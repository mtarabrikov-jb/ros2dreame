//! Direct data source (ava OFF): open the MCU (`/dev/ttyS4`) + LDS (`/dev/ttyS3`)
//! serial ports ourselves and drive them in-process - no external daemon, no TCP
//! hop. Ported from `w10-mcud`: sustain the MCU protocol (MotorCtrl 50Hz + pong
//! to 0x0f + the periodic SetLED/SetCleaning/0x14/0x26/0x1d frames) so the board
//! stays healthy, keep a live cliff/bumper hazard gate, and keep the LDS turret
//! spinning. The RX stream is decoded straight into ROS 2 messages (Status ->
//! `/odom`, LDS -> `/scan`) via the shared `tap` builders.
//!
//! v1: telemetry + turret. Drive is present (watchdog + clamp + hazard gate) but
//! left disabled until `/cmd_vel` is wired - the robot does not move.

use std::fs::{File, OpenOptions};
use std::io::{Read, Write};
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::io::AsRawFd;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicU8, Ordering};
use std::sync::mpsc::Sender;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use dreame_w10_proto::lds::LdsScanner;
use dreame_w10_proto::{encode_frame, encode_motor_ctrl, parse_body, FrameScanner, Msg};

use crate::tap::{battery_msg, currents_msg, imu_from_status10, odom_from_status, Sweep, Tap};

const MAX_LINEAR_MM_S: f32 = 150.0;
const MAX_ROT_RAD_S: f32 = 1.5;
const WATCHDOG_MS: u64 = 500; // no fresh command for this long -> stop
const T_PING: u8 = 0x0f;

/// Shared driver state. `main` holds this to feed `/cmd_vel` later; the turret is
/// on and drive is disabled at startup (no motion until commanded).
pub struct Shared {
    start: Instant,
    shutdown: AtomicBool,
    hazard: AtomicBool,
    lidar_on: AtomicBool,
    // observe mode: stop driving the MCU (no MotorCtrl, no nav frames, turret
    // off) so the robot stays "idle/docked" - the only state in which the
    // vendor firmware lets the OV8856 RGB camera stream (see REVERSE_ENGINEERING;
    // RGB is dead in any active/nav mode). Keeps SetLED/SetCleaning heartbeats +
    // pong so telemetry (odom/imu) still flows; loses /scan (turret parked).
    observe: AtomicBool,
    // drive command (for a future /cmd_vel), gated by watchdog + clamp + hazard
    enabled: AtomicBool,
    linear_bits: AtomicU32,
    rot_bits: AtomicU32,
    last_cmd_ms: AtomicU64,
    // actuator levels (0..~150), sent in the periodic SetCleaning frame.
    side_brush: AtomicU8,
    main_brush: AtomicU8,
    fan: AtomicU8,
    pump: AtomicU8,
}

impl Shared {
    fn now_ms(&self) -> u64 {
        self.start.elapsed().as_millis() as u64
    }
    /// Set a drive command (mm/s, rad/s). Latches the watchdog. For /cmd_vel.
    pub fn set_drive(&self, linear_mm_s: f32, rot_rad_s: f32) {
        self.linear_bits.store(linear_mm_s.to_bits(), Ordering::Relaxed);
        self.rot_bits.store(rot_rad_s.to_bits(), Ordering::Relaxed);
        self.last_cmd_ms.store(self.now_ms(), Ordering::Relaxed);
        self.enabled.store(true, Ordering::Relaxed);
    }
    pub fn set_side_brush(&self, v: u8) {
        self.side_brush.store(v, Ordering::Relaxed);
    }
    pub fn set_main_brush(&self, v: u8) {
        self.main_brush.store(v, Ordering::Relaxed);
    }
    pub fn set_fan(&self, v: u8) {
        self.fan.store(v, Ordering::Relaxed);
    }
    pub fn set_pump(&self, v: u8) {
        self.pump.store(v, Ordering::Relaxed);
    }
}

fn clampf(v: f32, lo: f32, hi: f32) -> f32 {
    v.max(lo).min(hi)
}

/// Open a serial port in raw mode at `baud` 8N1.
fn open_serial(path: &str, baud: libc::speed_t) -> std::io::Result<File> {
    let f = OpenOptions::new()
        .read(true)
        .write(true)
        .custom_flags(libc::O_NOCTTY)
        .open(path)?;
    let fd = f.as_raw_fd();
    unsafe {
        let mut t: libc::termios = std::mem::zeroed();
        if libc::tcgetattr(fd, &mut t) != 0 {
            return Err(std::io::Error::last_os_error());
        }
        libc::cfmakeraw(&mut t);
        libc::cfsetispeed(&mut t, baud);
        libc::cfsetospeed(&mut t, baud);
        t.c_cflag |= libc::CLOCAL | libc::CREAD;
        t.c_cc[libc::VMIN] = 0;
        t.c_cc[libc::VTIME] = 1; // 0.1s read timeout
        if libc::tcsetattr(fd, libc::TCSANOW, &t) != 0 {
            return Err(std::io::Error::last_os_error());
        }
    }
    Ok(f)
}

fn send(w: &Mutex<File>, typ: u8, payload: &[u8]) {
    let mut buf = [0u8; 64];
    if let Some(n) = encode_frame(typ, payload, &mut buf) {
        if let Ok(mut f) = w.lock() {
            let _ = f.write_all(&buf[..n]);
        }
    }
}

/// RX: answer pings, track hazard, and decode Status -> `/odom`.
fn rx_loop(mut rd: File, w: Arc<Mutex<File>>, sh: Arc<Shared>, tx: Sender<Tap>) {
    let mut sc = FrameScanner::new();
    let mut buf = [0u8; 4096];
    let mut gyro_z_dps: f32 = 0.0;
    let (mut wl, mut wr, mut load) = (0i16, 0i16, 0i16);
    while !sh.shutdown.load(Ordering::Relaxed) {
        let n = match rd.read(&mut buf) {
            Ok(0) => continue,
            Ok(n) => n,
            Err(_) => {
                thread::sleep(Duration::from_millis(5));
                continue;
            }
        };
        for &b in &buf[..n] {
            let Some(body) = sc.push(b) else { continue };
            let Ok((typ, payload)) = parse_body(body) else { continue };
            if typ == T_PING && payload.len() >= 4 {
                send(&w, T_PING, &payload[..4]); // echo the ping's timestamp
                continue;
            }
            match Msg::decode(typ, payload) {
                Msg::Status10ms(s) => {
                    gyro_z_dps = s.gyro_deg_s()[2];
                    let _ = tx.send(Tap::Imu(Box::new(imu_from_status10(&s))));
                }
                Msg::Status20ms(s) => {
                    let _ = tx.send(Tap::Currents(Box::new(currents_msg(
                        wl, wr, s.roller_current, s.sidebrush_current, load,
                    ))));
                    let odom = odom_from_status(&s, gyro_z_dps);
                    if tx.send(Tap::Odom(Box::new(odom))).is_err() {
                        return;
                    }
                }
                Msg::Status100ms(s) => {
                    wl = s.left_current;
                    wr = s.right_current;
                    load = s.load;
                }
                Msg::Triggers(t) => {
                    // bumpers/wheel-float (raw[0] bits 4-7) or any cliff (raw[1]).
                    let hz = (t.raw[0] & 0xF0) != 0 || t.raw[1] != 0;
                    sh.hazard.store(hz, Ordering::Relaxed);
                    let _ = tx.send(Tap::Triggers {
                        dock: t.dock_sta(),
                        bumper: t.left_bumper() || t.right_bumper(),
                        cliff: t.any_cliff(),
                    });
                }
                Msg::Battery(b) => {
                    let _ = tx.send(Tap::Battery(Box::new(battery_msg(&b))));
                }
                _ => {}
            }
        }
    }
}

/// TX: MotorCtrl at 50 Hz plus the periodic frames `ava` emits (which also keep
/// the LDS turret spinning while `lidar_on`).
fn tx_loop(w: Arc<Mutex<File>>, sh: Arc<Shared>) {
    let mut tick: u64 = 0;
    let mut mbuf = [0u8; 64];
    // In observe (parked, turret off) ros2dreame emits the MCU camera-AI-reset
    // frame `0x1d [0x05, 0x00]`. ava's node_signal::AIReset2ComProcess builds
    // exactly this from a CAMERA_AI_RESET msg (byte0=0x00 = reset; 0x01 = enable,
    // which is what nav sends). Sent with the camera CLOSED it un-wedges a
    // turret-wedged RGB isp0 off-dock - no ava, no dock, no reboot (found by
    // disassembling node_signal.so; verified end to end). One-shot: the RGB helper
    // must open AFTER the reset lands. W10_CAM_SYNC_VAL overrides byte0;
    // W10_NO_CAM_SYNC disables it.
    let cam_sync = std::env::var_os("W10_NO_CAM_SYNC").is_none();
    let cam_sync_val: u8 = std::env::var("W10_CAM_SYNC_VAL").ok()
        .and_then(|s| s.parse().ok()).unwrap_or(0);
    while !sh.shutdown.load(Ordering::Relaxed) {
        let observe = sh.observe.load(Ordering::Relaxed);
        // SetLED / SetCleaning idle: harmless heartbeats ava always sends; keep
        // them in both modes so the MCU stays alive and streams telemetry.
        if tick % 25 == 5 {
            send(&w, 0x02, &[0x21]); // SetLED / heartbeat
        }
        if tick % 50 == 10 {
            // SetCleaning `[side, main, fan, pump, mode, 0]`. Idle keeps ava's
            // exact frame; any commanded actuator switches to vacuum mode (0x03).
            // (The MCU "active mode" byte does NOT gate the ToF - that was a red
            // herring; the ToF just needs ava fully dead. See docs/MCU + the
            // dreame-vacuum-livestream phase3 notes.)
            let (sb, mb, fan, pump) = (
                sh.side_brush.load(Ordering::Relaxed),
                sh.main_brush.load(Ordering::Relaxed),
                sh.fan.load(Ordering::Relaxed),
                sh.pump.load(Ordering::Relaxed),
            );
            let p = if sb | mb | fan | pump == 0 {
                [0x00, 0x01, 0x00, 0x00, 0x00, 0x00]
            } else {
                [sb, mb, fan, pump, 0x03, 0x00]
            };
            send(&w, 0x01, &p);
        }
        if observe {
            // Idle/parked: a zero MotorCtrl keeps the MCU streaming telemetry
            // (odom/imu), but NO nav frames and turret off -> not an active/nav
            // mode, so the RGB camera can stream. (If a zero MotorCtrl still
            // trips active mode and kills RGB, drop this send.)
            if let Some(n) = encode_motor_ctrl(1, 0.0, 0.0, &mut mbuf) {
                if let Ok(mut f) = w.lock() {
                    let _ = f.write_all(&mbuf[..n]);
                }
            }
            if cam_sync && tick % 50 == 40 {
                send(&w, 0x1d, &[0x05, cam_sync_val]); // MCU camera-AI-reset -> un-wedge RGB isp0
            }
            tick = tick.wrapping_add(1);
            thread::sleep(Duration::from_millis(20));
            continue;
        }
        let fresh =
            sh.now_ms().wrapping_sub(sh.last_cmd_ms.load(Ordering::Relaxed)) < WATCHDOG_MS;
        let (mut lin, rot) = if sh.enabled.load(Ordering::Relaxed) && fresh {
            (
                clampf(f32::from_bits(sh.linear_bits.load(Ordering::Relaxed)), -MAX_LINEAR_MM_S, MAX_LINEAR_MM_S),
                clampf(f32::from_bits(sh.rot_bits.load(Ordering::Relaxed)), -MAX_ROT_RAD_S, MAX_ROT_RAD_S),
            )
        } else {
            (0.0, 0.0)
        };
        if sh.hazard.load(Ordering::Relaxed) && lin != 0.0 {
            lin = 0.0; // never translate into a detected cliff/bump
        }
        if let Some(n) = encode_motor_ctrl(1, lin, rot, &mut mbuf) {
            if let Ok(mut f) = w.lock() {
                let _ = f.write_all(&mbuf[..n]);
            }
        }
        let lidar = sh.lidar_on.load(Ordering::Relaxed);
        if tick % 50 == 20 {
            send(&w, 0x14, if lidar { &[0x04, 0x01] } else { &[0x04, 0x00] });
        }
        if tick % 50 == 30 {
            let p: &[u8] = if lidar {
                &[0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04]
            } else {
                &[0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04]
            };
            send(&w, 0x26, p);
        }
        if lidar && tick % 200 == 40 {
            send(&w, 0x1d, &[0x05, 0x01]); // laser enable re-pulse
        }
        tick = tick.wrapping_add(1);
        thread::sleep(Duration::from_millis(20)); // ~50 Hz
    }
    if let Some(n) = encode_motor_ctrl(1, 0.0, 0.0, &mut mbuf) {
        if let Ok(mut f) = w.lock() {
            let _ = f.write_all(&mbuf[..n]); // leave motors stopped
        }
    }
}

/// LDS: read ttyS3, de-frame, accumulate sweeps -> `/scan`.
fn lds_loop(mut rd: File, sh: Arc<Shared>, tx: Sender<Tap>) {
    let mut sc = LdsScanner::new();
    let mut sweep = Sweep::new();
    let mut buf = [0u8; 4096];
    while !sh.shutdown.load(Ordering::Relaxed) {
        let n = match rd.read(&mut buf) {
            Ok(0) => continue,
            Ok(n) => n,
            Err(_) => {
                thread::sleep(Duration::from_millis(5));
                continue;
            }
        };
        for &b in &buf[..n] {
            if let Some(f) = sc.push(b) {
                if let Some(scan) = sweep.push(&f) {
                    if tx.send(Tap::Scan(Box::new(scan))).is_err() {
                        return;
                    }
                }
            }
        }
    }
}

/// Open the serial ports and start the driver threads. Returns the shared state
/// (turret already enabled; drive disabled until `/cmd_vel`). Exits the process
/// if the MCU port can't be opened (ava still running?).
pub fn run(mcu_path: &str, lds_path: &str, observe: bool, tx: Sender<Tap>) -> Arc<Shared> {
    let sh = Arc::new(Shared {
        start: Instant::now(),
        shutdown: AtomicBool::new(false),
        hazard: AtomicBool::new(false),
        // turret spins for /scan in nav mode only. W10_NO_TURRET forces it off
        // even in nav (still drive-capable, but no LDS/scan): used to test whether
        // the spinning LDS turret is what disrupts the OV8856/isp0 MIPI timing and
        // stalls RGB in nav.
        lidar_on: AtomicBool::new(!observe && std::env::var_os("W10_NO_TURRET").is_none()),
        observe: AtomicBool::new(observe),
        enabled: AtomicBool::new(false),
        linear_bits: AtomicU32::new(0),
        rot_bits: AtomicU32::new(0),
        last_cmd_ms: AtomicU64::new(0),
        side_brush: AtomicU8::new(0),
        main_brush: AtomicU8::new(0),
        fan: AtomicU8::new(0),
        pump: AtomicU8::new(0),
    });

    let rd = match open_serial(mcu_path, libc::B115200) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("direct: cannot open {mcu_path} ({e}) - is ava stopped?");
            std::process::exit(1);
        }
    };
    let wr = rd.try_clone().expect("clone MCU fd");
    let w = Arc::new(Mutex::new(wr));
    if observe {
        log::info!("direct: MCU {mcu_path} OBSERVE mode (idle heartbeats only, turret off; RGB can stream, no /scan)");
    } else {
        log::info!("direct: driving MCU {mcu_path} (MotorCtrl 50Hz + pong + heartbeats), turret on");
    }

    {
        let (w, sh, tx) = (w.clone(), sh.clone(), tx.clone());
        thread::spawn(move || rx_loop(rd, w, sh, tx));
    }
    {
        let (w, sh) = (w.clone(), sh.clone());
        thread::spawn(move || tx_loop(w, sh));
    }
    match open_serial(lds_path, libc::B230400) {
        Ok(lds_rd) => {
            log::info!("direct: LDS {lds_path} -> /scan");
            let (sh, tx) = (sh.clone(), tx);
            thread::spawn(move || lds_loop(lds_rd, sh, tx));
        }
        Err(e) => log::warn!("direct: cannot open {lds_path} ({e}); /scan disabled"),
    }
    sh
}
