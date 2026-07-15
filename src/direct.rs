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

use crate::tap::{battery_msg, imu_from_status10, odom_from_status, Sweep, Tap};

const MAX_LINEAR_MM_S: f32 = 150.0;
const MAX_ROT_RAD_S: f32 = 1.5;
const WATCHDOG_MS: u64 = 500; // no fresh command for this long -> stop
const T_PING: u8 = 0x0f;

/// Shared driver state. `main` holds this to feed `/cmd_vel` later; the turret is
/// on and drive is disabled at startup (no motion until commanded).
pub struct Shared {
    start: Instant,
    shutdown: AtomicBool,
    // Directional drive gate from the Triggers frame. hazard_fwd blocks forward
    // motion (front bumper, front/mid cliff, or a wheel drop); hazard_rev blocks
    // reverse (rear/mid cliff, or a wheel drop). Split so the robot can back AWAY
    // from a front bump instead of being frozen against it.
    hazard_fwd: AtomicBool,
    hazard_rev: AtomicBool,
    // Front bumper contact (raw[0] bits 4/5) - drives the bump-escape reflex in
    // tx_loop (back away, then turn away from the hit side). bump_bits: bit0 = left,
    // bit1 = right, for the turn direction.
    bumper: AtomicBool,
    bump_bits: AtomicU8,
    lidar_on: AtomicBool,
    // observe mode: stop driving the MCU (no MotorCtrl, no nav frames, turret
    // off) so the robot stays "idle/docked" - the only state in which the
    // vendor firmware lets the OV8856 RGB camera stream (see REVERSE_ENGINEERING;
    // RGB is dead in any active/nav mode). Keeps SetLED/SetCleaning heartbeats +
    // pong so telemetry (odom/imu) still flows; loses /scan (turret parked).
    observe: AtomicBool,
    // drive command (for a future /cmd_vel), gated by watchdog + clamp + hazard
    enabled: AtomicBool,
    // W10_AUTO: paused while the user drives things manually (a /set_turret click
    // takes over park/drive; /set_auto true resumes motion-based auto-switching).
    auto_paused: AtomicBool,
    linear_bits: AtomicU32,
    rot_bits: AtomicU32,
    last_cmd_ms: AtomicU64,
    // last time a NON-zero drive command arrived - the auto mode (W10_AUTO) parks
    // (turret off -> RGB+IR) after this goes stale and drives (turret on -> /scan+IR)
    // while it is fresh. Distinct from last_cmd_ms (the drive watchdog), which a
    // zero "stop" command still refreshes.
    last_move_ms: AtomicU64,
    // actuator levels (0..~150), sent in the periodic SetCleaning frame.
    side_brush: AtomicU8,
    main_brush: AtomicU8,
    fan: AtomicU8,
    mop: AtomicU8,
    // Base-station function, driven via the 0x26 frame (0 = idle, 1 = dry the mop
    // pads (dock fan), 2 = wash the mop pads (dock water pump)). Reverse-engineered
    // by snooping ava's ttyS4 while triggering wash/dry from Valetudo.
    station: AtomicU8,
    // Dock LCD screen/status code. The 0x26 frame's byte0 is the dock "mode" the
    // dock MCU renders on its own LCD (byte0=0x14 idle, 0x0d wash, 0x0e dry are the
    // known ones; the dock has more screen codes - see VacuumTiger DOCK_PROTOCOL.md).
    // When idle (station=0) and this is non-zero, we send an idle-shaped 0x26 with
    // byte0 = this code, so the robot MCU relays it to the dock and the LCD shows it.
    // 0 = leave the dock alone. Byte0->screen mapping beyond idle/wash/dry is not yet
    // verified on the LCD; treat values as experimental (drive /set_dock_screen and
    // watch the panel). The frame is otherwise the safe idle frame (no pump/fan).
    dock_screen: AtomicU8,
    // Raw 8-byte 0x26 dock payload override, packed little-endian into a u64 (0 =
    // none). Sent verbatim as the 0x26 frame while idle, overriding dock_screen -
    // the full lever for building the byte0->screen table and driving progress
    // screens (byte7) by hand. UNSAFE: byte6 != 0 can start the dock pump/fan, so
    // only set byte6/params deliberately. Real frames set byte7=0x02, so a valid
    // frame is never all-zero; all-zero clears the override.
    dock_frame: AtomicU64,
}

impl Shared {
    fn now_ms(&self) -> u64 {
        self.start.elapsed().as_millis() as u64
    }
    /// Set a drive command (mm/s, rad/s). Latches the watchdog. For /cmd_vel.
    pub fn set_drive(&self, linear_mm_s: f32, rot_rad_s: f32) {
        self.linear_bits.store(linear_mm_s.to_bits(), Ordering::Relaxed);
        self.rot_bits.store(rot_rad_s.to_bits(), Ordering::Relaxed);
        let now = self.now_ms();
        self.last_cmd_ms.store(now, Ordering::Relaxed);
        if linear_mm_s != 0.0 || rot_rad_s != 0.0 {
            self.last_move_ms.store(now, Ordering::Relaxed);
        }
        self.enabled.store(true, Ordering::Relaxed);
    }
    /// ms since the last non-zero drive command (auto mode's drive/park decision).
    /// Returns u64::MAX before the first motion command so we start/stay PARKED.
    pub fn idle_move_ms(&self) -> u64 {
        let last = self.last_move_ms.load(Ordering::Relaxed);
        if last == 0 {
            return u64::MAX;
        }
        self.now_ms().saturating_sub(last)
    }
    pub fn is_parked(&self) -> bool {
        self.observe.load(Ordering::Relaxed)
    }
    /// Auto mode: park (turret OFF -> RGB reset+capture) vs drive (turret ON ->
    /// /scan). Flips both `observe` (tx_loop path) and `lidar_on` (turret) together.
    pub fn set_parked(&self, parked: bool) {
        self.observe.store(parked, Ordering::Relaxed);
        self.lidar_on.store(!parked, Ordering::Relaxed);
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
    /// Rotating mop pads (the two spinning mop discs). SetCleaning byte[3]. The
    /// robot has no water pump - only these pads; the base station's pump + drying
    /// fan are separate (MIoT/dock, not this MCU).
    pub fn set_mop(&self, v: u8) {
        self.mop.store(v, Ordering::Relaxed);
    }
    /// Base-station function (0 = idle, 1 = dry, 2 = wash) -> the 0x26 frame.
    /// 2 (wash) runs the dock water pump - do not leave it on unattended.
    pub fn set_station(&self, v: u8) {
        self.station.store(v, Ordering::Relaxed);
    }
    /// Dock LCD screen/status code -> the 0x26 frame's byte0 (dock "mode"), relayed
    /// by the robot MCU to the dock and rendered on its LCD. 0 = leave the dock
    /// alone; non-zero sends an idle-shaped 0x26 (no pump/fan) with byte0 = the code
    /// while station is idle. Known: 0x14 idle, 0x0d wash, 0x0e dry; other dock
    /// screen codes are experimental (see VacuumTiger DOCK_PROTOCOL.md). Ignored
    /// while a wash/dry is running (those drive the screen themselves).
    pub fn set_dock_screen(&self, v: u8) {
        self.dock_screen.store(v, Ordering::Relaxed);
    }
    /// Raw 8-byte 0x26 dock payload override (from `/set_dock_frame`). >=8 bytes ->
    /// the first 8 are sent verbatim as the periodic 0x26 while idle (overrides
    /// `set_dock_screen`); fewer than 8 (or all-zero) clears the override. Full
    /// manual control for RE - byte0=screen, byte6!=0 drives pump/fan (careful).
    pub fn set_dock_frame(&self, bytes: &[u8]) {
        let packed = if bytes.len() >= 8 {
            u64::from_le_bytes(bytes[..8].try_into().unwrap())
        } else {
            0
        };
        self.dock_frame.store(packed, Ordering::Relaxed);
    }
    /// Turret (LDS) currently commanded on (driving) vs off (parked).
    pub fn turret_on(&self) -> bool {
        self.lidar_on.load(Ordering::Relaxed)
    }
    /// Commanded actuator levels: (fan, side_brush, main_brush, mop).
    pub fn levels(&self) -> (u8, u8, u8, u8) {
        (
            self.fan.load(Ordering::Relaxed),
            self.side_brush.load(Ordering::Relaxed),
            self.main_brush.load(Ordering::Relaxed),
            self.mop.load(Ordering::Relaxed),
        )
    }
    /// Manual turret toggle from the GUI: on -> drive state (turret spins, /scan,
    /// cameras -> ToF/IR, RGB wedged); off -> park state (turret off, RGB
    /// un-wedge reset, both cameras). Pauses the W10_AUTO motion auto-switch so
    /// the manual choice sticks; publish /set_auto true to resume auto.
    pub fn set_turret(&self, on: bool) {
        self.auto_paused.store(true, Ordering::Relaxed);
        self.set_parked(!on);
    }
    /// Resume (true) / pause (false) the W10_AUTO motion-based auto-switch.
    pub fn set_auto(&self, on: bool) {
        self.auto_paused.store(!on, Ordering::Relaxed);
    }
    pub fn auto_paused(&self) -> bool {
        self.auto_paused.load(Ordering::Relaxed)
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
    // RX frame-change debug (W10_RX_DEBUG=1): log a frame only when its payload
    // changes vs the last of that type - finds which frame/bit a button toggles.
    let rx_debug = std::env::var("W10_RX_DEBUG").is_ok();
    let mut last_frame: std::collections::HashMap<u8, Vec<u8>> = std::collections::HashMap::new();
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
            // skip the high-rate counter frames (they change every frame -> spam)
            if rx_debug
                && !matches!(typ, 0x01 | 0x02 | 0x03 | 0x05 | 0x12 | 0x2c)
                && last_frame.get(&typ).map(Vec::as_slice) != Some(payload)
            {
                last_frame.insert(typ, payload.to_vec());
                let hex: String = payload.iter().map(|b| format!("{:02x}", b)).collect();
                eprintln!("RXD type=0x{:02x} len={} [{}]", typ, payload.len(), hex);
            }
            // Base-station buttons: 0x23 dock-status frame byte0 (bit0=Home,
            // bit2=Start/Stop; bit4=0x10 is a constant docked flag). Verified live.
            if typ == 0x23 && !payload.is_empty() {
                let b0 = payload[0];
                let _ = tx.send(Tap::DockButton { home: b0 & 0x01 != 0, start: b0 & 0x04 != 0 });
            }
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
                    let _ = tx.send(Tap::Currents([wl, wr, s.roller_current, s.sidebrush_current, load]));
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
                    // Directional drive gate. raw[0]: bits 4/5 = front bumpers,
                    // bits 6/7 = wheel drop (lift -> block both ways). raw[1] = 6
                    // cliff sensors: bits 0-3 = front+mid (block forward), bits 1,2,4,5
                    // = mid+rear (block reverse); mid (1,2, position unverified) blocks
                    // both to be safe. So the robot can still back away from a front bump.
                    let bumper = (t.raw[0] & 0x30) != 0;
                    let wheel = (t.raw[0] & 0xC0) != 0;
                    let cliff_fwd = (t.raw[1] & 0x0F) != 0; // bits 0-3: front-left/mid/mid/front-right
                    let cliff_rev = (t.raw[1] & 0x36) != 0; // bits 1,2,4,5: mid + rear
                    sh.hazard_fwd.store(bumper || wheel || cliff_fwd, Ordering::Relaxed);
                    sh.hazard_rev.store(wheel || cliff_rev, Ordering::Relaxed);
                    sh.bumper.store(bumper, Ordering::Relaxed);
                    sh.bump_bits.store((t.raw[0] >> 4) & 0x03, Ordering::Relaxed); // bit0=left, bit1=right
                    let _ = tx.send(Tap::Triggers {
                        dock: t.dock_sta(),
                        bumper_bits: (t.left_bumper() as u8) | ((t.right_bumper() as u8) << 1),
                        cliff_bits: t.cliff_flags() & 0x3f,
                        wheel_bits: (t.left_wheel_floating() as u8) | ((t.right_wheel_floating() as u8) << 1),
                        fan_oc: t.fan_overcurrent(),
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
/// ava's mop-wash steps, captured by snooping ttyS4 during a Valetudo-triggered
/// wash. Each step: (0x26 pump frame, mop-pad level, duration ms). Wet wash (water
/// on + pads high, pump speed ramps) -> water off, pads scrub the worked-in water.
/// Replayed on /set_station 2, then followed by the dry stages (DRY_SCREENS). During
/// a step the SetCleaning frame drives the pads (`[00 01 00 <pad> 00 00]`, mop mode
/// 00). byte0 of each frame is also the dock LCD screen (0x0d = "Mop pad cleaning").
const WASH_STEPS: &[([u8; 8], u8, u64)] = &[
    ([0x0d, 0x00, 0x46, 0, 0, 0, 0, 0x02], 0xd6, 20_700), // wet (slow pump) + pads high
    ([0x0d, 0x78, 0x46, 0, 0, 0, 0, 0x02], 0x00, 8_000),  // wet (fast pump) + pads off
    ([0x0d, 0x64, 0x46, 0, 0, 0, 0, 0x02], 0x2a, 4_300),  // wet + pads low
    ([0x0d, 0x64, 0x00, 0, 0, 0, 0, 0x02], 0x2a, 12_000), // water off + pads low
    ([0x0d, 0x00, 0x00, 0, 0, 0, 0, 0x02], 0x2a, 15_000), // idle pump + pads low
    ([0x0d, 0x00, 0x00, 0, 0, 0, 0, 0x02], 0x4b, 65_000), // scrub + pads medium
    ([0x0d, 0x64, 0x00, 0, 0, 0, 0, 0x02], 0xd6, 10_000), // scrub + pads high
    ([0x0d, 0x64, 0x00, 0, 0, 0, 0, 0x02], 0x00, 8_000),  // pads off
];

/// Dock drying-fan stages: the fan runs the whole time (byte3=0x78, byte6=0x01) while
/// byte0 walks the dock's four "Mop pad dehydrating N/4" LCD screens (0x0e=1/4,
/// 0x65/0x66/0x67=2/4..4/4, verified live) so the panel shows real dry progress
/// instead of a static 1/4. Used by /set_station 1 (dry) and appended after
/// WASH_STEPS for /set_station 2. Per-stage duration = W10_DRY_STAGE_MS (default 5 min).
const DRY_SCREENS: [u8; 4] = [0x0e, 0x65, 0x66, 0x67];

/// The active step at `elapsed_ms` into a timed (0x26 frame, pad, duration) sequence,
/// or None once it is over.
fn seq_step(steps: &[([u8; 8], u8, u64)], elapsed_ms: u64) -> Option<&([u8; 8], u8, u64)> {
    let mut acc = 0u64;
    for s in steps {
        acc += s.2;
        if elapsed_ms < acc {
            return Some(s);
        }
    }
    None
}

fn tx_loop(w: Arc<Mutex<File>>, sh: Arc<Shared>) {
    let mut tick: u64 = 0;
    let mut mbuf = [0u8; 64];
    let mut wash_start_ms: u64 = 0; // 0 = not running a wash cycle
    // Dock off-pulse: the base station keeps its pump/fan running until told to
    // stop, so when /set_station returns to 0 we send the idle 0x26 a few times to
    // actively stop it (then go quiet so parked-mode RGB is undisturbed).
    let mut prev_station: u8 = 0;
    let mut off_pulse: u32 = 5; // also pulse on startup, to stop any dock cycle left running
    // Dry stages (advancing dehydrating screen + fan). Per-stage duration is
    // W10_DRY_STAGE_MS (default 5 min -> 20 min total); a short value eases testing.
    let dry_ms: u64 = std::env::var("W10_DRY_STAGE_MS").ok().and_then(|v| v.parse().ok()).unwrap_or(300_000);
    let dry_steps: Vec<([u8; 8], u8, u64)> = DRY_SCREENS
        .iter()
        .map(|&c| ([c, 0, 0, 0x78, 0, 0, 0x01, 0x02], 0x00u8, dry_ms))
        .collect();
    // Full wash = the wet-wash steps, then the dry stages.
    let wash_seq: Vec<([u8; 8], u8, u64)> = WASH_STEPS.iter().cloned().chain(dry_steps.iter().cloned()).collect();
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
    // MCU 0x14 init register group. ava streams FOUR 0x14 `[reg][val]` frames every
    // cycle: reg 0x01=01, 0x00=01, 0x09=02, 0x04=00 (0x04 = lidar). Without these the
    // MCU does NOT scan the 6 downward cliff sensors - Triggers raw[1] stays 0 under
    // de-ava'd ros2dreame even on a full lift; with the group raw[1] fires 0x00->0x3f
    // exactly like ava (verified live). We send regs 0x00/0x01/0x09 here and leave
    // reg 0x04 to the lidar logic below (it toggles 04 00/01 with the turret; sending
    // a second fixed 04 here would fight it in nav). Harmless to RGB in observe
    // (verified: video2 keeps streaming). Escape hatch: W10_NO_MCU_INIT disables it.
    let mcu_init = std::env::var_os("W10_NO_MCU_INIT").is_none();
    // Bump-escape reflex: on a front bumper hit, reverse a little then turn away from
    // the hit side, overriding the planner for a moment (a vacuum recovers from an
    // LDS-invisible obstacle by backing off and turning, not by pushing). Durations
    // are env-tunable; W10_NO_BUMP_ESCAPE disables it. phase: 0 idle, 1 backing, 2 turning.
    let bump_escape = std::env::var_os("W10_NO_BUMP_ESCAPE").is_none();
    let back_ms: u64 = std::env::var("W10_BUMP_BACK_MS").ok().and_then(|v| v.parse().ok()).unwrap_or(700);
    let turn_ms: u64 = std::env::var("W10_BUMP_TURN_MS").ok().and_then(|v| v.parse().ok()).unwrap_or(1100);
    const BUMP_BACK_MM_S: f32 = 80.0;
    const BUMP_TURN_RAD_S: f32 = 0.6;
    let mut esc_phase: u8 = 0;
    let mut esc_start: u64 = 0;
    let mut esc_turn: f32 = -1.0;
    while !sh.shutdown.load(Ordering::Relaxed) {
        let observe = sh.observe.load(Ordering::Relaxed);
        if mcu_init && tick % 50 == 15 {
            send(&w, 0x14, &[0x01, 0x01]);
            send(&w, 0x14, &[0x00, 0x01]);
            send(&w, 0x14, &[0x09, 0x02]);
        }
        // SetLED / SetCleaning idle: harmless heartbeats ava always sends; keep
        // them in both modes so the MCU stays alive and streams telemetry.
        if tick % 25 == 5 {
            send(&w, 0x02, &[0x21]); // SetLED / heartbeat
        }
        // Base-station cycles: station=2 = full wash (WASH_STEPS wet steps + dry
        // stages), station=1 = dry only (dry stages). Drives the mop pads (SetCleaning
        // below) and the pump/fan (0x26 below); each frame's byte0 also drives the
        // matching dock LCD screen. After the last step the driver returns station to 0
        // (idle + off-pulse). Sent in both modes since the robot is parked while docked.
        let mut station = sh.station.load(Ordering::Relaxed);
        let (mut wash_pump, mut wash_pad): (Option<[u8; 8]>, Option<u8>) = (None, None);
        // station 2 = full wash (wet steps + dry stages); station 1 = dry only. Both
        // are timed sequences whose 0x26 byte0 also walks the matching dock LCD screen
        // (0x0d cleaning during wash, 0x0e/0x65/0x66/0x67 = dehydrating 1/4..4/4 on dry).
        let seq: Option<&[([u8; 8], u8, u64)]> = match station {
            2 => Some(&wash_seq),
            1 => Some(&dry_steps),
            _ => None,
        };
        if let Some(seq) = seq {
            if wash_start_ms == 0 {
                wash_start_ms = sh.now_ms();
            }
            match seq_step(seq, sh.now_ms().saturating_sub(wash_start_ms)) {
                Some(s) => {
                    wash_pump = Some(s.0);
                    wash_pad = Some(s.1);
                }
                None => {
                    sh.station.store(0, Ordering::Relaxed); // cycle finished
                    station = 0;
                    wash_start_ms = 0;
                }
            }
        } else {
            wash_start_ms = 0;
        }
        if station == 0 && prev_station != 0 {
            off_pulse = 5; // just switched off -> pulse the idle 0x26 to stop the dock
        }
        prev_station = station;
        if tick % 50 == 10 {
            // SetCleaning `[side, main, fan, mop, mode, 0]`. During a wash the cycle
            // drives the mop pads (mode 00); otherwise the commanded actuator levels
            // (any set -> vacuum mode 0x03; idle keeps ava's exact frame). The MCU
            // active-mode byte does NOT gate the ToF (red herring) - see docs/MCU.
            let p = if let Some(pad) = wash_pad {
                [0x00, 0x01, 0x00, pad, 0x00, 0x00]
            } else {
                let (sb, mb, fan, mop) = (
                    sh.side_brush.load(Ordering::Relaxed),
                    sh.main_brush.load(Ordering::Relaxed),
                    sh.fan.load(Ordering::Relaxed),
                    sh.mop.load(Ordering::Relaxed),
                );
                if sb | mb | fan | mop == 0 {
                    [0x00, 0x01, 0x00, 0x00, 0x00, 0x00]
                } else {
                    [sb, mb, fan, mop, 0x03, 0x00]
                }
            };
            send(&w, 0x01, &p);
        }
        // Base-station (dock) 0x26 - sent in BOTH modes (robot parked while docked).
        if tick % 50 == 30 {
            let p: Option<[u8; 8]> = if let Some(pump) = wash_pump {
                Some(pump)
            } else {
                match station {
                    _ if off_pulse > 0 => {
                        off_pulse -= 1;
                        // Dock stop/idle = ava's exact frame `26 15 ..` (byte0=0x15).
                        // This is what STOPS a running wash/dry - snooped from ava's
                        // ttyS4 as the "stop mop drying" command (Valetudo). byte0=0x14
                        // (an earlier guess) does NOT abort a dry; 0x15 does.
                        Some([0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02]) // idle/stop -> dock off
                    }
                    // Idle: a raw 0x26 override (/set_dock_frame) wins; else a screen
                    // code (/set_dock_screen) -> idle-shaped 0x26 with byte0 = code
                    // (no pump/fan, byte6=0); else leave the dock alone.
                    _ => {
                        let raw = sh.dock_frame.load(Ordering::Relaxed);
                        if raw != 0 {
                            Some(raw.to_le_bytes())
                        } else {
                            match sh.dock_screen.load(Ordering::Relaxed) {
                                // Docked/parked idle = ava's `26 15 ..` heartbeat, sent
                                // CONTINUOUSLY (like ava streams it). This keeps a
                                // stopped dock idle and is what makes /set_station 0
                                // actually ABORT a wash/dry - a few frames don't stick.
                                0 if observe => Some([0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02]),
                                0 => None, // nav mode: the nav-heartbeat 0x26 (below) drives it
                                s => Some([s, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02]),
                            }
                        }
                    }
                }
            };
            if let Some(p) = p {
                send(&w, 0x26, &p);
            }
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
        let (mut lin, mut rot) = if sh.enabled.load(Ordering::Relaxed) && fresh {
            (
                clampf(f32::from_bits(sh.linear_bits.load(Ordering::Relaxed)), -MAX_LINEAR_MM_S, MAX_LINEAR_MM_S),
                clampf(f32::from_bits(sh.rot_bits.load(Ordering::Relaxed)), -MAX_ROT_RAD_S, MAX_ROT_RAD_S),
            )
        } else {
            (0.0, 0.0)
        };
        // Directional hazard gate: block forward into a front bump/cliff, block
        // reverse into a rear cliff - but still allow backing away from a front bump.
        if lin > 0.0 && sh.hazard_fwd.load(Ordering::Relaxed) {
            lin = 0.0;
        } else if lin < 0.0 && sh.hazard_rev.load(Ordering::Relaxed) {
            lin = 0.0;
        }
        // Bump-escape reflex (overrides the planner). Start on a fresh bumper hit.
        if bump_escape && esc_phase == 0 && sh.bumper.load(Ordering::Relaxed) {
            esc_phase = 1;
            esc_start = sh.now_ms();
            // turn away from the hit side: left-only -> turn right (CW, -); right-only
            // -> turn left (CCW, +); head-on -> default right.
            let b = sh.bump_bits.load(Ordering::Relaxed);
            esc_turn = if b == 0x01 { -1.0 } else if b == 0x02 { 1.0 } else { -1.0 };
        }
        if esc_phase == 1 {
            // back away, unless a rear cliff/lift blocks it -> skip to turning
            if sh.now_ms().saturating_sub(esc_start) < back_ms && !sh.hazard_rev.load(Ordering::Relaxed) {
                lin = -BUMP_BACK_MM_S;
                rot = 0.0;
            } else {
                esc_phase = 2;
                esc_start = sh.now_ms();
            }
        }
        if esc_phase == 2 {
            if sh.now_ms().saturating_sub(esc_start) < turn_ms {
                lin = 0.0;
                rot = esc_turn * BUMP_TURN_RAD_S;
            } else {
                esc_phase = 0; // reflex done -> planner resumes
            }
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
        if tick % 50 == 30 && station == 0 {
            // nav-mode 0x26 idle heartbeat (mcud value; keeps the MCU in nav mode /
            // turret spinning). The dock command (station != 0) is sent above so it
            // also works while parked. See docs/MCU.md for the 0x26 dock semantics.
            let p: [u8; 8] = if lidar {
                [0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04]
            } else {
                [0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04]
            };
            send(&w, 0x26, &p);
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
        hazard_fwd: AtomicBool::new(false),
        hazard_rev: AtomicBool::new(false),
        bumper: AtomicBool::new(false),
        bump_bits: AtomicU8::new(0),
        // turret spins for /scan in nav mode only. W10_NO_TURRET forces it off
        // even in nav (still drive-capable, but no LDS/scan): used to test whether
        // the spinning LDS turret is what disrupts the OV8856/isp0 MIPI timing and
        // stalls RGB in nav.
        lidar_on: AtomicBool::new(!observe && std::env::var_os("W10_NO_TURRET").is_none()),
        observe: AtomicBool::new(observe),
        enabled: AtomicBool::new(false),
        auto_paused: AtomicBool::new(false),
        last_move_ms: AtomicU64::new(0),
        linear_bits: AtomicU32::new(0),
        rot_bits: AtomicU32::new(0),
        last_cmd_ms: AtomicU64::new(0),
        side_brush: AtomicU8::new(0),
        main_brush: AtomicU8::new(0),
        fan: AtomicU8::new(0),
        mop: AtomicU8::new(0),
        station: AtomicU8::new(0),
        dock_screen: AtomicU8::new(0),
        dock_frame: AtomicU64::new(0),
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
    {
        // Periodic actuator/turret state telemetry -> /state/* (not event-driven
        // like the currents/odom decoded from the MCU stream).
        let (sh, tx) = (sh.clone(), tx.clone());
        thread::spawn(move || {
            while !sh.shutdown.load(Ordering::Relaxed) {
                let (fan, side_brush, main_brush, mop) = sh.levels();
                if tx
                    .send(Tap::State { turret: sh.turret_on(), fan, side_brush, main_brush, mop })
                    .is_err()
                {
                    return;
                }
                thread::sleep(Duration::from_millis(400));
            }
        });
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
