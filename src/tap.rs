//! Decode helpers + the TCP tap data source.
//!
//! The message builders (`odom_from_status`, `Sweep`/`build_scan`) are shared by
//! both data sources: the `direct` serial driver (ava off) and this `tap` reader
//! (ava on - reads `avatap-relay`'s `mcu-rx` 7701 / `lds-rx` 7702 mirror over TCP,
//! read-only, alongside the vendor `ava`). Finished ROS 2 messages go over an
//! mpsc channel to the publisher loop in `main`.

use std::io::Read;
use std::net::TcpStream;
use std::sync::mpsc::Sender;
use std::thread;
use std::time::Duration;

use dreame_w10_proto::lds::{LdsFrame, LdsScanner, LDS_ANGLE_FULL};
use dreame_w10_proto::{parse_body, Battery, FrameScanner, Msg, Status10ms, Status20ms};

use crate::msg::{
    self, BatteryState, Header, Imu, Int16MultiArray, LaserScan, Odometry, Point, Pose,
    PoseWithCovariance, Twist, TwistWithCovariance, Vector3,
};

/// A finished message from a reader thread, ready to publish.
pub enum Tap {
    Odom(Box<Odometry>),
    Scan(Box<LaserScan>),
    Image(Box<crate::msg::CompressedImage>),
    Imu(Box<Imu>),
    Battery(Box<BatteryState>),
    Triggers { dock: bool, bumper: bool, cliff: bool },
    /// [wheel_left, wheel_right, main_brush, side_brush, load] raw i16 currents.
    Currents([i16; 5]),
    /// Actuator/turret state telemetry (published periodically, not event-driven).
    State { turret: bool, fan: u8, side_brush: u8, main_brush: u8, pump: u8 },
}

// --- LDS -> LaserScan geometry (W10) -----------------------------------------
// The W10 LDS is a fixed ~126 deg rear arc, spinning CW. These map the raw
// sensor angle to a REP-103 (CCW, forward = 0) laser frame; calibrated on this
// robot (see VacuumTiger/sangamio docs/dreame_w10.md). Baked into the scan angle
// only; the laser's XY mounting offset belongs in TF, not here.
const ANGLE_SCALE: f32 = -1.0; // CW sensor -> CCW ROS
const ANGLE_OFFSET: f32 = 0.506; // rad, zero the sensor toward robot forward
const SCAN_INCREMENT: f32 = std::f32::consts::PI / 180.0; // 1 deg bins
const RANGE_MIN: f32 = 0.05;
const RANGE_MAX: f32 = 8.0;
const FSA_WRAP: i32 = 8192; // fsa drop > 45 deg (u16 units) = new sweep

const TAU: f32 = std::f32::consts::TAU;

fn wrap_pi(a: f32) -> f32 {
    let mut a = a % TAU;
    if a > std::f32::consts::PI {
        a -= TAU;
    } else if a < -std::f32::consts::PI {
        a += TAU;
    }
    a
}

/// `Status20ms` (pose/velocity) + IMU yaw rate -> `nav_msgs/Odometry`.
pub(crate) fn odom_from_status(s: &Status20ms, gyro_z_dps: f32) -> Odometry {
    let x_m = s.x_mm10 as f64 / 10.0 / 1000.0;
    let y_m = s.y_mm10 as f64 / 10.0 / 1000.0;
    let yaw = (s.yaw_deg() as f64).to_radians();
    let v_lin = (s.left_vel as f64 + s.right_vel as f64) / 2.0 / 1000.0;
    let v_ang = (gyro_z_dps as f64).to_radians();
    Odometry {
        header: Header { stamp: msg::now(), frame_id: "odom".into() },
        child_frame_id: "base_link".into(),
        pose: PoseWithCovariance {
            pose: Pose {
                position: Point { x: x_m, y: y_m, z: 0.0 },
                orientation: msg::yaw_to_quat(yaw),
            },
            covariance: [0.0; 36],
        },
        twist: TwistWithCovariance {
            twist: Twist {
                linear: Vector3 { x: v_lin, y: 0.0, z: 0.0 },
                angular: Vector3 { x: 0.0, y: 0.0, z: v_ang },
            },
            covariance: [0.0; 36],
        },
    }
}

/// `Status10ms` (IMU) -> `sensor_msgs/Imu`. Gyro deg/s -> rad/s, accel g -> m/s2.
/// No absolute orientation (covariance[0] = -1 marks it unknown).
pub(crate) fn imu_from_status10(s: &Status10ms) -> Imu {
    let g = s.gyro_deg_s();
    let a = s.accel_g();
    let mut imu = Imu {
        header: Header { stamp: msg::now(), frame_id: "base_link".into() },
        angular_velocity: Vector3 {
            x: (g[0] as f64).to_radians(),
            y: (g[1] as f64).to_radians(),
            z: (g[2] as f64).to_radians(),
        },
        linear_acceleration: Vector3 {
            x: a[0] as f64 * 9.80665,
            y: a[1] as f64 * 9.80665,
            z: a[2] as f64 * 9.80665,
        },
        ..Default::default()
    };
    imu.orientation_covariance[0] = -1.0;
    imu
}

/// `Battery` (0x2b) -> `sensor_msgs/BatteryState`.
pub(crate) fn battery_msg(b: &Battery) -> BatteryState {
    let charging = b.charge_voltage_mv > 1000;
    BatteryState {
        header: Header { stamp: msg::now(), frame_id: "base_link".into() },
        voltage: b.voltage_v(),
        temperature: b.temperature_ddeg as f32 / 10.0,
        current: b.current_ma as f32 / 1000.0,
        charge: f32::NAN,
        capacity: f32::NAN,
        design_capacity: f32::NAN,
        percentage: (b.soc_percent() / 100.0).clamp(0.0, 1.0),
        power_supply_status: if charging { 1 } else { 2 }, // CHARGING / DISCHARGING
        power_supply_health: 0,      // UNKNOWN
        power_supply_technology: 3,  // LION
        present: true,
        cell_voltage: Vec::new(),
        cell_temperature: Vec::new(),
        location: String::new(),
        serial_number: String::new(),
    }
}

/// Motor currents -> `std_msgs/Int16MultiArray` (the combined `/motor_currents`
/// topic). `[wheel_left, wheel_right, main_brush, side_brush, load]` (raw i16).
/// The same values are also published individually as `/current/<name>`.
pub(crate) fn currents_array(c: [i16; 5]) -> Int16MultiArray {
    Int16MultiArray { layout: Default::default(), data: c.to_vec() }
}

/// Build a LaserScan from one accumulated sweep of (raw_angle_rad, dist_m).
fn build_scan(pts: &[(f32, f32)]) -> Option<LaserScan> {
    if pts.is_empty() {
        return None;
    }
    let mut tp: Vec<(f32, f32)> = pts
        .iter()
        .map(|&(a, d)| (wrap_pi(ANGLE_SCALE * a + ANGLE_OFFSET), d))
        .collect();
    tp.sort_by(|a, b| a.0.total_cmp(&b.0));
    let angle_min = tp.first().unwrap().0;
    let angle_max = tp.last().unwrap().0;
    let n = (((angle_max - angle_min) / SCAN_INCREMENT).round() as usize).saturating_add(1);
    if n == 0 || n > 4096 {
        return None;
    }
    let mut ranges = vec![f32::INFINITY; n];
    for (a, d) in tp {
        let i = (((a - angle_min) / SCAN_INCREMENT).round() as usize).min(n - 1);
        if d < ranges[i] {
            ranges[i] = d; // keep the nearest return in a bin
        }
    }
    Some(LaserScan {
        header: Header { stamp: msg::now(), frame_id: "laser".into() },
        angle_min,
        angle_max,
        angle_increment: SCAN_INCREMENT,
        time_increment: 0.0,
        scan_time: 0.0,
        range_min: RANGE_MIN,
        range_max: RANGE_MAX,
        ranges,
        intensities: Vec::new(),
    })
}

/// Accumulates LDS packets into arc sweeps. Push each decoded frame; a full
/// LaserScan is returned when the arc restarts (fsa jumps backwards).
pub(crate) struct Sweep {
    pts: Vec<(f32, f32)>,
    prev_fsa: Option<u16>,
}

impl Sweep {
    pub(crate) fn new() -> Self {
        Self { pts: Vec::with_capacity(256), prev_fsa: None }
    }
    pub(crate) fn push(&mut self, f: &LdsFrame) -> Option<LaserScan> {
        let mut done = None;
        if let Some(pf) = self.prev_fsa {
            if (pf as i32 - f.fsa as i32) > FSA_WRAP {
                done = build_scan(&self.pts);
                self.pts.clear();
            }
        }
        self.prev_fsa = Some(f.fsa);
        for k in 0..f.samples.len() {
            let s = f.samples[k];
            if s.valid && s.dist_mm != 0 {
                let a = f.sample_angle(k) as f32 / LDS_ANGLE_FULL as f32 * TAU;
                self.pts.push((a, s.dist_mm as f32 / 1000.0));
            }
        }
        done
    }
}

/// LDS tap: connect to `lds-rx`, de-frame, accumulate sweeps, publish.
pub fn lds_reader(addr: String, tx: Sender<Tap>) {
    let mut buf = [0u8; 4096];
    loop {
        let mut stream = match TcpStream::connect(&addr) {
            Ok(s) => {
                let _ = s.set_read_timeout(Some(Duration::from_secs(2)));
                log::info!("lds: connected to {addr}");
                s
            }
            Err(e) => {
                log::warn!("lds: connect {addr}: {e}; retry");
                thread::sleep(Duration::from_millis(500));
                continue;
            }
        };
        let mut sc = LdsScanner::new();
        let mut sweep = Sweep::new();
        loop {
            let n = match stream.read(&mut buf) {
                Ok(0) => break,
                Ok(n) => n,
                Err(e)
                    if e.kind() == std::io::ErrorKind::WouldBlock
                        || e.kind() == std::io::ErrorKind::TimedOut =>
                {
                    continue
                }
                Err(e) => {
                    log::warn!("lds: read: {e}; reconnect");
                    break;
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
}

/// MCU tap: connect to `mcu-rx`, de-frame, publish `/odom` per Status20ms.
pub fn mcu_reader(addr: String, tx: Sender<Tap>) {
    let mut buf = [0u8; 4096];
    loop {
        let mut stream = match TcpStream::connect(&addr) {
            Ok(s) => {
                let _ = s.set_read_timeout(Some(Duration::from_secs(2)));
                log::info!("mcu: connected to {addr}");
                s
            }
            Err(e) => {
                log::warn!("mcu: connect {addr}: {e}; retry");
                thread::sleep(Duration::from_millis(500));
                continue;
            }
        };
        let mut fs = FrameScanner::new();
        let mut gyro_z_dps: f32 = 0.0;
        let (mut wl, mut wr, mut load) = (0i16, 0i16, 0i16);
        loop {
            let n = match stream.read(&mut buf) {
                Ok(0) => break,
                Ok(n) => n,
                Err(e)
                    if e.kind() == std::io::ErrorKind::WouldBlock
                        || e.kind() == std::io::ErrorKind::TimedOut =>
                {
                    continue
                }
                Err(e) => {
                    log::warn!("mcu: read: {e}; reconnect");
                    break;
                }
            };
            for &b in &buf[..n] {
                let Some(body) = fs.push(b) else { continue };
                let Ok((typ, payload)) = parse_body(body) else { continue };
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
                    Msg::Battery(b) => {
                        let _ = tx.send(Tap::Battery(Box::new(battery_msg(&b))));
                    }
                    Msg::Triggers(t) => {
                        let _ = tx.send(Tap::Triggers {
                            dock: t.dock_sta(),
                            bumper: t.left_bumper() || t.right_bumper(),
                            cliff: t.any_cliff(),
                        });
                    }
                    _ => {}
                }
            }
        }
    }
}
