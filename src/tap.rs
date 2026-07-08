//! Tap-mode data source: read the robot's MCU + LDS serial streams mirrored by
//! `avatap-relay` (TCP `mcu-rx` 7701 / `lds-rx` 7702), decode with the vendored
//! `dreame-w10-proto`, and turn them into ROS 2 messages. Read-only: this runs
//! alongside the vendor `ava`, which keeps driving.
//!
//! Each reader is its own blocking thread that reconnects on error (survives
//! relay/ava restarts), and pushes finished messages over an mpsc channel to the
//! publisher loop in `main`.

use std::io::Read;
use std::net::TcpStream;
use std::sync::mpsc::Sender;
use std::thread;
use std::time::Duration;

use dreame_w10_proto::lds::{LdsScanner, LDS_ANGLE_FULL};
use dreame_w10_proto::{parse_body, FrameScanner, Msg};

use crate::msg::{
    self, Header, LaserScan, Odometry, Point, Pose, PoseWithCovariance, Twist,
    TwistWithCovariance, Vector3,
};

/// A finished message from a reader thread, ready to publish.
pub enum Tap {
    Odom(Box<Odometry>),
    Scan(Box<LaserScan>),
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
        // keep the nearest return in a bin
        if d < ranges[i] {
            ranges[i] = d;
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

/// LDS reader: connect to `lds-rx`, de-frame, accumulate one arc sweep, publish.
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
        let mut pts: Vec<(f32, f32)> = Vec::with_capacity(256);
        let mut prev_fsa: Option<u16> = None;
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
                let Some(f) = sc.push(b) else { continue };
                // sweep boundary: fsa jumped backwards (arc restarted)
                if let Some(pf) = prev_fsa {
                    if (pf as i32 - f.fsa as i32) > FSA_WRAP {
                        if let Some(scan) = build_scan(&pts) {
                            if tx.send(Tap::Scan(Box::new(scan))).is_err() {
                                return;
                            }
                        }
                        pts.clear();
                    }
                }
                prev_fsa = Some(f.fsa);
                for k in 0..f.samples.len() {
                    let s = f.samples[k];
                    if !s.valid || s.dist_mm == 0 {
                        continue;
                    }
                    let a = f.sample_angle(k) as f32 / LDS_ANGLE_FULL as f32 * TAU;
                    pts.push((a, s.dist_mm as f32 / 1000.0));
                }
            }
        }
    }
}

/// MCU reader: connect to `mcu-rx`, de-frame, publish `/odom` on each pose
/// update (Status20ms @20ms). Twist angular comes from the IMU yaw rate.
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
                    Msg::Status10ms(s) => gyro_z_dps = s.gyro_deg_s()[2],
                    Msg::Status20ms(s) => {
                        let x_m = s.x_mm10 as f64 / 10.0 / 1000.0;
                        let y_m = s.y_mm10 as f64 / 10.0 / 1000.0;
                        let yaw = (s.yaw_deg() as f64).to_radians();
                        let v_lin = (s.left_vel as f64 + s.right_vel as f64) / 2.0 / 1000.0;
                        let v_ang = (gyro_z_dps as f64).to_radians();
                        let odom = Odometry {
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
                        };
                        if tx.send(Tap::Odom(Box::new(odom))).is_err() {
                            return;
                        }
                    }
                    _ => {}
                }
            }
        }
    }
}
