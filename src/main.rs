//! ros2dreame - standalone ROS 2 bridge for the Dreame Bot W10 (r2104).
//!
//! Milestone 3: tap-mode reader + TF. Connects to `avatap-relay` (`mcu-rx` 7701 +
//! `lds-rx` 7702), decodes with vendored `dreame-w10-proto`, and publishes as
//! standard ROS 2 topics - one static musl binary, no ROS 2 install, no chroot:
//!   /scan  sensor_msgs/LaserScan    (best-effort sensor QoS)
//!   /odom  nav_msgs/Odometry        (best-effort sensor QoS)
//!   /tf         tf2_msgs/TFMessage  odom -> base_link (per odom update)
//!   /tf_static  tf2_msgs/TFMessage  base_link -> laser (once, transient-local)
//! With TF, rviz (Fixed Frame = odom) renders /scan and the odometry pose.

mod cam;
mod direct;
mod msg;
mod tap;

use std::thread;

use ros2_client::ros2::{policy, Duration, QosPolicies, QosPolicyBuilder};
use ros2_client::{Context, MessageTypeName, Name, NodeName, NodeOptions};

use crate::msg::{Header, Odometry, Quaternion, TFMessage, Transform, TransformStamped, Vector3};
use crate::tap::Tap;

// Laser mounting on the robot (base_link -> laser). The scan angles are already
// rotated into the robot frame in tap::build_scan, so this is position-only
// (identity rotation): the LDS sits ~87mm behind center (see dreame_w10 calib).
const LASER_X: f64 = -0.087;
const LASER_Z: f64 = 0.05;

/// Sensor data: best-effort, keep-last 5, volatile (rmw sensor-data profile).
fn sensor_qos() -> QosPolicies {
    QosPolicyBuilder::new()
        .reliability(policy::Reliability::BestEffort)
        .history(policy::History::KeepLast { depth: 5 })
        .durability(policy::Durability::Volatile)
        .build()
}

/// /tf: reliable, keep-last 100, volatile (tf2 default).
fn tf_qos() -> QosPolicies {
    QosPolicyBuilder::new()
        .reliability(policy::Reliability::Reliable {
            max_blocking_time: Duration::from_millis(100),
        })
        .history(policy::History::KeepLast { depth: 100 })
        .durability(policy::Durability::Volatile)
        .build()
}

/// Camera images: reliable, keep-last 2. CompressedImage JPEGs are large (many
/// KB -> multiple RTPS fragments); over best-effort WiFi a single lost fragment
/// drops the whole sample, so reliable (fragment retransmit) is needed to get
/// complete frames through. Keep-last 2 bounds latency if the reader lags.
fn image_qos() -> QosPolicies {
    // BEST-EFFORT, not reliable: two full-res JPEG streams (RGB+IR) are many RTPS
    // fragments each, and over WiFi a reliable writer thrashes on retransmits -
    // it saturates RustDDS (send window full, PollEventSender EAGAIN) to the point
    // that INCOMING subs like /cmd_vel are starved and the robot won't drive.
    // Best-effort drops the odd fragment (a lost frame here and there) but keeps
    // DDS - and control - responsive. Subscribers must also be best-effort.
    QosPolicyBuilder::new()
        .reliability(policy::Reliability::BestEffort)
        .history(policy::History::KeepLast { depth: 2 })
        .durability(policy::Durability::Volatile)
        .build()
}

/// /tf_static: reliable, transient-local (late subscribers still get it).
fn tf_static_qos() -> QosPolicies {
    QosPolicyBuilder::new()
        .reliability(policy::Reliability::Reliable {
            max_blocking_time: Duration::from_millis(100),
        })
        .history(policy::History::KeepLast { depth: 1 })
        .durability(policy::Durability::TransientLocal)
        .build()
}

/// Build the odom -> base_link transform from an Odometry message.
fn odom_tf(o: &Odometry) -> TFMessage {
    TFMessage {
        transforms: vec![TransformStamped {
            header: o.header.clone(),
            child_frame_id: o.child_frame_id.clone(),
            transform: Transform {
                translation: Vector3 {
                    x: o.pose.pose.position.x,
                    y: o.pose.pose.position.y,
                    z: o.pose.pose.position.z,
                },
                rotation: o.pose.pose.orientation.clone(),
            },
        }],
    }
}

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let (tx, rx) = std::sync::mpsc::channel::<Tap>();

    // W10_AUTO: auto-switch turret + cameras with motion. Driving (fresh /cmd_vel)
    // -> turret ON -> /scan + IR; idle -> turret OFF, RGB un-wedge reset, both
    // cameras (RGB + IR). ros2dreame owns the w10-camd helper in this mode.
    let auto = std::env::var_os("W10_AUTO").is_some();

    // Data source. Default: DIRECT (ava OFF) - open /dev/ttyS4 + /dev/ttyS3 and
    // drive the MCU/LDS in-process (one binary, no external daemon). Set
    // W10_MCU_ADDR (host:port) to use TAP mode instead (ava ON, read
    // avatap-relay's mirror over TCP) for development.
    let drive = if let Ok(mcu_addr) = std::env::var("W10_MCU_ADDR") {
        let lds_addr = std::env::var("W10_LDS_ADDR").unwrap_or_else(|_| "127.0.0.1:7702".into());
        log::info!("data source: TAP (mcu {mcu_addr}, lds {lds_addr})");
        {
            let tx = tx.clone();
            thread::spawn(move || tap::mcu_reader(mcu_addr, tx));
        }
        {
            let tx = tx.clone();
            thread::spawn(move || tap::lds_reader(lds_addr, tx));
        }
        None
    } else {
        let mcu = std::env::var("W10_MCU").unwrap_or_else(|_| "/dev/ttyS4".into());
        let lds = std::env::var("W10_LDS").unwrap_or_else(|_| "/dev/ttyS3".into());
        // Observe/park mode (W10_OBSERVE): stay idle so the RGB camera can stream
        // (firmware kills RGB in any active/nav mode); no /scan. Default is nav.
        let observe = auto || std::env::var("W10_OBSERVE").is_ok(); // auto starts parked
        log::info!("data source: DIRECT (ava off; mcu {mcu}, lds {lds}, observe={observe}, auto={auto})");
        Some(direct::run(&mcu, &lds, observe, tx.clone()))
    };

    // Cameras: read JPEG frames from the vendored w10-camd helper over a tmpfs
    // shm ring (no HTTP), publish as CompressedImage. "camera" (RGB shm) always;
    // "camera_ir" (ToF shm) when the helper also runs ToF (W10_CAM_IR). frame_id
    // routes the topic.
    let rgb_shm = std::env::var("W10_CAM_SHM").unwrap_or_else(|_| "/tmp/ros2cam.shm".into());
    let ir_shm = std::env::var("W10_CAM_SHM_IR").unwrap_or_else(|_| "/tmp/ros2cam_ir.shm".into());
    let mut cams: Vec<(&str, String)> = vec![("camera", rgb_shm)];
    if auto || std::env::var("W10_CAM_IR").is_ok() {
        cams.push(("camera_ir", ir_shm));
    }
    for (frame, path) in &cams {
        let (p, f, txc) = (path.clone(), frame.to_string(), tx.clone());
        thread::spawn(move || cam::cam_reader(p, f, txc));
    }
    drop(tx);

    // Auto mode: one thread flips turret/park with motion; another runs the
    // w10-camd helper to match (tof while driving, both while parked).
    if auto {
        if let Some(sh) = drive.clone() {
            let s = sh.clone();
            thread::spawn(move || {
                const HOLD_MS: u64 = 3000; // stay in DRIVING this long after motion stops
                loop {
                    // Skip while paused (a /set_turret click took manual control);
                    // use is_parked() as truth since manual control can change it.
                    if !s.auto_paused() {
                        let want = s.idle_move_ms() > HOLD_MS;
                        if want != s.is_parked() {
                            s.set_parked(want);
                            log::info!("auto: {}", if want { "PARKED (turret off; RGB+IR)" } else { "DRIVING (turret on; /scan+IR)" });
                        }
                    }
                    thread::sleep(std::time::Duration::from_millis(400));
                }
            });
            let s = sh.clone();
            let camd = std::env::var("W10_CAMD").unwrap_or_else(|_| "/data/ros2dreame/w10-camd".into());
            thread::spawn(move || camera_manager(s, camd));
        } else {
            log::warn!("W10_AUTO ignored: only works in DIRECT mode (no W10_MCU_ADDR)");
        }
    }

    let context = Context::new().expect("create ROS 2 context");
    let mut node = context
        .new_node(
            NodeName::new("/", "ros2dreame").expect("valid node name"),
            NodeOptions::new().enable_rosout(true),
        )
        .expect("create node");

    let scan_topic = node
        .create_topic(
            &Name::new("/", "scan").unwrap(),
            MessageTypeName::new("sensor_msgs", "LaserScan"),
            &sensor_qos(),
        )
        .expect("scan topic");
    let odom_topic = node
        .create_topic(
            &Name::new("/", "odom").unwrap(),
            MessageTypeName::new("nav_msgs", "Odometry"),
            &sensor_qos(),
        )
        .expect("odom topic");
    let tf_topic = node
        .create_topic(
            &Name::new("/", "tf").unwrap(),
            MessageTypeName::new("tf2_msgs", "TFMessage"),
            &tf_qos(),
        )
        .expect("tf topic");
    let tf_static_topic = node
        .create_topic(
            &Name::new("/", "tf_static").unwrap(),
            MessageTypeName::new("tf2_msgs", "TFMessage"),
            &tf_static_qos(),
        )
        .expect("tf_static topic");

    let scan_pub = node
        .create_publisher::<msg::LaserScan>(&scan_topic, None)
        .expect("scan pub");
    let odom_pub = node
        .create_publisher::<Odometry>(&odom_topic, None)
        .expect("odom pub");
    let tf_pub = node
        .create_publisher::<TFMessage>(&tf_topic, None)
        .expect("tf pub");
    let tf_static_pub = node
        .create_publisher::<TFMessage>(&tf_static_topic, None)
        .expect("tf_static pub");

    // Extra telemetry: IMU, battery, and the Triggers booleans (dock/bumper/cliff).
    let mk_pub = |node: &mut ros2_client::Node, ns: &str, name: &str, pkg: &str, ty: &str| {
        let topic = node
            .create_topic(&Name::new(ns, name).unwrap(), MessageTypeName::new(pkg, ty), &sensor_qos())
            .expect("topic");
        topic
    };
    let imu_pub = {
        let t = mk_pub(&mut node, "/", "imu", "sensor_msgs", "Imu");
        node.create_publisher::<msg::Imu>(&t, None).expect("imu pub")
    };
    let battery_pub = {
        let t = mk_pub(&mut node, "/", "battery", "sensor_msgs", "BatteryState");
        node.create_publisher::<msg::BatteryState>(&t, None).expect("battery pub")
    };
    let dock_pub = {
        let t = mk_pub(&mut node, "/", "dock", "std_msgs", "Bool");
        node.create_publisher::<msg::Bool>(&t, None).expect("dock pub")
    };
    let bumper_pub = {
        let t = mk_pub(&mut node, "/", "bumper", "std_msgs", "Bool");
        node.create_publisher::<msg::Bool>(&t, None).expect("bumper pub")
    };
    let cliff_pub = {
        let t = mk_pub(&mut node, "/", "cliff", "std_msgs", "Bool");
        node.create_publisher::<msg::Bool>(&t, None).expect("cliff pub")
    };
    // Individual cliff/floor sensors (the 6-bit mask + one Bool per sensor). Bit
    // order (raw[1] low 6 bits = global bits 8..13): front_left, mid_left, mid_right,
    // front_right, rear_left, rear_right. front/rear corners verified; the two mid
    // sensors' exact positions are TBD (named tentatively). true = sees no floor.
    let cliff_flags_pub = {
        let t = mk_pub(&mut node, "/cliff", "flags", "std_msgs", "UInt8");
        node.create_publisher::<msg::UInt8>(&t, None).expect("cliff flags pub")
    };
    const CLIFF_NAMES: [&str; 6] =
        ["front_left", "mid_left", "mid_right", "front_right", "rear_left", "rear_right"];
    let mut cliff_sensor_pubs = Vec::with_capacity(6);
    for name in CLIFF_NAMES {
        let t = mk_pub(&mut node, "/cliff", name, "std_msgs", "Bool");
        cliff_sensor_pubs.push(node.create_publisher::<msg::Bool>(&t, None).expect("cliff sensor pub"));
    }
    // MCU fan/suction overcurrent fault (Triggers bit 42). The MCU has no analog
    // fan current - this flag is the only per-fan signal (fan draw itself shows on
    // /battery.current). true = the MCU tripped the fan's overcurrent protection.
    let fan_oc_pub = {
        let t = mk_pub(&mut node, "/fault", "fan", "std_msgs", "Bool");
        node.create_publisher::<msg::Bool>(&t, None).expect("fan fault pub")
    };
    // Base-station buttons (0x23 dock-status byte0: bit0=Home, bit2=Start/Stop;
    // verified live). true while held.
    let home_pub = {
        let t = mk_pub(&mut node, "/", "dock_button_home", "std_msgs", "Bool");
        node.create_publisher::<msg::Bool>(&t, None).expect("home button pub")
    };
    let start_pub = {
        let t = mk_pub(&mut node, "/", "dock_button_start", "std_msgs", "Bool");
        node.create_publisher::<msg::Bool>(&t, None).expect("start button pub")
    };
    let currents_pub = {
        let t = mk_pub(&mut node, "/", "motor_currents", "std_msgs", "Int16MultiArray");
        node.create_publisher::<msg::Int16MultiArray>(&t, None).expect("currents pub")
    };
    // Named per-motor currents /current/<name> (std_msgs/Int16) - easier to plot
    // than the /motor_currents array. Order matches [wl, wr, main, side, load].
    let mut current_pubs = Vec::new();
    for n in ["wheel_left", "wheel_right", "main_brush", "side_brush", "load"] {
        let t = mk_pub(&mut node, "/current", n, "std_msgs", "Int16");
        current_pubs.push(node.create_publisher::<msg::Int16>(&t, None).expect("current pub"));
    }
    // Actuator/turret state /state/* telemetry (from the periodic Tap::State).
    let turret_pub = {
        let t = mk_pub(&mut node, "/state", "turret", "std_msgs", "Bool");
        node.create_publisher::<msg::Bool>(&t, None).expect("turret pub")
    };
    let mode_pub = {
        let t = mk_pub(&mut node, "/state", "mode", "std_msgs", "String");
        node.create_publisher::<msg::StringMsg>(&t, None).expect("mode pub")
    };
    let mut level_pubs = Vec::new();
    for n in ["fan", "side_brush", "main_brush", "mop"] {
        let t = mk_pub(&mut node, "/state", n, "std_msgs", "UInt8");
        level_pubs.push(node.create_publisher::<msg::UInt8>(&t, None).expect("level pub"));
    }

    // Camera publishers: /<frame>/image_raw/compressed (image_transport compressed).
    let mut img_pubs: Vec<(String, ros2_client::Publisher<msg::CompressedImage>)> = Vec::new();
    for (frame, _path) in &cams {
        let topic = node
            .create_topic(
                &Name::new(&format!("/{frame}/image_raw"), "compressed").unwrap(),
                MessageTypeName::new("sensor_msgs", "CompressedImage"),
                &image_qos(),
            )
            .expect("image topic");
        let p = node
            .create_publisher::<msg::CompressedImage>(&topic, None)
            .expect("image pub");
        img_pubs.push((frame.to_string(), p));
    }

    // /cmd_vel teleop -> drive (direct mode only). The drive path in direct.rs is
    // gated by a 500 ms command watchdog + speed clamp + cliff/bumper hazard, so
    // a dropped/stale command stops the robot. Best-effort sub matches most
    // teleop/nav publishers; the watchdog covers any loss.
    if let Some(drive) = drive.clone() {
        let cmd_topic = node
            .create_topic(
                &Name::new("/", "cmd_vel").unwrap(),
                MessageTypeName::new("geometry_msgs", "Twist"),
                &sensor_qos(),
            )
            .expect("cmd_vel topic");
        let cmd_sub = node
            .create_subscription::<msg::Twist>(&cmd_topic, Some(sensor_qos()))
            .expect("cmd_vel sub");
        {
            let drive = drive.clone();
            thread::spawn(move || {
                log::info!("cmd_vel: subscribed (Twist -> MotorCtrl)");
                loop {
                    match cmd_sub.take() {
                        Ok(Some((t, _))) => {
                            let lin_mm_s = (t.linear.x * 1000.0) as f32; // m/s -> mm/s
                            let rot = t.angular.z as f32; // rad/s
                            drive.set_drive(lin_mm_s, rot);
                        }
                        _ => thread::sleep(std::time::Duration::from_millis(10)),
                    }
                }
            });
        }

        // Actuators: std_msgs/UInt8 levels -> the periodic SetCleaning frame.
        for (name, set) in [
            ("set_fan", direct::Shared::set_fan as fn(&direct::Shared, u8)),
            ("set_side_brush", direct::Shared::set_side_brush),
            ("set_main_brush", direct::Shared::set_main_brush),
            ("set_mop", direct::Shared::set_mop),
            ("set_station", direct::Shared::set_station),
            ("set_dock_screen", direct::Shared::set_dock_screen),
        ] {
            let topic = node
                .create_topic(&Name::new("/", name).unwrap(), MessageTypeName::new("std_msgs", "UInt8"), &sensor_qos())
                .expect("actuator topic");
            let sub = node
                .create_subscription::<msg::UInt8>(&topic, Some(sensor_qos()))
                .expect("actuator sub");
            let d = drive.clone();
            thread::spawn(move || loop {
                match sub.take() {
                    Ok(Some((m, _))) => set(&d, m.data),
                    _ => thread::sleep(std::time::Duration::from_millis(20)),
                }
            });
        }
        log::info!("actuators: subscribed (/set_fan /set_side_brush /set_main_brush /set_mop /set_station /set_dock_screen)");

        // Raw dock 0x26 frame (std_msgs/UInt8MultiArray, first 8 bytes) - full manual
        // control of the base-station frame for RE (byte0=screen, byte7 progress, ...).
        // Overrides /set_dock_screen while idle; byte6 != 0 drives pump/fan (careful).
        {
            let topic = node
                .create_topic(&Name::new("/", "set_dock_frame").unwrap(), MessageTypeName::new("std_msgs", "UInt8MultiArray"), &sensor_qos())
                .expect("dock_frame topic");
            let sub = node
                .create_subscription::<msg::UInt8MultiArray>(&topic, Some(sensor_qos()))
                .expect("dock_frame sub");
            let d = drive.clone();
            thread::spawn(move || loop {
                match sub.take() {
                    Ok(Some((m, _))) => d.set_dock_frame(&m.data),
                    _ => thread::sleep(std::time::Duration::from_millis(20)),
                }
            });
            log::info!("dock: subscribed (/set_dock_frame - raw 8-byte 0x26)");
        }

        // Manual turret + auto toggle (std_msgs/Bool, for GUI on/off buttons).
        // /set_turret true = drive state (turret + /scan, IR; RGB drops), false =
        // park state (both cameras). It pauses W10_AUTO; /set_auto true resumes it.
        for (name, set) in [
            ("set_turret", direct::Shared::set_turret as fn(&direct::Shared, bool)),
            ("set_auto", direct::Shared::set_auto),
        ] {
            let topic = node
                .create_topic(&Name::new("/", name).unwrap(), MessageTypeName::new("std_msgs", "Bool"), &sensor_qos())
                .expect("bool topic");
            let sub = node
                .create_subscription::<msg::Bool>(&topic, Some(sensor_qos()))
                .expect("bool sub");
            let d = drive.clone();
            thread::spawn(move || loop {
                match sub.take() {
                    Ok(Some((m, _))) => set(&d, m.data),
                    _ => thread::sleep(std::time::Duration::from_millis(20)),
                }
            });
        }
        log::info!("manual: subscribed (/set_turret /set_auto)");
    }

    // Static base_link -> laser, published once (transient-local keeps it for
    // late subscribers like rviz).
    let static_tf = TFMessage {
        transforms: vec![TransformStamped {
            header: Header { stamp: msg::now(), frame_id: "base_link".into() },
            child_frame_id: "laser".into(),
            transform: Transform {
                translation: Vector3 { x: LASER_X, y: 0.0, z: LASER_Z },
                rotation: Quaternion::default(), // identity
            },
        }],
    };
    if let Err(e) = tf_static_pub.publish(static_tf) {
        log::warn!("tf_static publish: {e:?}");
    }

    let cam_names: Vec<&str> = cams.iter().map(|(f, _)| *f).collect();
    log::info!("ros2dreame up; publishing /scan /odom /tf /tf_static + cameras {cam_names:?}");

    let (mut n_scan, mut n_odom, mut n_img, mut n_curr, mut n_imu) = (0u64, 0u64, 0u64, 0u64, 0u64);
    // Per-camera publish-rate cap (~6 fps). Two full-res JPEG streams over WiFi are
    // many RTPS fragments each; publishing every frame floods RustDDS's event
    // channels (PollEventSender EAGAIN, status channel full) and starves incoming
    // subs like /cmd_vel - the robot stops responding to drive commands.
    let mut last_img: std::collections::HashMap<String, std::time::Instant> = std::collections::HashMap::new();
    // Per-camera min interval in ms (W10_IMG_MS, default 160 = ~6 fps). Higher =
    // fewer frames = the network thread is freer to receive /cmd_vel promptly.
    let img_ms: u128 = std::env::var("W10_IMG_MS").ok().and_then(|s| s.parse().ok()).unwrap_or(160);
    for m in rx {
        match m {
            Tap::Scan(s) => {
                if let Err(e) = scan_pub.publish(*s) {
                    log::warn!("scan publish: {e:?}");
                } else {
                    n_scan += 1;
                    if n_scan % 50 == 0 {
                        log::info!("scans {n_scan}, odom {n_odom}, images {n_img}");
                    }
                }
            }
            Tap::Odom(o) => {
                n_odom += 1;
                if n_odom % 2 == 0 {
                    // ~25 Hz (source 50 Hz) - eases the DDS load; still smooth.
                    let tf = odom_tf(&o);
                    let _ = odom_pub.publish(*o);
                    let _ = tf_pub.publish(tf);
                }
            }
            Tap::Image(img) => {
                let fid = img.header.frame_id.clone();
                let now = std::time::Instant::now();
                let due = last_img.get(&fid).map_or(true, |t| now.duration_since(*t).as_millis() >= img_ms);
                if due {
                    if let Some((_, p)) = img_pubs.iter().find(|(f, _)| *f == fid) {
                        let _ = p.publish(*img);
                        n_img += 1;
                        last_img.insert(fid, now);
                    }
                }
            }
            Tap::Imu(i) => {
                n_imu += 1;
                if n_imu % 4 == 0 {
                    let _ = imu_pub.publish(*i); // ~25 Hz (source ~100 Hz)
                }
            }
            Tap::Battery(b) => {
                let _ = battery_pub.publish(*b);
            }
            Tap::Triggers { dock, bumper, cliff_bits, fan_oc } => {
                let _ = dock_pub.publish(msg::Bool { data: dock });
                let _ = bumper_pub.publish(msg::Bool { data: bumper });
                let _ = cliff_pub.publish(msg::Bool { data: cliff_bits != 0 }); // aggregate
                let _ = cliff_flags_pub.publish(msg::UInt8 { data: cliff_bits });
                for (i, p) in cliff_sensor_pubs.iter().enumerate() {
                    let _ = p.publish(msg::Bool { data: cliff_bits & (1 << i) != 0 });
                }
                let _ = fan_oc_pub.publish(msg::Bool { data: fan_oc });
            }
            Tap::DockButton { home, start } => {
                let _ = home_pub.publish(msg::Bool { data: home });
                let _ = start_pub.publish(msg::Bool { data: start });
            }
            Tap::Currents(c) => {
                // Decimate to ~5 Hz: the MCU streams currents at 50 Hz, but 6 topics
                // (array + 5 named) x 50 Hz floods RustDDS over WiFi and starves
                // incoming subs like /cmd_vel. A dashboard doesn't need 50 Hz.
                n_curr += 1;
                if n_curr % 10 == 0 {
                    let _ = currents_pub.publish(crate::tap::currents_array(c));
                    for (p, &v) in current_pubs.iter().zip(c.iter()) {
                        let _ = p.publish(msg::Int16 { data: v });
                    }
                }
            }
            Tap::State { turret, fan, side_brush, main_brush, mop } => {
                let _ = turret_pub.publish(msg::Bool { data: turret });
                let _ = mode_pub.publish(msg::StringMsg {
                    data: if turret { "DRIVING" } else { "PARKED" }.into(),
                });
                for (p, v) in level_pubs.iter().zip([fan, side_brush, main_brush, mop]) {
                    let _ = p.publish(msg::UInt8 { data: v });
                }
            }
        }
    }
}

/// Auto mode camera helper manager: run `w10-camd tof` while driving (turret on
/// wedges RGB isp0, so only the ToF/isp1 stream is useful) and `w10-camd both`
/// while parked (turret off -> RGB recovers, both ISPs stream). On the park switch
/// wait for the turret to spin down and the RGB reset (tx_loop's 0x1d 05 00) to
/// land before opening video2. `both` forks a ToF child, so the helper is spawned
/// in its own session (setsid) and stopped by killing the whole process group.
fn camera_manager(sh: std::sync::Arc<direct::Shared>, camd: String) {
    use std::os::unix::process::CommandExt;
    use std::process::{Child, Command, Stdio};
    use std::time::Duration;
    let mut child: Option<Child> = None;
    let mut pgid: i32 = 0;
    let mut cur = String::new();
    loop {
        let want = if sh.is_parked() { "both" } else { "tof" };
        if cur != want {
            if pgid > 0 {
                unsafe { libc::kill(-pgid, libc::SIGKILL) };
            }
            if let Some(mut c) = child.take() {
                let _ = c.wait(); // reap the helper parent (its ToF fork is init-reaped)
            }
            // PARK: let the turret coast to a stop + the RGB un-wedge reset land
            // before (re)opening video2, else a still-spinning turret re-wedges it.
            thread::sleep(Duration::from_millis(if want == "both" { 3000 } else { 500 }));
            let mut cmd = Command::new(&camd);
            cmd.arg(want)
                .stdin(Stdio::null())
                .stdout(Stdio::null())
                .stderr(Stdio::null());
            unsafe {
                cmd.pre_exec(|| {
                    libc::setsid();
                    Ok(())
                });
            }
            match cmd.spawn() {
                Ok(c) => {
                    pgid = c.id() as i32; // setsid made pid == pgid
                    child = Some(c);
                    log::info!("auto-cam: w10-camd {want}");
                }
                Err(e) => {
                    pgid = 0;
                    log::warn!("auto-cam: spawn {camd} {want}: {e}");
                }
            }
            cur = want.to_string();
        }
        thread::sleep(Duration::from_millis(300));
    }
}
