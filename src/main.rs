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
    QosPolicyBuilder::new()
        .reliability(policy::Reliability::Reliable {
            max_blocking_time: Duration::from_millis(200),
        })
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
        let observe = std::env::var("W10_OBSERVE").is_ok();
        log::info!("data source: DIRECT (ava off; mcu {mcu}, lds {lds}, observe={observe})");
        Some(direct::run(&mcu, &lds, observe, tx.clone()))
    };

    // Cameras: read JPEG frames from the vendored w10-camd helper over a tmpfs
    // shm ring (no HTTP), publish as CompressedImage. "camera" (RGB shm) always;
    // "camera_ir" (ToF shm) when the helper also runs ToF (W10_CAM_IR). frame_id
    // routes the topic.
    let rgb_shm = std::env::var("W10_CAM_SHM").unwrap_or_else(|_| "/tmp/ros2cam.shm".into());
    let ir_shm = std::env::var("W10_CAM_SHM_IR").unwrap_or_else(|_| "/tmp/ros2cam_ir.shm".into());
    let mut cams: Vec<(&str, String)> = vec![("camera", rgb_shm)];
    if std::env::var("W10_CAM_IR").is_ok() {
        cams.push(("camera_ir", ir_shm));
    }
    for (frame, path) in &cams {
        let (p, f, txc) = (path.clone(), frame.to_string(), tx.clone());
        thread::spawn(move || cam::cam_reader(p, f, txc));
    }
    drop(tx);

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
    let currents_pub = {
        let t = mk_pub(&mut node, "/", "motor_currents", "std_msgs", "Int16MultiArray");
        node.create_publisher::<msg::Int16MultiArray>(&t, None).expect("currents pub")
    };

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
            ("set_water_pump", direct::Shared::set_pump),
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
        log::info!("actuators: subscribed (/set_fan /set_side_brush /set_main_brush /set_water_pump)");
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

    let (mut n_scan, mut n_odom, mut n_img) = (0u64, 0u64, 0u64);
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
                let tf = odom_tf(&o);
                let _ = odom_pub.publish(*o);
                let _ = tf_pub.publish(tf);
                n_odom += 1;
            }
            Tap::Image(img) => {
                let fid = img.header.frame_id.clone();
                if let Some((_, p)) = img_pubs.iter().find(|(f, _)| *f == fid) {
                    let _ = p.publish(*img);
                    n_img += 1;
                }
            }
            Tap::Imu(i) => {
                let _ = imu_pub.publish(*i);
            }
            Tap::Battery(b) => {
                let _ = battery_pub.publish(*b);
            }
            Tap::Triggers { dock, bumper, cliff } => {
                let _ = dock_pub.publish(msg::Bool { data: dock });
                let _ = bumper_pub.publish(msg::Bool { data: bumper });
                let _ = cliff_pub.publish(msg::Bool { data: cliff });
            }
            Tap::Currents(c) => {
                let _ = currents_pub.publish(*c);
            }
        }
    }
}
