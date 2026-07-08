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

    let mcu_addr = std::env::var("W10_MCU_ADDR").unwrap_or_else(|_| "127.0.0.1:7701".into());
    let lds_addr = std::env::var("W10_LDS_ADDR").unwrap_or_else(|_| "127.0.0.1:7702".into());

    let (tx, rx) = std::sync::mpsc::channel::<Tap>();
    {
        let tx = tx.clone();
        thread::spawn(move || tap::mcu_reader(mcu_addr, tx));
    }
    {
        let tx = tx.clone();
        thread::spawn(move || tap::lds_reader(lds_addr, tx));
    }

    // Cameras: read MJPEG from go2rtc (fed by the vendor ava stack OR the no-ava
    // w10-cam stack). "camera" always; "camera_ir" when W10_CAM_IR is set (the
    // no-ava dual-camera stack publishes both). frame_id routes to its topic.
    let cam_addr = std::env::var("W10_CAM_ADDR").unwrap_or_else(|_| "127.0.0.1:1984".into());
    let mut cams: Vec<(&str, &str)> = vec![("camera", "camera")]; // (go2rtc src, frame_id)
    if std::env::var("W10_CAM_IR").is_ok() {
        cams.push(("camera_ir", "camera_ir"));
    }
    for (src, frame) in &cams {
        let (a, s, f, txc) = (cam_addr.clone(), src.to_string(), frame.to_string(), tx.clone());
        thread::spawn(move || cam::cam_reader(a, s, f, txc));
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

    // Camera publishers: /<frame>/image_raw/compressed (image_transport compressed).
    let mut img_pubs: Vec<(String, ros2_client::Publisher<msg::CompressedImage>)> = Vec::new();
    for (_src, frame) in &cams {
        let topic = node
            .create_topic(
                &Name::new(&format!("/{frame}/image_raw"), "compressed").unwrap(),
                MessageTypeName::new("sensor_msgs", "CompressedImage"),
                &sensor_qos(),
            )
            .expect("image topic");
        let p = node
            .create_publisher::<msg::CompressedImage>(&topic, None)
            .expect("image pub");
        img_pubs.push((frame.to_string(), p));
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

    let cam_names: Vec<&str> = cams.iter().map(|(_, f)| *f).collect();
    log::info!(
        "ros2dreame up; publishing /scan /odom /tf /tf_static + cameras {cam_names:?}"
    );

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
        }
    }
}
