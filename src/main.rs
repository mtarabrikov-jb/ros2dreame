//! ros2dreame - standalone ROS 2 bridge for the Dreame Bot W10 (r2104).
//!
//! Milestone 2: tap-mode reader. Connects to the robot's `avatap-relay`
//! (`mcu-rx` 7701 + `lds-rx` 7702), decodes with the vendored `dreame-w10-proto`,
//! and publishes `/odom` (nav_msgs/Odometry) + `/scan` (sensor_msgs/LaserScan) as
//! standard ROS 2 topics - one static musl binary, no ROS 2 install, no chroot.
//!
//! Addresses default to loopback (running ON the robot next to the relay); set
//! W10_MCU_ADDR / W10_LDS_ADDR to run against a remote relay for development.

mod msg;
mod tap;

use std::thread;

use ros2_client::{
    Context, MessageTypeName, Name, NodeName, NodeOptions, DEFAULT_PUBLISHER_QOS,
};

use crate::msg::{LaserScan, Odometry};
use crate::tap::Tap;

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let mcu_addr = std::env::var("W10_MCU_ADDR").unwrap_or_else(|_| "127.0.0.1:7701".into());
    let lds_addr = std::env::var("W10_LDS_ADDR").unwrap_or_else(|_| "127.0.0.1:7702".into());

    // Reader threads -> mpsc -> the publisher loop (which owns the publishers).
    let (tx, rx) = std::sync::mpsc::channel::<Tap>();
    {
        let tx = tx.clone();
        thread::spawn(move || tap::mcu_reader(mcu_addr, tx));
    }
    thread::spawn(move || tap::lds_reader(lds_addr, tx));

    let context = Context::new().expect("create ROS 2 context");
    let mut node = context
        .new_node(
            NodeName::new("/", "ros2dreame").expect("valid node name"),
            NodeOptions::new().enable_rosout(true),
        )
        .expect("create node");

    let scan_topic = node
        .create_topic(
            &Name::new("/", "scan").expect("topic name"),
            MessageTypeName::new("sensor_msgs", "LaserScan"),
            &DEFAULT_PUBLISHER_QOS,
        )
        .expect("create scan topic");
    let odom_topic = node
        .create_topic(
            &Name::new("/", "odom").expect("topic name"),
            MessageTypeName::new("nav_msgs", "Odometry"),
            &DEFAULT_PUBLISHER_QOS,
        )
        .expect("create odom topic");

    let scan_pub = node
        .create_publisher::<LaserScan>(&scan_topic, None)
        .expect("create scan publisher");
    let odom_pub = node
        .create_publisher::<Odometry>(&odom_topic, None)
        .expect("create odom publisher");

    log::info!("ros2dreame up (tap mode); publishing /scan + /odom");

    let (mut n_scan, mut n_odom) = (0u64, 0u64);
    for m in rx {
        match m {
            Tap::Scan(s) => {
                if let Err(e) = scan_pub.publish(*s) {
                    log::warn!("scan publish: {e:?}");
                } else {
                    n_scan += 1;
                    if n_scan % 20 == 0 {
                        log::info!("published {n_scan} scans, {n_odom} odom");
                    }
                }
            }
            Tap::Odom(o) => {
                if let Err(e) = odom_pub.publish(*o) {
                    log::warn!("odom publish: {e:?}");
                } else {
                    n_odom += 1;
                }
            }
        }
    }
}
