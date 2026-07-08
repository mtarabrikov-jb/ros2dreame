//! ros2dreame - standalone ROS 2 bridge for the Dreame Bot W10 (r2104).
//!
//! Goal: ONE static Rust binary that drives/observes the robot's MCU + LDS +
//! cameras and republishes everything through standard ROS 2 interfaces
//! (sensor_msgs / nav_msgs / geometry_msgs), with no ROS 2 install on the robot
//! (pure-Rust RustDDS via ros2-client) and no dependency on the SangamIO daemon.
//!
//! This file is milestone 0: bring up a ROS 2 node and publish a heartbeat, to
//! prove the ros2-client + RustDDS + static-musl-aarch64 toolchain END TO END
//! before wiring the real topics. Once the static build links and `ros2 topic
//! echo /ros2dreame/heartbeat` sees it, the sensor/actuator/camera publishers go
//! in on top (see README).

use std::thread::sleep;
use std::time::Duration;

use ros2_client::{
    Context, MessageTypeName, Name, NodeName, NodeOptions, DEFAULT_PUBLISHER_QOS,
};

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let context = Context::new().expect("create ROS 2 context");
    let mut node = context
        .new_node(
            NodeName::new("/", "ros2dreame").expect("valid node name"),
            NodeOptions::new().enable_rosout(true),
        )
        .expect("create node");

    let topic = node
        .create_topic(
            &Name::new("/ros2dreame", "heartbeat").expect("valid topic name"),
            MessageTypeName::new("std_msgs", "String"),
            &DEFAULT_PUBLISHER_QOS,
        )
        .expect("create topic");

    let publisher = node
        .create_publisher::<String>(&topic, None)
        .expect("create publisher");

    log::info!("ros2dreame up; publishing /ros2dreame/heartbeat at 1 Hz");

    let mut i: u64 = 0;
    loop {
        match publisher.publish(format!("ros2dreame alive {i}")) {
            Ok(()) => log::debug!("heartbeat {i}"),
            Err(e) => log::warn!("publish failed: {e:?}"),
        }
        i += 1;
        sleep(Duration::from_secs(1));
    }
}
