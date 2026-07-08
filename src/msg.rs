//! Minimal ROS 2 message structs that ros2dreame publishes.
//!
//! Field ORDER and types match the upstream `.msg` definitions exactly - CDR is
//! positional, so a wrong order/type deserializes to garbage on the ROS 2 side.
//! Serialized by ros2-client via serde/CDR; publishing only needs `Serialize`
//! (ros2-client `create_publisher<D: Serialize>`), `Deserialize` is for symmetry.

use serde::{Deserialize, Serialize};
use serde_big_array::BigArray;

/// builtin_interfaces/Time
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct Time {
    pub sec: i32,
    pub nanosec: u32,
}

/// std_msgs/Header
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct Header {
    pub stamp: Time,
    pub frame_id: String,
}

/// sensor_msgs/LaserScan
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct LaserScan {
    pub header: Header,
    pub angle_min: f32,
    pub angle_max: f32,
    pub angle_increment: f32,
    pub time_increment: f32,
    pub scan_time: f32,
    pub range_min: f32,
    pub range_max: f32,
    pub ranges: Vec<f32>,
    pub intensities: Vec<f32>,
}

// --- geometry_msgs ---

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}
impl Default for Quaternion {
    fn default() -> Self {
        Self { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } // identity, not all-zero
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct Pose {
    pub position: Point,
    pub orientation: Quaternion,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct Twist {
    pub linear: Vector3,
    pub angular: Vector3,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct PoseWithCovariance {
    pub pose: Pose,
    #[serde(with = "BigArray")]
    pub covariance: [f64; 36],
}
impl Default for PoseWithCovariance {
    fn default() -> Self {
        Self { pose: Pose::default(), covariance: [0.0; 36] }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct TwistWithCovariance {
    pub twist: Twist,
    #[serde(with = "BigArray")]
    pub covariance: [f64; 36],
}
impl Default for TwistWithCovariance {
    fn default() -> Self {
        Self { twist: Twist::default(), covariance: [0.0; 36] }
    }
}

/// nav_msgs/Odometry
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct Odometry {
    pub header: Header,
    pub child_frame_id: String,
    pub pose: PoseWithCovariance,
    pub twist: TwistWithCovariance,
}

// --- tf2 ---

/// geometry_msgs/Transform
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct Transform {
    pub translation: Vector3,
    pub rotation: Quaternion,
}

/// geometry_msgs/TransformStamped
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct TransformStamped {
    pub header: Header,
    pub child_frame_id: String,
    pub transform: Transform,
}

/// tf2_msgs/TFMessage
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct TFMessage {
    pub transforms: Vec<TransformStamped>,
}

/// Wall-clock ROS time (`builtin_interfaces/Time`) from the system clock.
pub fn now() -> Time {
    let d = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default();
    Time { sec: d.as_secs() as i32, nanosec: d.subsec_nanos() }
}

/// Quaternion for a yaw rotation (rad) about +Z.
pub fn yaw_to_quat(yaw: f64) -> Quaternion {
    Quaternion { x: 0.0, y: 0.0, z: (yaw * 0.5).sin(), w: (yaw * 0.5).cos() }
}
