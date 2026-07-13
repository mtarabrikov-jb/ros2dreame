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

/// sensor_msgs/CompressedImage
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct CompressedImage {
    pub header: Header,
    pub format: String, // "jpeg"
    pub data: Vec<u8>,
}

/// sensor_msgs/Imu ([f64; 9] <= 32 so no BigArray needed).
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct Imu {
    pub header: Header,
    pub orientation: Quaternion,
    pub orientation_covariance: [f64; 9],
    pub angular_velocity: Vector3,
    pub angular_velocity_covariance: [f64; 9],
    pub linear_acceleration: Vector3,
    pub linear_acceleration_covariance: [f64; 9],
}

/// sensor_msgs/BatteryState
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct BatteryState {
    pub header: Header,
    pub voltage: f32,
    pub temperature: f32,
    pub current: f32,
    pub charge: f32,
    pub capacity: f32,
    pub design_capacity: f32,
    pub percentage: f32, // 0..1
    pub power_supply_status: u8,
    pub power_supply_health: u8,
    pub power_supply_technology: u8,
    pub present: bool,
    pub cell_voltage: Vec<f32>,
    pub cell_temperature: Vec<f32>,
    pub location: String,
    pub serial_number: String,
}

/// std_msgs/Bool
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct Bool {
    pub data: bool,
}

/// std_msgs/UInt8
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct UInt8 {
    pub data: u8,
}

/// std_msgs/Int16
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct Int16 {
    pub data: i16,
}

/// std_msgs/String (named `StringMsg` to avoid shadowing std::string::String)
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct StringMsg {
    pub data: String,
}

/// std_msgs/MultiArrayDimension
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct MultiArrayDimension {
    pub label: String,
    pub size: u32,
    pub stride: u32,
}

/// std_msgs/MultiArrayLayout
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct MultiArrayLayout {
    pub dim: Vec<MultiArrayDimension>,
    pub data_offset: u32,
}

/// std_msgs/Int16MultiArray
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct Int16MultiArray {
    pub layout: MultiArrayLayout,
    pub data: Vec<i16>,
}

/// std_msgs/UInt8MultiArray
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct UInt8MultiArray {
    pub layout: MultiArrayLayout,
    pub data: Vec<u8>,
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
