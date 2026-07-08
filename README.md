# ros2dreame

Standalone ROS 2 bridge for the **Dreame Bot W10 (r2104)**, as **one static Rust
binary**. It exposes the robot's sensors, actuators and cameras through standard
ROS 2 interfaces (`sensor_msgs`, `nav_msgs`, `geometry_msgs`) so a host running
Nav2 / SLAM / rviz can drive and map the robot.

Unlike a Python bridge on a companion PC, ros2dreame runs **on the robot itself**:
it uses the pure-Rust [`ros2-client`](https://github.com/jhelovuo/ros2-client)
(RustDDS) so there is **no ROS 2 install, no `rcl`/`rmw`, no C DDS** on the robot,
and it does **not** depend on the SangamIO daemon. The robot's old glibc (2.23) is
irrelevant because the binary is fully static musl.

## Status

Milestone 0 (current): node + heartbeat, to prove the `ros2-client` + RustDDS +
static-musl-aarch64 toolchain end to end. Next milestones add the real topics.

## Planned ROS 2 interface

Published:
- `/scan` `sensor_msgs/LaserScan` - LDS arc (W10 is a ~126 deg rear arc, not 360)
- `/odom` `nav_msgs/Odometry` + `/tf` `odom -> base_link`
- `/imu` `sensor_msgs/Imu`
- `/battery` `sensor_msgs/BatteryState`
- `/bumper`, `/cliff`, `/dock` - contact / cliff / dock-connected + raw dock-IR
- motor currents (drive L/R, brushes, load) - diagnostics
- `/camera/image_raw/compressed`, `/camera_ir/image_raw/compressed`
  `sensor_msgs/CompressedImage` (+ `CameraInfo`)

Subscribed (direct mode):
- `/cmd_vel` `geometry_msgs/Twist` -> `MotorCtrl`
- services: suction / brushes / water pump / LED / LDS motor

## Data sources (two modes, mirroring the SangamIO dreame_w10 driver)

- **tap** (read-only, coexists with the vendor `ava`): connect to `avatap-relay`
  TCP channels `mcu-rx` (7701), `lds-rx` (7702). Publishes sensors + cameras.
- **direct** (replaces `ava`): open `/dev/ttyS4` (MCU) + `/dev/ttyS3` (LDS),
  run the control loop, and actuate from `/cmd_vel` + services.

Cameras come from the existing `ava_cam_relay` MJPEG (`:8090` RGB, `:8091` IR/ToF)
wrapped as `CompressedImage` - no robot-side change to the camera stack.

## Vendored protocol

`dreame-proto/` is a copy of `VacuumTiger/dreame-w10/proto` (pure `no_std`, no
deps, MIT) - the Dreame MCU/LDS framing, decode (`Status20ms/10ms/100ms`,
`Triggers`, `Battery`) and command encode (`MotorCtrl`, `SetCleaning`, `SetLED`).
It is shared, not part of SangamIO. If the upstream protocol changes, re-copy the
two files under `dreame-proto/src/`.

## Build

Static aarch64 musl (for the robot) - use the build script, which wires up the
bundled `rust-lld` cross-linker (see the note in `build/build-aarch64.sh`):

```sh
rustup target add aarch64-unknown-linux-musl   # one-time
./build/build-aarch64.sh
```

Produces `target/aarch64-unknown-linux-musl/release/ros2dreame` - a fully static
`ELF aarch64` (`ldd` -> "not a dynamic executable"). Deploy with `cat over ssh`
(the robot has no sftp-server): `cat ros2dreame | ssh root@<ip> 'cat > /data/ros2dreame/ros2dreame && chmod +x $_'`.

Native (fast API checks on the dev host): `cargo build`.

## Verified (milestone 0)

On the robot (`r2104`, kernel 4.9, glibc 2.23): the static binary runs, RustDDS
opens the RTPS ports (7400/7401/7410/7411), creates the `DomainParticipant`, and
publishes the ROS 2 topic `rt/ros2dreame/heartbeat`. Confirms pure-Rust ROS 2
(ros2-client + RustDDS) works on this hardware with no ROS 2 install and no chroot.

Notes:
- Set `ROS_DISTRO=jazzy` on the host (ros2-client is built for Jazzy; matches the
  `vacuum_ros2_bridge` target).
- The robot has no IPv6, so RustDDS logs harmless `raw_send ... [::1] ... Address
  family not supported` warnings. Ignorable (or disable IPv6 locators in config).
