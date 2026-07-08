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

## Data source: TCP relay (same wire in both modes)

`ros2dreame` reads the raw framed MCU + LDS streams over TCP - `mcu-rx` (7701),
`lds-rx` (7702) - and decodes them with `dreame-proto`. What feeds those ports
decides whether `ava` is on or off:

- **ava OFF (full autonomy, the goal):** the vendored **`w10-mcud`** opens
  `/dev/ttyS4` + `/dev/ttyS3` itself, sustains the MCU (MotorCtrl 50Hz +
  heartbeats + ping/pong, own watchdog/clamp/cliff-gate), and serves 7701/7702
  (+ control 7705). No vendor daemon, no SangamIO.
- **ava ON (development tap):** `avatap-relay` mirrors `ava`'s serial I/O
  read-only onto the same 7701/7702. Lets you develop against live telemetry
  while `ava` keeps the robot safe.

Either way `ros2dreame` is unchanged. Cameras come from go2rtc MJPEG
(`/api/stream.mjpeg?src=camera[_ir]`), fed by the no-ava `w10-cam` stack (ava
off) or `ava`'s own relay (ava on) - wrapped as `CompressedImage`, no decode.

## Full autonomy, ava OFF

Everything below builds from THIS repo. `deploy/direct-mode.sh` (run on the
robot) freezes both ava watchdogs, kills ava, starts `w10-mcud` + the camera
stack + `ros2dreame`, and enables the LDS turret:

```sh
deploy/direct-mode.sh start [rgb|tof|both]   # ava off, full stack up
deploy/direct-mode.sh restore                # ava back
```

(The camera stack `w10-cam`/`noava-cam.sh` lives in the separate
`dreame-vacuum-livestream` project and needs the vendor `libsunxicamera.so`; it
is a runtime dependency, not vendored here.)

## Vendored (self-contained)

- `dreame-proto/` - the Dreame MCU/LDS protocol (pure `no_std`, no deps): framing,
  decode (`Status20ms/10ms/100ms`, `Triggers`, `Battery`) + encode (`MotorCtrl`,
  `SetCleaning`, `SetLED`). Copied from `VacuumTiger/dreame-w10/proto`.
- `w10-mcud/` - the standalone MCU driver (deps: `libc` + `dreame-proto`). Copied
  from `VacuumTiger/dreame-w10/mcud` (branch `dreame_w10_control`).

Both are plain copies so this repo builds the whole ava-off stack on its own. To
resync, re-copy the sources from upstream.

## Build

Static aarch64 musl (for the robot) - use the build script, which wires up the
bundled `rust-lld` cross-linker (see the note in `build/build-aarch64.sh`):

```sh
rustup target add aarch64-unknown-linux-musl   # one-time
./build/build-aarch64.sh
```

Builds the whole workspace - `ros2dreame` AND `w10-mcud` - as fully static
`ELF aarch64` (`ldd` -> "not a dynamic executable") under
`target/aarch64-unknown-linux-musl/release/`. Deploy both with `cat over ssh`
(the robot has no sftp-server), e.g. `cat ros2dreame | ssh root@<ip> 'cat > /data/ros2dreame/ros2dreame && chmod +x $_'` (and likewise `w10-mcud` + `deploy/direct-mode.sh`).

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
