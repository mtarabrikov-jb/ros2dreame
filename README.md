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

Working, ava OFF, one binary (verified on the robot -> a Jazzy container):
- `/scan` `sensor_msgs/LaserScan` - LDS arc (W10 is a ~126 deg rear arc, not 360)
- `/odom` `nav_msgs/Odometry` + `/tf` (`odom -> base_link`) + `/tf_static`
  (`base_link -> laser`)
- `/camera/image_raw/compressed`, `/camera_ir/image_raw/compressed`
  `sensor_msgs/CompressedImage`

Planned next:
- `/imu`, `/battery`, `/bumper`, `/cliff`, `/dock` (+ raw dock-IR), motor currents
- `/cmd_vel` `geometry_msgs/Twist` -> `MotorCtrl` (the direct driver already has
  the gated drive path; `Shared::set_drive` just needs a subscriber) + actuator
  services (suction / brushes / water pump / LED)

## Data sources

Two, selected at runtime. Both decode with `dreame-proto` and feed the same
publisher, so the ROS 2 side is identical:

- **DIRECT (default; ava OFF):** `src/direct.rs` opens `/dev/ttyS4` + `/dev/ttyS3`
  itself and drives the MCU in-process - MotorCtrl 50 Hz + `0x0f` pong +
  the periodic SetLED/SetCleaning/`0x14`/`0x26`/`0x1d` frames, with a command
  watchdog, speed clamp and a live cliff/bumper hazard gate - and spins the LDS
  turret. Status -> `/odom`, LDS -> `/scan`. No external daemon, no vendor `ava`,
  no SangamIO. (Ported from `w10-mcud`; drive is present but disabled until
  `/cmd_vel` is wired - the robot does not move yet.)
- **TAP (dev; ava ON):** set `W10_MCU_ADDR=host:7701` to read `avatap-relay`'s
  read-only serial mirror over TCP instead. Lets you develop against live
  telemetry while `ava` keeps the robot safe.

Cameras come from go2rtc MJPEG (`/api/stream.mjpeg?src=camera[_ir]`), fed by the
no-ava `w10-cam` stack (ava off) or `ava`'s relay (ava on) - wrapped as
`CompressedImage`, no decode.

## Full autonomy, ava OFF (one binary, one script)

`deploy/direct-mode.sh` (run on the robot) freezes both ava watchdogs, kills ava
(freeing `ttyS4`/`ttyS3`), brings up the camera stack, and starts `ros2dreame` -
which then drives the MCU/LDS itself:

```sh
deploy/direct-mode.sh start [rgb|tof|both]   # ava off (default both)
deploy/direct-mode.sh restore                # ava back
```

(The camera stack `w10-cam`/`noava-cam.sh` lives in the separate
`dreame-vacuum-livestream` project and needs the vendor `libsunxicamera.so`; it
is a runtime dependency, not part of this repo.)

## Vendored

`dreame-proto/` - the Dreame MCU/LDS protocol (pure `no_std`, no deps): framing,
decode (`Status20ms/10ms/100ms`, `Triggers`, `Battery`) + encode (`MotorCtrl`,
`SetCleaning`, `SetLED`). Copied from `VacuumTiger/dreame-w10/proto`; the direct
driver (`src/direct.rs`) is ported from `VacuumTiger/dreame-w10/mcud`. Re-copy
`dreame-proto/src/*` to resync the protocol.

## Build

Static aarch64 musl (for the robot) - use the build script, which wires up the
bundled `rust-lld` cross-linker (see the note in `build/build-aarch64.sh`):

```sh
rustup target add aarch64-unknown-linux-musl   # one-time
./build/build-aarch64.sh
```

Produces one fully static `ELF aarch64` (`ldd` -> "not a dynamic executable"):
`target/aarch64-unknown-linux-musl/release/ros2dreame`. Deploy it plus
`deploy/direct-mode.sh` with `cat over ssh` (the robot has no sftp-server), e.g.
`cat ros2dreame | ssh root@<ip> 'cat > /data/ros2dreame/ros2dreame && chmod +x $_'`.

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
