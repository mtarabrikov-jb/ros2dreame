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
- `/camera/image_raw/compressed` (RGB), `/camera_ir/image_raw/compressed` (IR)
  `sensor_msgs/CompressedImage`. Both verified end to end. RGB and `/scan` are
  **mutually exclusive**: the spinning **LDS turret** disrupts the OV8856 MIPI and
  wedges isp0 (recoverable only by an ava reprime or reboot), so RGB streams only
  with the turret off. RGB comes from **observe mode** (turret parked), `/scan` +
  IR from **nav mode** (turret spinning). Driving itself is fine for RGB - it is
  the turret, not motion: `W10_NO_TURRET=1` drives with RGB+IR, minus `/scan`
  (see [docs/MCU.md](docs/MCU.md)).

- `/imu` `sensor_msgs/Imu`, `/battery` `sensor_msgs/BatteryState`,
  `/dock` `/bumper` `/cliff` `std_msgs/Bool` (from Triggers)
- **motor currents** `/current/{wheel_left,wheel_right,main_brush,side_brush,load}`
  `std_msgs/Int16` (also combined in `/motor_currents` `Int16MultiArray`)
- **state** `/state/turret` `Bool` + `/state/mode` `String` (DRIVING/PARKED) +
  `/state/{fan,side_brush,main_brush,mop}` `UInt8` (commanded actuator
  levels) - handy for a Foxglove / rqt dashboard
- `/cmd_vel` `geometry_msgs/Twist` -> `MotorCtrl` (teleop; gated by watchdog +
  clamp + cliff/bump hazard). Verified: drove the robot 7.6 cm from the container.
- **actuator control** `/set_{fan,side_brush,main_brush,mop}` `std_msgs/UInt8`
  (0 = off, ~1-150 = level -> the SetCleaning frame). `mop` = the two rotating mop
  pads (byte[3]); the robot has no water pump. Publish from a GUI to toggle.
- **base station (dock) control** `/set_station` `std_msgs/UInt8`: 0 = idle/**stop**,
  1 = **dry** the mop pads (dock fan/heater; the LCD walks "dehydrating 1/4..4/4" via
  `DRY_SCREENS`), 2 = **run the full mop-wash cycle** (a self-timed state machine
  replayed from ava: wet wash with a pulsing pump + rotating pads -> scrub -> the dry
  stages; `WASH_STEPS` in `src/direct.rs`). Driven via the `0x26` frame (pump/fan +
  byte0=dock LCD screen) + SetCleaning (pads); RE'd from ava, see
  [docs/MCU.md](docs/MCU.md). Only run wash docked + attended - it pumps water into
  the base. **`/set_station 0` aborts** a running wash/dry - it streams ava's exact
  `0x15` stop frame (a `0x14` idle does not stop a dry; verified live). Verified
  end-to-end on the robot: start dry -> LCD animates 1/4..4/4 -> `0` aborts it.
- **dock LCD status** `/set_dock_screen` `std_msgs/UInt8`: sets the status screen the
  base station shows on its own LCD (the `0x26` byte0 "mode" the dock renders; the
  robot MCU relays it to the dock over RF). Non-zero while idle sends a safe
  idle-shaped `0x26` (no pump/fan) with that screen code; `0` leaves the dock alone;
  ignored during a wash/dry. Known codes `0x14` idle / `0x0d` wash / `0x0e` dry; more
  in the dock RE ([docs/MCU.md](docs/MCU.md)). byte0->screen mapping beyond those is
  experimental - drive it and watch the panel (`make dock-sweep`).
- **raw dock frame** `/set_dock_frame` `std_msgs/UInt8MultiArray`: first 8 bytes ->
  the `0x26` payload verbatim (overrides `/set_dock_screen` while idle) - full manual
  control for RE (byte0=screen, byte7=progress). Caution: byte6 != 0 runs the pump/fan.
- **turret control** `/set_turret` `std_msgs/Bool`: true = drive state (turret +
  `/scan` + IR, RGB drops), false = park state (turret off, both cameras). In
  `W10_AUTO` it takes manual control (pauses the motion auto-switch); `/set_auto`
  `Bool` true resumes auto. Good for GUI on/off buttons (Foxglove Button panel).

Planned next:
- raw dock-IR; LED control

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

## Cameras

The vendored helper `w10-camd` (`cam-helper/`) captures a frame, JPEG-encodes it,
and writes it into a tmpfs shared-memory ring (seqlock; `cam-helper/ros2cam_shm.h`)
- `/tmp/ros2cam.shm` (RGB) or `/tmp/ros2cam_ir.shm` (IR/ToF). `ros2dreame` reads the
ring and publishes each frame as `sensor_msgs/CompressedImage`. No HTTP/MJPEG
server, no `go2rtc`, no `ava_cam_relay` - frames go straight into ROS topics.

- **RGB** (OV8856, /dev/video2, isp0): driven via the vendor `libsunxicamera.so`
  (NV21 672x504). Runs in `observe` mode -> `/camera` (needs the **LDS turret
  off** - spinning it wedges isp0). If it was wedged by a prior turret run,
  `observe` un-wedges it off-dock (no `ava`/reboot) by sending the MCU
  camera-AI-reset frame `0x1d [0x05,0x00]` before opening video2 - see
  [docs/MCU.md](docs/MCU.md).
- **IR/ToF** (ofilm0092 Sunny iToF, /dev/video1, isp1): driven with **raw V4L2**
  (MPLANE BG12 224x1558) + the ToF media pipeline + the sensor's i2c enable
  registers on `/dev/i2c-2` @0x3d (`libsunxicamera` is RGB-only and can't bring the
  ToF up). Runs in nav mode -> `/camera_ir` (structured-light IR; `ir_process.h`
  decodes the 9 sub-frames to grayscale).

**CRITICAL:** `ava` must be fully dead before a camera opens - while it lives it
holds the video nodes and floods isp0 with frame errors, and the capture comes out
as pure noise (this was a long red herring). `direct-mode.sh` freezes both
watchdogs, kills `ava`, and waits for it to exit first, then runs ONE camera per
mode (`observe`->RGB, nav->ToF). Full hardware/kernel RE:
`../dreame-vacuum-livestream/phase3-noava/README.md`.

Why a separate helper (not folded into ros2dreame): driving the camera needs the
vendor `.so`, which must be `dlopen`'d - and a fully static musl binary has no
dynamic loader, so it can't. `w10-camd` is therefore a small DYNAMIC binary (only
`<= GLIBC_2.17` symbols, so it loads on the robot's glibc 2.23); ros2dreame stays
static. The one irreducible runtime dependency is `/usr/lib/libsunxicamera.so`
(the Allwinner ISP driver, part of the robot OS) - not vendorable, only dlopen'd.

Image topics use **best-effort** QoS. Reliable was tried first (a JPEG is many
RTPS fragments, and one lost fragment drops the whole frame), but two reliable
JPEG streams saturate RustDDS's single network thread over WiFi - the reliable
retransmit/ACK traffic blocks it and **starves the incoming `/cmd_vel`, so the
robot stops responding to drive commands**. Best-effort images (plus a lower JPEG
quality `W10_JPEG_Q` and a per-camera rate cap `W10_IMG_MS`) keep the thread free
for control; a dropped frame just shows the next one. Note: cold (just-started)
publishers still take ~10-20 s to discover over WiFi - a **warm**, already-connected
publisher (a persistent Foxglove/rqt session) drives with no perceptible lag.

## Full autonomy, ava OFF (one static binary + one dynamic helper, one script)

`deploy/direct-mode.sh` (run on the robot) freezes the ava reboot+respawn
watchdogs, kills ava (freeing `ttyS4`/`ttyS3` + the cameras), and starts
`w10-camd` + `ros2dreame`:

```sh
deploy/direct-mode.sh start      # nav mode: /scan /odom /tf + IR camera
deploy/direct-mode.sh observe    # park mode: RGB camera (turret off; no /scan)
deploy/direct-mode.sh auto       # auto-switch: drive -> /scan+IR; stop -> RGB+IR
deploy/direct-mode.sh restore    # ava back
```

**`auto`** is the useful one: ros2dreame follows `/cmd_vel` and switches itself.
While driving it spins the turret (`/scan` + IR, RGB is wedged); ~3 s after motion
stops it parks the turret, sends the RGB un-wedge reset (`0x1d [05 00]`, see
[docs/MCU.md](docs/MCU.md)), and runs **both** cameras -> `/camera` (RGB) +
`/camera_ir` (IR) at once. It owns the `w10-camd` helper (starts `tof`<->`both`
itself). So: drive the vacuum with the map + IR, stop, and look at RGB + IR
together - no ava, no dock, no reboot. (Two JPEG streams over WiFi are heavy, so
`auto` defaults to `W10_JPEG_Q=35` @ `W10_IMG_MS=200` (~5 fps/camera, best-effort)
to keep `/cmd_vel` responsive while driving; raise the quality if you are not
driving. Drive from a warm publisher - a cold node's first command lags ~10-20 s
on WiFi discovery.)

**Freezing the watchdogs is mandatory, not cosmetic:** the vendor `monitor.sh`
reboots (then factory-resets) the robot if ava is not alive, so `ava_off` freezes
it first - see [docs/MCU.md](docs/MCU.md). `start` (nav) and `observe` (park) are
mutually exclusive only because of the LDS turret (it wedges the RGB isp0), not
because of motion - the same doc covers the `W10_NO_TURRET` drive-with-RGB path.

## GUI (host side)

See every topic and drive the vacuum from a GUI, in Docker, with no ROS 2
install (Linux + an X server). See [host/README.md](host/README.md):

```sh
cd host
make up      # build + start the ROS 2 Jazzy desktop container (net=host, X11)
make rqt     # topics, images, plots, message publisher (actuators)
make steer   # drive sliders -> /cmd_vel
make rviz    # /scan /odom /tf + pose
```

## Vendored (self-contained)

- `dreame-proto/` - the Dreame MCU/LDS protocol (pure `no_std`, no deps): framing,
  decode (`Status20ms/10ms/100ms`, `Triggers`, `Battery`) + encode (`MotorCtrl`,
  `SetCleaning`, `SetLED`). From `VacuumTiger/dreame-w10/proto`; the direct driver
  (`src/direct.rs`) is ported from `VacuumTiger/dreame-w10/mcud`.
- `cam-helper/` - `w10-camd.c` (camera driver + JPEG + shm publisher, merged from
  `w10-cam` + `ava_cam_relay`) plus `ros2cam_shm.h`/`ir_process.h`/`jpeg_gray.h`. `make docker`
  cross-builds it for the robot (dynamic aarch64).

## Build

Static aarch64 musl (for the robot) - use the build script, which wires up the
bundled `rust-lld` cross-linker (see the note in `build/build-aarch64.sh`):

```sh
rustup target add aarch64-unknown-linux-musl   # one-time
./build/build-aarch64.sh                        # -> ros2dreame (static musl)
( cd cam-helper && make docker )                # -> w10-camd (dynamic aarch64)
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
