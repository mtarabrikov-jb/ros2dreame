# Dreame W10 MCU + LDS notes (as used by ros2dreame)

The robot's motion/sensor microcontroller (a GD32) is on `/dev/ttyS4`; the LDS
(lidar) turret is on `/dev/ttyS3`. In `ava` OFF mode `ros2dreame` opens and
drives both itself (`src/direct.rs`, ported from `VacuumTiger/dreame-w10/mcud`).
The wire protocol is decoded/encoded by the vendored `dreame-proto` crate.

## Serial

- MCU  `/dev/ttyS4` @ **115200** 8N1, raw.
- LDS  `/dev/ttyS3` @ **230400** 8N1, raw (read-only; the turret spins only when
  told via the MCU nav frames below).

Framing: `<` .. `>` with `?` escaping; body = `[len][type][payload][crc16]`
(CRC-16/Modbus). See `dreame-proto/src/lib.rs`.

## What the driver SENDS (to keep the MCU healthy)

The MCU is a watchdogged device: it only keeps streaming telemetry while the host
sustains the protocol the way `ava` does. `src/direct.rs` replays that, ~50 Hz:

- **`MotorCtrl` (0x00)** `<flag, linear mm/s, rotational rad/s>` at 50 Hz. Gated
  by an enable + a 500 ms command watchdog + a speed clamp (150 mm/s, 1.5 rad/s)
  + a live cliff/bumper hazard gate (from Triggers). Idle = `(0, 0)`.
- **pong to `0x0f`** ping - echo the ping's first 4 bytes (the com-fault
  handshake). Done in the RX loop.
- **heartbeats** `ava` emits: `SetLED` (0x02), `SetCleaning` idle (0x01), and the
  **nav frames** `0x14` / `0x26` / `0x1d` that also keep the **LDS turret**
  spinning (they carry `ava`'s nav values).

Safety: on exit the driver sends a zero `MotorCtrl` so the motors stop; the
hazard gate forces linear velocity to 0 whenever a cliff/bump is detected.

## Base station (dock) control: the `0x26` frame

The dock's **mop-drying fan** and **mop-washing water pump** live in the base
station, not on the robot. The robot relays their commands to the dock over the
charging contacts as the **`0x26`** MCU frame (8-byte payload). Reverse-engineered
by disassembling `node_signal.so` (`AvaCleanDockProcess -> CastComMsg(0x26, buf, 8)`)
and then snooping ava's `ttyS4` writes (LD_PRELOAD `write` hook) while triggering
mop wash/dry from Valetudo (`MopDockClean`/`MopDockDryManualTriggerCapability`):

| byte0 (mode) | 1 | 2 | 3 | 4 | 5 | 6 | 7 | meaning |
|---|---|---|---|---|---|---|---|---|
| `14`/`64` | 00 | 00 | 00 | 00 | 00 | 00 | 02 | idle heartbeat |
| `0e` | 00 | 00 | **78** | 00 | 00 | **01** | 02 | **dry** (dock fan): byte3=0x78 time, byte6=0x01 on |
| `0d` | **64** | **46** | 00 | 00 | 00 | 00 | 02 | **wash** (dock water pump): byte1=0x64 pump rate, byte2=0x46 water (byte1=0 = slow) |

`src/direct.rs` sends these when `station` is set (`/set_station` `0`/`1`/`2`). The
idle heartbeat keeps `byte7=0x04` (the mcud value, benign); the dock commands use
ava's `byte7=0x02`. No `0x25` frame is used on this dock. Only trigger `wash` (the
water pump) when the robot is docked and attended - it pumps water into the base.

## What the MCU REPORTS (decoded -> ROS 2)

- `Status20ms` (0x01, 20 ms): `left_vel`, `right_vel`, `yaw`, `pose_x/y` ->
  **`/odom`** (`nav_msgs/Odometry`) + **`/tf`** `odom->base_link`.
- `Status10ms` (0x02, 10 ms): gyro/accel + wheel-travel increments (IMU; `/imu`
  is planned).
- `Triggers` (0x00): bumpers, wheel-float, cliff/floor sensors, dock, faults
  (drives the hazard gate; `/bumper`/`/cliff`/`/dock` planned).
- `Battery` (0x2b): voltage / SoC / charging (`/battery` planned).
- LDS packets (`/dev/ttyS3`) -> arc sweeps -> **`/scan`** (`sensor_msgs/LaserScan`,
  a ~126 deg rear arc, not 360).

The MCU streams `Status*` **only while it is being driven** (MotorCtrl + the nav
heartbeats). A bare zero MotorCtrl alone is not enough - so in observe mode
(below) there is no `/odom`/`/scan`.

## nav vs observe mode (the RGB constraint)

RGB (`OV8856`/isp0) and `/scan` (the LDS) are **mutually exclusive**, but the
cause is the **spinning LDS turret**, not "driving" or "any active mode". Proven
with `ava` verifiably dead (freeze all four ava watchdogs, see below - an earlier
`ava` respawn confound made this look like an "active mode" policy):

- observe, or nav with the turret off: RGB streams cleanly, **0** isp0 errors.
- nav with the turret spinning: the turret disrupts the OV8856 MIPI - a
  continuous `[VIN_ERR] isp0 frame error, size 0, hblank max.. min..` (the
  horizontal-blank timing jitters ~2x) - and RGB stalls within a few seconds. No
  regulator/clock change is logged at the transition, so it is a physical
  disturbance (turret-motor EMI / shared-rail droop), not a firmware mode switch.
- The disturbance **wedges isp0 persistently**: once the turret has spun, a plain
  reopen of `/dev/video2` keeps erroring (`size 0`) even after the turret stops,
  even with the sensor left idle during the turret, and even after a 60 s wait. It
  does NOT clear by waiting, reopening, a proactive stop-before-turret, a VIN
  driver unbind (which oopses the driver), the `resetsync` ioctl, or persistent
  streaming.
- **Off-dock un-wedge (SOLVED):** send the MCU **camera-AI-reset** frame
  `0x1d [0x05, 0x00]` with the turret off and the camera CLOSED, then re-open the
  RGB camera - isp0 recovers, RGB streams clean, no `ava`/dock/reboot needed. Byte0
  must be **`0x00`** (reset); `0x01` (what nav sends) does NOT clear it. Found by
  disassembling `node_signal.so`: `AvaNodeSignal::AIReset2ComProcess(ava_camera_ai_reset_msg*)`
  builds `CastComMsg(0x1d, {0x05, byte0}, 2)` (siblings: `{0x04,..}` = stereo cam,
  `{0x01,..}` = ToF). `ros2dreame` emits this in observe mode (see `direct.rs`
  `cam_sync`); `direct-mode.sh observe` starts ros2dreame FIRST so the reset lands
  before `w10-camd` opens video2. Verified end to end (real 672x504 RGB recovered
  from a turret-wedged isp0).
- The IR/ToF sensor (`ofilm0092`, isp1 - a separate ISP + MIPI lane) is unaffected.

So driving itself is fine for RGB: `W10_NO_TURRET=1` runs nav (drive-capable,
active MCU) with the turret parked, and RGB + IR both stream. The trade-off is
**RGB vs `/scan`**, not RGB vs motion. `ros2dreame` exposes:

| mode | MCU driving | LDS turret | topics | camera |
|------|-------------|------------|--------|--------|
| **nav** (`direct-mode.sh start`, default) | MotorCtrl + nav frames | spinning | `/scan` `/odom` `/tf` | **IR** (`/camera_ir`) |
| **observe** (`direct-mode.sh observe`, `W10_OBSERVE=1`) | idle | parked | `/odom` `/tf` | **RGB** (`/camera`) |
| nav + `W10_NO_TURRET=1` | MotorCtrl + nav frames | parked | `/odom` `/tf` (no `/scan`) | **RGB** (+ IR) |

Verified on the robot (ava dead): observe delivers real 672x504 colour RGB
(`sensor_msgs/CompressedImage`) end to end; nav delivers `/scan` + `/odom` + the
IR feed; nav+`W10_NO_TURRET` drives with RGB+IR at ~13 fps and 0 isp0 errors.

## keeping ava off (the reboot watchdog)

Stopping `ava` is not just "kill it": the vendor `/etc/rc.d/monitor.sh` probes it
with `avacmd media status_get` and, after 3 failed probes (~90 s), **reboots the
robot** - and `factory_reset.sh monitor_rescue_brick` **factory-resets** it if it
is still down after that reboot. So `ava` can only be stopped if `monitor.sh` is
stopped too. `direct-mode.sh` `ava_off` freezes (`kill -STOP`) the whole ava
reboot+respawn set and only then kills ava:

- `monitor.sh` - the rebooter/factory-resetter (**the** safety-critical freeze).
- `exec_monitor.sh` + `exec_proc` - the launcher chain that respawns ava (else it
  comes back and grabs `ttyS4` + `video1`/`video2` from under `ros2dreame`).
- `sys_monitor.sh` - ava memory/status monitor.

Each is an init child, and init does not respawn a *stopped* child, so the freeze
holds for the session; `ava_off` verifies `monitor.sh` reached state `T` before
touching ava and aborts otherwise. `restore`/`ava_on` touches
`/tmp/restart_ava.mark` (tells `monitor.sh` the downtime was intentional), resumes
the monitors, and relaunches real ava. Do **not** bind a stub over `/usr/bin/ava`:
that leaves `monitor.sh` running, the stub fails the health probe, and it triggers
exactly the reboot + factory-reset above.
