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

## MCU init: the `0x14` register group (cliff enable)

`0x14` is not a single "nav on" frame - it is a **register write**: `<0x14, reg,
val>`. ava streams a group of four every cycle: `reg 0x01=01`, `0x00=01`,
`0x09=02`, `0x04=00` (`0x04` = LDS turret: `01` on / `00` off). Without regs
`0x00`/`0x01`/`0x09` the MCU **does not scan the 6 downward cliff sensors** -
`Triggers` `raw[1]` stays `0x00` even on a full lift. Sending them makes `raw[1]`
fire `0x00`->`0x3f` exactly like ava (verified live: lift -> `02 03 07 13 1a 1f
3f`, the six bits filling in). `src/direct.rs` streams `0x00`/`0x01`/`0x09` in
both modes (reg `0x04` is left to the turret logic, which toggles `04 00`/`04 01`
- a second fixed `04` here would fight it in nav). Harmless to the RGB camera in
observe (verified: `video2` keeps streaming, 0 misses). Escape hatch:
`W10_NO_MCU_INIT=1` disables the group. Found by snooping ava's `ttyS4` boot
writes (LD_PRELOAD read+write hook) and diffing the frame set against ros2dreame's
- the cliff enable is **not** GPIO/PWM/sysfs (ava does no sysfs/gpio writes at
boot; it only opens `/dev/i2c-2`/`i2c-3` for the ToF sensor), it is this MCU
register group.

## Base station (dock) control: the `0x26` frame

The dock's **mop-drying fan** and **mop-washing water pump** live in the base
station, not on the robot. The robot relays their commands to the dock over the
charging contacts as the **`0x26`** MCU frame (8-byte payload). Reverse-engineered
by disassembling `node_signal.so` (`AvaCleanDockProcess -> CastComMsg(0x26, buf, 8)`)
and then snooping ava's `ttyS4` writes (LD_PRELOAD `write` hook) while triggering
mop wash/dry from Valetudo (`MopDockClean`/`MopDockDryManualTriggerCapability`):

| byte0 (mode) | 1 | 2 | 3 | 4 | 5 | 6 | 7 | meaning |
|---|---|---|---|---|---|---|---|---|
| `15` | 00 | 00 | 00 | 00 | 00 | 00 | 02 | **docked idle / STOP** (charging screen). ava streams this continuously; this exact frame is what aborts a running wash/dry |
| `0e` | 00 | 00 | **78** | 00 | 00 | **01** | 02 | **dry** 1/4 (dock fan/heater): byte3=0x78 time, byte6=0x01 on |
| `65`/`66`/`67` | 00 | 00 | 78 | 00 | 00 | 01 | 02 | **dry** 2/4, 3/4, 4/4 - byte0 advances the "dehydrating N/4" LCD screen |
| `0d` | **64** | **46** | 00 | 00 | 00 | 00 | 02 | **wash** (dock water pump): byte1=0x64 pump rate, byte2=0x46 water (byte1=0 = slow) |

`byte0` doubles as the dock **LCD screen** code (`0x0d`="Mop pad cleaning",
`0x0e`/`0x65`-`0x67`="dehydrating 1/4..4/4", `0x15`="100%"/charging - full table in
[`DOCK_PROTOCOL.md`](../../VacuumTiger/dreame-w10/docs/DOCK_PROTOCOL.md)).

`src/direct.rs` drives these from `/set_station` (`0`/`1`/`2`):
- **`0` = idle / STOP** - streams the `0x15` docked-idle frame **continuously** (in
  observe/parked mode). This is what **aborts** a running wash/dry: the dock latches
  its own autonomous cycle once started, and only ava's exact `0x15` frame, streamed,
  stops it. **`byte0=0x14` (an earlier guess) does NOT abort a dry; `0x15` does** -
  verified live against a Valetudo "stop mop drying" snoop. A few pulses are not
  enough; it must be streamed.
- **`1` = dry** - runs the dehydrating fan/heater and walks byte0 through the four
  screens (`0x0e`->`0x65`->`0x66`->`0x67` = 1/4..4/4) over `W10_DRY_STAGE_MS` per
  stage (default 5 min -> 20 min). `DRY_SCREENS` in `src/direct.rs`.
- **`2` = wash** - the full cycle (`WASH_STEPS`, snooped from a Valetudo wash: wet
  wash with a pulsing pump + rotating pads -> scrub) followed by the dry stages;
  byte0=`0x0d` ("Mop pad cleaning") through the wash portion.

Only trigger a wash when docked + attended - it pumps water into the base. The pads
do NOT rotate while docked in vacuum mode 0x03 (mop mode 00). **Verified end-to-end
live**: `/set_station 1` -> the LCD walks dehydrating 1/4..4/4 with the fan on;
`/set_station 0` aborts it (fan off, back to "100%"). No `0x25` frame is used on this
dock; the nav-mode idle heartbeat uses `byte7=0x04` (mcud value), dock commands
`byte7=0x02`.

### The dock is a second MCU (below the `0x26` frame)

The `0x26` frame above is only Layer 1 (`ava` -> robot MCU over `ttyS4`). The robot
MCU is a *bridge*: it re-encodes dock commands and sends them to a **separate
microcontroller inside the dock** over a sub-GHz **RF** link (the dock has its own
GD32 MCU, LCD, wash/dry pumps + heater + fan, radio and ymodem bootloader). The
robot MCU <-> dock MCU frame is `AA 55 | len | cmd | payload | crc16 | 0D 0A`
(CRC-16, CR/LF trailer); `cmd 0x80` is the dock control/display frame (9-byte
payload = screen code, UI set, progress %, status) and `cmd 0x01` + `"BT"` triggers
the dock's OTA bootloader. This second layer, the dock LCD (`Ux.bin` images on the
dock's FAT32), and how the dock firmware (`/UIMA.bin` / `/UIMB.bin`) is flashed are
reverse-engineered in full in
[`VacuumTiger/dreame-w10/docs/DOCK_PROTOCOL.md`](../../VacuumTiger/dreame-w10/docs/DOCK_PROTOCOL.md).
(The earlier "over the charging contacts" description of this link was an untested
assumption; the firmware shows a real radio. The exact carrier — RF vs. a
charging-pin serial link when docked — is not yet confirmed on the wire.)

### Driving the dock LCD status (`/set_dock_screen`)

The dock's LCD status is chosen by the **`0x26` frame's byte0** (the dock "mode").
Traced in the robot MCU (`mcu.bin`): the ttyS4 `0x26` payload feeds `FUN_08015866`,
which fills a dock-state struct (`dst[0]=byte0` mode, `dst[1..3]=byte1..3` params,
`dst[6]=byte6*4+1` actuator mask, percent from `byte7`); a periodic task
`FUN_0801679c` derives the screen code from `byte0` and packs the 9-byte `cmd 0x80`
payload the MCU relays to the dock (`FUN_08009238`). So byte0 is what the dock
renders. Known values: `0x14` idle, `0x0d` wash, `0x0e` dry (the ones we send for
`/set_station`); the dock firmware has many more screen codes (see
[`DOCK_PROTOCOL.md`](../../VacuumTiger/dreame-w10/docs/DOCK_PROTOCOL.md)).

`ros2dreame` exposes this as **`/set_dock_screen`** (`std_msgs/UInt8`): while the
dock is idle (`/set_station 0`), a non-zero value sends an **idle-shaped `0x26`**
(`[code,0,0,0,0,0,0,0x02]` — no pump/fan, byte6=0) with byte0 = the code, so the
robot MCU relays it and the LCD shows that screen; `0` leaves the dock alone. It is
ignored while a wash/dry is running (those drive the screen themselves).
`src/direct.rs` (`set_dock_screen`, the `0x26` idle branch).

For full manual control there is also **`/set_dock_frame`** (`std_msgs/UInt8MultiArray`):
its first 8 bytes are sent verbatim as the idle `0x26` (overrides `/set_dock_screen`),
so you can drive byte0 = screen, byte7 = progress, etc. by hand. Caution: byte6 != 0
can start the dock pump/fan.

**byte0 -> screen table (captured live).** The full mapping is in
[`DOCK_PROTOCOL.md`](../../VacuumTiger/dreame-w10/docs/DOCK_PROTOCOL.md#byte0---dock-lcd-screen-verified-live-r2104-w10-dock)
- e.g. `0x03` Cleaning, `0x0c` 100%/charging, `0x0d` mop-wash, `0x0e` mop-dry,
`0x11` returning-to-wash, `0x17..0x21` the tank/child-lock/exception/"Mapping" alerts.
The dock LCD is on no camera, so it was read off the panel by hand. To re-capture or
extend it, with the robot **docked + idle** and ros2dreame running, sweep the codes
and watch the dock:
```
make dock-sweep                 # sweeps /set_dock_screen over candidate codes
DWELL=12 CODES="3 4 6 23 29" make dock-sweep   # focus a subset
```
**DDS gotcha (learned live):** a one-shot `ros2 topic pub --once` (and short
`-r`/`timeout` runs) frequently **drops** against the robot's RustDDS - the
FastDDS<->RustDDS discovery isn't done before the process exits, so the message
never lands and the screen doesn't change. `dock-sweep.sh` therefore uses a
**persistent rclpy publisher** (one node, ~2 s discovery, then holds each code a few
seconds). Publish the same way by hand if scripting it.
`host/dock-sweep.sh` is the sweep. The dock's screen-code classes (which file each
code loads) are in [`DOCK_PROTOCOL.md`](../../VacuumTiger/dreame-w10/docs/DOCK_PROTOCOL.md);
the ava-side status -> screen-code logic lives in `node_aether.so` `SendCleanDockMsg`
(too large to extract cleanly - the live sweep is faster). Same method that produced
the `0x26` wash/dry table.

## What the MCU REPORTS (decoded -> ROS 2)

- `Status20ms` (0x01, 20 ms): `left_vel`, `right_vel`, `yaw`, `pose_x/y` ->
  **`/odom`** (`nav_msgs/Odometry`) + **`/tf`** `odom->base_link`.
- `Status10ms` (0x02, 10 ms): gyro/accel + wheel-travel increments (IMU; `/imu`
  is planned).
- `Triggers` (0x00): bumpers, wheel-float, cliff/floor sensors, dock, faults
  (drives the hazard gate). `raw[1]` = the **6 downward cliff sensors** (bits 0-5).
  Published split: **`/cliff/<name>`** (`std_msgs/Bool`, one per sensor:
  `front_left` bit0, `mid_left` bit1, `mid_right` bit2, `front_right` bit3,
  `rear_left` bit4, `rear_right` bit5 - bits 0/3/4/5 confirmed from the global bit
  map, bits 1/2 are best-guess mid positions), plus **`/cliff/flags`**
  (`std_msgs/UInt8`, the raw 6-bit mask) and the aggregate **`/cliff`**
  (`std_msgs/Bool`, any sensor). The cliff scan only reports once the MCU 0x14 init
  group is sent (see above) - otherwise all stay 0.
- `Battery` (0x2b): voltage / SoC / charging (`/battery` planned).
- **Dock status** (`0x23`, 6 bytes, ~100 ms) -> the **base-station buttons**. byte0
  bit0 = **Home**, bit2 = **Start/Stop**, bit4 (`0x10`) = a constant docked flag;
  byte2 = tank flags (see MCU_PROTOCOL). ros2dreame publishes **`/dock_button_home`**
  and **`/dock_button_start`** (`std_msgs/Bool`, true while held). Found live: at rest
  0x23 = `10 00 00 00 00 42`; Home press -> `11 ..`, Start/Stop -> `14 ..`. (A generic
  RX frame-change logger, `W10_RX_DEBUG=1` in `src/direct.rs`, located the frame.)
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
