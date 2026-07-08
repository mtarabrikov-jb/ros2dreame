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

The vendor firmware only lets the **OV8856 RGB camera stream when the robot is
fully idle/docked**. In *any* active mode - cleaning, manual control, or our
driving the MCU/turret - the RGB pipeline dies with a continuous
`[VIN_ERR] isp0 frame error, size 0 / sunxi_isp_reset / 8856 pd io` loop (the
MIPI carries no pixel payload). This is a power/clock/firmware policy, **not** ISP
contention and **not** fixable in software - confirmed in
`dreame-vacuum-livestream/docs/REVERSE_ENGINEERING.md` (aggressive power-cycling
gave zero RGB frames). The IR/ToF sensor (`ofilm0092`, a different sensor on the
other ISP) is unaffected.

So the two states are mutually exclusive at the firmware level, and `ros2dreame`
exposes both:

| mode | MCU driving | LDS turret | topics | camera |
|------|-------------|------------|--------|--------|
| **nav** (`direct-mode.sh start`, default) | MotorCtrl + nav frames | spinning | `/scan` `/odom` `/tf` | **IR** (`/camera_ir`) |
| **observe** (`direct-mode.sh observe`, `W10_OBSERVE=1`) | idle (zero MotorCtrl, no nav) | parked | (none) | **RGB** (`/camera`) |

Verified on the robot: observe mode delivers real 672x504 colour RGB
(`sensor_msgs/CompressedImage`) end to end; nav mode delivers `/scan` + `/odom` +
the IR feed.
