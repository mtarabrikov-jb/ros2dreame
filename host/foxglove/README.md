# Foxglove dashboard

A ready [Foxglove Studio](https://foxglove.dev) layout for the robot: on/off
buttons for the fan, side brush, main brush and turret (plus a "resume AUTO"
button), the two camera feeds, a motor-current plot, a battery gauge, the
DRIVING/PARKED state, and bumper/cliff/dock indicators.

## Use

1. Start the bridge (a WebSocket server for all ROS 2 topics) in the GUI container:

   ```sh
   make foxglove          # from host/  (or: docker exec -d ros2dreame-gui bash -c \
                          #   'source /opt/ros/jazzy/setup.bash && ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765')
   ```

2. Open Foxglove: the desktop app, or https://app.foxglove.dev in Chrome/Edge.
   **Open connection -> Foxglove WebSocket ->** `ws://localhost:8765` (on this
   host) or `ws://<host-ip>:8765`.

3. **Layout -> Import from file ->** `ros2dreame.json`.

## Buttons

The control buttons publish to the topics ros2dreame subscribes to:

- Fan / Side brush / Main brush / Mop ON = `std_msgs/UInt8 {data: 100}`, OFF = `{data: 0}`.
  Mop = the two rotating mop pads (`/set_mop`); the robot itself has no water pump.
- Dock DRY / WASH / OFF = `/set_station` `std_msgs/UInt8` `{data: 1 / 2 / 0}` - the
  base station's mop-drying fan (1) and mop-washing water pump (2). **WASH pumps
  water into the base** - only use it docked and attended.
- Turret ON = `std_msgs/Bool {data: true}` (drive state: `/scan` + IR, RGB drops);
  OFF = `{data: false}` (park state: both cameras). A turret click pauses the
  `W10_AUTO` motion auto-switch.
- Resume AUTO = `/set_auto {data: true}` (hand control back to motion-based auto).

The **Teleop** panel (top-left) drives the wheels: it publishes
`geometry_msgs/Twist` to `/cmd_vel` (up/down = linear, left/right = rotate).
Driving MOVES the robot; the drive is gated by a 500 ms watchdog + speed clamp +
cliff/bump hazard, and in `W10_AUTO` it flips to the DRIVING state (turret + map +
IR) while you drive and parks ~3 s after you stop. Tune the speeds in the panel
settings (defaults 0.15 m/s / 0.6 rad/s).

## Notes

- The Image panels point at `/camera/image_raw/compressed` (RGB) and
  `/camera_ir/image_raw/compressed` (IR). If your Foxglove version doesn't
  auto-load them, just re-pick the topic in the panel.
- `/camera` (RGB) streams in the parked state; `/camera_ir` (IR) streams while
  driving and parked.
- The **battery draw (A)** plot shows `/battery.current` (signed: + = discharge).
  The MCU has no per-fan current sensor, so the suction fan is monitored here as
  its share of the total draw: ~0.3 A idle -> ~0.9 A with `/set_fan 100` (a ~0.6 A
  jump). Turn the brushes off and stay parked to read the fan's draw in isolation;
  the only fan-specific MCU signal otherwise is the `fan_overcurrent` fault flag.
