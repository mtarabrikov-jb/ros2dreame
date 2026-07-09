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

- Fan / Side brush / Main brush ON = `std_msgs/UInt8 {data: 100}`, OFF = `{data: 0}`.
- Turret ON = `std_msgs/Bool {data: true}` (drive state: `/scan` + IR, RGB drops);
  OFF = `{data: false}` (park state: both cameras). A turret click pauses the
  `W10_AUTO` motion auto-switch.
- Resume AUTO = `/set_auto {data: true}` (hand control back to motion-based auto).

## Notes

- The Image panels point at `/camera/image_raw/compressed` (RGB) and
  `/camera_ir/image_raw/compressed` (IR). If your Foxglove version doesn't
  auto-load them, just re-pick the topic in the panel.
- `/camera` (RGB) streams in the parked state; `/camera_ir` (IR) streams while
  driving and parked.
