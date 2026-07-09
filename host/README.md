# ros2dreame GUI (host side)

See every topic and drive the vacuum from a GUI, in Docker, without installing
ROS 2 on your machine. Requires **Linux + an X server** (the GUIs need a display)
and Docker with the `compose` plugin.

```sh
cd host
make up        # build (first run) + start the container + allow X access
make all       # launch ALL windows at once: rviz + rqt + steering + image view
# or individually:
make rqt       # the all-in-one GUI (topics, images, plots, publisher)
make steer     # drive sliders -> /cmd_vel (steer the vacuum with the mouse)
make rviz      # /scan /odom /tf + robot pose
make down      # stop
```

## Seeing all topics

- **`make rqt`** opens rqt. Add plugins from its menu (`Plugins >`):
  - **Topic Monitor** - every topic, live values (`/odom` `/imu` `/battery`
    `/dock` `/bumper` `/cliff` `/motor_currents` ...).
  - **Image View** - `make image` (or `make all`) starts a `republish`
    compressed->raw node per camera, so rqt shows the decoded frames directly:
    pick **`/camera_ir/repub`** (IR, nav mode) or **`/camera/repub`** (RGB,
    observe mode). rqt_image_view is finicky with compressed-only topics, hence
    the raw republish.
  - **Plot** - graph `/imu`, `/battery/percentage`, `/motor_currents/data[2]`
    (the main-brush current), etc.
  - **Message Publisher** - publish `/set_fan` `/set_main_brush` `/set_side_brush`
    `/set_mop` (`std_msgs/UInt8`) to run the actuators, or `/set_station` (0/1/2)
    for the dock.
- **`make image`** - just the camera viewer. **`make tf`** - the TF tree.
- **`make rviz`** - the spatial view (laser scan, odometry, frames).

## Driving the vacuum

- **`make steer`** - `rqt_robot_steering`: two sliders (linear / angular) that
  publish `geometry_msgs/Twist` on `/cmd_vel`. Move the sliders, the robot drives
  (nav mode; the on-robot driver clamps to 150 mm/s, 1.5 rad/s and stops on a
  cliff/bump or if commands stop for 500 ms).
- **`make teleop`** - keyboard teleop in the current terminal (`i/j/k/l`, space =
  stop). Needs the robot in nav mode.

## Mapping (SLAM)

**`make slam`** runs `slam_toolbox` on `/scan` + the TF chain ros2dreame already
publishes (`odom -> base_link -> laser`) and opens rviz with `rviz/slam.rviz` (the
`/map` occupancy grid + live `/scan`, fixed frame `map`). It builds a map live.

- **The turret must be spinning** for `/scan` (parked = no scan): click **Turret
  ON** in Foxglove, or `make steer` and drive.
- **Drive around** to grow the map - a stationary robot only maps the visible arc.
- **Foxglove:** import `foxglove/slam.json` (a 3D panel: `/map` + `/scan` + TF).
- ros2dreame already does the job the makerspet tutorial splits across four nodes
  (`vacuum_bridge` + `robot_state_publisher` + `ekf`): it publishes `/scan` and the
  full `odom -> base_link -> laser` TF, so only `slam_toolbox` is added on top.
- Caveat: the W10 LDS is a **~123 deg rear arc** (not 360), so the map is coarser
  than a 360 lidar (harder loop-closure, more drift). Config: `slam/dreame_slam.yaml`.

## Notes

- The robot must be running ros2dreame (`deploy/direct-mode.sh start` for nav, or
  `observe` for the RGB camera). `ROS_DOMAIN_ID` must match (0 by default).
- The container uses the default Fast-DDS transports (SHM for local large-image
  republish + UDP to reach the robot's RustDDS). To force UDP-only, set
  `FASTRTPS_DEFAULT_PROFILES_FILE=/cfg/fastdds_no_shm.xml` in `docker-compose.yml`.
- Cameras go **only** over ROS topics now (no HTTP/MJPEG): `w10-camd` -> tmpfs shm
  -> `ros2dreame` -> `/camera/image_raw/compressed` (RGB, observe) or
  `/camera_ir/image_raw/compressed` (IR, nav). They are **reliable** QoS (large
  JPEGs fragment); set rqt/rviz image displays to Reliable or they show nothing.
