# ros2dreame GUI (host side)

See every topic and drive the vacuum from a GUI, in Docker, without installing
ROS 2 on your machine. Requires **Linux + an X server** (the GUIs need a display)
and Docker with the `compose` plugin.

```sh
cd host
make up        # build (first run) + start the container + allow X access
make rqt       # the all-in-one GUI (topics, images, plots, publisher)
make steer     # drive sliders -> /cmd_vel (steer the vacuum with the mouse)
make rviz      # /scan /odom /tf + robot pose
make down      # stop
```

## Seeing all topics

- **`make rqt`** opens rqt. Add plugins from its menu (`Plugins >`):
  - **Topic Monitor** - every topic, live values (`/odom` `/imu` `/battery`
    `/dock` `/bumper` `/cliff` `/motor_currents` ...).
  - **Image View** - pick the **base** topic `/camera_ir/image_raw` (IR, nav
    mode) or `/camera/image_raw` (RGB, observe mode); the `compressed` transport
    (bundled via `image-transport-plugins`) decodes it. Set Reliability = Reliable.
  - **Plot** - graph `/imu`, `/battery/percentage`, `/motor_currents/data[2]`
    (the main-brush current), etc.
  - **Message Publisher** - publish `/set_fan` `/set_main_brush` `/set_side_brush`
    `/set_water_pump` (`std_msgs/UInt8`) to run the actuators.
- **`make image`** - just the camera viewer. **`make tf`** - the TF tree.
- **`make rviz`** - the spatial view (laser scan, odometry, frames).

## Driving the vacuum

- **`make steer`** - `rqt_robot_steering`: two sliders (linear / angular) that
  publish `geometry_msgs/Twist` on `/cmd_vel`. Move the sliders, the robot drives
  (nav mode; the on-robot driver clamps to 150 mm/s, 1.5 rad/s and stops on a
  cliff/bump or if commands stop for 500 ms).
- **`make teleop`** - keyboard teleop in the current terminal (`i/j/k/l`, space =
  stop). Needs the robot in nav mode.

## Notes

- The robot must be running ros2dreame (`deploy/direct-mode.sh start` for nav, or
  `observe` for the RGB camera). `ROS_DOMAIN_ID` must match (0 by default).
- The container uses Fast-DDS forced to UDP (`fastdds_no_shm.xml`) so it reaches
  the robot's RustDDS over the network reliably.
- Camera topics are **reliable** QoS (large JPEGs fragment); set rqt/rviz image
  displays to Reliable or they show nothing.
