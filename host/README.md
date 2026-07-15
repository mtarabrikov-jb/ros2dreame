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
- The W10 LDS is a **full 360 deg** scan (8 m; verified live: 359 beams / 358 deg,
  valid returns all around), so loop-closure and mapping are as good as any 360
  lidar. Config: `slam/dreame_slam.yaml`.

## Autonomous navigation & exploration (Nav2)

- **`make nav2`** - Nav2 on a **saved** map (`map_server` + `amcl`): set the start
  pose (rviz `2D Pose Estimate`), then send goals (`Nav2 Goal`). The turret must
  spin for `/scan`. Config: `nav2/nav2_params.yaml`, launch: `nav2/nav2_minimal.launch.py`.
- **`make explore`** - autonomous frontier exploration with **`explore_lite`**
  (m-explore-ros2): it drives to the boundary between mapped and unknown space,
  repeats until the map is closed, then returns home (`return_to_init`). Launches
  slam (fresh map) + Nav2 in SLAM mode (`slam:=true` = no map_server/amcl) + the
  explore node + rviz. Turret must spin. Config: `explore/explore.yaml`.
  - This is the maintained alternative to **auto_mapper** (a ~450-line demo node
    with naive first-frontier pick and no unreachable-frontier blacklist).
    explore_lite is the ROS 2 port of the classic `m-explore`. It is **not packaged
    for Jazzy** - the image builds it from source (`Dockerfile`); `make build` to
    rebuild after pulling.
  - The LDS is a full 360 deg scan, so frontier detection has full surround
    coverage; `min_frontier_size` is kept modest for a home-scale map.

## Fall & contact protection (hazard -> costmap)

Nav2 has no concept of a hole in the floor, or a bump felt but not seen by the
laser. `nav2/hazard_costmap.py` bridges ros2dreame's three MCU hazard sensor groups
into Nav2 costmap obstacles so the planner avoids them:

- `/cliff/flags` (6 drop sensors) + `/wheel_drop/flags` (2 drive-wheel drops) ->
  `/cliff/obstacles` + `/wheel_drop/obstacles` -> a **persistent drop layer**
  (`cliff_layer`, local + global, `clearing:False`): a seen ledge/stair stays avoided.
- `/bumper/flags` (2 front bumpers) -> `/bumper/obstacles` -> a **transient contact
  layer** (`bump_layer`, local only): the rolling window expires it as the robot
  moves off (a bumped chair is not a permanent wall).
- Runs automatically inside `make nav2` / `make explore` (launch arg `hazard:=true`).
  Standalone for testing: **`make hazard`**.
- This is the *planning*-level guard; the *immediate* stop is already on the robot
  (ros2dreame's MCU hazard gate zeroes forward speed the instant a cliff/bump fires).
- The per-sensor mounting offsets in `hazard_costmap.py` are **approximate** - measure
  the W10 and override via the `*_offsets_x/_y` ROS params for precise mark placement.

## Clock sync (required for SLAM / Nav2 / explore)

ROS 2 stamps every message and tf2 rejects transforms whose time is outside its
buffer. If the **robot and this host clocks differ by more than ~1 s**, tf2 in the
container drops the robot's `/scan` + `/tf` ("timestamp earlier than all the data in
the transform cache"), **SLAM never publishes a map**, and Nav2's costmaps never
activate. The Dreame has no NTP client (only busybox `date`/`rdate`/`adjtimex`) and
its clock drifts (~0.1 s/h) and boots off by seconds.

- **ros2dreame syncs its own clock** - no action needed. At startup, before it
  creates its DDS participant, it queries public NTP (by IP - the robot has internet
  but no DNS) and hard-**steps** the clock to UTC; then a background thread **slews**
  it (adjtime, never steps) to hold it there. Pre-DDS stepping means RustDDS starts
  with the right time; slewing keeps ROS/DDS timestamps monotonic afterwards. Since
  this host is NTP-synced too, robot == UTC == host. Disable with `W10_NO_TIMESYNC`.
  See `src/timesync.rs`. The widened `transform_tolerance` (explore 1.5 s, slam
  1.0 s) absorbs the sub-second residual.
- Do **not** run a continuous clock-stepping loop (e.g. `while true; ssh date -s`) -
  repeatedly stepping the robot's system clock, especially backward, desyncs the
  FastDDS<->RustDDS participants and drops robot<->container comms. Slew, don't step.
- **`make timesync` is a fallback only** - for when the robot has no internet, it
  does a one-shot `ssh root@$ROBOT date -s` from this host. Set `ROBOT` via
  `export ROBOT=...` or a local gitignored `host/.env` (copy `host/.env.example`);
  the IP is never committed. Not needed when the robot has internet.

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
