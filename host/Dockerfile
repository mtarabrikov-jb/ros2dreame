# Host-side ROS 2 GUI for ros2dreame (the companion-PC side). Based on the
# official ROS 2 Jazzy desktop image (rviz2 + rqt + demos) plus the plugins that
# make a vacuum usable from a GUI: robot-steering (drive), image-view (cameras),
# the common rqt plugins (topic monitor / plot / message publisher) and keyboard
# teleop.
FROM osrf/ros:jazzy-desktop

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-rqt \
        ros-jazzy-rqt-common-plugins \
        ros-jazzy-rqt-robot-steering \
        ros-jazzy-rqt-image-view \
        ros-jazzy-rqt-tf-tree \
        ros-jazzy-teleop-twist-keyboard \
        ros-jazzy-image-transport-plugins \
        ros-jazzy-slam-toolbox \
        ros-jazzy-nav2-map-server \
        ros-jazzy-foxglove-bridge \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
        git \
        python3-colcon-common-extensions \
        python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# explore_lite (m-explore-ros2) - autonomous frontier exploration, the maintained
# alternative to auto_mapper. Not packaged for Jazzy, so build from source. It runs
# on top of Nav2 (NavigateToPose) + slam_toolbox (/map); see host/README.md and
# host/explore/explore.yaml. Deps (nav2_costmap_2d, nav2_msgs) come from navigation2.
RUN . /opt/ros/jazzy/setup.sh \
    && mkdir -p /opt/explore_ws/src \
    && git clone --depth 1 https://github.com/robo-friends/m-explore-ros2.git /opt/explore_ws/src/m-explore-ros2 \
    && cd /opt/explore_ws \
    && (rosdep update || true) \
    && (rosdep install --from-paths src --ignore-src -y || true) \
    && colcon build --merge-install \
    && rm -rf build log src/m-explore-ros2/.git

# Auto-source ROS + the explore_lite overlay in EVERY shell. `docker exec bash -c`
# runs a NON-interactive shell, which does not read .bashrc - so `make`/exec would
# otherwise have to prefix every command with `source .../setup.bash`. BASH_ENV is
# the file bash sources for non-interactive shells, so pointing it at a setup script
# makes `docker exec ... bash -c 'ros2 ...'` just work, no per-command source.
RUN printf 'source /opt/ros/jazzy/setup.bash\nif [ -f /opt/explore_ws/install/setup.bash ]; then source /opt/explore_ws/install/setup.bash; fi\n' > /opt/ros_env.bash
ENV BASH_ENV=/opt/ros_env.bash
# .bashrc for interactive shells (make shell); BASH_ENV covers non-interactive exec.
RUN echo 'source /opt/ros_env.bash' >> /root/.bashrc
