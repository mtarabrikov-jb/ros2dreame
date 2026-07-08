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
    && rm -rf /var/lib/apt/lists/*

# Source ROS in every shell (docker exec bypasses the entrypoint).
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc
