# Quick launch for the host-side ROS 2 GUI (see docker-compose.yml).
#   make up      build (first time) + start + allow X
#   make rviz | rqt | steer | image | tf | teleop | topics | shell | down
DC   := docker compose
SVC  := ros2dreame-gui
SRC  := source /opt/ros/jazzy/setup.bash &&
GUI  := $(DC) exec -d $(SVC) bash -c   # detached: GUI opens, make returns
RUN  := $(DC) exec $(SVC) bash -c      # attached (stdin): terminal tools

.PHONY: build up all down shell topics rviz rqt steer image tf teleop foxglove slam
build:
	$(DC) build
up:
	-xhost +local: >/dev/null 2>&1 || true
	$(DC) up -d
	@echo ">> up. Try: make all  |  make rqt  |  make rviz  |  make steer"
all: up rviz rqt steer image   ## up + every GUI window at once
	@echo ">> launched: rviz, rqt, robot-steering, image view"
down:
	$(DC) down
shell:
	$(DC) exec $(SVC) bash
topics:
	$(RUN) '$(SRC) ros2 topic list'
foxglove:                   ## foxglove_bridge WebSocket (ws://localhost:8765) - import host/foxglove/ros2dreame.json
	-$(DC) exec -T $(SVC) bash -c 'ros2 pkg prefix foxglove_bridge >/dev/null 2>&1 || (apt-get update -qq && apt-get install -y -qq ros-jazzy-foxglove-bridge)'
	-$(DC) exec -T $(SVC) pkill -9 -f '[f]oxglove_bridge' 2>/dev/null || true
	$(GUI) '$(SRC) ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 -p address:=0.0.0.0'
	@echo ">> foxglove_bridge on ws://localhost:8765 -- open app.foxglove.dev, connect, import host/foxglove/ros2dreame.json"
slam:                       ## slam_toolbox live map + rviz (turret must spin for /scan)
	-$(DC) exec -T $(SVC) bash -c 'ros2 pkg prefix slam_toolbox >/dev/null 2>&1 || (apt-get update -qq && apt-get install -y -qq ros-jazzy-slam-toolbox)'
	$(GUI) '$(SRC) ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/cfg/slam/dreame_slam.yaml'
	$(GUI) '$(SRC) rviz2 -d /cfg/rviz/slam.rviz'
	@echo ">> slam_toolbox + rviz up. /scan needs the turret spinning (Foxglove Turret ON, or make steer + drive)."
	@echo ">> Foxglove: import host/foxglove/slam.json (3D panel: /map + /scan + TF). Drive to grow the map."
rviz:                       ## /scan /odom /tf + pose
	$(GUI) '$(SRC) rviz2 -d /cfg/rviz/dreame.rviz'
rqt:                        ## everything: topics, images, plots, publisher
	$(GUI) '$(SRC) rqt'
steer:                      ## GUI drive sliders -> /cmd_vel
	$(GUI) '$(SRC) ros2 run rqt_robot_steering rqt_robot_steering'
REPUB = ros2 run image_transport republish --ros-args -p in_transport:=compressed -p out_transport:=raw
image:                      ## camera viewer: republishes compressed->raw so rqt just shows it
	-$(DC) exec -T $(SVC) pkill -f '[i]mage_republisher'
	$(GUI) '$(SRC) $(REPUB) -r in/compressed:=/camera_ir/image_raw/compressed -r out:=/camera_ir/repub'
	$(GUI) '$(SRC) $(REPUB) -r in/compressed:=/camera/image_raw/compressed    -r out:=/camera/repub'
	$(GUI) '$(SRC) ros2 run rqt_image_view rqt_image_view'
	@echo ">> in rqt_image_view pick /camera_ir/repub (IR, nav) or /camera/repub (RGB, observe)"
tf:                         ## TF tree
	$(GUI) '$(SRC) ros2 run rqt_tf_tree rqt_tf_tree'
teleop:                     ## keyboard teleop -> /cmd_vel (needs this terminal)
	$(RUN) '$(SRC) ros2 run teleop_twist_keyboard teleop_twist_keyboard'
