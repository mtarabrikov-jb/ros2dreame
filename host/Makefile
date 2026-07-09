# Quick launch for the host-side ROS 2 GUI (see docker-compose.yml).
# All ROS packages are baked into the image (Dockerfile) and all config lives in
# mounts (rviz/ slam/ maps/), so the container survives recreation with ZERO
# runtime installs. Detached launches log to the container's stdout -> follow
# everything with `make logs` (docker compose logs -f).
#   make up      build (first time) + start + allow X
#   make slam | foxglove | rviz | rqt | steer | image | tf | teleop | save-map | logs | down
DC   := docker compose
SVC  := ros2dreame-gui
SRC  := source /opt/ros/jazzy/setup.bash &&
LOG  := >/proc/1/fd/1 2>&1             # detached procs -> container stdout (make logs)
GUI  := $(DC) exec -d $(SVC) bash -c   # detached: GUI opens, make returns
RUN  := $(DC) exec $(SVC) bash -c      # attached (stdin): terminal tools
NAME ?= dreame_map                     # map basename for `make save-map`

.PHONY: build up all down logs shell topics rviz rqt steer image tf teleop foxglove slam localize save-map
build:
	$(DC) build
up:
	-xhost +local: >/dev/null 2>&1 || true
	$(DC) up -d
	@echo ">> up. Try: make slam  |  make foxglove  |  make rviz  |  make steer  |  make logs"
all: up slam rqt steer image foxglove   ## up + full stack: slam (map+rviz), rqt, steer, image, foxglove
	@echo ">> launched: slam+rviz (map), rqt, robot-steering, image view, foxglove_bridge (ws://localhost:8765)"
down:
	$(DC) down
logs:                       ## follow all container stdout (slam, foxglove, rviz, ...)
	$(DC) logs -f
shell:
	$(DC) exec $(SVC) bash
topics:
	$(RUN) '$(SRC) ros2 topic list'
foxglove:                   ## foxglove_bridge WebSocket (ws://localhost:8765) - import host/foxglove/slam.json
	-$(DC) exec -T $(SVC) pkill -9 -f '[f]oxglove_bridge' 2>/dev/null || true
	$(GUI) '$(SRC) ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 -p address:=0.0.0.0 $(LOG)'
	@echo ">> foxglove_bridge on ws://localhost:8765 -- in app.foxglove.dev import host/foxglove/{full,slam,ros2dreame}.json (full = map + all controls in one window)"
slam:                       ## slam_toolbox live map (auto-loads /cfg/maps/dreame_map if present) + rviz
	$(GUI) '$(SRC) ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/cfg/slam/dreame_slam.yaml $(LOG)'
	$(GUI) '$(SRC) rviz2 -d /cfg/rviz/slam.rviz $(LOG)'
	@echo ">> slam + rviz up (saved map auto-loaded). Turret must spin for /scan. Logs: make logs"
localize:                   ## drive on the saved map WITHOUT changing it (mode: localization) + rviz
	$(DC) exec -T $(SVC) bash -c 'sed "s/mode: mapping/mode: localization/" /cfg/slam/dreame_slam.yaml > /tmp/dreame_localize.yaml'
	$(GUI) '$(SRC) ros2 launch slam_toolbox localization_launch.py slam_params_file:=/tmp/dreame_localize.yaml $(LOG)'
	$(GUI) '$(SRC) rviz2 -d /cfg/rviz/slam.rviz $(LOG)'
	@echo ">> LOCALIZATION + rviz up: saved map is fixed (NOT modified). Set start pose via rviz 2D Pose Estimate. Stop mapping (make slam) first - they conflict. Logs: make logs"
save-map:                   ## save current map to host/maps/ (NAME=dreame_map): .pgm/.yaml + slam .posegraph/.data
	$(RUN) '$(SRC) ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: /cfg/maps/$(NAME)}"'
	$(RUN) '$(SRC) ros2 run nav2_map_server map_saver_cli -f /cfg/maps/$(NAME) --ros-args -p save_map_timeout:=15.0'
	-$(DC) exec -T $(SVC) bash -c 'chown $$(stat -c "%u:%g" /cfg/maps) /cfg/maps/$(NAME).* 2>/dev/null'
	@echo ">> saved host/maps/$(NAME).{pgm,yaml,posegraph,data}"
rviz:                       ## /scan /odom /tf + pose
	$(GUI) '$(SRC) rviz2 -d /cfg/rviz/dreame.rviz $(LOG)'
rqt:                        ## everything: topics, images, plots, publisher
	$(GUI) '$(SRC) rqt $(LOG)'
steer:                      ## GUI drive sliders -> /cmd_vel
	$(GUI) '$(SRC) ros2 run rqt_robot_steering rqt_robot_steering $(LOG)'
REPUB = ros2 run image_transport republish --ros-args -p in_transport:=compressed -p out_transport:=raw
image:                      ## camera viewer: republishes compressed->raw so rqt just shows it
	-$(DC) exec -T $(SVC) pkill -f '[i]mage_republisher'
	$(GUI) '$(SRC) $(REPUB) -r in/compressed:=/camera_ir/image_raw/compressed -r out:=/camera_ir/repub $(LOG)'
	$(GUI) '$(SRC) $(REPUB) -r in/compressed:=/camera/image_raw/compressed    -r out:=/camera/repub $(LOG)'
	$(GUI) '$(SRC) ros2 run rqt_image_view rqt_image_view $(LOG)'
	@echo ">> in rqt_image_view pick /camera_ir/repub (IR, nav) or /camera/repub (RGB, observe)"
tf:                         ## TF tree
	$(GUI) '$(SRC) ros2 run rqt_tf_tree rqt_tf_tree $(LOG)'
teleop:                     ## keyboard teleop -> /cmd_vel (needs this terminal)
	$(RUN) '$(SRC) ros2 run teleop_twist_keyboard teleop_twist_keyboard'
