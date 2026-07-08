# Quick launch for the host-side ROS 2 GUI (see docker-compose.yml).
#   make up      build (first time) + start + allow X
#   make rviz | rqt | steer | image | tf | teleop | topics | shell | down
DC   := docker compose
SVC  := ros2dreame-gui
SRC  := source /opt/ros/jazzy/setup.bash &&
GUI  := $(DC) exec -d $(SVC) bash -c   # detached: GUI opens, make returns
RUN  := $(DC) exec $(SVC) bash -c      # attached (stdin): terminal tools

.PHONY: build up all down shell topics rviz rqt steer image tf teleop
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
rviz:                       ## /scan /odom /tf + pose
	$(GUI) '$(SRC) rviz2 -d /cfg/rviz/dreame.rviz'
rqt:                        ## everything: topics, images, plots, publisher
	$(GUI) '$(SRC) rqt'
steer:                      ## GUI drive sliders -> /cmd_vel
	$(GUI) '$(SRC) ros2 run rqt_robot_steering rqt_robot_steering'
image:                      ## camera viewer (/camera, /camera_ir)
	$(GUI) '$(SRC) ros2 run rqt_image_view rqt_image_view'
tf:                         ## TF tree
	$(GUI) '$(SRC) ros2 run rqt_tf_tree rqt_tf_tree'
teleop:                     ## keyboard teleop -> /cmd_vel (needs this terminal)
	$(RUN) '$(SRC) ros2 run teleop_twist_keyboard teleop_twist_keyboard'
