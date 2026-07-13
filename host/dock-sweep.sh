#!/usr/bin/env bash
# Build / re-capture the dock-LCD byte0 -> screen table by sweeping screen codes and
# watching the base station's own panel. The dock LCD is not on any camera, so this
# is a human-in-the-loop capture: for each code the screen is held for a few seconds;
# you write down what the dock shows. The captured table lives in
# VacuumTiger/dreame-w10/docs/DOCK_PROTOCOL.md.
#
# Preconditions: robot DOCKED + idle (/set_station 0), ros2dreame running, run from
# inside the ros2dreame-gui container (ROS 2 sourced). Env: CODES="3 4 6 23" to pick
# codes, DWELL=<seconds> per code (default 12).
#
# IMPORTANT: uses a PERSISTENT rclpy publisher, NOT `ros2 topic pub --once`. Against
# the robot's RustDDS, one-shot pubs (and short `-r`/timeout runs) routinely drop -
# FastDDS<->RustDDS discovery isn't finished before the process exits, so nothing
# lands and the screen never changes. One long-lived node (discover once, then keep
# publishing) is reliable.
set -u
source /opt/ros/jazzy/setup.bash 2>/dev/null || true

# Dreame W10 (r2104) known meaningful codes; edit CODES to focus.
DEFAULT="1 2 3 4 6 7 9 12 13 14 15 16 17 18 23 24 25 26 27 29 30 31 32 33 101 102 103"
CODES=${CODES:-$DEFAULT}
DWELL=${DWELL:-12}

CODES="$CODES" DWELL="$DWELL" python3 - <<'PY'
import os, time, rclpy
from std_msgs.msg import UInt8
from rclpy.qos import qos_profile_sensor_data
codes = [int(x) for x in os.environ["CODES"].split()]
dwell = float(os.environ["DWELL"])
rclpy.init()
n = rclpy.create_node("dock_sweep")
p = n.create_publisher(UInt8, "/set_dock_screen", qos_profile_sensor_data)
print("== discovering (2s) ==", flush=True)
time.sleep(2)
for c in codes:
    print(f">> code {c} (0x{c:02x}) - LOOK NOW ({dwell:.0f}s)", flush=True)
    end = time.time() + dwell
    while time.time() < end:
        p.publish(UInt8(data=c)); time.sleep(0.15)
print(">> done - restoring idle (send /set_dock_screen 0 to release the dock)", flush=True)
for _ in range(20):
    p.publish(UInt8(data=0)); time.sleep(0.1)
n.destroy_node(); rclpy.shutdown()
PY
echo "== done. Note byte0 -> screen for DOCK_PROTOCOL.md =="
