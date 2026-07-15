# Minimal, stable Nav2 for ros2dreame. Only the nodes we actually need, each in
# its own lifecycle manager listing exactly those nodes - so the Jazzy extras
# (route_server / collision_monitor / docking_server / waypoint_follower) can't
# abort the bringup. bond_timeout is raised so the manager doesn't tear the stack
# down under CPU / WiFi-/tf jitter. The controller (RPP) publishes /cmd_vel
# directly (no velocity_smoother / collision_monitor in the cmd_vel path).
#
# Args:
#   slam:=false   localization on a saved map (map_server + amcl). Default.
#   slam:=true    NO map_server/amcl - use with `make slam` (slam_toolbox provides
#                 /map + the map->odom TF). This is the exploration/mapping setup.
#   hazard:=true  run hazard_costmap.py (cliff/wheel-drop/bumper -> costmap). Default.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")
    slam = LaunchConfiguration("slam")
    hazard = LaunchConfiguration("hazard")
    lm = {"autostart": True, "use_sim_time": False, "bond_timeout": 20.0, "attempt_respawn_reconnection": True}

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value="/cfg/nav2/nav2_params.yaml"),
        DeclareLaunchArgument("map", default_value="/cfg/maps/dreame_map.yaml"),
        DeclareLaunchArgument("slam", default_value="false"),
        DeclareLaunchArgument("hazard", default_value="true"),

        # --- localization: map_server + amcl (skipped when slam:=true) ---
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             output="screen", parameters=[params, {"yaml_filename": map_yaml}],
             condition=UnlessCondition(slam)),
        Node(package="nav2_amcl", executable="amcl", name="amcl",
             output="screen", parameters=[params], condition=UnlessCondition(slam)),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_localization", output="screen",
             parameters=[{**lm, "node_names": ["map_server", "amcl"]}],
             condition=UnlessCondition(slam)),

        # --- hazard sensors -> costmap obstacles (cliff / wheel-drop / bumper) ---
        ExecuteProcess(cmd=["python3", "/cfg/nav2/hazard_costmap.py"],
                       output="screen", condition=IfCondition(hazard)),

        # --- navigation: controller + smoother + planner + behaviors + bt ---
        Node(package="nav2_controller", executable="controller_server", name="controller_server",
             output="screen", parameters=[params]),
        Node(package="nav2_smoother", executable="smoother_server", name="smoother_server",
             output="screen", parameters=[params]),
        Node(package="nav2_planner", executable="planner_server", name="planner_server",
             output="screen", parameters=[params]),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server",
             output="screen", parameters=[params]),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator",
             output="screen", parameters=[params]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{**lm, "node_names": [
                 "controller_server", "smoother_server", "planner_server",
                 "behavior_server", "bt_navigator"]}]),
    ])
