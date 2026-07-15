#!/usr/bin/env python3
"""hazard_costmap - turn the vacuum's contact/drop sensors into Nav2 costmap obstacles.

Nav2 has no concept of a "hole in the floor" or a "bump felt but not seen". This
node converts ros2dreame's three MCU hazard sensor groups into PointCloud2 clouds
of virtual obstacles (a point at each fired sensor's position on the ground), which
the costmaps mark as LETHAL so the planner routes around them:

  /cliff/flags       (UInt8, 6 bits)  -> /cliff/obstacles       drop/ledge sensors
  /wheel_drop/flags  (UInt8, 2 bits)  -> /wheel_drop/obstacles  drive-wheel drop (edge)
  /bumper/flags      (UInt8, 2 bits)  -> /bumper/obstacles       front contact bumper

Wiring (nav2_params.yaml): cliff + wheel_drop feed a persistent drop layer (local +
global, clearing:False - a seen ledge stays avoided). bumper feeds a transient
contact layer (local only - the rolling window expires it as the robot moves off).

This is the *planning*-level guard. The *immediate* stop is already on the robot:
ros2dreame's MCU hazard gate zeroes linear velocity the instant a cliff/bump fires
(src/direct.rs). This node only makes Nav2 remember and avoid.

All offsets are in base_link (x forward, y left), APPROXIMATE (the W10 is ~0.35 m
across, robot_radius 0.18) - measure and override via the *_offsets_x/_y params.
Bit orders match ros2dreame: cliff bit0=front_left,1=mid_left,2=mid_right,
3=front_right,4=rear_left,5=rear_right; bumper/wheel_drop bit0=left,1=right.
"""
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import UInt8
from sensor_msgs.msg import PointCloud2, PointField

# name -> (flags_topic, cloud_topic, default [(x,y)] offsets per bit)
HAZARDS = {
    "cliff": ("/cliff/flags", "/cliff/obstacles", [
        (0.15, 0.08), (0.10, 0.14), (0.10, -0.14),
        (0.15, -0.08), (-0.15, 0.08), (-0.15, -0.08),
    ]),
    "wheel_drop": ("/wheel_drop/flags", "/wheel_drop/obstacles", [
        (0.0, 0.15), (0.0, -0.15),
    ]),
    # Bumper: the LDS missed this obstacle (below the scan plane / transparent). The
    # W10's front bumper is FLAT with rounded corners, so mark a flat line across the
    # front (corners pulled back slightly) - not a forward-bulging arc - on ANY bumper
    # bit, so the whole front reads blocked (not 2 points the planner threads between).
    # The immediate reaction is the ros2dreame bump-escape reflex (back off + turn);
    # this mark just stops Nav2 re-planning straight back into the same spot.
    "bumper": ("/bumper/flags", "/bumper/obstacles", [
        (0.17, 0.0), (0.17, 0.07), (0.17, -0.07), (0.155, 0.14), (0.155, -0.14),
    ]),
}

# Hazards marked as the whole cluster on any bit set (vs one point per bit) - a bumper
# hit means "a wall across my front", so mark the arc, not just the two contact points.
ARC_ON_ANY = {"bumper"}


class HazardCostmap(Node):
    def __init__(self):
        super().__init__("hazard_costmap")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("point_z", 0.10)      # obstacle height in base_link [m]
        self.declare_parameter("publish_rate", 5.0)  # Hz - keep marks fresh for the costmap
        self.frame_id = self.get_parameter("frame_id").value
        self.point_z = float(self.get_parameter("point_z").value)

        self.sources = []  # (mask_getter_index, offsets, publisher)
        self.masks = {}
        for name, (flags_topic, cloud_topic, offs) in HAZARDS.items():
            ox = list(self.declare_parameter(f"{name}_offsets_x", [x for x, _ in offs]).value)
            oy = list(self.declare_parameter(f"{name}_offsets_y", [y for _, y in offs]).value)
            if len(ox) != len(offs) or len(oy) != len(offs):
                self.get_logger().warn(f"{name}: offsets must be length {len(offs)}; using defaults")
                ox = [x for x, _ in offs]
                oy = [y for _, y in offs]
            offsets = list(zip(ox, oy))
            pub = self.create_publisher(PointCloud2, cloud_topic, qos_profile_sensor_data)
            self.masks[name] = 0
            self.create_subscription(
                UInt8, flags_topic,
                lambda m, n=name: self.masks.__setitem__(n, m.data),
                qos_profile_sensor_data)
            self.sources.append((name, offsets, pub))

        rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1.0 / rate if rate > 0 else 0.2, self._tick)
        self.get_logger().info(
            f"hazard_costmap: {'/'.join(HAZARDS)} flags -> obstacle clouds "
            f"(frame {self.frame_id}, z {self.point_z} m)")

    def _tick(self):
        for name, offsets, pub in self.sources:
            mask = self.masks[name]
            if name in ARC_ON_ANY:
                points = [(x, y, self.point_z) for x, y in offsets] if mask else []
            else:
                points = [
                    (offsets[i][0], offsets[i][1], self.point_z)
                    for i in range(len(offsets))
                    if mask & (1 << i)
                ]
            pub.publish(self._cloud(points))

    def _cloud(self, points):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(points)
        msg.is_dense = True
        msg.data = b"".join(struct.pack("<fff", x, y, z) for x, y, z in points)
        return msg


def main():
    rclpy.init()
    node = HazardCostmap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
