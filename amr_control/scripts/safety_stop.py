#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


def scan_points(scan):
    r = np.array(scan.ranges)
    a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
    ok = np.isfinite(r) & (r > scan.range_min)
    return r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])


class SafetyStop(Node):
    def __init__(self):
        super().__init__('safety_stop')
        self.k = self.declare_parameter('k', 2.0).value
        self.d_min = self.declare_parameter('d_min', 0.3).value
        self.half_width = self.declare_parameter('half_width', 0.15).value
        self.body_front = self.declare_parameter('body_front', 0.14).value   # LiDAR to front edge distance
        self.body_back = self.declare_parameter('body_back', 0.14).value     # LiDAR -> back edge
        self.scans = {}              # 'scan' / 'fleet_scan' -> latest LaserScan
        self.speed = 0.0             # measured speed (odometry)
        self.stopped = False

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.stop_pub = self.create_publisher(Bool, 'safety_stop', 10)   # tells the smoother
        self.create_subscription(Twist, 'cmd_vel_smooth', self.on_cmd, 10)
        self.create_subscription(Odometry, 'odom', lambda msg: setattr(
            self, 'speed', msg.twist.twist.linear.x), 10)
        for topic in ('scan', 'fleet_scan'):
            self.create_subscription(LaserScan, topic, lambda msg, t=topic: self.scans.update(
                {t: msg}), qos_profile_sensor_data)

    def nearest_in_lane(self, forward):
        """Gap between the robot's front or back) edge and the nearest obstacle in its lane."""
        nearest = math.inf
        edge = self.body_front if forward else self.body_back
        for scan in self.scans.values():
            x, y = scan_points(scan)
            ahead = (x if forward else -x) - edge           # distance from the robot's edge
            # points closer than 2 cm to the edge are the robot itself its LiDAR sometimes sees its own front plate when the body pitches); points inside the body are skipped too
            in_lane = (ahead > 0.02) & (np.abs(y) < self.half_width)
            if in_lane.any():
                nearest = min(nearest, float(ahead[in_lane].min()))
        return nearest

    def on_cmd(self, cmd):
        v = max(abs(self.speed), abs(cmd.linear.x))      # be safe: the higher of the two
        d_safe = self.k * v * v + self.d_min
        moving = abs(cmd.linear.x) > 1e-3
        distance = self.nearest_in_lane(forward=cmd.linear.x >= 0.0) if moving else math.inf

        if self.stopped:
            d_safe += 0.05                  # drive on only once clearly clear (no on/off flicker)
        if distance < d_safe:                              # OVERRIDE: halt now
            cmd = Twist(angular=cmd.angular)
            if not self.stopped:
                self.get_logger().warn(f'SAFETY STOP: obstacle at {distance:.2f} m '
                                       f'< d_safe {d_safe:.2f} m')
            self.stopped = True
            self.stop_pub.publish(Bool(data=True))
        elif self.stopped and moving:
            self.get_logger().info('safety stop released')
            self.stopped = False
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = SafetyStop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
