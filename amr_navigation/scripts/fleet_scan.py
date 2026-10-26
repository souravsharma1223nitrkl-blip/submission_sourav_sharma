#!/usr/bin/env python3


import math

import numpy as np
from grid_tools import get_pose, run
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

ROBOTS = {'amr1': 0.22, 'amr2': 0.12}   # name: radius (m)
MARGIN = 0.05                           # m added around each robot
BEAMS = 1440                            # 0.25 deg apart
RANGE = 30.0                            # m, whole warehouse


class FleetScan(Node):
    def __init__(self):
        super().__init__('fleet_scan')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.local_plan, self.speed = {}, {}
        for r in ROBOTS:
            self.create_subscription(Path, f'/{r}/local_plan',
                                     lambda msg, r=r: self.local_plan.update({r: msg}), 10)
            self.create_subscription(Odometry, f'/{r}/odom', lambda msg, r=r: self.speed.update(
                {r: abs(msg.twist.twist.linear.x)}), 10)
        self.pubs = {r: self.create_publisher(LaserScan, f'/{r}/fleet_scan', 10) for r in ROBOTS}
        self.prediction_pubs = {r: self.create_publisher(LaserScan, f'/{r}/fleet_prediction', 10)
                                for r in ROBOTS}
        self.create_timer(0.1, self.publish_scans)

    def projected_path(self, robot):
        """Points (map frame) of the robot's local plan, if it is moving; else []."""
        plan = self.local_plan.get(robot)
        if plan is None or self.speed.get(robot, 0.0) < 0.05:
            return []
        try:
            ox, oy, oyaw = get_pose(self.tf_buffer, plan.header.frame_id)   # odom -> map
        except TransformException:
            return []
        c, s = math.cos(oyaw), math.sin(oyaw)
        return [(ox + c * p.pose.position.x - s * p.pose.position.y,
                 oy + s * p.pose.position.x + c * p.pose.position.y)
                for p in plan.poses[::3]] # converting from the robot local frame to the global map frame

    def publish_scans(self):
        try:
            body = {r: get_pose(self.tf_buffer, f'{r}/base_footprint') for r in ROBOTS}
            lidar = {r: get_pose(self.tf_buffer, f'{r}/base_scan') for r in ROBOTS}
        except TransformException:
            return
        for robot in ROBOTS:
            now, future = [], []        # circles: other robots now / along their trajectory
            for other, radius in ROBOTS.items():
                if other != robot:
                    now.append((body[other][0], body[other][1], radius + MARGIN))
                    future += [(x, y, radius) for x, y in self.projected_path(other)]
            self.pubs[robot].publish(self.make_scan(robot, lidar[robot], now))
            self.prediction_pubs[robot].publish(self.make_scan(robot, lidar[robot], now + future))

    def make_scan(self, robot, lidar_pose, circles):
        x, y, yaw = lidar_pose
        angles = yaw + np.linspace(-np.pi, np.pi, BEAMS, endpoint=False)
        ranges = np.full(BEAMS, np.inf)
        for cx, cy, r in circles:
            ranges = np.minimum(ranges, self.ray_circle(x, y, angles, cx, cy, r))
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = f'{robot}/base_scan'
        scan.angle_min, scan.angle_increment = -np.pi, 2 * np.pi / BEAMS
        scan.angle_max = scan.angle_min + (BEAMS - 1) * scan.angle_increment
        scan.range_min, scan.range_max = 0.05, RANGE
        scan.ranges = ranges.astype(np.float32).tolist()
        return scan

    @staticmethod
    def ray_circle(x, y, angles, cx, cy, r):
        """Distance along each ray from (x, y) to the circle (cx, cy, r); inf if it misses."""
        dx, dy = cx - x, cy - y
        along = dx * np.cos(angles) + dy * np.sin(angles)    # closest approach along the ray
        miss2 = dx * dx + dy * dy - along ** 2                # squared distance ray <-> centre
        hit = (along > 0) & (miss2 < r * r)
        dist = np.full(len(angles), np.inf)
        dist[hit] = along[hit] - np.sqrt(r * r - miss2[hit])
        dist[(dist <= 0) | (dist > RANGE)] = np.inf          # inside the circle / too far
        return dist


if __name__ == '__main__':
    run(FleetScan)
