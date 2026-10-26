#!/usr/bin/env python3


import numpy as np
import rclpy
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


def min_distance(a, b):
    """Smallest distance between any point of a and any point of b (Nx2 arrays)."""
    if len(a) == 0 or len(b) == 0:
        return np.inf
    return np.min(np.hypot(a[:, None, 0] - b[None, :, 0], a[:, None, 1] - b[None, :, 1]))


class TrafficControl(Node):
    def __init__(self):
        super().__init__('traffic_control')
        self.first = self.declare_parameter('priority_robot', 'amr1').value
        self.second = self.declare_parameter('yielding_robot', 'amr2').value
        self.look_ahead = self.declare_parameter('look_ahead', 2.0).value
        self.conflict_distance = self.declare_parameter('conflict_distance', 0.7).value
        self.release_after = self.declare_parameter('release_after', 2.0).value

        self.plans = {}                  # robot -> (time received, Nx2 array of path points)
        self.holding = False
        self.last_conflict = -1e9
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        for robot in (self.first, self.second):
            self.create_subscription(Path, f'/{robot}/plan',
                                     lambda msg, r=robot: self.on_plan(r, msg), 10)
        self.hold_pub = self.create_publisher(Bool, f'/{self.second}/hold', 10)
        self.create_timer(0.2, self.check)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_plan(self, robot, msg):
        points = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
        self.plans[robot] = (self.now(), points)

    def position_and_path(self, robot):
        """Where the robot is now, and the next look_ahead metres of its plan (Nx2 arrays)."""
        t = self.tf_buffer.lookup_transform('map', f'{robot}/base_footprint', Time())
        pos = np.array([[t.transform.translation.x, t.transform.translation.y]])
        received, points = self.plans.get(robot, (-1e9, np.zeros((0, 2))))
        if self.now() - received > 3.0 or len(points) == 0:   # not navigating right now
            return pos, np.zeros((0, 2))
        start = np.argmin(np.hypot(*(points - pos).T))      # closest plan point
        steps = np.hypot(*np.diff(points[start:], axis=0).T)
        end = start + np.searchsorted(np.cumsum(steps), self.look_ahead) + 1
        return pos, points[start:end + 1]

    def check(self):
        try:
            pos1, path1 = self.position_and_path(self.first)
            pos2, path2 = self.position_and_path(self.second)
        except TransformException:
            return
        gap = min_distance(np.vstack([pos1, path1]), np.vstack([pos2, path2]))
        conflict = bool(len(path1) > 0 and len(path2) > 0 and gap < self.conflict_distance)
        if conflict:
            self.last_conflict = self.now()

        hold = conflict or (self.holding and self.now() - self.last_conflict < self.release_after)
        if hold != self.holding:
            self.get_logger().info(
                f'{self.second} YIELDS to {self.first} (paths {gap:.2f} m apart): controlled stop'
                if hold else f'conflict over: {self.second} drives on')
        self.holding = hold
        self.hold_pub.publish(Bool(data=hold))


def main():
    rclpy.init()
    node = TrafficControl()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
