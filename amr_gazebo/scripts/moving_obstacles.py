#!/usr/bin/env python3


import math

import rclpy
import yaml
from gazebo_msgs.srv import SetEntityState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

RATE = 10.0               # Hz
FLOOR_Z = 0.0342          # floor height of the AWS warehouse
ROBOTS = ['amr1', 'amr2']


class MovingObstacle:
    """One pedestrian or cart walking back and forth along its waypoints."""

    def __init__(self, config):
        self.name = config['name']
        self.waypoints = config['waypoints']
        self.speed = config['speed']
        self.pause = config['pause']
        self.x, self.y = self.waypoints[0]       # start where the world file puts it
        self.yaw = 0.0
        self.index = 1                           # waypoint we are walking to
        self.step = 1                            # +1 forward along the list, -1 back
        self.wait = 0.0

    def turn_around(self):
        self.step = -self.step
        self.index += self.step                  # back to the waypoint we came from

    def update(self, dt, robots, stop_distance):
        if self.wait > 0.0:                      # pausing at a waypoint
            self.wait -= dt
            return
        tx, ty = self.waypoints[self.index]
        dx, dy = tx - self.x, ty - self.y
        distance = math.hypot(dx, dy)
        if distance < 0.01:                      # arrived: pause, then the next waypoint
            self.wait = self.pause
            if not 0 <= self.index + self.step < len(self.waypoints):
                self.step = -self.step           # end of the list: walk back
            self.index += self.step
            return
        step = min(self.speed * dt, distance)
        new_x, new_y = self.x + step * dx / distance, self.y + step * dy / distance
        for rx, ry in robots:                    # a robot ahead is too close: turn around
            gap_now = math.hypot(self.x - rx, self.y - ry)
            gap_next = math.hypot(new_x - rx, new_y - ry)
            if gap_next < stop_distance and gap_next < gap_now:
                self.turn_around()
                return
        self.x, self.y, self.yaw = new_x, new_y, math.atan2(dy, dx)


class MovingObstaclesNode(Node):
    def __init__(self):
        super().__init__('moving_obstacles')
        config_file = self.declare_parameter('config_file', '').value
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
        self.stop_distance = config['stop_distance']
        self.obstacles = [MovingObstacle(c) for c in config['obstacles']]

        # robot positions come from TF (world -> amrX/odom -> amrX/base_footprint)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.set_state = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.create_timer(1.0 / RATE, self.on_timer)

    def robot_positions(self):
        positions = []
        for robot in ROBOTS:
            try:
                t = self.tf_buffer.lookup_transform('world', f'{robot}/base_footprint', Time())
                positions.append((t.transform.translation.x, t.transform.translation.y))
            except TransformException:
                pass                             # robot not spawned yet
        return positions

    def on_timer(self):
        if not self.set_state.service_is_ready():
            return
        robots = self.robot_positions()
        for obstacle in self.obstacles:
            obstacle.update(1.0 / RATE, robots, self.stop_distance)
            request = SetEntityState.Request()
            request.state.name = obstacle.name
            request.state.reference_frame = 'world'
            request.state.pose.position.x = obstacle.x
            request.state.pose.position.y = obstacle.y
            request.state.pose.position.z = FLOOR_Z
            request.state.pose.orientation.z = math.sin(obstacle.yaw / 2.0)
            request.state.pose.orientation.w = math.cos(obstacle.yaw / 2.0)
            self.set_state.call_async(request)   # fire and forget


def main():
    rclpy.init()
    node = MovingObstaclesNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
