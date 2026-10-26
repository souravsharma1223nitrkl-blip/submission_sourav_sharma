#!/usr/bin/env python3

import numpy as np
from grid_tools import FREE, LATCHED, OCCUPIED, UNKNOWN, Grid, get_pose, run
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

INPUTS = ['/amr1/map_selective', '/amr2/map']
ROBOTS = ['amr1', 'amr2']


class MapFusion(Node):
    def __init__(self):
        super().__init__('map_fusion')
        self.grid = Grid(self)
        self.clear_radius = self.declare_parameter('robot_clear_radius', 0.30).value   # m
        self.maps = {}                             # topic -> latest map (numpy grid)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        for topic in INPUTS:
            self.create_subscription(OccupancyGrid, topic,
                                     lambda msg, t=topic: self.store(t, msg), LATCHED)
        self.pub = self.create_publisher(OccupancyGrid, '/map', LATCHED)
        self.create_timer(1.0, self.merge)

    def store(self, topic, msg):
        self.maps[topic] = self.grid.from_msg(msg)

    def merge(self):
        if not self.maps:
            return
        merged = self.grid.empty(UNKNOWN)
        for m in self.maps.values():
            merged = np.maximum(merged, m)       # highest value wins, cell by cell

        for robot in ROBOTS:                     # the robots are not obstacles
            try:
                x, y, _ = get_pose(self.tf_buffer, f'{robot}/base_footprint')
            except TransformException:
                continue
            near = (self.grid.x - x) ** 2 + (self.grid.y - y) ** 2 < self.clear_radius ** 2
            merged[near & (merged == OCCUPIED)] = FREE

        self.pub.publish(self.grid.to_msg(merged, self.get_clock().now().to_msg()))


if __name__ == '__main__':
    run(MapFusion)
