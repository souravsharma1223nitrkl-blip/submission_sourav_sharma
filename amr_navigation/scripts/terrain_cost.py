#!/usr/bin/env python3


import numpy as np
import yaml
from grid_tools import LATCHED, UNKNOWN, Grid, run
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node


class TerrainCost(Node):
    def __init__(self):
        super().__init__('terrain_cost')
        self.grid = Grid(self)
        terrain_file = self.declare_parameter('terrain_file', '').value
        flat = self.declare_parameter('flat_grade', 0.02).value     # 2 %
        steep = self.declare_parameter('max_grade', 0.10).value     # 10 %
        low = self.declare_parameter('min_cost', 70).value
        high = self.declare_parameter('max_cost', 95).value

        height = self.floor_height(terrain_file)
        dh_dy, dh_dx = np.gradient(height, self.grid.res)
        slope = np.hypot(dh_dx, dh_dy)

        cost = self.grid.empty(UNKNOWN)
        ramp = (slope > flat) & (slope <= steep)
        cost[ramp] = np.round(low + (high - low) * (slope[ramp] - flat) / (steep - flat))
        cost[slope > steep] = 100

        self.pub = self.create_publisher(OccupancyGrid, '/terrain_cost', LATCHED)
        self.pub.publish(self.grid.to_msg(cost, self.get_clock().now().to_msg()))
        self.get_logger().info(f'terrain cost: {ramp.sum()} ramp cells, '
                               f'{(slope > steep).sum()} not drivable')

    def floor_height(self, path):
        g = self.grid
        height = np.zeros((g.height, g.width))
        with open(path, 'r') as f:
            plan = yaml.safe_load(f)
        for p in plan['platforms']:                      # raised flat area
            x0, y0, x1, y1 = p['area']
            height[(g.x >= x0) & (g.x <= x1) & (g.y >= y0) & (g.y <= y1)] = p['height']
        for r in plan['ramps']:                          # height rises from `from` to `to`
            (fx, fy), (tx, ty) = r['from'], r['to']
            length = np.hypot(tx - fx, ty - fy)
            ux, uy = (tx - fx) / length, (ty - fy) / length          # direction up the ramp
            along = ((g.x - fx) * ux + (g.y - fy) * uy) / length     # 0 at the foot, 1 at top
            across = abs(-(g.x - fx) * uy + (g.y - fy) * ux)         # distance from centre line
            on_ramp = (along >= 0) & (along <= 1) & (across <= r['width'] / 2)
            h0, h1 = r['heights']
            height[on_ramp] = h0 + along[on_ramp] * (h1 - h0)
        return height


if __name__ == '__main__':
    run(TerrainCost)
