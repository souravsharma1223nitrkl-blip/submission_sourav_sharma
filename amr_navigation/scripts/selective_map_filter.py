#!/usr/bin/env python3

import numpy as np
from grid_tools import LATCHED, UNKNOWN, Grid, get_pose, run
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class SelectiveMapFilter(Node):
    def __init__(self):
        super().__init__('selective_map_filter')
        self.grid = Grid(self)
        self.n = round(self.declare_parameter('tile_size', 1.0).value / self.grid.res)
        self.base_period = self.declare_parameter('base_period', 4.0).value   # s
        self.max_period = self.declare_parameter('max_period', 64.0).value    # s
        tiles = (self.grid.height // self.n, self.grid.width // self.n)

        self.sent = self.grid.empty()                # what the merged map has from AMR-1
        self.last_sent = np.full(tiles, -1e9)        # time each tile was last sent
        self.visits = np.zeros(tiles, dtype=int)     # how often AMR-1 drove into each tile
        self.robot_tile = None
        self.stats = {'explore': 0, 'refresh': 0, 'postponed': 0}   # tile updates
        # seconds between two refreshes of a tile, by how often AMR-1 drove through it
        self.gaps = {'1x': [], '2x': [], '3x+': []}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(OccupancyGrid, '/amr1/map_selective', LATCHED)
        self.create_subscription(OccupancyGrid, '/amr1/map', self.on_map, LATCHED)
        self.create_timer(0.5, self.count_visits)
        self.create_timer(10.0, self.print_stats)

    def tiles(self, cells):
        """Cell mask -> tile mask (a tile is True if any of its cells is True)."""
        n = self.n
        return cells.reshape(self.grid.height // n, n, self.grid.width // n, n).any(axis=(1, 3))

    def cells(self, tiles):
        """Tile mask -> cell mask."""
        return tiles.repeat(self.n, axis=0).repeat(self.n, axis=1)

    def count_visits(self):
        try:
            x, y, _ = get_pose(self.tf_buffer, 'amr1/base_footprint')
        except TransformException:
            return
        cell = self.grid.cell(x, y)
        if cell is None:
            return
        tile = (cell[0] // self.n, cell[1] // self.n)
        if tile != self.robot_tile:            # AMR-1 just drove into this tile
            self.robot_tile = tile
            self.visits[tile] += 1

    def on_map(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        new_map = self.grid.from_msg(msg)

        changed = (new_map != UNKNOWN) & (new_map != self.sent)
        unexplored = changed & (self.sent == UNKNOWN)

        changed_tiles = self.tiles(changed)
        explore_tiles = self.tiles(unexplored)
        period = np.minimum(self.base_period * 2.0 ** np.maximum(self.visits - 1, 0),
                            self.max_period)
        refresh_tiles = changed_tiles & ~explore_tiles & (now - self.last_sent >= period)
        send_tiles = explore_tiles | refresh_tiles

        self.stats['explore'] += int(explore_tiles.sum())
        self.stats['refresh'] += int(refresh_tiles.sum())
        waiting = changed_tiles & ~send_tiles      # changed, but postponed
        self.stats['postponed'] += int(waiting.sum())
        repeat = refresh_tiles & (self.last_sent > 0)    # tiles sent before: measure the gap
        for name, tiles in (('1x', self.visits <= 1), ('2x', self.visits == 2),
                            ('3x+', self.visits >= 3)):
            self.gaps[name] += (now - self.last_sent)[repeat & tiles].tolist()
        if not send_tiles.any():
            return
        update = self.cells(send_tiles) & changed
        self.sent[update] = new_map[update]
        self.last_sent[send_tiles] = now
        self.pub.publish(self.grid.to_msg(self.sent, msg.header.stamp))

    def print_stats(self):
        s = self.stats
        self.get_logger().info(
            f"tile updates: {s['explore']} sent at once (unexplored), {s['refresh']} refreshes "
            f"sent, {s['postponed']} postponed | {int((self.visits > 0).sum())} tiles visited, "
            f'max {self.visits.max()} visits')
        self.get_logger().info('typical (median) time between refreshes of a tile driven '
                               'through ' + ', '.join(f'{name}: {np.median(gaps):.0f} s'
                                                      for name, gaps in self.gaps.items() if gaps))


if __name__ == '__main__':
    run(SelectiveMapFilter)
