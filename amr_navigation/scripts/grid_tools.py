import array
import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time

UNKNOWN, FREE, OCCUPIED = -1, 0, 100

# "Latched" topic: a node that subscribes later still gets the last map
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Grid:
    def __init__(self, node):
        self.res = node.declare_parameter('grid.resolution', 0.05).value
        self.x0 = node.declare_parameter('grid.origin_x', -7.5).value
        self.y0 = node.declare_parameter('grid.origin_y', -11.0).value
        self.width = node.declare_parameter('grid.width', 300).value     # cells along x
        self.height = node.declare_parameter('grid.height', 440).value   # cells along y
        # world x  y of every cell centre 
        xs = self.x0 + (np.arange(self.width) + 0.5) * self.res
        ys = self.y0 + (np.arange(self.height) + 0.5) * self.res
        self.x, self.y = np.meshgrid(xs, ys)

    def empty(self, value=UNKNOWN):
        return np.full((self.height, self.width), value, dtype=np.int8)

    def cell(self, x, y):
        """(row, col) of a world point, or None if it is outside the grid."""
        row, col = int((y - self.y0) / self.res), int((x - self.x0) / self.res)
        if 0 <= row < self.height and 0 <= col < self.width:
            return row, col
        return None

    def from_msg(self, msg):
        """Copy a map message from slam_toolbox into the grid"""
        src = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        dr = round((msg.info.origin.position.y - self.y0) / self.res)   # row offset
        dc = round((msg.info.origin.position.x - self.x0) / self.res)   # column offset
        out = self.empty()
        r0, r1 = max(dr, 0), min(dr + src.shape[0], self.height)       # overlapping part
        c0, c1 = max(dc, 0), min(dc + src.shape[1], self.width)
        if r0 < r1 and c0 < c1:
            out[r0:r1, c0:c1] = src[r0 - dr:r1 - dr, c0 - dc:c1 - dc]
        return out

    def to_msg(self, grid, stamp):
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = stamp
        msg.info.resolution = float(self.res)
        msg.info.width, msg.info.height = self.width, self.height
        msg.info.origin.position.x, msg.info.origin.position.y = float(self.x0), float(self.y0)
        msg.info.origin.orientation.w = 1.0
        msg.data = array.array('b', grid.astype(np.int8).tobytes()) #8 bit signed char to match ros occupancy grid, was causing errors because of this
        return msg


def get_pose(tf_buffer, frame):
    """(x, y, yaw) of a TF frame in the 'map' frame. Raises TransformException if unknown."""
    t = tf_buffer.lookup_transform('map', frame, Time())
    q = t.transform.rotation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return t.transform.translation.x, t.transform.translation.y, yaw


def run(node_class):
    """Start ROS, run one node until Ctrl+C, shut down quietly."""
    rclpy.init()
    node = node_class()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
