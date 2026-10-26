#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan


class SensorValidator(Node):
    def __init__(self):
        super().__init__('sensor_validator')
        self.max_rate = self.declare_parameter('max_angular_velocity', 2.0).value   # rad/s
        self.beams = self.declare_parameter('scan_beams', 360).value
        self.count = {'scan ok': 0, 'scan dropped': 0, 'imu ok': 0, 'imu dropped': 0}
        self.last_warning = {}       # sensor -> time of the last warning (max 1 per second)

        self.scan_pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, 'imu', qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan_raw', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, 'imu_raw', self.on_imu, qos_profile_sensor_data)
        self.create_timer(30.0, self.report)

    def on_scan(self, msg):
        if len(msg.ranges) != self.beams:
            return self.reject('scan', f'{len(msg.ranges)} beams instead of {self.beams}')
        # NaN fails both comparisons, so it becomes inf too
        msg.ranges = [r if msg.range_min <= r <= msg.range_max else math.inf for r in msg.ranges]
        self.count['scan ok'] += 1
        self.scan_pub.publish(msg)

    def on_imu(self, msg):
        w = msg.angular_velocity
        rate = math.sqrt(w.x ** 2 + w.y ** 2 + w.z ** 2)
        if not math.isfinite(rate) or rate > self.max_rate:
            return self.reject('imu', f'angular velocity {rate:.2f} rad/s is above the plausible '
                                      f'{self.max_rate:.2f} rad/s for this robot')
        self.count['imu ok'] += 1
        self.imu_pub.publish(msg)

    def reject(self, sensor, reason):
        self.count[f'{sensor} dropped'] += 1
        if time.monotonic() - self.last_warning.get(sensor, 0.0) > 1.0:
            self.last_warning[sensor] = time.monotonic()
            self.get_logger().warn(f'{sensor.upper()} REJECTED: {reason}')

    def report(self):
        self.get_logger().info(', '.join(f'{k}: {v}' for k, v in self.count.items()))


def main():
    rclpy.init()
    node = SensorValidator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
