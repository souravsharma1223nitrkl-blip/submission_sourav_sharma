#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

RATE = 20.0          # Hz
DT = 1.0 / RATE


def clamp(value, low, high):
    return max(low, min(high, value))


class MotionSmoother(Node):
    def __init__(self):
        super().__init__('motion_smoother')
        for name, default in [('max_speed', 0.26), ('max_accel', 0.3), ('max_decel', 0.5),
                              ('max_jerk', 1.0), ('max_angular_accel', 1.0),
                              ('payload_factor', 0.5), ('carrying_payload', False)]:
            self.declare_parameter(name, default)

        self.target = Twist()        # what Nav2 wants to publish as the speed 
        self.target_time = 0.0
        self.hold = False            # traffic control says: stop and wait safety failsafe mechanism
        self.v = 0.0                 # what we send now (linear, angular)
        self.w = 0.0
        self.a = 0.0                 # current linear acceleration

        self.pub = self.create_publisher(Twist, 'cmd_vel_smooth', 10)
        self.create_subscription(Twist, 'cmd_vel_nav', self.on_cmd, 10)
        self.create_subscription(Bool, 'hold', lambda msg: setattr(self, 'hold', msg.data), 10)
        self.create_subscription(Bool, 'safety_stop', self.on_safety_stop, 10)
        self.create_timer(DT, self.step)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, msg):
        self.target, self.target_time = msg, self.now()

    def on_safety_stop(self, msg):
        """The safety node halted the robot: start again from standstill (smoothly)."""
        if msg.data:
            self.v, self.a = 0.0, 0.0

    def p(self, name):
        return self.get_parameter(name).value

    def step(self):
        # target speed: Nav2's command, or 0 if holding / Nav2 went quiet
        if self.hold or self.now() - self.target_time > 0.5:
            v_goal, w_goal = 0.0, 0.0 # adding this in case of a navigation failure 
        else:
            v_goal, w_goal = self.target.linear.x, self.target.angular.z

        # limits for our current state
        f = self.p('payload_factor') if self.p('carrying_payload') else 1.0
        accel = f * self.p('max_accel') * (1.0 - 0.5 * min(abs(self.v) / self.p('max_speed'), 1.0))
        decel = f * self.p('max_decel')
        jerk = f * self.p('max_jerk')
        speeding_up = abs(v_goal) > abs(self.v) and v_goal * self.v >= 0.0
        limit = accel if speeding_up else decel

        # (sqrt(2 * jerk * dv))
        dv = v_goal - self.v
        a_wanted = math.copysign(min(limit, math.sqrt(2.0 * jerk * abs(dv))), dv)
        # change the acceleration by at most jerk * dt per step (= jerk limit)
        self.a = clamp(a_wanted, self.a - jerk * DT, self.a + jerk * DT)
        self.v += self.a * DT
        if abs(v_goal - self.v) < 0.002 and abs(self.a) <= jerk * DT:   # arrived
            self.v, self.a = v_goal, 0.0

        # 4. angular: acceleration limit only
        dw = f * self.p('max_angular_accel') * DT
        self.w = clamp(w_goal, self.w - dw, self.w + dw)

        out = Twist()
        out.linear.x, out.angular.z = self.v, self.w
        self.pub.publish(out)


def main():
    rclpy.init()
    node = MotionSmoother()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()