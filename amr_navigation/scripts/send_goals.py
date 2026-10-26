#!/usr/bin/env python3


import math
import os
import sys

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def make_pose(x, y, yaw=0.0):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x, pose.pose.position.y = float(x), float(y)
    pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return pose


def read_goals(args, config):
    """Command line -> {robot: (label, [poses])}."""
    if args == ['--tour']:
        return {robot: ('mapping tour', [make_pose(x, y) for x, y in points])
                for robot, points in config['tours'].items()}
    goals = {}
    for arg in args:                               # e.g. amr1=heavy_storage or amr1=2.0,-3.5
        robot, target = arg.split('=')
        if target in config['zones']:
            x, y, yaw = config['zones'][target]
        else:
            x, y = (float(v) for v in target.split(','))
            yaw = 0.0
        goals[robot] = (target, [make_pose(x, y, yaw)])
    return goals


def main():
    sys.stdout.reconfigure(line_buffering=True)    # show every line at once, also in a log file
    rclpy.init()
    node = Node('send_goals', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', value=True)])
    config_file = os.path.join(get_package_share_directory('amr_navigation'),
                               'config', 'zones.yaml')
    with open(config_file, 'r') as f:
        goals = read_goals(rclpy.utilities.remove_ros_args(sys.argv)[1:], yaml.safe_load(f))
    if not goals:
        print(__doc__)
        return

    def sim_time():
        return node.get_clock().now().nanoseconds * 1e-9

    print('waiting for the simulation clock and Nav2 ...')
    while sim_time() == 0.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    # 1. send all goals at once
    handles = {}                                   # robot -> accepted goal
    results = {}                                   # robot -> future of the result
    done = {}                                      # robot -> result text
    progress = {}                                  # robot -> latest feedback from Nav2
    for robot, (label, poses) in goals.items():
        if len(poses) > 1:
            client = ActionClient(node, FollowWaypoints, f'/{robot}/follow_waypoints')
            goal = FollowWaypoints.Goal(poses=poses)
        else:
            client = ActionClient(node, NavigateToPose, f'/{robot}/navigate_to_pose')
            goal = NavigateToPose.Goal(pose=poses[0])
        while not client.wait_for_server(timeout_sec=5.0):
            print(f'  still waiting for the Nav2 of {robot} ...')
        handle = client.send_goal_async(
            goal, feedback_callback=lambda msg, r=robot: progress.update({r: msg.feedback}))
        rclpy.spin_until_future_complete(node, handle)
        if not handle.result().accepted:
            done[robot] = 'goal rejected'
            continue
        handles[robot] = handle.result()
        results[robot] = handles[robot].get_result_async()
        print(f'{robot} -> {label}')
    start = sim_time()

    # 2. wait until every robot is done (and show progress every 10 s)
    last_print = sim_time()
    while len(done) < len(goals):
        rclpy.spin_once(node, timeout_sec=0.5)
        if sim_time() - last_print > 10.0:
            last_print = sim_time()
            for robot in results:                  # ask again (a reply can get lost under load)
                if robot not in done and not results[robot].done():
                    results[robot] = handles[robot].get_result_async()
            print(' | '.join(
                f'{r}: waypoint {fb.current_waypoint + 1}' if hasattr(fb, 'current_waypoint')
                else f'{r}: {fb.distance_remaining:.1f} m to go'
                for r, fb in progress.items() if r not in done))
        for robot, future in results.items():
            if future.done() and robot not in done:
                ok = future.result().status == GoalStatus.STATUS_SUCCEEDED
                done[robot] = f"{'reached' if ok else 'FAILED'} after {sim_time() - start:.0f} s"
                print(f'{robot}: {done[robot]}')

    print('\nRESULTS')
    for robot, (label, _) in goals.items():
        print(f'  {robot}  {label:15s} {done[robot]}')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass       # Ctrl+C stops this script only; Nav2 keeps driving to the goals
