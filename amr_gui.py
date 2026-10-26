#!/usr/bin/env python3
"""
The workspace is ~/amr_ws by default (change it in the window, or AMR_WS=/path python3 amr_gui.py).

Maintainer: Sourav <souravsharma1321@gmail.com>
"""

import importlib.util
import math
import os
import queue
import shlex
import signal
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import ttk

import yaml

ROS_SETUP = '/opt/ros/humble/setup.bash'
WORKSPACE = os.path.expanduser(os.environ.get('AMR_WS', '~/amr_ws'))

# Started from a terminal without ROS sourced? Restart once with ROS + the workspace sourced.
if 'AMR_GUI_SOURCED' not in os.environ and importlib.util.find_spec('rclpy') is None:
    os.environ['AMR_GUI_SOURCED'] = '1'
    setup = os.path.join(WORKSPACE, 'install', 'setup.bash')
    os.execvp('bash', ['bash', '-c', f'source {ROS_SETUP}; source {shlex.quote(setup)}; '
                       'exec python3 "$0" "$@"', os.path.abspath(__file__)] + sys.argv[1:])

try:                                    # live view (map, robots, paths) needs ROS
    import numpy as np
    import rclpy
    from nav_msgs.msg import OccupancyGrid, Odometry, Path
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import Bool
    ROS_OK = True
except ImportError:
    ROS_OK = False

TITLE = 'Sourav Kumar Sharma - AMR Warehouse Project'
SUBTITLE = ('Adaptive Navigation and Conflict-Aware Path Planning (Logistics)  ·  '
            'ROS 2 Humble · Gazebo Classic · slam_toolbox · Nav2')

ROBOTS = {
    'amr1': {'label': 'AMR-1  heavy lead', 'color': '#d9480f', 'start': (-3.70, -9.40),
             'info': 'TurtleBot3 Waffle Pi · 8 m LiDAR · max 0.26 m/s\n'
                     'priority robot · gentle acceleration · loop closure on'},
    'amr2': {'label': 'AMR-2  scout', 'color': '#1971c2', 'start': (1.70, 9.20),
             'info': 'TurtleBot3 Burger · 3.5 m LiDAR · max 0.35 m/s\n'
                     'light and quick · yields to AMR-1 at crossings'},
}
DEFAULT_GOALS = {'amr1': 'heavy_storage', 'amr2': 'packing_bay_4'}
STAY = '(stay)'

LAUNCH_OPTIONS = {
    'Full system (Sections 1-4)': 'ros2 launch amr_control fleet.launch.py',
    'Mapping + navigation (Section 2)': 'ros2 launch amr_navigation navigation.launch.py',
    'Simulation only (Section 1)': 'ros2 launch amr_gazebo sim.launch.py',
}
# simulation log lines shown when "show all simulation output" is off
KEY_WORDS = ('WARN', 'ERROR', 'SAFETY', 'YIELDS', 'conflict over', 'REJECTED',
             'Managed nodes are active', 'process has died')
IGNORE = ('tick rate',)                 # harmless timing warnings on a slow PC

# Warehouse area drawn on the map (metres, map frame) and drawing scale
X_MIN, X_MAX, Y_MIN, Y_MAX = -6.5, 6.5, -10.5, 10.5
SCALE = 26                              # pixels per metre
W, H = int((X_MAX - X_MIN) * SCALE), int((Y_MAX - Y_MIN) * SCALE)


def to_px(x, y):
    return (x - X_MIN) * SCALE, (Y_MAX - y) * SCALE


def map_image(msg):
    """OccupancyGrid -> PPM image of the drawn area (white free, black wall, grey unknown)."""
    info = msg.info
    grid = np.array(msg.data, dtype=np.int16).reshape(info.height, info.width)
    xs = X_MIN + (np.arange(W) + 0.5) / SCALE            # world x of every image column
    ys = Y_MAX - (np.arange(H) + 0.5) / SCALE            # world y of every image row
    cols = np.floor((xs - info.origin.position.x) / info.resolution).astype(int)
    rows = np.floor((ys - info.origin.position.y) / info.resolution).astype(int)
    ok_c = (cols >= 0) & (cols < info.width)
    ok_r = (rows >= 0) & (rows < info.height)
    image = np.full((H, W), 225, np.uint8)               # outside the map: unknown
    cells = grid[np.ix_(rows[ok_r], cols[ok_c])]
    image[np.ix_(ok_r, ok_c)] = np.where(cells < 0, 225, 255 - cells * 255 // 100)
    rgb = np.repeat(image[:, :, None], 3, axis=2)
    return f'P6 {W} {H} 255\n'.encode() + rgb.tobytes()


class LiveData:
    """Listens to ROS topics in a background thread and keeps the latest values."""

    def __init__(self):
        self.map, self.map_new = None, False
        self.pose, self.speed, self.plan = {}, {}, {}
        self.hold = False
        rclpy.init()
        self.node = rclpy.create_node('amr_gui')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        for robot in ROBOTS:
            self.node.create_subscription(Odometry, f'/{robot}/odom',
                                          lambda msg, r=robot: self.on_odom(r, msg), 10)
            self.node.create_subscription(Path, f'/{robot}/plan',
                                          lambda msg, r=robot: self.on_plan(r, msg), 10)
        self.node.create_subscription(Bool, '/amr2/hold',
                                      lambda msg: setattr(self, 'hold', msg.data), 10)
        threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()

    def on_map(self, msg):
        self.map, self.map_new = msg, True

    def on_odom(self, robot, msg):              # robot odometry is in the world (= map) frame
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose[robot] = (p.x, p.y, 2.0 * math.atan2(q.z, q.w))
        self.speed[robot] = msg.twist.twist.linear.x

    def on_plan(self, robot, msg):
        self.plan[robot] = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]


class App:
    def __init__(self, root):
        self.root = root
        self.sim = None                         # running "ros2 launch ..." process
        self.goals = None                       # running "send_goals.py" process
        self.lines = queue.Queue()              # (source, text) from the processes
        self.nav_ready = 0
        self.status = {robot: 'idle' for robot in ROBOTS}
        self.zones, self.tours = {}, {}
        self.live = LiveData() if ROS_OK else None
        self.map_photo = None

        root.title(TITLE)
        ttk.Style().theme_use('clam')
        self.build()
        self.load_config()
        root.protocol('WM_DELETE_WINDOW', self.on_close)
        self.tick()

    # ---------------------------------------------------------------- window layout
    def build(self):
        head = ttk.Frame(self.root, padding=(12, 8))
        head.pack(fill='x')
        ttk.Label(head, text=TITLE, font=('Helvetica', 17, 'bold')).pack(anchor='w')
        ttk.Label(head, text=SUBTITLE, foreground='#555').pack(anchor='w')

        body = ttk.Frame(self.root, padding=(12, 0))
        body.pack(fill='both', expand=True)

        # left: live map
        left = ttk.LabelFrame(body, text='Warehouse (live)', padding=6)
        left.pack(side='left', fill='y')
        self.canvas = tk.Canvas(left, width=W, height=H, bg='#e1e1e1', highlightthickness=0)
        self.canvas.pack()
        self.map_item = self.canvas.create_image(0, 0, anchor='nw')
        ttk.Label(left, foreground='#555', justify='left', text=(
            'white = free   black = wall / rack   grey = not explored yet\n'
            'dashed = your selection (straight line)   solid = path planned by Nav2')).pack()

        right = ttk.Frame(body, padding=(12, 0, 0, 0))
        right.pack(side='left', fill='both', expand=True)

        # 1. start / stop
        box = ttk.LabelFrame(right, text='1. Start the warehouse', padding=8)
        box.pack(fill='x')
        row = ttk.Frame(box)
        row.pack(fill='x')
        ttk.Label(row, text='Workspace:').pack(side='left')
        self.workspace = tk.StringVar(value=WORKSPACE)
        ttk.Entry(row, textvariable=self.workspace, width=34).pack(side='left', padx=6)
        self.launch = tk.StringVar(value=list(LAUNCH_OPTIONS)[0])
        ttk.Combobox(box, textvariable=self.launch, values=list(LAUNCH_OPTIONS),
                     state='readonly', width=40).pack(anchor='w', pady=6)
        row = ttk.Frame(box)
        row.pack(fill='x')
        self.show_gazebo = tk.BooleanVar(value=True)
        self.show_rviz = tk.BooleanVar(value=False)
        ttk.Checkbutton(row, text='Gazebo window', variable=self.show_gazebo).pack(side='left')
        ttk.Checkbutton(row, text='RViz', variable=self.show_rviz).pack(side='left', padx=10)
        row = ttk.Frame(box)
        row.pack(fill='x', pady=(6, 0))
        ttk.Button(row, text='Start simulation', command=self.start_sim).pack(side='left')
        ttk.Button(row, text='Stop everything', command=self.stop_all).pack(side='left', padx=6)
        self.sim_status = ttk.Label(box, foreground='#555')
        self.sim_status.pack(anchor='w', pady=(6, 0))

        # 2. goals
        box = ttk.LabelFrame(right, text='2. Choose where the robots go', padding=8)
        box.pack(fill='x', pady=8)
        self.mode = tk.StringVar(value='zones')
        ttk.Radiobutton(box, text='Send each robot to a zone', value='zones',
                        variable=self.mode).pack(anchor='w')
        self.goal, self.combo = {}, {}
        for robot, spec in ROBOTS.items():
            row = ttk.Frame(box)
            row.pack(fill='x', padx=(22, 0), pady=2)
            ttk.Label(row, text=spec['label'], width=18, foreground=spec['color']).pack(side='left')
            self.goal[robot] = tk.StringVar(value=DEFAULT_GOALS[robot])
            self.combo[robot] = ttk.Combobox(row, textvariable=self.goal[robot],
                                             state='readonly', width=18)
            self.combo[robot].pack(side='left')
        ttk.Radiobutton(box, text='Mapping tour (both robots explore the whole warehouse)',
                        value='tour', variable=self.mode).pack(anchor='w', pady=(4, 0))
        row = ttk.Frame(box)
        row.pack(fill='x', pady=(8, 0))
        ttk.Button(row, text='Send goals', command=self.send_goals).pack(side='left')
        ttk.Button(row, text='Cancel goals', command=self.cancel_goals).pack(side='left', padx=6)

        # 3. robots
        box = ttk.LabelFrame(right, text='3. Robots', padding=8)
        box.pack(fill='x')
        self.robot_status = {}
        for robot, spec in ROBOTS.items():
            ttk.Label(box, text=spec['label'], foreground=spec['color'],
                      font=('Helvetica', 11, 'bold')).pack(anchor='w')
            ttk.Label(box, text=spec['info'], foreground='#555').pack(anchor='w')
            self.robot_status[robot] = ttk.Label(box)
            self.robot_status[robot].pack(anchor='w', pady=(0, 6))

        ttk.Label(right, foreground='#555', justify='left', wraplength=430, text=(
            'How it works: "Start" launches Gazebo, both robots, sensor validation, SLAM, Nav2 '
            'and the motion/safety control. "Send goals" runs send_goals.py, which gives each '
            "robot's Nav2 its goal; Nav2 plans on the map the robots are building and replans "
            'every second as new walls appear.')).pack(anchor='w')

        # log
        box = ttk.LabelFrame(self.root, text='Commands and messages', padding=6)
        box.pack(fill='both', expand=True, padx=12, pady=8)
        self.show_all = tk.BooleanVar(value=False)
        ttk.Checkbutton(box, text='show all simulation output',
                        variable=self.show_all).pack(anchor='w')
        self.log_box = tk.Text(box, height=9, font='TkFixedFont', wrap='none')
        scroll = ttk.Scrollbar(box, command=self.log_box.yview)
        self.log_box.configure(yscrollcommand=scroll.set)
        scroll.pack(side='right', fill='y')
        self.log_box.pack(fill='both', expand=True)
        for tag, color in [('cmd', '#2b8a3e'), ('warn', '#e67700'), ('error', '#c92a2a')]:
            self.log_box.tag_configure(tag, foreground=color)

    def load_config(self):
        """Zones and tours from the package's zones.yaml (installed or source copy)."""
        ws = self.workspace.get()
        for path in [f'{ws}/install/amr_navigation/share/amr_navigation/config/zones.yaml',
                     f'{ws}/src/amr_warehouse/amr_navigation/config/zones.yaml']:
            if os.path.exists(path):
                with open(path) as f:
                    config = yaml.safe_load(f)
                self.zones, self.tours = config['zones'], config['tours']
                break
        else:
            self.log(f'zones.yaml not found in {ws} - is the workspace built?', 'error')
        for robot in ROBOTS:
            self.combo[robot]['values'] = [STAY] + list(self.zones)
        self.canvas.delete('zones')
        for name, (x, y, _) in self.zones.items():
            px, py = to_px(x, y)
            self.canvas.create_oval(px - 4, py - 4, px + 4, py + 4, fill='#2b8a3e',
                                    outline='', tags='zones')
            anchor = 'w' if px < 50 else 'e' if px > W - 50 else 'center'
            self.canvas.create_text(px, py - 10, text=name, font=('Helvetica', 8), anchor=anchor,
                                    fill='#2b8a3e', tags='zones')

    # ---------------------------------------------------------------- shell commands
    def run(self, command, source):
        """Run a command in bash with ROS + the workspace sourced; its output goes to the log."""
        ws = shlex.quote(self.workspace.get())
        script = (f'[ -f {ROS_SETUP} ] && source {ROS_SETUP}; '
                  f'source {ws}/install/setup.bash && {command}')
        self.log(f'$ {command}', 'cmd')
        process = subprocess.Popen(['bash', '-c', script], stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True, bufsize=1,
                                   start_new_session=True)       # own group: easy to stop
        threading.Thread(target=self.read_output, args=(process, source), daemon=True).start()
        return process

    def read_output(self, process, source):
        for line in process.stdout:
            self.lines.put((source, line.rstrip()))
        self.lines.put((source, None))           # finished

    @staticmethod
    def running(process):
        return process is not None and process.poll() is None

    @staticmethod
    def interrupt(process):
        if App.running(process):
            os.killpg(process.pid, signal.SIGINT)    # like Ctrl+C in the terminal

    def start_sim(self):
        if self.running(self.sim):
            return self.log('the simulation is already running', 'warn')
        self.load_config()
        command = (f'{LAUNCH_OPTIONS[self.launch.get()]} '
                   f'headless:={str(not self.show_gazebo.get()).lower()} '
                   f'rviz:={str(self.show_rviz.get()).lower()}')
        self.nav_ready = 0
        self.sim = self.run(command, 'sim')

    def stop_all(self):
        for process in (self.goals, self.sim):
            self.interrupt(process)
        self.log('stopping ... (Gazebo can take a few seconds)', 'warn')
        self.root.after(10000, self.force_stop)

    def force_stop(self):
        for process in (self.goals, self.sim):
            if self.running(process):
                os.killpg(process.pid, signal.SIGKILL)
        for name in ('gzserver', 'gzclient'):        # leftover Gazebo processes
            subprocess.run(['pkill', '-9', '-x', name], check=False)

    def send_goals(self):
        if self.mode.get() == 'tour':
            arguments = '--tour'
        else:
            arguments = ' '.join(f'{robot}={var.get()}' for robot, var in self.goal.items()
                                 if var.get() != STAY)
            if not arguments:
                return self.log('choose a zone for at least one robot', 'warn')
        self.interrupt(self.goals)                   # a new goal replaces the old one
        for robot in ROBOTS:
            self.status[robot] = 'goal sent, waiting for Nav2 ...'
            if self.live:
                self.live.plan.pop(robot, None)
        self.goals = self.run(f'ros2 run amr_navigation send_goals.py {arguments}', 'goals')

    def cancel_goals(self):
        self.interrupt(self.goals)
        services = ' '.join(f'/{robot}/{action}/_action/cancel_goal' for robot in ROBOTS
                            for action in ('navigate_to_pose', 'follow_waypoints'))
        # an empty CancelGoal request (zero goal id + zero time) cancels all goals
        self.run(f'for s in {services}; do ros2 service call $s action_msgs/srv/CancelGoal '
                 '"{}" > /dev/null && echo "cancelled $s"; done', 'cancel')
        for robot in ROBOTS:
            self.status[robot] = 'cancelled'

    # ---------------------------------------------------------------- updates (every 0.3 s)
    def tick(self):
        self.read_lines()
        self.draw()
        self.update_labels()
        self.root.after(300, self.tick)

    def read_lines(self):
        while not self.lines.empty():
            source, line = self.lines.get()
            if line is None:
                self.log(f'[{source}] finished', 'cmd')
                continue
            if source == 'sim':
                if 'Managed nodes are active' in line:
                    self.nav_ready += 1
                interesting = any(k in line for k in KEY_WORDS)
                if not self.show_all.get() and (not interesting or any(k in line for k in IGNORE)):
                    continue
            if source == 'goals':                    # e.g. "amr1: 3.2 m to go | amr2: waypoint 4"
                for part in line.split(' | '):
                    for robot in ROBOTS:
                        if part.strip().startswith(robot):
                            self.status[robot] = part.strip()[len(robot):].lstrip(' :->')
            tag = 'error' if 'ERROR' in line or 'died' in line else (
                'warn' if 'WARN' in line or 'FAILED' in line else None)
            self.log(f'[{source}] {line}', tag)

    def log(self, text, tag=None):
        self.log_box.insert('end', text + '\n', tag)
        if int(self.log_box.index('end-1c').split('.')[0]) > 2000:
            self.log_box.delete('1.0', '500.0')
        self.log_box.see('end')

    def draw(self):
        c = self.canvas
        c.delete('dyn')
        live = self.live
        if live and live.map_new:                    # new shared map from map_fusion
            live.map_new = False
            self.map_photo = tk.PhotoImage(data=map_image(live.map))
            c.itemconfigure(self.map_item, image=self.map_photo)

        for robot, spec in ROBOTS.items():
            start = (*spec['start'], 0.0)
            x, y, yaw = live.pose.get(robot, start) if live else start
            # your selection: straight dashed lines to the goal / along the tour
            if self.mode.get() == 'tour':
                points = [(x, y)] + [tuple(p) for p in self.tours.get(robot, [])]
            elif self.goal[robot].get() in self.zones:
                points = [(x, y), tuple(self.zones[self.goal[robot].get()][:2])]
            else:
                points = []
            if len(points) > 1:
                c.create_line(*[v for p in points for v in to_px(*p)], fill=spec['color'],
                              dash=(4, 4), width=1, tags='dyn')
            # the path Nav2 planned (hidden once the goal is finished)
            plan = live.plan.get(robot, []) if live else []
            finished = any(w in self.status[robot] for w in ('reached', 'FAILED', 'cancelled'))
            if len(plan) > 1 and not finished:
                c.create_line(*[v for p in plan for v in to_px(*p)], fill=spec['color'],
                              width=3, tags='dyn')
            # the robot
            px, py = to_px(x, y)
            c.create_oval(px - 7, py - 7, px + 7, py + 7, fill=spec['color'], outline='white',
                          width=2, tags='dyn')
            c.create_line(px, py, px + 14 * math.cos(yaw), py - 14 * math.sin(yaw),
                          fill=spec['color'], width=3, tags='dyn')
            c.create_text(px + 12, py + 12, text=robot.upper(), anchor='w',
                          font=('Helvetica', 9, 'bold'), fill=spec['color'], tags='dyn')

    def update_labels(self):
        sim = 'running' if self.running(self.sim) else 'stopped'
        live = 'on' if self.live else 'off (ROS not found - start from a ROS terminal)'
        self.sim_status.configure(
            text=f'Simulation: {sim}   ·   Nav2 ready: {self.nav_ready}/2   ·   live view: {live}')
        for robot in ROBOTS:
            speed = self.live.speed.get(robot, 0.0) if self.live else 0.0
            text = f'speed {abs(speed):.2f} m/s   ·   {self.status[robot]}'
            if robot == 'amr2' and self.live and self.live.hold:
                text += '   ·   YIELDING to AMR-1'
            self.robot_status[robot].configure(text=text)

    def on_close(self):
        for process in (self.goals, self.sim):
            self.interrupt(process)
        if self.live:
            rclpy.try_shutdown()
        self.root.destroy()


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == '__main__':
    main()
