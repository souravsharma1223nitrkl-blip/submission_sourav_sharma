# AMR Warehouse: Adaptive Navigation & Conflict-Aware Path Planning

ROS 2 Humble · Gazebo Classic 11 · Ubuntu 22.04

Two different robots in a simulated logistics warehouse. They build one shared map together,
plan around a ramp their LiDARs cannot see, avoid each other (the scout yields to the heavy
robot), drive smoothly, and stop before they hit anything. All sensor data is validated first.

| Robot | Body | Role |
|---|---|---|
| AMR-1 `amr1` | TurtleBot3 Waffle Pi, 8 m LiDAR | heavy lead / mapper, has priority |
| AMR-2 `amr2` | TurtleBot3 Burger, 3.5 m LiDAR | fast scout, yields |

Maintainer: Sourav (souravsharma1321@gmail.com)

## Packages

| Package | Section | What it does |
|---|---|---|
| `amr_description` | 1 | robot models (SDF/xacro) for both robots |
| `amr_gazebo` | 1 | warehouse world with mezzanine ramp, pedestrians, robot spawning |
| `amr_bsp` | 4 | sensor validation: checks every LiDAR and IMU message |
| `amr_navigation` | 2 | SLAM per robot, shared map, ramp cost, Nav2, goal script  |
| `amr_control` | 3 | motion smoothing, yielding between robots, safety stop |

## Install

Ubuntu 22.04 with ROS 2 Humble already installed:

```bash
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-turtlebot3-gazebo ros-humble-xacro \
  ros-humble-navigation2 ros-humble-slam-toolbox ros-humble-teleop-twist-keyboard \
  ros-humble-tf2-tools python3-numpy python3-yaml

mkdir -p ~/amr_ws/src && cd ~/amr_ws/src
git clone -b ros2 https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git
# copy this folder here as ~/amr_ws/src/amr_warehouse
```

## Build

```bash
cd ~/amr_ws
colcon build --symlink-install
source install/setup.bash
```

Run `source ~/amr_ws/install/setup.bash` in every new terminal.

## Run

Start everything (Gazebo, both robots, SLAM, Nav2, control):

```bash
ros2 launch amr_control fleet.launch.py

# if RVIZ do not launch 
ros2 run rviz2 rviz2 -d ~/amr_ws/install/amr_navigation/share/amr_navigation/rviz/navigation.rviz --ros-args -p use_sim_time:=true
```

Wait until "Managed nodes are active" appears twice in the log, then send goals in a second terminal.

Lighter options:

```bash
ros2 launch amr_control fleet.launch.py headless:=true rviz:=false   # no Gazebo window, no RViz
ros2 launch amr_navigation navigation.launch.py                      # Sections 1, 2, 4 only
ros2 launch amr_gazebo sim.launch.py                                 # simulation only
```

## Send goals

```bash
ros2 run amr_navigation send_goals.py amr1=heavy_storage amr2=packing_bay_4   # both robots at once
ros2 run amr_navigation send_goals.py amr1=2.0,-3.5                           # x,y in metres
ros2 run amr_navigation send_goals.py --tour                                  # map the whole warehouse
```

Zones: `base`, `scout_dock`, `heavy_storage`, `packing_bay_1` … `packing_bay_5`, `rescue_zone`.
Goals can also be set in RViz with the "Nav2 Goal" tool (one per robot).

## Stop

Ctrl+C in the launch terminal. If Gazebo keeps running:

```bash
pkill -9 -x gzserver; pkill -9 -x gzclient
```

## Credits

- AWS RoboMaker small warehouse world (MIT-0): https://github.com/aws-robotics/aws-robomaker-small-warehouse-world
- Robot models derived from TurtleBot3 simulations (Apache-2.0): https://github.com/ROBOTIS-GIT/turtlebot3_simulations
