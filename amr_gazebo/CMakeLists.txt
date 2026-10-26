cmake_minimum_required(VERSION 3.8)
project(amr_gazebo)

find_package(ament_cmake REQUIRED)

# Install world, models, launch files, configs and RViz settings into share/amr_gazebo
install(DIRECTORY launch worlds models config rviz DESTINATION share/${PROJECT_NAME})

# Install the Python node so `ros2 run amr_gazebo moving_obstacles.py` finds it
install(PROGRAMS scripts/moving_obstacles.py DESTINATION lib/${PROJECT_NAME})

ament_package()
