cmake_minimum_required(VERSION 3.8)
project(amr_navigation)

find_package(ament_cmake REQUIRED)

# Launch files, configs and RViz settings -> share/amr_navigation
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})

# Python nodes -> lib/amr_navigation (so `ros2 run amr_navigation <script>` finds them).
# grid_tools.py is the helper module they import.
install(PROGRAMS
  scripts/selective_map_filter.py
  scripts/map_fusion.py
  scripts/terrain_cost.py
  scripts/fleet_scan.py
  scripts/send_goals.py
  scripts/grid_tools.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
