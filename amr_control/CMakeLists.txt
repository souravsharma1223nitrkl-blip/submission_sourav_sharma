cmake_minimum_required(VERSION 3.8)
project(amr_control)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/motion_smoother.py
  scripts/traffic_control.py
  scripts/safety_stop.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
