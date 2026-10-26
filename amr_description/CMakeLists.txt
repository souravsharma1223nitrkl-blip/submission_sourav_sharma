cmake_minimum_required(VERSION 3.8)
project(amr_description)

find_package(ament_cmake REQUIRED)

# Install the robot model files so launch files can find them
install(DIRECTORY sdf DESTINATION share/${PROJECT_NAME})

ament_package()
