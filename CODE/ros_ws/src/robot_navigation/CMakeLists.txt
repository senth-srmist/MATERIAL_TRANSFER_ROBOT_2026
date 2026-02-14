cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

find_package(ament_cmake REQUIRED)

# Install config and behavior trees
install(DIRECTORY
  config
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
