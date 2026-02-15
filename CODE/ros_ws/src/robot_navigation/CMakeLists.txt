cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)

# Install config, behavior trees, and launch
install(DIRECTORY
  config
  behavior_trees
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
