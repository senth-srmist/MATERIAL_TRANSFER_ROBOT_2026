cmake_minimum_required(VERSION 3.8)
project(system_supervisor)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME} "msg/NodeHealth.msg"
                           "msg/RobotHealth.msg" DEPENDENCIES std_msgs)

# Install Python nodes (renamed to avoid shadowing package name)
install(
  PROGRAMS scripts/system_supervisor.py
  RENAME supervisor_node.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
