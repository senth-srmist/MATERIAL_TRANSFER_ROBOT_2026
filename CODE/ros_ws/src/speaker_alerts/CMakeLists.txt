cmake_minimum_required(VERSION 3.8)
project(speaker_alerts)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate Speak.srv
rosidl_generate_interfaces(${PROJECT_NAME} "srv/Speak.srv")

# Install Python node
install(PROGRAMS scripts/speaker_alerts.py DESTINATION lib/${PROJECT_NAME})

ament_package()
