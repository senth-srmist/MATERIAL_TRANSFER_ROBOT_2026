cmake_minimum_required(VERSION 3.8)
project(mission_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tile_manager REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME} "srv/NavigateToRoom.srv")

# Install Python scripts
file(GLOB PYTHON_SCRIPTS scripts/*.py)

install(PROGRAMS ${PYTHON_SCRIPTS} DESTINATION lib/${PROJECT_NAME})

# Install config if any
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
