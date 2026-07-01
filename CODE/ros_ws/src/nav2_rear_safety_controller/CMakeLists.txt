cmake_minimum_required(VERSION 3.8)
project(nav2_rear_safety_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ======================
# FIND DEPENDENCIES
# ======================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)

# ======================
# INCLUDE DIRECTORIES
# ======================
include_directories(
  include
)

# ======================
# CREATE PLUGIN LIBRARY
# ======================
add_library(nav2_rear_safety_controller SHARED
  src/rear_safety_controller.cpp
)

# ======================
# LINK DEPENDENCIES
# ======================
ament_target_dependencies(nav2_rear_safety_controller
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_util
  nav2_costmap_2d
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
  pluginlib
)

# ======================
# EXPORT PLUGIN
# ======================
pluginlib_export_plugin_description_file(nav2_core plugin.xml)

# ======================
# INSTALL
# ======================
install(
  TARGETS nav2_rear_safety_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# ======================
# TESTING (OPTIONAL)
# ======================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()