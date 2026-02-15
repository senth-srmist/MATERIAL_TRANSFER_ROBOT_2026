cmake_minimum_required(VERSION 3.8)
project(tile_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==================== DEPENDENCIES ====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# ==================== C++ BT PLUGIN ====================
add_library(tile_manager_bt_plugin SHARED
  src/tile_check_plugin.cpp
)

# CRITICAL: Export all symbols so BT_RegisterNodesFromPlugin is visible
set_target_properties(tile_manager_bt_plugin PROPERTIES
  CXX_VISIBILITY_PRESET default
  VISIBILITY_INLINES_HIDDEN OFF
)

target_include_directories(tile_manager_bt_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(tile_manager_bt_plugin
  rclcpp
  nav2_msgs
  nav2_behavior_tree
  behaviortree_cpp_v3
  tf2_ros
  tf2
  geometry_msgs
  ament_index_cpp
)

target_link_libraries(tile_manager_bt_plugin
  yaml-cpp
)

# ==================== INSTALL ====================

# C++ library
install(TARGETS tile_manager_bt_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Headers
install(DIRECTORY include/
  DESTINATION include
)

# Config, maps, behavior trees
install(DIRECTORY
  config
  maps
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

# Python modules (keeping for benchmark tools)
install(DIRECTORY
  tile_manager/
  DESTINATION lib/python3/dist-packages/tile_manager
  FILES_MATCHING PATTERN "*.py"
)

# Python executables
install(PROGRAMS
  tile_manager/tile_switcher.py
  tile_manager/benchmark_node.py
  tile_manager/benchmark_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Launch files if any
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  OPTIONAL
)

# Resource index
install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

# ==================== EXPORT ====================
ament_export_include_directories(include)
ament_export_libraries(tile_manager_bt_plugin)
ament_export_dependencies(
  rclcpp
  behaviortree_cpp_v3
  nav2_behavior_tree
  nav2_msgs
  tf2_ros
)

ament_package()
