cmake_minimum_required(VERSION 3.8)
project(nav2_depth_obstacle_layer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(OpenCV REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(speech_pkg REQUIRED)

# Optional: ZED messages for human detection
find_package(zed_msgs QUIET)

# =============================================================================
# Costmap Layer Library
# =============================================================================
set(LAYER_DEPS
  rclcpp
  nav2_costmap_2d
  pluginlib
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_geometry
)

add_library(${PROJECT_NAME}_layer SHARED
  src/depth_obstacle_layer.cpp
)

target_include_directories(${PROJECT_NAME}_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_layer
  ${OpenCV_LIBRARIES}
)

if(zed_msgs_FOUND)
  target_compile_definitions(${PROJECT_NAME}_layer PRIVATE HAVE_ZED_MSGS)
  list(APPEND LAYER_DEPS zed_msgs)
  message(STATUS "ZED messages found - human masking enabled")
endif()

ament_target_dependencies(${PROJECT_NAME}_layer ${LAYER_DEPS})

pluginlib_export_plugin_description_file(nav2_costmap_2d depth_obstacle_layer_plugin.xml)

# =============================================================================
# BT Nodes Library
# =============================================================================
set(BT_DEPS
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  behaviortree_cpp_v3
  speech_pkg
)

add_library(${PROJECT_NAME}_bt_nodes SHARED
  src/bt_nodes/human_blocking_path.cpp
  src/bt_nodes/wait_until_human_clears.cpp
  src/bt_nodes/bt_register_nodes.cpp
)

target_include_directories(${PROJECT_NAME}_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(zed_msgs_FOUND)
  target_compile_definitions(${PROJECT_NAME}_bt_nodes PRIVATE HAVE_ZED_MSGS)
  list(APPEND BT_DEPS zed_msgs)
endif()

ament_target_dependencies(${PROJECT_NAME}_bt_nodes ${BT_DEPS})

# =============================================================================
# Install
# =============================================================================
install(TARGETS ${PROJECT_NAME}_layer ${PROJECT_NAME}_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES depth_obstacle_layer_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_layer ${PROJECT_NAME}_bt_nodes)
ament_export_dependencies(${LAYER_DEPS} ${BT_DEPS})

ament_package()
