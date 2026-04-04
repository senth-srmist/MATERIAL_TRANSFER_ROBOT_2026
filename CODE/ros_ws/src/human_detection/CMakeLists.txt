cmake_minimum_required(VERSION 3.8)
project(human_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(zed_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service
rosidl_generate_interfaces(${PROJECT_NAME} "srv/TriggerAlarm.srv" DEPENDENCIES
                           std_msgs)

# ==============================================================================
# Costmap Layer Plugin
# ==============================================================================
add_library(human_costmap_layer SHARED src/human_costmap_layer.cpp)

target_include_directories(
  human_costmap_layer
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  human_costmap_layer
  rclcpp
  nav2_costmap_2d
  pluginlib
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  zed_msgs)

# Register the costmap layer plugin
pluginlib_export_plugin_description_file(nav2_costmap_2d
                                         human_costmap_layer_plugin.xml)

# ==============================================================================
# BT Nodes Library
# ==============================================================================
add_library(
  human_bt_nodes SHARED src/human_blocking_path.cpp src/raise_human_alarm.cpp
                        src/wait_human_clears.cpp)

target_include_directories(
  human_bt_nodes PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(human_bt_nodes rclcpp behaviortree_cpp_v3
                          nav2_behavior_tree geometry_msgs zed_msgs)

# Link generated service interfaces to BT nodes if needed
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
                              "rosidl_typesupport_cpp")
target_link_libraries(human_bt_nodes ${cpp_typesupport_target})

# ==============================================================================
# Install
# ==============================================================================
install(
  TARGETS human_costmap_layer human_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

install(FILES human_costmap_layer_plugin.xml DESTINATION share/${PROJECT_NAME})

# ==============================================================================
# Export
# ==============================================================================
ament_export_include_directories(include)
ament_export_libraries(human_costmap_layer human_bt_nodes)
ament_export_dependencies(rclcpp nav2_costmap_2d pluginlib behaviortree_cpp_v3
                          nav2_behavior_tree zed_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
