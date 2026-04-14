cmake_minimum_required(VERSION 3.8)
project(depth_costmap_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(cv_bridge REQUIRED)

add_library(depth_image_costmap_layer SHARED
  src/depth_image_costmap_layer.cpp
)

target_include_directories(depth_image_costmap_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(depth_image_costmap_layer
  rclcpp nav2_costmap_2d pluginlib tf2_ros tf2_geometry_msgs
  geometry_msgs sensor_msgs image_geometry cv_bridge
)

pluginlib_export_plugin_description_file(nav2_costmap_2d
  depth_costmap_layer_plugin.xml
)

install(TARGETS depth_image_costmap_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES depth_costmap_layer_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(depth_image_costmap_layer)

ament_package()