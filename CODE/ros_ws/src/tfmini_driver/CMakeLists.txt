cmake_minimum_required(VERSION 3.8)
project(tfmini_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# Dependencies
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# =========================
# Executable
# =========================
add_executable(tfmini_node src/tfmini_node.cpp)

ament_target_dependencies(tfmini_node
  rclcpp
  sensor_msgs
)

# =========================
# Install
# =========================
install(TARGETS
  tfmini_node
  DESTINATION lib/${PROJECT_NAME}
)

# =========================
# Testing (optional)
# =========================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# =========================
# IMPORTANT: LAST LINE
# =========================
ament_package()