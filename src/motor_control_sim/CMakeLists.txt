cmake_minimum_required(VERSION 3.8)
project(motor_control_sim)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(motor_control_msgs REQUIRED)

# Include directories
include_directories(include)

# Motor Simulator Node
add_executable(motor_simulator src/motor_simulator_node.cpp)
ament_target_dependencies(motor_simulator
  rclcpp
  geometry_msgs
  std_msgs
  motor_control_msgs
)
target_include_directories(motor_simulator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Cmd Vel Publisher (test utility)
add_executable(cmd_vel_publisher src/cmd_vel_publisher.cpp)
ament_target_dependencies(cmd_vel_publisher
  rclcpp
  geometry_msgs
)
target_include_directories(cmd_vel_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install executables
install(TARGETS
  motor_simulator
  cmd_vel_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Unit testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_motor_model test/test_motor_model.cpp)
  target_include_directories(test_motor_model PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
endif()

ament_package()
