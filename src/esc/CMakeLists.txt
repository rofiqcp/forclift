cmake_minimum_required(VERSION 3.18)
project(esc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)

add_executable(esc_driver src/esc_driver_node.cpp src/vesc_board.cpp)
target_include_directories(esc_driver PUBLIC include)
ament_target_dependencies(esc_driver rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs)

add_executable(esc_command_mux src/esc_command_mux.cpp)
ament_target_dependencies(esc_command_mux rclcpp geometry_msgs sensor_msgs std_msgs)

add_executable(esc_keyboard_teleop src/teleop_keyboard_global.cpp)
ament_target_dependencies(esc_keyboard_teleop rclcpp geometry_msgs std_msgs)

add_executable(winch_serial_node src/winch_serial_node.cpp)
ament_target_dependencies(winch_serial_node rclcpp std_msgs geometry_msgs sensor_msgs)

# Nav2 ControllerServer wrapper belongs to ESC because this package owns the
# Ackermann actuator interface from cmd_vel through steering/speed commands.
add_executable(ackermann_controller_server src/ackermann_controller_server.cpp)
ament_target_dependencies(ackermann_controller_server rclcpp nav2_controller)

add_library(esc_aware_progress_checker SHARED src/esc_aware_progress_checker.cpp)
ament_target_dependencies(
  esc_aware_progress_checker
  rclcpp rclcpp_lifecycle std_msgs geometry_msgs nav2_core nav2_controller pluginlib)
pluginlib_export_plugin_description_file(nav2_core esc_progress_checker.xml)

install(
  TARGETS esc_aware_progress_checker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  TARGETS esc_driver esc_command_mux esc_keyboard_teleop winch_serial_node ackermann_controller_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY tools DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()
