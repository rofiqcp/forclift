cmake_minimum_required(VERSION 3.18)
project(esc)

# ROS 2 Humble must use the system Python, not an unrelated activated venv
# (e.g. ~/.hermes/venv). ament_cmake executes package_xml_2_cmake.py through
# Python3::Interpreter, so a stale virtualenv here breaks even pure C++ packages.
if(EXISTS "/usr/bin/python3")
  set(Python3_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "ROS system Python" FORCE)
  set(PYTHON_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "ROS system Python (legacy)" FORCE)
endif()


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME esc_runtime_self_check
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/esc_runtime_self_check.py)
  add_test(
    NAME esc_build_directory_self_check
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/esc_build_directory_self_check.py)
endif()

# Defensive build-directory creation. GNU depfiles (*.o.d) are written under
# CMakeFiles/<target>.dir/src/. A partially interrupted/stale colcon build can
# leave the target makefiles while those directories are missing; create them
# at configure time so a normal `colcon build` cannot fail only because the
# depfile parent directory disappeared.
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/motor_teleop.dir/src")
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/ackermann_controller_server.dir/src")

# Manual keyboard/gamepad input. This node NEVER owns the STM UART.
add_executable(motor_teleop src/motor_teleop.cpp)
ament_target_dependencies(motor_teleop rclcpp geometry_msgs std_msgs)

# Single actuator owner: TELEOP/NAV2 mux + Ackermann conversion + robust STM UART.
add_executable(ackermann_controller_server src/ackermann_controller_server.cpp)
ament_target_dependencies(ackermann_controller_server rclcpp rcl_interfaces geometry_msgs nav_msgs std_msgs)

install(
  TARGETS motor_teleop ackermann_controller_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
