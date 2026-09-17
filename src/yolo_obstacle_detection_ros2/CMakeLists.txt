cmake_minimum_required(VERSION 3.10)

# CMake >= 3.18: let CMake/NVCC detect and populate the CUDA architecture
# instead of leaving CUDA_ARCHITECTURES empty (CMP0104 OLD behaviour).
# This removes the developer warning on Jetson while keeping the build
# portable across Jetson GPU generations instead of hard-coding sm_87/sm_72.
if(POLICY CMP0104)
  cmake_policy(SET CMP0104 NEW)
endif()

project(yolo_obstacle_detection_ros2)

# ROS 2 Humble on Ubuntu 22.04 must use the system Python 3.10.
# Keep rosidl-generated Python interfaces ABI-compatible with rclpy.
set(Python3_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "ROS 2 Humble Python" FORCE)
set(PYTHON_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "ROS 2 Humble Python" FORCE)
set(Python_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "ROS 2 Humble Python" FORCE)
find_package(Python3 3.10 EXACT COMPONENTS Interpreter REQUIRED)
message(STATUS "[YOLO-PYTHON] ROS 2 Humble Python locked to ${Python3_EXECUTABLE} (${Python3_VERSION})")

# Fail early with one readable error if a partial archive/update has replaced
# this package. These files are part of the production package and must exist.
set(YOLO_REQUIRED_FILES
  msg/Obstacle.msg
  msg/ObstacleArray.msg
  msg/Pose2DYaw.msg
  msg/DockingState.msg
  msg/PalletPose.msg
  msg/FloorMarking.msg
  msg/AlignmentState.msg
  src/obstacle_detector_node.cpp
  src/pallet_pose_estimator.cpp
  src/astra_rgb_v4l2_node.cpp
  include/yolo_obstacle_detection_ros2/astra_rgb_driver.hpp
  include/yolo_obstacle_detection_ros2/gpu_preprocess.hpp
  src/gpu_preprocess.cu
  hole_block_alignment/hole_block_alignment_node.py
)
set(YOLO_MISSING_FILES "")
foreach(_required IN LISTS YOLO_REQUIRED_FILES)
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_required}")
    list(APPEND YOLO_MISSING_FILES "${_required}")
  endif()
endforeach()
if(YOLO_MISSING_FILES)
  string(JOIN "\n  - " _missing_text ${YOLO_MISSING_FILES})
  message(FATAL_ERROR
    "[YOLO-SOURCE] Package is incomplete. Missing required files:\n  - ${_missing_text}\n"
    "Restore the complete yolo_obstacle_detection_ros2 package before building.")
endif()
message(STATUS "[YOLO-SOURCE] package integrity preflight PASS")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  # Keep strict host warnings on C++ sources without forwarding -Wpedantic to
  # nvcc/CUDA headers. The latter generated thousands of harmless GCC line-
  # directive warnings on Jetson even though the package built successfully.
  add_compile_options(
    $<$<COMPILE_LANGUAGE:CXX>:-Wall>
    $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
    $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>
    $<$<COMPILE_LANGUAGE:CXX>:-O3>
    $<$<COMPILE_LANGUAGE:CUDA>:-O3>
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)
pkg_check_modules(GLIB REQUIRED glib-2.0)
include(CheckLanguage)

set(TENSORRT_FOUND FALSE)
find_path(TENSORRT_INCLUDE_DIR NvInfer.h PATHS /usr/include/aarch64-linux-gnu /usr/include /usr/src/tensorrt/include)
find_library(NVINFER_LIBRARY nvinfer PATHS /usr/lib/aarch64-linux-gnu /usr/lib)
find_library(CUDART_LIBRARY cudart PATHS /usr/local/cuda/targets/aarch64-linux/lib /usr/local/cuda/lib64 /usr/lib/aarch64-linux-gnu)
find_path(CUDA_RUNTIME_INCLUDE_DIR cuda_runtime.h PATHS /usr/local/cuda/include /usr/local/cuda/targets/aarch64-linux/include)
if(TENSORRT_INCLUDE_DIR AND NVINFER_LIBRARY AND CUDART_LIBRARY AND CUDA_RUNTIME_INCLUDE_DIR)
  # PART-3 uses TensorRT named-I/O (getNbIOTensors/setTensorAddress/enqueueV3),
  # which is the stable JetPack 6 / TensorRT >= 8.5 path.  If an older
  # TensorRT is installed, compile the OpenCV-CUDA fallback instead of
  # failing later at C++ compile time.
  set(_trt_version_header "${TENSORRT_INCLUDE_DIR}/NvInferVersion.h")
  set(_trt_major 0)
  set(_trt_minor 0)
  if(EXISTS "${_trt_version_header}")
    file(STRINGS "${_trt_version_header}" _trt_major_line REGEX "^#define NV_TENSORRT_MAJOR")
    file(STRINGS "${_trt_version_header}" _trt_minor_line REGEX "^#define NV_TENSORRT_MINOR")
    string(REGEX REPLACE ".*NV_TENSORRT_MAJOR[ \t]+([0-9]+).*" "\\1" _trt_major "${_trt_major_line}")
    string(REGEX REPLACE ".*NV_TENSORRT_MINOR[ \t]+([0-9]+).*" "\\1" _trt_minor "${_trt_minor_line}")
  endif()
  if((_trt_major GREATER 8) OR (_trt_major EQUAL 8 AND _trt_minor GREATER_EQUAL 5))
    set(TENSORRT_FOUND TRUE)
    add_compile_definitions(HAS_TENSORRT=1)
    message(STATUS "[YOLO-TRT] TensorRT ${_trt_major}.${_trt_minor} named-I/O path ENABLED")
  else()
    add_compile_definitions(HAS_TENSORRT=0)
    message(WARNING "[YOLO-TRT] TensorRT ${_trt_major}.${_trt_minor} is too old for the named-I/O path; using OpenCV CUDA fallback")
  endif()
else()
  add_compile_definitions(HAS_TENSORRT=0)
endif()

# The CUDA language is optional.  TensorRT inference remains available without
# nvcc (CPU letterbox + pinned buffers), while Jetson builds with nvcc compile
# the GPU preprocessing kernel for both the executable and component targets.
if(TENSORRT_FOUND)
  check_language(CUDA)
  if(CMAKE_CUDA_COMPILER)
    enable_language(CUDA)
    message(STATUS "[YOLO-CUDA] CUDA compiler ${CMAKE_CUDA_COMPILER}; GPU preprocess ENABLED")
  else()
    message(WARNING "[YOLO-CUDA] nvcc not found; TensorRT uses CPU preprocess + pinned buffers")
  endif()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  "msg/Pose2DYaw.msg"
  "msg/DockingState.msg"
  "msg/PalletPose.msg"
  "msg/FloorMarking.msg"
  "msg/AlignmentState.msg"
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

function(configure_obstacle_target target_name)
  ament_target_dependencies(${target_name}
    rclcpp rclcpp_components sensor_msgs std_msgs geometry_msgs tf2_ros tf2_geometry_msgs builtin_interfaces)
  target_include_directories(${target_name} PRIVATE include)
  target_link_libraries(${target_name} ${OpenCV_LIBRARIES} ${cpp_typesupport_target})
  if(TENSORRT_FOUND)
    target_include_directories(${target_name} PRIVATE ${TENSORRT_INCLUDE_DIR} ${CUDA_RUNTIME_INCLUDE_DIR})
    target_link_libraries(${target_name} ${NVINFER_LIBRARY} ${CUDART_LIBRARY})
    if(CMAKE_CUDA_COMPILER)
      target_sources(${target_name} PRIVATE src/gpu_preprocess.cu)
      set_target_properties(${target_name} PROPERTIES CUDA_STANDARD 14 CUDA_STANDARD_REQUIRED ON)
      target_compile_definitions(${target_name} PRIVATE HAS_CUDA_PREPROCESS=1)
    else()
      target_compile_definitions(${target_name} PRIVATE HAS_CUDA_PREPROCESS=0)
    endif()
  else()
    target_compile_definitions(${target_name} PRIVATE HAS_CUDA_PREPROCESS=0)
  endif()
endfunction()

# Standalone executable remains available as a deployment fallback.
add_executable(obstacle_detector_node src/obstacle_detector_node.cpp)
configure_obstacle_target(obstacle_detector_node)

# Low-rate COCO semantic companion. Reuses the same TensorRT/GPU-preprocess
# implementation but emits only person detections, so the pallet model remains
# unchanged and runtime overhead is bounded by max_inference_fps=3.
add_executable(warehouse_person_detector_node src/obstacle_detector_node.cpp)
target_compile_definitions(warehouse_person_detector_node PRIVATE WAREHOUSE_COCO_PROFILE=1)
configure_obstacle_target(warehouse_person_detector_node)

# PART-5 Stage-3: component build enables same-process camera -> YOLO
# intra-process delivery, avoiding DDS serialization between these two nodes.
add_library(obstacle_detector_component SHARED src/obstacle_detector_node.cpp)
target_compile_definitions(obstacle_detector_component PRIVATE YOLO_COMPONENT_BUILD=1)
configure_obstacle_target(obstacle_detector_component)
rclcpp_components_register_nodes(obstacle_detector_component "ObstacleDetectorNode")

# V30 production profile: pallet_pose_estimator is intentionally not built.
# The current vehicle perception contract is RGB camera + YOLO object detection
# + hole-block alignment. Keeping the unused cv_bridge-linked target out of the
# production build prevents OpenCV 4.5/4.10 linker conflicts on this Jetson.

# HOLE-GUIDED BLOCK ALIGNMENT (Python) — consumes existing YOLO ObstacleArray.
install(PROGRAMS
  hole_block_alignment/hole_block_alignment_node.py
  tools/build_yolo_tensorrt_engine.sh
  tools/check_camera_usb.sh
  tools/verify_source_integrity.sh
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY hole_block_alignment
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.py" EXCLUDE)

include_directories(include)

# Internal V4L2 Astra RGB driver — no external astra_camera_ros2 dependency.
# Publishes /camera/color/image_raw + /camera/color/camera_info via Linux V4L2 API.
function(configure_camera_target target_name)
  ament_target_dependencies(${target_name} rclcpp rclcpp_components sensor_msgs std_msgs)
  target_include_directories(${target_name} PRIVATE ${GST_INCLUDE_DIRS} ${GLIB_INCLUDE_DIRS} include)
  target_link_libraries(${target_name} ${OpenCV_LIBRARIES} ${GST_LIBRARIES} ${GLIB_LIBRARIES})
endfunction()

add_executable(astra_rgb_v4l2_node src/astra_rgb_v4l2_node.cpp)
configure_camera_target(astra_rgb_v4l2_node)
add_library(astra_rgb_component SHARED src/astra_rgb_v4l2_node.cpp)
target_compile_definitions(astra_rgb_component PRIVATE YOLO_COMPONENT_BUILD=1)
configure_camera_target(astra_rgb_component)
rclcpp_components_register_nodes(astra_rgb_component "astra_rgb_driver::AstraRGBNode")

install(TARGETS obstacle_detector_node warehouse_person_detector_node astra_rgb_v4l2_node obstacle_detector_component astra_rgb_component
         ARCHIVE DESTINATION lib
         LIBRARY DESTINATION lib
         RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  PATTERN "*.bak*" EXCLUDE)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/models")
  install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
endif()
ament_package()
