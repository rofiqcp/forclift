cmake_minimum_required(VERSION 3.18)
project(perception LANGUAGES CXX)

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

option(PERCEPTION_BUILD_CPU "Build the direct .pt LibTorch CPU backend when PyTorch is available" ON)
option(PERCEPTION_REQUIRE_CPU "Fail configure if the LibTorch CPU backend cannot be built" OFF)
set(PERCEPTION_TORCH_PYTHON "" CACHE FILEPATH "Python interpreter whose torch installation supplies LibTorch")
# Runtime target policy: Jetson/aarch64 builds TensorRT by default; x86_64 Mini-PC
# defaults to CPU LibTorch even if a CUDA toolkit happens to be installed.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(_PERCEPTION_GPU_DEFAULT ON)
else()
  set(_PERCEPTION_GPU_DEFAULT OFF)
endif()
option(PERCEPTION_BUILD_GPU "Build the CUDA/TensorRT backend when dependencies exist" ${_PERCEPTION_GPU_DEFAULT})
option(ASTRA_YOLOP_FAST_MATH "Enable CUDA fast math for Astra YOLOP kernels" ON)
set(ASTRA_CUDA_ROOT "" CACHE PATH "CUDA toolkit root; empty means auto-detect")
if(NOT ASTRA_CUDA_ROOT)
  set(_PERCEPTION_CUDA_CANDIDATES)
  if(DEFINED ENV{CUDA_HOME} AND NOT "$ENV{CUDA_HOME}" STREQUAL "")
    list(APPEND _PERCEPTION_CUDA_CANDIDATES "$ENV{CUDA_HOME}")
  endif()
  list(APPEND _PERCEPTION_CUDA_CANDIDATES
    "/usr/local/cuda"
    "/usr/local/cuda-12.6"
    "/usr/local/cuda-12.4"
    "/usr/local/cuda-12.2"
    "/usr/local/cuda-11.8")
  foreach(_CUDA_CANDIDATE IN LISTS _PERCEPTION_CUDA_CANDIDATES)
    if(EXISTS "${_CUDA_CANDIDATE}/bin/nvcc")
      set(ASTRA_CUDA_ROOT "${_CUDA_CANDIDATE}" CACHE PATH "CUDA toolkit root" FORCE)
      break()
    endif()
  endforeach()
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    $<$<COMPILE_LANGUAGE:CXX>:-Wall>
    $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
    $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>)
endif()

# Some CUDA-enabled OpenCV packages still use legacy FindCUDA internally.
# Keep that compatibility path available, but never make it a requirement for
# the CPU backend.
if(POLICY CMP0146)
  cmake_policy(SET CMP0146 OLD)
endif()
if(EXISTS "${ASTRA_CUDA_ROOT}")
  set(CUDA_TOOLKIT_ROOT_DIR "${ASTRA_CUDA_ROOT}" CACHE PATH "Legacy CUDA root" FORCE)
  set(CUDAToolkit_ROOT "${ASTRA_CUDA_ROOT}" CACHE PATH "CUDA toolkit root" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(Threads REQUIRED)

# Prime variables needed by a CUDA-enabled static OpenCVConfig. This lookup is
# quiet and optional: a normal distro OpenCV CPU package never enters it.
if(PERCEPTION_BUILD_GPU AND EXISTS "${ASTRA_CUDA_ROOT}/bin/nvcc")
  set(CUDA_USE_STATIC_CUDA_RUNTIME OFF CACHE BOOL "Use shared CUDA runtime" FORCE)
  find_package(CUDA QUIET)
  if(CUDA_FOUND)
    set(_PERCEPTION_NPPI_LIBRARIES)
    foreach(_NPPI_VAR
        CUDA_nppial_LIBRARY CUDA_nppicc_LIBRARY CUDA_nppicom_LIBRARY
        CUDA_nppidei_LIBRARY CUDA_nppif_LIBRARY CUDA_nppig_LIBRARY
        CUDA_nppim_LIBRARY CUDA_nppist_LIBRARY CUDA_nppisu_LIBRARY
        CUDA_nppitc_LIBRARY)
      if(DEFINED ${_NPPI_VAR} AND ${_NPPI_VAR} AND NOT "${${_NPPI_VAR}}" MATCHES "NOTFOUND")
        list(APPEND _PERCEPTION_NPPI_LIBRARIES "${${_NPPI_VAR}}")
      endif()
    endforeach()
    list(REMOVE_DUPLICATES _PERCEPTION_NPPI_LIBRARIES)
    if(_PERCEPTION_NPPI_LIBRARIES)
      set(CUDA_nppi_LIBRARY "${_PERCEPTION_NPPI_LIBRARIES}" CACHE STRING "OpenCV NPP compatibility" FORCE)
    endif()
  endif()
endif()
find_package(
  OpenCV REQUIRED
  COMPONENTS core imgproc imgcodecs highgui videoio)

# CPU backend: direct .pt via LibTorch. ament tetap memakai /usr/bin/python3,
# tetapi lokasi LibTorch boleh berasal dari interpreter PyTorch lain yang valid.
# Ini menghindari kasus Torch ada di user/venv tetapi target CPU diam-diam dilewati.
set(PERCEPTION_CPU_AVAILABLE OFF)
set(_PERCEPTION_TORCH_PYTHON "")
if(PERCEPTION_BUILD_CPU)
  set(_PERCEPTION_TORCH_PYTHON_CANDIDATES)
  if(PERCEPTION_TORCH_PYTHON)
    list(APPEND _PERCEPTION_TORCH_PYTHON_CANDIDATES "${PERCEPTION_TORCH_PYTHON}")
  endif()
  if(DEFINED ENV{PERCEPTION_TORCH_PYTHON} AND NOT "$ENV{PERCEPTION_TORCH_PYTHON}" STREQUAL "")
    list(APPEND _PERCEPTION_TORCH_PYTHON_CANDIDATES "$ENV{PERCEPTION_TORCH_PYTHON}")
  endif()
  list(APPEND _PERCEPTION_TORCH_PYTHON_CANDIDATES "${Python3_EXECUTABLE}" "/usr/bin/python3")
  find_program(_PERCEPTION_PATH_PYTHON3 NAMES python3)
  if(_PERCEPTION_PATH_PYTHON3)
    list(APPEND _PERCEPTION_TORCH_PYTHON_CANDIDATES "${_PERCEPTION_PATH_PYTHON3}")
  endif()
  list(REMOVE_DUPLICATES _PERCEPTION_TORCH_PYTHON_CANDIDATES)

  foreach(_PY IN LISTS _PERCEPTION_TORCH_PYTHON_CANDIDATES)
    if(NOT _PY OR NOT EXISTS "${_PY}")
      continue()
    endif()
    execute_process(
      COMMAND "${_PY}" -c "import torch; print(torch.utils.cmake_prefix_path)"
      OUTPUT_VARIABLE _TORCH_PREFIX_TEST
      RESULT_VARIABLE _TORCH_IMPORT_RESULT
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET)
    if(_TORCH_IMPORT_RESULT EQUAL 0 AND _TORCH_PREFIX_TEST)
      set(_PERCEPTION_TORCH_PYTHON "${_PY}")
      set(TORCH_CMAKE_PREFIX "${_TORCH_PREFIX_TEST}")
      break()
    endif()
  endforeach()

  if(_PERCEPTION_TORCH_PYTHON)
    execute_process(
      COMMAND "${_PERCEPTION_TORCH_PYTHON}" -c
        "import torch; f=getattr(torch,'compiled_with_cxx11_abi',None); a=f() if callable(f) else getattr(torch._C,'_GLIBCXX_USE_CXX11_ABI',None); print('1' if a is True else ('0' if a is False else 'unknown'))"
      OUTPUT_VARIABLE TORCH_CXX11_ABI
      RESULT_VARIABLE TORCH_ABI_RESULT
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET)

    if(TORCH_ABI_RESULT EQUAL 0 AND NOT TORCH_CXX11_ABI STREQUAL "0")
      list(PREPEND CMAKE_PREFIX_PATH "${TORCH_CMAKE_PREFIX}")
      set(_PERCEPTION_SAVED_MESSAGE_LOG_LEVEL "${CMAKE_MESSAGE_LOG_LEVEL}")
      set(CMAKE_MESSAGE_LOG_LEVEL ERROR)
      find_package(Torch QUIET)
      if(_PERCEPTION_SAVED_MESSAGE_LOG_LEVEL)
        set(CMAKE_MESSAGE_LOG_LEVEL "${_PERCEPTION_SAVED_MESSAGE_LOG_LEVEL}")
      else()
        unset(CMAKE_MESSAGE_LOG_LEVEL)
      endif()
      if(Torch_FOUND)
        set(PERCEPTION_CPU_AVAILABLE ON)
        separate_arguments(TORCH_CXX_FLAGS_LIST NATIVE_COMMAND "${TORCH_CXX_FLAGS}")
        message(STATUS "LibTorch CPU ditemukan dari ${_PERCEPTION_TORCH_PYTHON}; ABI=${TORCH_CXX11_ABI}; prefix=${TORCH_CMAKE_PREFIX}")
      endif()
    endif()
  endif()

  if(NOT PERCEPTION_CPU_AVAILABLE)
    if(_PERCEPTION_TORCH_PYTHON AND TORCH_CXX11_ABI STREQUAL "0")
      set(_CPU_REASON "PyTorch ditemukan di ${_PERCEPTION_TORCH_PYTHON}, tetapi CXX11_ABI=0 tidak kompatibel dengan ROS 2 Humble C++ pada target gabungan ini")
    elseif(_PERCEPTION_TORCH_PYTHON)
      set(_CPU_REASON "PyTorch ditemukan di ${_PERCEPTION_TORCH_PYTHON}, tetapi TorchConfig.cmake/LibTorch tidak dapat dimuat")
    else()
      set(_CPU_REASON "tidak ada interpreter Python yang dapat import torch dan menyediakan torch.utils.cmake_prefix_path")
    endif()
    if(PERCEPTION_REQUIRE_CPU)
      message(FATAL_ERROR
        "Backend CPU YOLOPv2 WAJIB tetapi tidak dapat dibangun: ${_CPU_REASON}.\n"
        "Cek dengan: /usr/bin/python3 -c \"import torch; print(torch.__version__); print(torch.utils.cmake_prefix_path); print(getattr(torch._C,'_GLIBCXX_USE_CXX11_ABI','unknown'))\"\n"
        "Jika Torch berada di interpreter lain, build dengan -DPERCEPTION_TORCH_PYTHON=/path/ke/python3")
    else()
      message(STATUS "Backend CPU YOLOPv2 dilewati: ${_CPU_REASON}; camera-only tetap dibangun.")
    endif()
  endif()
endif()

# perception_mode:=off still opens/publishes the camera for the GUI, but this
# target never loads a model and has no Torch/ONNX/TensorRT inference path.
add_executable(camera_only_node src/camera_only_node.cpp)
target_include_directories(
  camera_only_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(camera_only_node rclcpp sensor_msgs std_msgs)
target_link_libraries(camera_only_node ${OpenCV_LIBS} Threads::Threads)

set(PERCEPTION_RUNTIME_TARGETS camera_only_node)
if(PERCEPTION_CPU_AVAILABLE)
  add_executable(perception_cpu_node src/astra_yolop_cpu_pt_node.cpp)
  target_include_directories(
    perception_cpu_node PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS})
  ament_target_dependencies(
    perception_cpu_node
    rclcpp sensor_msgs std_msgs geometry_msgs nav_msgs visualization_msgs vision_msgs)
  target_link_libraries(perception_cpu_node ${TORCH_LIBRARIES} ${OpenCV_LIBS} Threads::Threads)
  target_compile_options(perception_cpu_node PRIVATE ${TORCH_CXX_FLAGS_LIST})

  # Embed the actual LibTorch runtime directories into the ELF.  This makes the
  # installed ROS executable self-contained with respect to the Python wheel
  # location and avoids relying on shell-specific LD_LIBRARY_PATH exports.
  execute_process(
    COMMAND "${_PERCEPTION_TORCH_PYTHON}" -c
      "import os,torch; print(os.path.join(os.path.dirname(torch.__file__),'lib'))"
    OUTPUT_VARIABLE TORCH_PYTHON_LIB_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
  set(TORCH_RUNTIME_DIRS)
  if(TORCH_PYTHON_LIB_DIR AND IS_DIRECTORY "${TORCH_PYTHON_LIB_DIR}")
    list(APPEND TORCH_RUNTIME_DIRS "${TORCH_PYTHON_LIB_DIR}")
  endif()
  foreach(_TORCH_LIB IN LISTS TORCH_LIBRARIES)
    if(IS_ABSOLUTE "${_TORCH_LIB}" AND EXISTS "${_TORCH_LIB}")
      get_filename_component(_TORCH_LIB_DIR "${_TORCH_LIB}" DIRECTORY)
      list(APPEND TORCH_RUNTIME_DIRS "${_TORCH_LIB_DIR}")
    endif()
  endforeach()
  list(REMOVE_DUPLICATES TORCH_RUNTIME_DIRS)
  set_target_properties(perception_cpu_node PROPERTIES
    BUILD_RPATH "${TORCH_RUNTIME_DIRS}"
    INSTALL_RPATH "${TORCH_RUNTIME_DIRS}"
    INSTALL_RPATH_USE_LINK_PATH TRUE)
  list(APPEND PERCEPTION_RUNTIME_TARGETS perception_cpu_node)
endif()

# Optional high-performance TensorRT backend. Missing CUDA/TensorRT produces a
# warning and a CPU-only package, not a configure failure.
set(PERCEPTION_GPU_AVAILABLE OFF)
if(PERCEPTION_BUILD_GPU)
  include(CheckLanguage)
  if(DEFINED CMAKE_CUDA_COMPILER AND "${CMAKE_CUDA_COMPILER}" MATCHES "NOTFOUND")
    unset(CMAKE_CUDA_COMPILER CACHE)
  endif()
  if(EXISTS "${ASTRA_CUDA_ROOT}/bin/nvcc" AND NOT DEFINED CMAKE_CUDA_COMPILER)
    set(CMAKE_CUDA_COMPILER "${ASTRA_CUDA_ROOT}/bin/nvcc" CACHE FILEPATH "Preferred NVCC")
  endif()
  check_language(CUDA)
  if(CMAKE_CUDA_COMPILER)
    enable_language(CUDA)
    set(CMAKE_CUDA_STANDARD 17)
    set(CMAKE_CUDA_STANDARD_REQUIRED ON)
    set(CMAKE_CUDA_EXTENSIONS OFF)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
      set(CMAKE_CUDA_ARCHITECTURES 87 CACHE STRING "CUDA architecture")
    elseif(NOT CMAKE_CUDA_ARCHITECTURES)
      set(CMAKE_CUDA_ARCHITECTURES 86 CACHE STRING "CUDA architecture")
    endif()

    find_package(CUDAToolkit QUIET)
    find_path(
      TENSORRT_INCLUDE_DIR NvInfer.h
      HINTS /usr/include/aarch64-linux-gnu /usr/include/x86_64-linux-gnu /usr/include)
    find_library(
      NVINFER_LIBRARY nvinfer
      HINTS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/lib)

    if(CUDAToolkit_FOUND AND TENSORRT_INCLUDE_DIR AND NVINFER_LIBRARY)
      set(PERCEPTION_GPU_AVAILABLE ON)
    else()
      message(STATUS
        "CUDA compiler ditemukan, tetapi CUDAToolkit/TensorRT tidak lengkap; "
        "backend GPU dilewati tanpa menggagalkan build.")
    endif()
  else()
    message(STATUS
      "CUDA compiler tidak ditemukan. perception_cpu_node tetap dibangun; backend GPU dilewati.")
  endif()
endif()

if(PERCEPTION_GPU_AVAILABLE)
  # Keep the historic executable name for backward compatibility.
  add_executable(perception_node src/astra_yolop_gpu_node.cpp)
  set_source_files_properties(src/astra_yolop_gpu_node.cpp PROPERTIES LANGUAGE CUDA)
  set_target_properties(
    perception_node PROPERTIES
    CUDA_SEPARABLE_COMPILATION OFF
    CUDA_RESOLVE_DEVICE_SYMBOLS OFF
    POSITION_INDEPENDENT_CODE ON
    CUDA_ARCHITECTURES "${CMAKE_CUDA_ARCHITECTURES}")
  target_include_directories(
    perception_node PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${TENSORRT_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${CUDAToolkit_INCLUDE_DIRS})
  target_compile_options(
    perception_node PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-deprecated-declarations>
    $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>
    $<$<COMPILE_LANGUAGE:CUDA>:--extended-lambda>
    $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=-Wall,-Wextra,-Wno-deprecated-declarations>
    $<$<AND:$<CONFIG:Release>,$<COMPILE_LANGUAGE:CXX>>:-O3>
    $<$<AND:$<CONFIG:Release>,$<COMPILE_LANGUAGE:CXX>>:-DNDEBUG>
    $<$<AND:$<CONFIG:Release>,$<COMPILE_LANGUAGE:CUDA>>:-O3>
    $<$<AND:$<CONFIG:Release>,$<COMPILE_LANGUAGE:CUDA>>:-DNDEBUG>)
  if(ASTRA_YOLOP_FAST_MATH)
    target_compile_options(perception_node PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--use_fast_math>)
  endif()
  ament_target_dependencies(
    perception_node
    rclcpp sensor_msgs std_msgs geometry_msgs nav_msgs visualization_msgs vision_msgs)
  target_link_libraries(
    perception_node ${NVINFER_LIBRARY} CUDA::cudart ${OpenCV_LIBS} Threads::Threads)
  list(APPEND PERCEPTION_RUNTIME_TARGETS perception_node)
  message(STATUS
    "Perception backends: CPU=${PERCEPTION_CPU_AVAILABLE} GPU=ON (CUDA ${CUDAToolkit_VERSION}, TensorRT ${NVINFER_LIBRARY})")
else()
  message(STATUS "Perception backends: CPU=${PERCEPTION_CPU_AVAILABLE} GPU=OFF; camera-only=ON")
endif()

if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(perception_safety_core_self_check test/perception_safety_core_self_check.cpp)
  target_include_directories(perception_safety_core_self_check PRIVATE include)
  add_test(NAME perception_safety_core_self_check COMMAND perception_safety_core_self_check)
  add_test(
    NAME perception_backend_contract_self_check
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/perception_backend_contract_self_check.py)
  add_test(
    NAME perception_cpu_pt_runtime_self_check
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/perception_cpu_pt_runtime_self_check.py)
  add_test(
    NAME perception_runtime_robustness_self_check
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/perception_runtime_robustness_self_check.py)
endif()

add_executable(bbox_static_test_logger src/bbox_static_test_logger.cpp)
target_compile_features(bbox_static_test_logger PRIVATE cxx_std_17)
ament_target_dependencies(bbox_static_test_logger rclcpp std_msgs)

add_executable(fit_bbox_calibration src/fit_bbox_calibration.cpp)
target_compile_features(fit_bbox_calibration PRIVATE cxx_std_17)

add_executable(converter_pt_to_onnx_to_engine src/converter_pt_to_onnx_to_engine.cpp)
target_compile_features(converter_pt_to_onnx_to_engine PRIVATE cxx_std_17)

install(
  TARGETS
    ${PERCEPTION_RUNTIME_TARGETS}
    bbox_static_test_logger
    fit_bbox_calibration
    converter_pt_to_onnx_to_engine
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config rviz models DESTINATION share/${PROJECT_NAME})
install(FILES CPU_GPU_BACKEND_GUIDE.md DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS tools/perception_backend_preflight.py tools/download_yolopv2.py tools/check_libtorch_cpu.py
  DESTINATION lib/${PROJECT_NAME})
install(
  PROGRAMS tools/yolopv2_official_export_v19_4.py
  DESTINATION lib/${PROJECT_NAME})

# Model TorchScript CPU boleh berada di source/share package. TensorRT engine
# Jetson tetap eksternal karena bersifat platform-specific. Runtime memvalidasi
# path sebelum perception dijalankan.
ament_package()
