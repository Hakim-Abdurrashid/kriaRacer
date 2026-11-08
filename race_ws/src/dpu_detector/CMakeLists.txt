cmake_minimum_required(VERSION 3.8)
project(dpu_detector)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  add_compile_options(-mcpu=cortex-a53)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# Vitis AI 3.5 runtime (installed by setup/install_dpu_vai35.sh). Without it nothing is built, so the
# workspace still builds on machines without a DPU.
find_path(VART_INCLUDE vart/runner_ext.hpp PATHS /usr/include)
find_library(VART_RUNNER vart-runner PATHS /usr/lib)
find_library(XIR_LIB xir PATHS /usr/lib)
find_library(UNILOG_LIB unilog PATHS /usr/lib)
if(NOT VART_INCLUDE OR NOT VART_RUNNER OR NOT XIR_LIB)
  message(WARNING "dpu_detector: Vitis AI runtime (VART) not found - skipping the build")
  ament_package()
  return()
endif()

# VART is built against glog 0.5 (shipped in /usr/lib by the Vitis AI zip); Ubuntu's glog 0.4 has the
# same soname. Link the 0.5 file directly and search /usr/lib first so the loader picks it.
find_file(GLOG05 libglog.so.0.5.0 PATHS /usr/lib NO_DEFAULT_PATH)
set(CMAKE_BUILD_RPATH /usr/lib)
set(CMAKE_INSTALL_RPATH /usr/lib)

add_library(yolo_dpu src/yolo_dpu.cpp)
target_include_directories(yolo_dpu PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> ${VART_INCLUDE})
target_include_directories(yolo_dpu SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(yolo_dpu ${VART_RUNNER} ${XIR_LIB} ${UNILOG_LIB} ${OpenCV_LIBS})
if(GLOG05)   # keep it as a direct dependency of every binary (Ubuntu links with --as-needed)
  target_link_libraries(yolo_dpu "-Wl,--no-as-needed" ${GLOG05} "-Wl,--as-needed")
endif()

add_executable(dpu_detect_images src/dpu_detect_images.cpp)
target_link_libraries(dpu_detect_images yolo_dpu)

add_executable(dpu_detector_node src/dpu_detector_node.cpp)
target_link_libraries(dpu_detector_node yolo_dpu)
ament_target_dependencies(dpu_detector_node rclcpp sensor_msgs std_msgs)

install(TARGETS dpu_detect_images dpu_detector_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
