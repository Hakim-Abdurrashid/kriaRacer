cmake_minimum_required(VERSION 3.8)
project(race_core)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  add_compile_options(-mcpu=cortex-a53)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)
find_package(OpenMP REQUIRED)

# ROS-free modules (unit-tested without ROS)
add_library(race_core_lib
  src/follow_gap.cpp src/occupancy_map.cpp src/particle_filter.cpp src/raceline.cpp
  src/controller.cpp src/opponent.cpp src/overtake.cpp)
target_include_directories(race_core_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(race_core_lib yaml-cpp ${OpenCV_LIBS} OpenMP::OpenMP_CXX)
target_include_directories(race_core_lib SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})

add_executable(race_core_node src/race_core_node.cpp)
target_link_libraries(race_core_node race_core_lib)
ament_target_dependencies(race_core_node rclcpp sensor_msgs ackermann_msgs nav_msgs geometry_msgs
  std_msgs std_srvs tf2_ros)

add_executable(race_sim src/race_sim.cpp)
target_link_libraries(race_sim race_core_lib)
ament_target_dependencies(race_sim rclcpp sensor_msgs ackermann_msgs nav_msgs geometry_msgs
  std_msgs tf2_ros)

add_executable(ftg_replay src/ftg_replay.cpp)
target_link_libraries(ftg_replay race_core_lib)

install(TARGETS race_core_node race_sim ftg_replay DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS tools/make_raceline.py tools/ftg_parity.py tools/lap_report.py tools/sim_eval.sh
  tools/calib_report.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_follow_gap test/test_follow_gap.cpp)
  target_link_libraries(test_follow_gap race_core_lib)
  ament_add_gtest(test_localization test/test_localization.cpp)
  target_link_libraries(test_localization race_core_lib)
  ament_add_gtest(test_overtake test/test_overtake.cpp)
  target_link_libraries(test_overtake race_core_lib)
endif()

ament_package()
