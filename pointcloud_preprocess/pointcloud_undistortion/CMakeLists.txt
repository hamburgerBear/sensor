cmake_minimum_required(VERSION 3.8)
project(pointcloud_undistortion)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(pointcloud_undistortion_node
  src/imu_process.cpp
  src/pointcloud_undistortion_node.cpp
  src/preprocess.cpp)
target_include_directories(pointcloud_undistortion_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
ament_target_dependencies(pointcloud_undistortion_node
  pcl_conversions
  rcl_interfaces
  rclcpp
  sensor_msgs)
target_compile_options(pointcloud_undistortion_node PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:-O3>
  $<$<AND:$<COMPILE_LANGUAGE:CXX>,$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>>>:-march=native>)
target_link_libraries(pointcloud_undistortion_node ${PCL_LIBRARIES})

install(
  DIRECTORY include/
  DESTINATION include)

install(
  DIRECTORY launch params rviz
  DESTINATION share/${PROJECT_NAME})

install(
  TARGETS pointcloud_undistortion_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
