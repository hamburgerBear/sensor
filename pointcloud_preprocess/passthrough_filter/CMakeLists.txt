cmake_minimum_required(VERSION 3.8)
project(passthrough_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(passthrough_filter_node src/passthrough_filter_node.cpp)
target_include_directories(passthrough_filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
ament_target_dependencies(passthrough_filter_node
  pcl_conversions
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros)
target_link_libraries(passthrough_filter_node ${PCL_LIBRARIES})

install(
  DIRECTORY include/
  DESTINATION include)

install(
  DIRECTORY launch params rviz
  DESTINATION share/${PROJECT_NAME})

install(
  TARGETS passthrough_filter_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
