#######################################################################################
cmake_minimum_required(VERSION 3.8)
#######################################################################################
project(ros2_heartbeat)
set(ROS2_HEARTBEAT_TARGET ${PROJECT_NAME})
#######################################################################################
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
#######################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
#######################################################################################
add_library(${ROS2_HEARTBEAT_TARGET} STATIC
  src/Monitor.cpp
)
#######################################################################################
target_include_directories(${ROS2_HEARTBEAT_TARGET} PRIVATE include)
#######################################################################################
target_compile_features(${ROS2_HEARTBEAT_TARGET} PRIVATE cxx_std_17)
#######################################################################################
ament_target_dependencies(${ROS2_HEARTBEAT_TARGET} rclcpp std_msgs ${THIS_PACKAGE_INCLUDE_DEPENDS})
#######################################################################################
ament_export_targets(${ROS2_HEARTBEAT_TARGET}_export HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
#######################################################################################
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${ROS2_HEARTBEAT_TARGET}
  EXPORT ${ROS2_HEARTBEAT_TARGET}_export
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
#######################################################################################
ament_package()
#######################################################################################
