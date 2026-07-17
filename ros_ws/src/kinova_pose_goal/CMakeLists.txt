cmake_minimum_required(VERSION 3.16)
project(kinova_pose_goal)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(pose_tracking_node src/pose_tracking_node.cpp)
target_compile_features(pose_tracking_node PUBLIC cxx_std_17)
ament_target_dependencies(
  pose_tracking_node
  geometry_msgs
  moveit_msgs
  rclcpp
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
)

add_executable(fake_target_frame_node src/fake_target_frame_node.cpp)
target_compile_features(fake_target_frame_node PUBLIC cxx_std_17)
ament_target_dependencies(
  fake_target_frame_node
  geometry_msgs
  rclcpp
  std_srvs
  tf2
  tf2_ros
)

install(
  TARGETS
    pose_tracking_node
    fake_target_frame_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
