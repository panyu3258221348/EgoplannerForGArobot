cmake_minimum_required(VERSION 2.8.3)
project(gazebo_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  gazebo_ros
  pcl_ros
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp gazebo_ros pcl_ros
)

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(gazebo_controller src/gazebo_controller.cpp)
target_link_libraries(gazebo_controller ${catkin_LIBRARIES})
