cmake_minimum_required(VERSION 2.8.3)
project(tabv_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  tf
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(tabv_controller src/tabv_controller.cpp)
target_link_libraries(tabv_controller ${catkin_LIBRARIES})

add_executable(ground_sim src/ground_sim.cpp)
target_link_libraries(ground_sim ${catkin_LIBRARIES})
