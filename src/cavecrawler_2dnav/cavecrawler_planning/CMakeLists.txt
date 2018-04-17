cmake_minimum_required(VERSION 2.8.3)
project(cavecrawler_planning)

add_definitions(-std=c++11) 

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  costmap_2d
  move_base
  move_base_msgs
  actionlib
  nav_core
  tf
  nav_msgs
  pluginlib
  cmake_modules
  visualization_msgs
  #ompl
)

#find_package(OMPL)
find_package(ompl)

catkin_package(
  CATKIN_DEPENDS roscpp
  DEPENDS system_lib ompl
)

#include_directories(
 # ${catkin_INCLUDE_DIRS}
  #${OMPL_INCLUDE_DIRS}
#)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${ompl_INCLUDE_DIRS}
)

#add_executable( rrt_global_planner src/rrt_global_planner.cpp)
add_library(rrt_global_planner_lib src/rrt_global_planner.cpp)
target_link_libraries(rrt_global_planner_lib ${catkin_LIBRARIES} ${OMPL_LIBRARIES} base_local_planner )


