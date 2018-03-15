cmake_minimum_required(VERSION 2.8.3)
project(cavecrawler_planning)

# add_definitions(-std=c++11) THIS WAS CAUSING SEGFAULT WITH CERTAIN PCL STUFF

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
)

catkin_package()



