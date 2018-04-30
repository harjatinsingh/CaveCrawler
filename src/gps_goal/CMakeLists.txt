cmake_minimum_required(VERSION 2.8.3)
project(gps_goal)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(gps_waypoint src/gps_waypoint.cpp)
target_link_libraries(gps_waypoint ${catkin_LIBRARIES})

