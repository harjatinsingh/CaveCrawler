cmake_minimum_required(VERSION 2.8.3)
project(cavecrawler_setup_tf)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})


# Build the tf_broadcaster executable
add_executable(tf_broadcaster src/tf_broadcaster.cpp)
# Add a build order dependency on geometry_msgs
# This ensures that geometry_msgs' msg headers are built before your executable

if(geometry_msgs_EXPORTED_TARGETS)
  add_dependencies(tf_broadcaster ${geometry_msgs_EXPORTED_TARGETS})
endif()

# Link against the catkin libraries
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})


