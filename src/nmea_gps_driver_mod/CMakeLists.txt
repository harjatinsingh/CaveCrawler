cmake_minimum_required(VERSION 2.8.3)
project(nmea_gps_driver_mod)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs 
    sensor_msgs
    geometry_msgs

    rospy
    message_filters
    geographic_msgs
    geodesy
    tf
  )

catkin_package(
   INCLUDE_DIRS 
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp sensor_msgs std_msgs sensor_msgs std_msgs geometry_msgs message_filters geographic_msgs geodesy tf
)

include_directories(
  ${catkin_INCLUDE_DIRS} 
)

#add_executable(navfix_to_utm_tf_converter src/navfix_to_utm_tf_converter.cpp)
#target_link_libraries(navfix_to_utm_tf_converter   ${catkin_LIBRARIES} )


#install(TARGETS navfix_to_utm_tf_converter  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
#install(PROGRAMS src/nmea_gps_driver_mod.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(PROGRAMS src/gpsbabel_nmea_publish.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})




