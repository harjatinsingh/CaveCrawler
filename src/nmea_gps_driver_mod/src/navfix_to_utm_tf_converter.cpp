#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <sensor_msgs/NavSatFix.h>
#include <geodesy/utm.h>
#include <sstream>
#include <tf/transform_broadcaster.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Vector3.h>

#include <cmath>
#include <limits>
#include <deque>
using namespace std;


string frame_origin, frame_child;
ros::Publisher curr_marker_pub;
ros::Publisher traj_marker_pub;
ros::Publisher gps_vel_pub;
visualization_msgs::Marker points;

unsigned int vel_filter_window = 15;
unsigned int del_time_max = 10.0;
deque<double> easting_filter;
deque<double> northing_filter;
deque<double> altitude_filter;
deque<double> time_filter;

/* ----------------------------------------------------- */
void navfixCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {

  double latitude =  msg->latitude;
  double longitude =  msg->longitude;
  double altitude = msg->altitude;

  ros::Time stamp = msg->header.stamp;
  int seq = msg->header.seq;

  if(isnan(altitude)) {
    altitude = 0;
  }

  geographic_msgs::GeoPoint geopoint = geodesy::toMsg (latitude, longitude, altitude); 	
  geodesy::UTMPoint utmpoint = geodesy::UTMPoint(geopoint);
  fprintf(stderr, "%d - %d.%06d)(%f) Receieved Lat/Long:, %0.9f %0.9f. Alt: %f. UTM: %d. %0.9f, %0.9f, %f\n", seq, stamp.sec, stamp.nsec, stamp.toSec(), latitude, longitude, altitude, utmpoint.zone, utmpoint.easting, utmpoint.northing, utmpoint.altitude);
  fflush(stderr);


  //filter the measurements to get a velocity and gps course measurement
  easting_filter.push_back(utmpoint.easting);
  northing_filter.push_back(utmpoint.northing);
  altitude_filter.push_back(utmpoint.altitude);
  time_filter.push_back(stamp.toSec());

  while(easting_filter.size() > vel_filter_window) {
    easting_filter.pop_front();
    northing_filter.pop_front();
    altitude_filter.pop_front();
    time_filter.pop_front();
  }

  double del_time = time_filter.back()-time_filter.front();
  double vel_easting, vel_northing, vel_altitude, vel_norm;
  if(del_time > del_time_max) {
    vel_easting = 0;
    vel_northing = 0;
    vel_altitude = 0;
  } else {
    vel_easting = (easting_filter.back()-easting_filter.front())/(del_time);
    vel_northing = (northing_filter.back()-northing_filter.front())/(del_time);
    vel_altitude = (altitude_filter.back()-altitude_filter.front())/(del_time);
  }
 
  geometry_msgs::Vector3Stamped gps_vel_msg;
  gps_vel_msg.header = msg->header;
  gps_vel_msg.vector.x = vel_easting;
  gps_vel_msg.vector.y = vel_northing;
  gps_vel_msg.vector.z = 0.0f;
  gps_vel_pub.publish(gps_vel_msg);

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(utmpoint.easting, utmpoint.northing, utmpoint.altitude) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, stamp, frame_origin, frame_child));


  //if visualization marker gets "crazy big" then clear them
  if(points.points.size() > 1000000) {
    points.points.clear();
  }
  points.header = msg->header;
  points.header.stamp = msg->header.stamp;
  points.action = visualization_msgs::Marker::ADD;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  points.scale.x = 1.5;
  points.scale.y = 1.5;
  points.color.r = 1.0f;
  points.color.a = 1.0;
  points.header.frame_id = frame_origin;
  geometry_msgs::Point p;
  p.x = utmpoint.easting;
  p.y = utmpoint.northing;
  p.z = utmpoint.altitude;



  points.points.push_back(p);

  traj_marker_pub.publish(points);

  visualization_msgs::Marker curr_point;
  curr_point.header = msg->header;
  curr_point.header.stamp = msg->header.stamp;
  curr_point.action = visualization_msgs::Marker::ADD;
  curr_point.id = 0;
  curr_point.type = visualization_msgs::Marker::POINTS;
  curr_point.scale.x = 4.0;
  curr_point.scale.y = 4.0;
  curr_point.color.g= 1.0f;
  curr_point.color.a = 1.0;
  curr_point.header.frame_id = frame_origin;
  geometry_msgs::Point curr_p;
  curr_p.x = utmpoint.easting;
  curr_p.y = utmpoint.northing;
  curr_p.z = utmpoint.altitude;
  curr_point.points.push_back(curr_p);
  curr_marker_pub.publish(curr_point);
 
}

/* ----------------------------------------------------- */
int main(int argc, char **argv) {

  ros::init(argc, argv, "navfix_to_utm_tf_converter");

  ros::NodeHandle nh_param("~");
  nh_param.getParam("frame_origin", frame_origin);
  nh_param.getParam("frame_child", frame_child);
  fprintf(stderr, "The transform frame_child : %s\n", frame_child.c_str());
  fprintf(stderr, "The transform frame_origin : %s\n",  frame_origin.c_str());

  /* */
  ros::NodeHandle nh;
  ros::Subscriber navfix_sub =  nh.subscribe("/fix", 30, navfixCallback);
  traj_marker_pub = nh.advertise<visualization_msgs::Marker>("gps_traj_marker", 10);
  curr_marker_pub = nh.advertise<visualization_msgs::Marker>("gps_curr_marker", 10);

  gps_vel_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/gps_vel", 10);

  ros::spin();
 
  return 0;
}

