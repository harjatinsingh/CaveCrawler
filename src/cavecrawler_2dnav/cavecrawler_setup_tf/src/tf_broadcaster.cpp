#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

void publish_tf()
{
  static tf::TransformBroadcaster br;
  br.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.5, 0.0, 0.5)), ros::Time::now(),"base_link", "camera"));
  
  static tf::TransformBroadcaster br2;
  br2.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.9, 0.0, 1.2)), ros::Time::now(),"base_link", "velodyne"));

}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){

    publish_tf();


    r.sleep();
  }
}