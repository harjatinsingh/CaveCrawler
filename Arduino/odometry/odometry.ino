/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup()
{
  Serial.begin(115200);
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}

///* 
//* Arduino ROS Node for Winebot 
//* Author: David Robinson
//*/
//
//#include <ros.h>
//#include <std_msgs/Float32MultiArray.h>
//
//#include <Encoder.h>
//Encoder myEnc(9, 10);
//
//bool debug = false; // Change this to begin ROS coms
//
//void setup() {
//  if (debug) {
//    Serial.begin(115200);
//  }
//  else setupROS();
//}
//
//float left_odom = 0;
//float left_turn = 0;
//float right_odom = 0;
//float right_turn = 0; 
//
//void loop() {
// 
////  long newPosition = myEnc.read();
//  
//  left_odom = left_odom + 1;
//  left_turn = left_turn + 1;
//  right_odom = right_odom + 1;
//  right_turn = right_turn + 1;
//  
//  if (debug) {
//      Serial.print("\t| LEFT | Odom: ");
//      Serial.print(left_odom);
//      Serial.print("\tTurn: ");
//      Serial.print(left_turn);
//      Serial.print("\t| RIGHT | Odom: ");
//      Serial.print(right_odom);
//      Serial.print("\tTurn: ");
//      Serial.print(right_turn);
//      Serial.println("");
//  }
//  else runROS();
//}
//
//// ROS Functions
//
//ros::NodeHandle nh;
//static int messageLength = 4;
//std_msgs::Float32MultiArray odometry;
//ros::Publisher odometry_pub("odometry", &odometry);
//
//void setupROS() {
//  nh.initNode();
//  setupMsg(odometry, odometry_pub);
//}
//
//void setupMsg(std_msgs::Float32MultiArray &msg, ros::Publisher &pub) {
//  msg.layout.dim =
//    (std_msgs::MultiArrayDimension *)
//    malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
//  msg.layout.dim[0].label = "height";
//  msg.layout.dim[0].size = messageLength;
//  msg.layout.dim[0].stride = 1;
//  msg.layout.data_offset = 0;
//  msg.data = (float *)malloc(sizeof(int) * 8);
//  msg.data_length = messageLength;
//  nh.advertise(pub);
//}
//
//// Data Rate Attenuation
//long int lastPubTime = 0;
//int dataRate  = 100; // ms or 10Hz
//
//void runROS() {
//  long int time = millis();
//  double dt = (time - lastPubTime);
//  if (dt > dataRate) {
//    sendMsg(odometry, odometry_pub);
//    lastPubTime = time;
//  }
//  nh.spinOnce();  
//}
//
//void sendMsg(std_msgs::Float32MultiArray &msg, ros::Publisher &pub) {
//  msg.data[0] = left_odom;
//  msg.data[1] = left_turn;
//  msg.data[2] = right_odom;
//  msg.data[3] = right_turn;
//  pub.publish(&msg);
//}

