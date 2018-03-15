/* 
* Arduino ROS Node for Winebot 
* Author: David Robinson
*/

#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

ros::NodeHandle nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

double x = 1.0;
double y = 0.0;
double theta = 1.57;

double left_odom = 0;
double left_turn = 0;
double right_odom = 0;
double right_turn = 0; 

char base_link[] = "/base_link";
char odom[] = "/odom";

long int lastPubTime = 0;
int dataRate  = 100; // ms or 10Hz

#include <Encoder.h>
Encoder myEnc(9, 10);

bool debug = false; // Switch Serial Print / ROS Publisher

void setup() {
  if (debug) {
    Serial.begin(9600);
  }
  else {
    nh.initNode();
    broadcaster.init(nh);
  }
}

void loop() {

  read_sensors();
  
  if (debug) { // Serial Print
      Serial.print("\t| LEFT | Odom: ");
      Serial.print(left_odom);
      Serial.print("\tTurn: ");
      Serial.print(left_turn);
      Serial.print("\t| RIGHT | Odom: ");
      Serial.print(right_odom);
      Serial.print("\tTurn: ");
      Serial.print(right_turn);
      Serial.println("");
  }
  else { // ROS Publisher

    long int time = millis();
    double dt = (time - lastPubTime);
    if (dt > dataRate) { // attenuate rate
  
      // drive in a circle
      double dx = 0.2;
      double dtheta = 0.18;
      x += cos(theta)*dx*0.1;
      y += sin(theta)*dx*0.1;
      theta += dtheta*0.1;
      if(theta > 3.14)
        theta=-3.14;
        
      // tf odom->base_link
      t.header.frame_id = odom;
      t.child_frame_id = base_link;
      
      t.transform.translation.x = x;
      t.transform.translation.y = y;
      
      t.transform.rotation = tf::createQuaternionFromYaw(theta);
      t.header.stamp = nh.now();
      
      broadcaster.sendTransform(t);
      nh.spinOnce();
  
    }
  }
}

void readSensor() {

  left_odom = left_odom + 1;
  left_turn = left_turn + 1;
  right_odom = right_odom + 1;
  right_turn = right_turn + 1;
  
}




