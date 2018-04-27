/*
  Arduino ROS Node for Winebot
  Author: David Robinson
*/

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <Encoder.h>

ros::NodeHandle nh;

geometry_msgs::QuaternionStamped odometry;
ros::Publisher odometry_pub("raw_odometry", &odometry);

Encoder left_enc(2, 5);
Encoder right_enc(4, 3);
float left_odom_prev = 0.0f;
float left_odom = 0.0f;
float right_odom_prev = 0.0f;
float right_odom = 0.0f;

static int left_turn_pin = A1;
static int right_turn_pin = A0;
float left_turn = 0.0f;
float right_turn = 0.0f;

long int old_time = 0;

bool debug = false; // Switch Serial Print / ROS Publisher

void setup() {

  if (debug) {
    Serial.begin(9600);
    Serial.println("Serial Connection @ 9600 Baud");
  }
  else {
    nh.getHardware()->setBaud(9600);
    nh.initNode();
    nh.advertise(odometry_pub);
  }
}

// Data Rate Attenuation
long int lastPubTime = 0;
int dataRate  = 20; // ms or 50Hz

void loop() {

  long int time = millis();
  double dt = (time - lastPubTime);
  if (dt > dataRate) {

    read_sensors(dt);

    if (debug) { // Serial Print
      Serial.print("TIME:   | ");
      Serial.print(dt/1000.0f);
      Serial.print("\t| LEFT | Odom: ");
      Serial.print(left_odom);
      Serial.print("\tTurn: ");
      Serial.print(left_turn);
      Serial.print("\t| RIGHT | Odom: ");
      Serial.print(-right_odom);
      Serial.print("\tTurn: ");
      Serial.print(right_turn);
      Serial.println("");
    }
    else { // ROS Publisher
      odometry.quaternion.x = left_odom;
      odometry.quaternion.y = -right_odom;
      odometry.quaternion.z = left_turn;
      odometry.quaternion.w = right_turn;
      odometry.header.stamp = nh.now();
      odometry_pub.publish(&odometry);
      nh.spinOnce();
    }
    // update time
    lastPubTime = time;
  }
}

void read_sensors(double dt) {

  // report delta in odom
  float left_new = left_enc.read();
  float right_new = right_enc.read();

  left_odom = (left_new - left_odom_prev)/dt;
  right_odom = (right_new - right_odom_prev)/dt;

  left_odom_prev = left_new;
  right_odom_prev = right_new;

  // get steering voltage 1.3 to 2.3V
  left_turn = analogRead(left_turn_pin);
  right_turn = analogRead(right_turn_pin);
}


