/*
  Arduino ROS Node for Winebot
  Author: David Robinson
*/

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <Encoder.h>

ros::NodeHandle nh;

static int messageLength = 4;
std_msgs::Int16MultiArray odometry;
ros::Publisher odometry_pub("odometry", &odometry);

Encoder left_enc(2, 5);
Encoder right_enc(4, 3);
long int left_odom_prev = 0;
int left_odom = 0;
long int right_odom_prev = 0;
int right_odom = 0;

static int left_turn_pin = A1;
static int right_turn_pin = A0;
int left_turn = 0;
int right_turn = 0;

float old_time = 0;

bool debug = false; // Switch Serial Print / ROS Publisher

void setup() {

  if (debug) {
    Serial.begin(9600);
  }
  else {
    nh.initNode();
    setupMsg(odometry, odometry_pub);
  }
}

void setupMsg(std_msgs::Int16MultiArray &msg, ros::Publisher &pub) {
  msg.layout.dim =
    (std_msgs::MultiArrayDimension *)
    malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  msg.layout.dim[0].label = "height";
  msg.layout.dim[0].size = messageLength;
  msg.layout.dim[0].stride = 1;
  msg.layout.data_offset = 0;
  msg.data = (int16_t *)malloc(sizeof(int) * 8);
  msg.data_length = messageLength;
  nh.advertise(pub);
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
    sendMsg(odometry, odometry_pub);
    nh.spinOnce();
  }
}

void read_sensors() {

  // report delta in odom
  long int left_new = left_enc.read();
  long int right_new = right_enc.read();

  float time_now = millis();

  left_odom = (left_new - left_odom_prev);
  right_odom = (right_new - right_odom_prev);

  left_odom_prev = left_new;
  right_odom_prev = right_new;

  old_time = time_now;

  // get steering voltage 1.3 to 2.3V
  left_turn = analogRead(left_turn_pin);
  right_turn = analogRead(right_turn_pin);

}

void sendMsg(std_msgs::Int16MultiArray &msg, ros::Publisher &pub) {
  msg.data[0] = left_odom;
  msg.data[1] = right_odom;
  msg.data[2] = left_turn;
  msg.data[3] = right_turn;
  pub.publish(&msg);
}



