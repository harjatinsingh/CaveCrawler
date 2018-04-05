#include <AS5045_scorpion.h>
#include <AS5045.h>

float left_dist;
float right_dist;
float left_past_dist;
float right_past_dist;
long dt_micros = 0;
long past_micros = 0;
int ODOM_DELAY_MS = 20;
float base_width = 1.10515;

//scorpion_wheels scorpion(7, 8, 5, 6, 101.6, 1.10515);//CSn, Clk, left, right, wheel diam, base_width
encoder right_wheel(6, 7, 5, 101.6, true);
encoder left_wheel(9, 10, 8, 101.6, false);

void setup() {
  Serial.begin(115200);
//  scorpion.setup_rotary_encoders();
//  scorpion.calibrate_rotary_encoders();
  right_wheel.setup_rotary_encoder();
  right_wheel.calibrate_rotary_encoder();
  left_wheel.setup_rotary_encoder();
  left_wheel.calibrate_rotary_encoder();
}

void loop() {
   float right_dist = right_wheel.rotary_data();
   float left_dist = left_wheel.rotary_data();
   dt_micros = micros() - past_micros;
   float right_v = (right_dist - right_past_dist) * 1000 / dt_micros;
   float left_v = (left_dist - left_past_dist) * 1000 / dt_micros;
   right_past_dist = right_dist;
   left_past_dist = left_dist;
   past_micros = micros();

   float vx = (left_v + right_v) /2;
   float vth = (right_v - left_v)/ base_width;
   Serial.print(left_dist);
   Serial.print("\t");
   Serial.print(right_dist);
   Serial.print("\t");
   Serial.print(left_v);
   Serial.print("\t");
   Serial.print(right_v);
   Serial.print("\t");
   Serial.print(vx);
   Serial.print("\t");
   Serial.println(vth);                                        
   delay(ODOM_DELAY_MS);

//   scorpion.dist_data();
//   dt_micros = micros() - past_micros;
//   left_dist = scorpion.left_dist;
//   right_dist = scorpion.right_dist;
//   float left_v = (left_dist - right_past_dist) * 1000 / dt_micros;
//   float right_v = (right_dist - right_past_dist) * 1000 / dt_micros;
//   left_past_dist = left_dist;
//   right_past_dist = right_dist;
//   past_micros = micros();
//   Serial.print(left_dist);
//   Serial.print("\t");
//   Serial.print(right_dist);
//   Serial.print("\t");
//   Serial.print(left_v);
//   Serial.print("\t");
//   Serial.println(right_v);
//   delay(ODOM_DELAY_MS);

//  scorpion.get_vx_vth();
//  
//  Serial.print(scorpion.left_dist);
//  Serial.print("\t");
//  Serial.print(scorpion.right_dist);
//  Serial.print("\t");
//  Serial.print(scorpion.vx);
//  Serial.print("\t");
//  Serial.println(scorpion.vth);
}
