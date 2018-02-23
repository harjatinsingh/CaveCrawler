
#include "HID-Project.h"

const int pinLed = LED_BUILTIN;
const int pinButton = 9;

String inString = "";

bool flag = 0;
bool forward_flag = 1;

int bits_recieved = 0;

//Variables for the Gamepad API
float robotSpeed = 0;
float wheelAngle = 0;
float maxSpeed = 50;
int mode = 0;
int forw_direction = 0;
int reset_amps = 0;

int m_speed =0;
int r_angle = 0;
int f_speed = 0;

void setup() {
  
  Serial1.begin(115200); //This is the UART, pipes to sensors attached to board
  
  pinMode(pinLed, OUTPUT);
  pinMode(pinButton, INPUT_PULLUP);

  // Sends a clean report to the host. This is important on any Arduino type.
  Gamepad.begin();
  
  //Serial1.print("$");  // Print three times individually
  //Serial1.print("$");
  //Serial1.print("$");  // Enter command mode
  //delay(100);  // Short delay, wait for the Mate to send back CMD
  //Serial1.println("W");  // Temporarily Change the baudrate to 9600, no parity
  delay(100);
  // 115200 can be too fast at times for NewSoftSerial to relay the data reliably
  //Serial1.begin(9600);  // Start bluetooth serial at 9600
  //Serial1.println("W");
  //Gamepad.press(5);
  //Gamepad.write();  
  //delay(20);
  //Gamepad.releaseAll();
}

void loop() {

/*    
  if (Serial1.available() > 0) {
    received_pc = Serial1.read();
    digitalWrite(pinLed, HIGH);
    delay(100);
    digitalWrite(pinLed, LOW);
    received_pc = (long) received_pc;
    int aux = received_pc/10;
    Serial1.print(aux, DEC);
  }*/

  while (Serial1.available() > 0) {
    int inChar = Serial1.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    }
         
    // if you get a newline, print the string, then the string's value:
    if (inChar == '\n') {
      robotSpeed = (inString.substring(0, 3).toFloat()-100)/100;
      wheelAngle = (inString.substring(3, 6).toFloat()-100)/100;
      maxSpeed = inString.substring(6, 9).toFloat()/100;
      mode = inString.substring(9,10).toInt();
      forw_direction = inString.substring(10,11).toInt();
      reset_amps = inString.substring(11).toInt();
      digitalWrite(pinLed, HIGH-digitalRead(pinLed));
      //Serial1.println(mode);
      
      if (mode == 1){
        Gamepad.press(6);
        Gamepad.write();  
        delay(20);
        Gamepad.releaseAll();          
      }
      else if (mode == 2){
        Gamepad.press(5);
        Gamepad.write();  
        delay(20);
        Gamepad.releaseAll();            
      }

      if (forw_direction == 1){
        if (!forward_flag){
          forward_flag = 1;
          Gamepad.press(9);
          Gamepad.write();  
          delay(20);
          Gamepad.releaseAll(); 
        }         
      }
      else if (forw_direction == 2){
        if (forward_flag){
          forward_flag = 0;
          Gamepad.press(9);
          Gamepad.write();  
          delay(20);
          Gamepad.releaseAll();   
        }         
      } 

      if (reset_amps == 1){
        Gamepad.press(7);
        Gamepad.press(8);
        Gamepad.press(10);
        Gamepad.press(11);
        Gamepad.write();  
        delay(20);
        Gamepad.releaseAll();                                   
      }
              
      //Serial1.print("Value:");
      //Serial1.println(robotSpeed);
      //Serial1.println(wheelAngle);
      //Serial1.println(maxSpeed);
      // clear the string for new input:
      inString = "";
    }
  }
 
  if (!flag){
    flag = 1;
    Gamepad.press(6);
    Gamepad.write();  
    delay(20);
    Gamepad.releaseAll();           
  }
        
  //function to get the speed of the robot
  if (robotSpeed > 0){
    f_speed =  65536 - 32768*robotSpeed;
    //Serial1.println(f_speed);
  }
   else{
    f_speed = -32767*robotSpeed;
  }    

  //function to get the angle of the robot
  if (r_angle >= 0){
    r_angle =  32767* wheelAngle;
  }
   else{
    r_angle =  65535 + 32767* wheelAngle;
  }

  //function to get the maximum speed of the robot
  if (maxSpeed >= 0.5){
    m_speed = 254* maxSpeed -127;
    //Serial1.println(m_speed);
  }
   else{
    m_speed = 254* maxSpeed + 128;
  }

  Gamepad.zAxis(m_speed);
  Gamepad.xAxis(r_angle);
  Gamepad.yAxis(f_speed);
  Gamepad.write();
  delay(10);  
  
}

