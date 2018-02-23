#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(3,2);
ros::NodeHandle  nh;
int Arr[90];
std_msgs::String str_msg;

ros::Publisher chatter("chater",&str_msg);

void messageCb( const std_msgs::Int32MultiArray& array){

    int i = 0;
    int j = 0;
    char ch[10];

  for(i = 0 ; i < 3 ; i++)
  {
    j = array.data[i];
    if(j<100 && j >9)
    {
      mySerial.print(0);
    }
    else if(j < 10)
    {
      mySerial.print(0);
      mySerial.print(0);
    }
    mySerial.print(j);
    str_msg.data = itoa(j,ch,10);
    chatter.publish(&str_msg); 
  }

  mySerial.print(array.data[3]);
  mySerial.print(array.data[4]);
  mySerial.print(array.data[5]);
  mySerial.print("\n");
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}


ros::Subscriber<std_msgs::Int32MultiArray> sub("bot", &messageCb );


void setup()
{ 

    mySerial.begin(115200);
//  mySerial.print("$");  // Print three times individually
//  mySerial.print("$");
//  mySerial.print("$");  // Enter command mode
//  delay(100);  // Short delay, wait for the Mate to send back CMD
//  //mySerial.println("U,9600,N");  // Temporarily Change the baudrate to 9600, no parity
//  // 115200 can be too fast at times for NewSoftSerial to relay the data reliably
//  //mySerial.begin(9600);
// // delay(100);
//  mySerial.println("C");
  delay(100);
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
}

void loop()
{    
  nh.spinOnce();
  delay(1);
}

