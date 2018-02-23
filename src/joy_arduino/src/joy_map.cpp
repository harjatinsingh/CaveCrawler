#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32MultiArray.h>

class JoyArduino
{
	public:
		JoyArduino();
	private:
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
		ros::NodeHandle nh;
		ros::Publisher ArduinoPub;
		ros::Subscriber JoySub;
};


JoyArduino::JoyArduino()
{
	//publish bot commands
	ArduinoPub = nh.advertise<std_msgs::Int32MultiArray>("bot",100);
	
	//joystick topic
	JoySub = nh.subscribe<sensor_msgs::Joy>("joy",10,&JoyArduino::joyCallback,this);

}

int ConvertToRange(float num)
{
	return ((int)(num*100+100));
}

int absoluteRange(float num)
{ 
	return (((int)(num*100 +100))/2);
}

void JoyArduino::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	std_msgs::Int32MultiArray PublishArray;
	PublishArray.data.clear();


	//map joystick axes to robot controls 

	//linear velocity mapping
	PublishArray.data.push_back(ConvertToRange(joy->axes[4]));
	
	//steering mapping
	PublishArray.data.push_back(ConvertToRange(-1*joy->axes[3]));
	
	//max robot speed
	PublishArray.data.push_back(absoluteRange(joy->axes[2]));


	//map joystick buttons to robot control
	if(joy->buttons[5] == 1)
	{
		PublishArray.data.push_back(1); //forward mode	
	}
	else if (joy->buttons[4] == 1)
	{
		PublishArray.data.push_back(2); //crab mode
	}
	else
	{
		PublishArray.data.push_back(0);//continue in previous mode
	}

	if(joy->buttons[8] == 1)	
	{
		PublishArray.data.push_back(1);	 //forward direction
	}
	else
	{
		PublishArray.data.push_back(0);//reverse direction
	}

	if(joy->buttons[6] == 1 && joy->buttons[7] == 1 && joy->buttons[9] == 1 &&joy->buttons[10] == 1)	
	{
		PublishArray.data.push_back(1);	//amplifiers reset
	}
	else
	{
		PublishArray.data.push_back(0);
	}

	
	ArduinoPub.publish(PublishArray);
			
}


int main(int argc,char** argv)
{
	ros::init(argc,argv,"joy_arduno");
	JoyArduino joy_arduno;
	ros::spin();
}
