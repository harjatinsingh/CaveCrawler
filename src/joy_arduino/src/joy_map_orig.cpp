#include<ros/ros.h>
#include<sensor_msgs/Joy.h>
#include<std_msgs/Int32MultiArray.h>

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
	ArduinoPub = nh.advertise<std_msgs::Int32MultiArray>("bot",100);
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
	float a,b;
	std_msgs::Int32MultiArray array;
	array.data.clear();
	a = joy->axes[1];
	b = joy->axes[2];
	array.data.push_back(ConvertToRange(joy->axes[4]));
	array.data.push_back(ConvertToRange(-1*joy->axes[3]));
	array.data.push_back(absoluteRange(joy->axes[2]));
	if(joy->buttons[5] == 1)
	{
		array.data.push_back(1);	
	}
	else if (joy->buttons[4] == 1)
	{
		array.data.push_back(2);
	}
	else
	{
		array.data.push_back(0);
	}

	if(joy->buttons[8] == 1)	
	{
		array.data.push_back(1);	
	}
	else
	{
		array.data.push_back(0);
	}

	if(joy->buttons[6] == 1 && joy->buttons[7] == 1 && joy->buttons[9] == 1 &&joy->buttons[10] == 1)	
	{
		array.data.push_back(1);	
	}
	else
	{
		array.data.push_back(0);
	}

	
	ArduinoPub.publish(array);
			
}


int main(int argc,char** argv)
{
	ros::init(argc,argv,"joy_arduno");
	JoyArduino joy_arduno;
	ros::spin();
}
