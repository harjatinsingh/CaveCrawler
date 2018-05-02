#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>

class JoyArduino
{
	public:
		JoyArduino();
	private:
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
		void controlCallback(const geometry_msgs::Twist::ConstPtr& msg);
		bool joy_control;
		bool crab_mode;
		ros::NodeHandle nh;
		ros::Publisher ArduinoPub;
		ros::Subscriber JoySub;
		ros::Subscriber cmdVelSub;
		ros::Subscriber ackermannMsg;
};


JoyArduino::JoyArduino()
{
	//publish bot commands
	ArduinoPub = nh.advertise<std_msgs::Int32MultiArray>("bot",100);

	//joystick topic
	JoySub = nh.subscribe<sensor_msgs::Joy>("joy",10,&JoyArduino::joyCallback,this);

	cmdVelSub = nh.subscribe<geometry_msgs::Twist>("cmd_vel",10,&JoyArduino::controlCallback,this);

	joy_control = true;
}

int ConvertToRange(float num)
{
	return ((int)(num*100+100));
}

int absoluteRange(float num)
{ 
	return (((int)(num*100 +100))/2);
}

void JoyArduino::controlCallback(const geometry_msgs::Twist::ConstPtr& msg)
{

	if(!joy_control){ //switch to autonomous control of robot
		ROS_WARN("Trying to drive robot autonomously");
		std_msgs::Int32MultiArray PublishArray;
		PublishArray.data.clear();

		//linear velocity mapping
		// ROS_INFO_STREAM("forward vel");
		// ROS_INFO_STREAM(msg->linear.x);
		// ROS_INFO_STREAM("angular vel");
		// ROS_INFO_STREAM(msg->angular.z);
		PublishArray.data.push_back(ConvertToRange(msg->linear.x));

		//steering mapping
		int bump = 0;
		if(-msg->angular.z > 0)
			bump =20;
		else if (-msg->angular.z < 0)
			bump = -20;	


		PublishArray.data.push_back(bump+ConvertToRange(-msg->angular.z));
		//max robot speed
		PublishArray.data.push_back(35);

		//if(crab_mode && msg->linear.x !=0){
		//	ROS_INFO_STREAM("SWITCHING TO  LINEAR  MODE");
		//	crab_mode = false;
		//	PublishArray.data.push_back(1);//switch to forward mode
		//} else if(!crab_mode && msg->linear.x==0 && msg->angular.z!= 0){
		//	crab_mode = true;
		//	ROS_INFO_STREAM("SWITCHING TO CRAB  MODE");
		//	PublishArray.data.push_back(2);//switch to crab mode
		//}else{
		//	if(crab_mode){
		//		ROS_INFO_STREAM("HOLDING CRAB  MODE");
		//	}else{
		//		ROS_INFO_STREAM("HOLDING LINEAR  MODE");
		//	}

		//	PublishArray.data.push_back(0);//continue in previous mode
		//}

		PublishArray.data.push_back(0);//continue in previous mode
		PublishArray.data.push_back(0);//continue in previous mode
		PublishArray.data.push_back(0);//continue in previous mode

		ROS_INFO_STREAM(PublishArray);
		ArduinoPub.publish(PublishArray);
	}

}

void JoyArduino::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	std_msgs::Int32MultiArray PublishArray;
	PublishArray.data.clear();

	//map joystick axes to robot controls 
	if(joy->buttons[3] == 1)
	{
		joy_control = !joy_control;
		if( joy_control){
			ROS_WARN("Toggling control of robot to joystick");
		}
		else{ 
			ROS_WARN("Toggling to autonomous control");
		}

	}

	if(joy_control){
		ROS_WARN("pushing joystick control");
		//linear velocity mapping
		ROS_INFO_STREAM("forward vel");
		ROS_INFO_STREAM(joy->axes[4]);
		PublishArray.data.push_back(ConvertToRange(joy->axes[4]));

		//steering mapping
		PublishArray.data.push_back(ConvertToRange(-1*joy->axes[3]));

		//max robot speed
		PublishArray.data.push_back(absoluteRange(joy->axes[2]));


		//map joystick buttons to robot control
		if(joy->buttons[5] == 1)
		{
			PublishArray.data.push_back(1); //forward mode	
			crab_mode = false;
		}
		else if (joy->buttons[4] == 1)
		{
			PublishArray.data.push_back(2); //crab mode
			crab_mode = true;
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
		ROS_INFO_STREAM(PublishArray);

		ArduinoPub.publish(PublishArray);
	}

}


int main(int argc,char** argv)
{
	ROS_WARN("Starting joy arduino");
	ros::init(argc,argv,"joy_arduno");
	JoyArduino joy_arduno;
	ros::spin();
}
