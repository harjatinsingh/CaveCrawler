To upload this code from any computer to the arduino, the following libraries must be installed:

1) Rosserial must be installed
	
sudo apt-get install ros-<distro>-rosserial-arduino
sudo apt-get install ros-<distro>-rosserial

2) Rosserial must be setup for the arduino IDE

cd <sketchbook>/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .

3) PJRC Encoder Library needs to installed

https://www.pjrc.com/teensy/td_libs_Encoder.html
