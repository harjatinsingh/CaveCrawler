To upload this code from any computer to the arduino, the following libraries must be installed:

1) Rosserial must be installed
	
_sudo apt-get install ros-<distro>-rosserial-arduino <br />
sudo apt-get install ros-<distro>-rosserial_ <br />

2) Rosserial must be setup for the arduino IDE

_cd <sketchbook>/libraries <br />
rm -rf ros_lib <br />
rosrun rosserial_arduino make_libraries.py ._

3) PJRC Encoder Library needs to installed

https://www.pjrc.com/teensy/td_libs_Encoder.html

Once the code is uploaded to the arduino, the rosnode can be launched with:

_roscore <br />
rosrun rosserial_python serial_node.py /dev/ttyACM* <br />
rostopic echo odometry_
