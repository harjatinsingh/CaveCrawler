#!/usr/bin/env python
import rospy
import math
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, Int16MultiArray
from geometry_msgs.msg import Point, Pose, Quaternion, QuaternionStamped, Twist, Vector3
import pdb

wheel_dist = 1
w = 0.9

def update_odom(data):
    raw_odom = data.data
    global left_odom
    global right_odom
    global left_turn_raw
    global right_turn_raw
    left_odom = raw_odom.quaternion.x # raw_odom[0]
    right_odom = raw_odom.quaternion.y # raw_odom[1]
    left_turn_raw = raw_odom.quaternion.z # raw_odom[2]
    right_turn_raw = raw_odom.quaternion.w #raw_odom[3]

    scale = 6e-3# 6e-6
    left_odom = left_odom * scale
    right_odom = right_odom * scale

    left_turn = (-0.0968 * left_turn_raw + 27.1744)*math.pi/180
    right_turn = (0.1112 * right_turn_raw - 31.0661)*math.pi/180

    # print "left_odom", left_odom, "right_odom", right_odom

    # get forward velocity
    v = (left_odom+right_odom)/2;

    if abs(left_odom+right_odom) > 0:
        direction = (left_odom+right_odom)/abs(left_odom+right_odom)
    else:
        direction = 1

    # get turn centre from angles
    turn_centre_left = 0.5 / math.tan(left_turn)
    turn_centre_right = 0.5 / math.tan(right_turn)
    turn_centre = (turn_centre_left + turn_centre_right)/2;

    # get turn centre from odom delta
    # turn_centre_delta = wheel_dist/2 * (left_odom+right_odom)/(left_odom-right_odom);

    # compute delta in heading
    # turn_centre = turn_centre_angle*(w) + turn_centre_delta*(1-w);

    if abs(turn_centre) < 1e5 and not v == 0:
        A = turn_centre**2 - v**2;
        try:
            d_heading = direction*math.atan2(v,math.sqrt(A));
        except:
            d_heading = 0;
    else:
        d_heading = 0;

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"

    # set the position
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, 0)
    odom.pose.pose = Pose(Point(0, 0, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(v, 0, 0), Vector3(0, 0, d_heading))

    # publish the message
    global odom_pub
    odom_pub.publish(odom)


def main():

    rospy.init_node('odom_cvt')

    global odom_pub
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    incoming_odom = rospy.Subscriber('/arduino/raw_odometry', QuaternionStamped, update_odom)


    r = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        r.sleep()

if __name__ == "__main__":
    main()
