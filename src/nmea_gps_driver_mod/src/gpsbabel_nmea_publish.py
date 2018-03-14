#!/usr/bin/env python
import rospy
import sys
import subprocess
import fcntl
import os
from time import sleep
from nmea_msgs.msg import Sentence

from std_msgs.msg import String


def nmea_publisher():
    rospy.init_node('nmea_sentence', anonymous=True)
    pub = rospy.Publisher('nmea_sentence', Sentence, queue_size=10)

    my_command=["gpsbabel -T -i garmin -f usb: -o nmea -F -"]

    print >> sys.stderr, 'Start gpsbabel process:', my_command
    

    sub_process = subprocess.Popen(my_command, close_fds=True, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    fcntl.fcntl(sub_process.stdout.fileno(), fcntl.F_SETFL, os.O_NONBLOCK)
    process_healthy = (sub_process.poll() is None)

    if(not process_healthy): 
      rospy.logerr("Did not start process successfully %s " % my_command)
      rospy.logerr("Returncode;", process.returncode)
      rospy.shutdown()
      sys.stdout.flush()

    frame_id = "gps_frame"
    sleep(0.1)
    gps_publish_rate = 25
    rate = rospy.Rate(gps_publish_rate)
    GGA = ''
    invalid_gps_counter = 0
    publish_flag = 0

    while (not rospy.is_shutdown()) and sub_process.poll() is None:
       try:
          data = sub_process.stdout.readline().rstrip()
          fields = data.split(',')
          rospy.loginfo("%s",str(data))
          #if GPGGA format, else other NMEA format
          if 'GGA' in fields[0]:
             invalid_gps_counter = 0
             GGA = data
             publish_flag = 1
       except:
          invalid_gps_counter += 1
          #wait for first GPS data
          if GGA == '':
             rospy.logwarn("Waiting for first GPS data")
             publish_flag = 0
          #no GPS data in last 5 seconds
          if(invalid_gps_counter > gps_publish_rate * 5):
             rospy.logerr("5+ seconds without new GPS data")
             publish_flag = 0
          pass

       if publish_flag == 0 and (invalid_gps_counter > gps_publish_rate * 5):
          rospy.logerr("No GPS published for more than 5 seconds")

       if publish_flag == 1:
          sentence = Sentence()
          sentence.header.stamp = rospy.get_rostime()
          sentence.header.frame_id = frame_id
          sentence.sentence = GGA
          pub.publish(sentence)
          sys.stdout.flush()
       rate.sleep()

    if  sub_process.poll() is None:
       sub_process.kill()
if __name__ == '__main__':
    try:
        nmea_publisher()
    except rospy.ROSInterruptException:
        pass
