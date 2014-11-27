#!/usr/bin/env python

import roslib#; roslib.load_manifest('lab7')

import rospy
from sensor_msgs.msg import Image

pub = None

def callback(data):
  pub.publish(data)

def main():
  global pub
  #Initialize the node
  rospy.init_node('display_node')

  pub = rospy.Publisher('/robot/xdisplay', Image)

  rospy.Subscriber('/cameras/left_hand_camera/image', Image, callback)
  rospy.spin()


if __name__ == '__main__':
    main()

