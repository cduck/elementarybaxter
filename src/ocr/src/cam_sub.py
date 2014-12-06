#!/usr/bin/env python

#Import the dependencies as described in example_pub.py
import rospy
from sensor_msgs.msg import Image
from ocr.msg import *
import numpy as np
import scipy as sp
import cv2
import os
import sys

image = None


im_loc = '/home/duckering_zeng/ee125project/src/ocr/src/in.png'
im_dest = 'andy@192.168.2.130:~/Dropbox/SchoolFiles2014-2015/EE125/FinalProject/ocr'
im_out_loc = 'andy@192.168.2.130:~/Dropbox/SchoolFiles2014-2015/EE125/FinalProject/ocr/out.txt'
im_out_dest = '/home/duckering_zeng/ee125project/src/ocr/src'

#Define the callback method which is called whenever this node receives a 
#message on its subscribed topic. The received message is passed as the 
#first argument to callback().
def callback(message):
    global image

    #Save image to file.
    image = np.zeros((message.height,message.width,3), np.uint8)
    #TODO: Change this for loop to improve speed...
    for channel in range(2,-1,-1):
        for i in range(0,message.height):
            for j in range(0,message.width):
                image[i,j,channel] = ord(message.data[i*4*message.width+j*4+channel])
    cv2.imwrite(im_loc,image)
    

    
    

#Define the method which contains the node's main functionality
def listener():

    #Run this program as a new node in the ROS computation graph
    #called /listener_<id>, where <id> is a randomly generated numeric
    #string. This randomly generated name means we can start multiple
    #copies of this node without having multiple nodes with the same
    #name, which ROS doesn't allow.
    rospy.init_node('ocr', anonymous=True)

    #Create a new instance of the rospy.Subscriber object which we can 
    #use to receive messages of type std_msgs/String from the topic /chatter.
    #Whenever a new message is received, the method callback() will be called
    #with the received message as its first argument.
    rospy.Subscriber("cameras/right_hand_camera/image", Image, callback)

    pub = rospy.Publisher('ocr_result', TextAndBox, queue_size=1)


    #Wait for messages to arrive on the subscribed topics, and exit the node
    #when it is killed with Ctrl+C
    global image
    while not rospy.is_shutdown():
      try:
	print 'Frame captured.'
        os.system('scp '+im_loc+' '+im_dest)
        print 'Waiting for server.'
        rospy.sleep(4.0)
        os.system('scp '+im_out_loc+' '+im_out_dest)

        try:
          with open(im_out_dest+'/out.txt','r') as fin:
              result = fin.read()
              print result

          msg = TextAndBox()
          lines = result.split('\n')
          msg.text = lines[0]
          boxStrings = lines[1].split(' ')
          msg.left = float(boxStrings[0])
          msg.top = float(boxStrings[1])
          msg.right = float(boxStrings[2])
          msg.bottom = float(boxStrings[3])
	  pub.publish(msg)
        except:
          if not rospy.is_shutdown():
            print 'Recieve output error:'
            print msg
            msg = TextAndBox()
            msg.text = '?'
            msg.left = 0
            msg.top = 0
            msg.right = 0
            msg.bottom = 40
	    pub.publish(msg)

        rospy.sleep(5.0)
        print 'Done.'
      except:
        sys.exit(1)
        break


#Python's syntax for a main() method
if __name__ == '__main__':
    listener()
