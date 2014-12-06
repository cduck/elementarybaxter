#!/usr/bin/env python

#Import the dependencies as described in example_pub.py
import rospy
from sensor_msgs.msg import Image
from ar_track_alvar.msg import *
from ocr.msg import *
import numpy as np
import scipy as sp
import sys
#import path_creation_lib as pcreate
from drawing.msg import *
import tf


useGripper = False
origin = (.4,-.1,-.15)
ihat = (0,1,0)
jhat = (-1,0,0)
khat = (0,0,1)
trans = None #np.array((ihat+(0,),jhat+(0,),khat+(0,),origin+(1,))).T
orientation = (1,1,0,0) #(0,1,0,.2)
#idlePosOrig = np.array((0,0,.3))


#idlePos = pcreate.transformWaypoint(idlePosOrig, trans)




if useGripper:
  raw_input('Press enter to load pen:')
  rospy.sleep(3)
  pc.closeGripper(pc.left_gripper)
  rospy.sleep(1)



counter = 0
allowedToSend = True
def callbackOcr(data):
  global counter, allowedToSend
  if not allowedToSend:
    print 'Can\'t solve.  Still busy writing an answer.'
    return
  #if recStr == '@q': sys.exit(0)
  #if recStr == '@u':
  #  msg = DrawPosition()
  #  msg.id = str(counter)
  #  msg.transform.flat = tuple(trans.flatten())
  #  msg.position = tuple(idlePos)
  #  pub2.publish(msg)
  #elif recStr == '@h':
  #  msg = DrawPosition()
  #  msg.id = str(counter)
  #  msg.transform.flat = tuple(trans.flatten())
  #  msg.position = tuple(idlePos)
  #  pub2.publish(msg)
  #else:
  print 'Solving...'
  prob = data.text
  if prob[-1] != '=':
    print 'I don\'t see an equals sign:',prob
  else:
    prob = prob.split('=')[0]
    sol = None
    try:
      sol = str(eval(prob))
    except:
      pass

    if sol != None and len(prob.split('~')) <= 1:
      print 'The solution to',prob,'is',sol
      msg = DrawCommand()
      counter += 1
      msg.id = str(counter)
      msg.text = sol

      top = (data.top) * 0.885
      bottom = (data.bottom) * 0.885
      x = (1-data.right) * 0.57
      s = (-top+bottom) * 0.85
      if s < 0.04: s = 0.04
      if s > .2: s = .2
      msg.size = s

      #print data.top, data.bottom, data.right
      #print top, bottom, x, s

      offsetX = 0.07 - x - 0.04
      offsetY = 0.07 - top + 0.1

      trans2 = np.array(((1,0,0,offsetX),(0,1,0,offsetY),(0,0,1,0),(0,0,0,1)))
      trans3 = trans.dot(trans2)
      msg.transform.flat = tuple(trans3.flatten())

      print offsetX, offsetY, '\n', trans, '\n', trans2, '\n', trans3

      pub.publish(msg)
      allowedToSend = False
    else:
      print 'I can\'t find a solution to',prob

def callback(data):
  global allowedToSend
  print data.id, ': Draw state:', data.msg
  if data.msg == 'done' and data.id == str(counter):
    print 'Confirmed'
    allowedToSend = True

def callbackTransform(data):
  global trans
  trans = np.array(data.flat).reshape((4,4))
  print 'Transform =\n',trans

def callbackTag(data):
  global trans
  NUM = 3
  marker = None
  for m in data.markers:
    if m.id == NUM:
      marker = m
      break
  if marker is None:
    return

  position = marker.pose.pose.position
  orientation = marker.pose.pose.orientation

  position = (position.x, position.y, position.z)
  orientation = (orientation.x, orientation.y, orientation.z, orientation.w)

  '''
      x: 0.343632340843+.07
      y: 0.472756026472-.07
      z: -0.149198124313
      x: 0.672691425208
      y: 0.739869023298
      z: 0.000913652870235
      w: 0.00890168827916
  '''
  position = (0.343632340843+.07, 0.472756026472-.07, -0.149198124313)
  orientation = (0.672691425208, 0.739869023298, 0.000913652870235, 0.00890168827916)

  rot = np.array(((0,-1,0,0),(1,0,0,0),(0,0,1,0),(0,0,0,1))) #tf.transformations.quaternion_matrix(orientation)
  translation = tf.transformations.translation_matrix(position)
  trans = np.array(((rot[0,0],rot[0,1],rot[0,2],translation[0,3]),(rot[1,0],rot[1,1],rot[1,2],translation[1,3]),(rot[2,0],rot[2,1],rot[2,2],translation[2,3]),(rot[3,0],rot[3,1],rot[3,2],translation[3,3])))
  #print 'Transform =\n',trans
  

rospy.init_node('solve_node')

pub = rospy.Publisher('draw_command', DrawCommand)
pub2 = rospy.Publisher('draw_position', DrawPosition)
rospy.Subscriber('draw_response', DrawResponse, callback)
rospy.Subscriber("ocr_result", TextAndBox, callbackOcr)
rospy.Subscriber("surface_transform", Transform, callbackTransform)
rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callbackTag)

#pc.executePlan(pc.planMoveToPose(pos=idlePos, orientation=orientation))
#pc.executePlan(pc.planMoveToPose(arm=pc.right_arm))
#pc.initLeftGripper()
#pc.initRightGripper()

#msg = DrawPosition()
#msg.position = tuple(idlePos)
#rospy.sleep(.2)
#pub2.publish(msg)

print 'Ready.'
rospy.spin()

