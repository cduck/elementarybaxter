#!/usr/bin/env python

import sys
#import path_control_lib as pc
import path_creation_lib as pcreate
import rospy
import numpy as np
from drawing.msg import *


useGripper = False
origin = (.4,.07,-.19)
ihat = (0,1,0)
jhat = (-1,0,0)
khat = (0,0,1)
trans = np.array((ihat+(0,),jhat+(0,),khat+(0,),origin+(1,))).T
orientation = (1,1,0,0) #(0,1,0,.2)
idlePosOrig = np.array((0,0,.3))


idlePos = pcreate.transformWaypoint(idlePosOrig, trans)




if useGripper:
  raw_input('Press enter to load pen:')
  rospy.sleep(3)
  pc.closeGripper(pc.left_gripper)
  rospy.sleep(1)



counter = 0
def inputAndSendText():
  global counter
  recStr = raw_input("Enter string (to quit: '@q'): ")
  if recStr == '@q': sys.exit(0)
  msg = DrawCommand()
  counter += 1
  msg.id = str(counter)
  msg.transform.flat = tuple(trans.flatten())
  msg.text = recStr
  pub.publish(msg)

def callback(data):
  print data.id, ': Draw state:', data.msg
  if data.msg == 'done' and data.id == str(counter):
    inputAndSendText()

rospy.init_node('draw_test_node')

pub = rospy.Publisher('draw_command', DrawCommand)
pub2 = rospy.Publisher('draw_position', DrawPosition)
rospy.Subscriber('draw_response', DrawResponse, callback)

#pc.executePlan(pc.planMoveToPose(pos=idlePos, orientation=orientation))
#pc.executePlan(pc.planMoveToPose(arm=pc.right_arm))
#pc.initLeftGripper()
#pc.initRightGripper()

msg = DrawPosition()
msg.position = tuple(idlePos)
rospy.sleep(.2)
pub2.publish(msg)

inputAndSendText()
rospy.spin()

