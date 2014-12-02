#!/usr/bin/env python

import sys
import path_control_lib as pc
import path_creation_lib as pcreate
import rospy
import numpy as np

#rospy.sleep(20)
rospy.init_node('moveit_node')
pc.init()


useGripper = False
origin = (.4,-.1,-.145)
ihat = (0,1,0)
jhat = (-1,0,0)
khat = (0,0,1)
trans = np.array((ihat+(0,),jhat+(0,),khat+(0,),origin+(1,))).T
orientation = (1,1,0,0) #(0,1,0,.2)
idlePosOrig = np.array((0,0,.3))

idlePos = pcreate.transformWaypoint(idlePosOrig, trans)




pc.executePlan(pc.planMoveToPose(pos=idlePos, orientation=orientation))
#pc.executePlan(pc.planMoveToPose(arm=pc.right_arm))
#pc.initLeftGripper()
#pc.initRightGripper()

if useGripper:
  raw_input('Press enter to load pen:')
  rospy.sleep(3)
  pc.closeGripper(pc.left_gripper)
  rospy.sleep(1)

while True:
  str = ''
  recStr = raw_input("Enter string (to quit: '@q'): ")
  if recStr == '@q': sys.exit(0)

  waypoints = pcreate.strToWaypoints(recStr,trans)
  print 'Waypoints:'
  print waypoints

  pc.executePlan(pc.planMoveToPose(waypoints[0],orientation))
  pc.executePlan(pc.planPath(waypoints, orientation, holdOrientation=True))

  #for p in waypoints:
  #  pc.executePlan(pc.planMoveToPosHoldOrientation(p, orientation))
  #if raw_input('Repeat: [Enter], Quit: q[Enter]\n') is 'q': break

