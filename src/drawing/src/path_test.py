#!/usr/bin/env python

import path_control_lib as pc
import rospy

rospy.init_node('moveit_node')
pc.init()

#rospy.sleep(20)

pc.executePlan(pc.planMoveToPose(pos=(.5,.5,.0), orientation=(1,0,1,0)))
#pc.executePlan(pc.planMoveToPose(arm=pc.right_arm))
#pc.initLeftGripper()
#pc.initRightGripper()

useGripper = False

'''
Square on whiteboard
start:
x: 0.569975589886
y: 0.627755696398
z: -0.112671622494
p1:
x: 0.775550653199
y: 0.515113384856
z: -0.1744279856
p2: 
x: 0.780138605451
y: 0.281300142469
z: -0.177751433842
p3:
x: 0.758225540375
y: 0.315142267893
z: -0.309447314654
'''

orientation = (.82,0,.57,0)
waypoints = (
(.57, .63, -.1),
(.82, .52, -.07),
(.82, .31, -.07),
(.8, .31, -.2),
(.8, .52, -.2),
(.82, .52, -.07),
(.57, .63, -.1))
'''
waypoints = (
(.5, .4, .2),
(.7, .4, .2),
(.7, .2, .2),
(.7, .2, .4),
(.7, .4, .4),
(.7, .4, .2),
(.5, .4, .2))
'''

pc.executePlan(pc.planMoveToPose(waypoints[0],orientation))

if useGripper:
  raw_input('Press enter to load pen:')
  rospy.sleep(3)
  pc.closeGripper(pc.left_gripper)
  rospy.sleep(1)

while True:
  pc.executePlan(pc.planPath(waypoints, orientation, holdOrientation=True))

  #for p in waypoints:
  #  pc.executePlan(pc.planMoveToPosHoldOrientation(p, orientation))
  if raw_input('Repeat: [Enter], Quit: q[Enter]\n') is 'q': break

  pc.executePlan(pc.planMoveToPose(waypoints[0],orientation))

