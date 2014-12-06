#!/usr/bin/env python

import rospy
import path_control_lib as pc
import path_creation_lib as pcreate
import rospy
import numpy as np
from drawing.msg import *

pub = None
trans = None
orientation = (1,1,0,0) #Best for drawing on horizontal surface

'''
Desired home angles
header: 
  seq: 2233596
  stamp: 
    secs: 1417862863
    nsecs: 753585523
  frame_id: ''
name: ['head_nod', 'head_pan', 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2', 'torso_t0']
position: [0.0, -0.042951462011718754, -1.9983934691345215, 2.376903228112793, 1.6712720664916993, 0.900446721459961, 2.1506410621582033, 1.7322478027404786, 1.69888372064209, 1.2559467686462402, 1.2896943459411623, 0.6450389205688477, -1.1190389834838868, 2.6526362744201664, -1.5700293346069336, 0.9219224524658204, -12.565987104803467]
velocity: [0.0, 0.001872535139322281, -0.009362675696611405, 0.010486196780204774, -0.004119577306509018, -0.0003745070278644562, -0.005617605417966843, 0.0026215491950511934, 0.0003745070278644562, -0.01610380219817162, 0.001872535139322281, -0.008613661640882493, -0.008239154613018036, 0.00898816866874695, -0.01872535139322281, -0.005617605417966843, 0.0]
effort: [0.0, 0.0, -10.324, 6.196, 0.592, -0.516, 0.432, -1.384, 0.128, 7.96, -13.692, -0.84, -10.452, -0.044, 0.536, 0.204, -20.48]

Desired home locations
Left:
pose: 
  position: 
    x: 0.561495187635
    y: 0.772525702265
    z: 0.0923686072884
  orientation: 
    x: -0.0673913669936
    y: 0.997582146799
    z: -0.00689465671297
    w: 0.0155154037089
twist: 
  linear: 
    x: 0.00476858255323
    y: -0.00801852651175
    z: -0.00540335683885
  angular: 
    x: -0.0337811504844
    y: -0.0223716649315
    z: -0.00446031976012
wrench: 
  force: 
    x: -1.10297226906
    y: 0.590402066708
    z: 7.61617136002
  torque: 
    x: 0.37381875515
    y: 0.455255240202
    z: -0.114333443344

Right:
pose: 
  position: 
    x: 0.540965800789
    y: 0.110993276752
    z: 0.484457265944
  orientation: 
    x: 0.996431522095
    y: 0.0526386589164
    z: 0.0567231028029
    w: -0.0337028629473
twist: 
  linear: 
    x: 0.0076993128757
    y: -0.0149241659842
    z: -0.00356675255098
  angular: 
    x: -0.0189258443983
    y: 0.00933993113828
    z: -0.0361095973864
wrench: 
  force: 
    x: -3.94997024536
    y: -4.74429750443
    z: 1.42103910446
  torque: 
    x: 1.47635197639
    y: -0.955144822598
    z: -0.164863973856
'''

leftArmHomePos = (0.561495187635,0.772525702265,0.0923686072884)
leftArmHomeOri = (1, 1, 0, 0)
rightArmHomePos = (0.540965800789,0.110993276752,0.484457265944)
rightArmHomeOri = (0.996431522095,0.0526386589164,0.0567231028029,-0.0337028629473)
rightArmDrawPos = (0.6,-0.5,0.484457265944)
rightArmDrawOri = rightArmHomeOri

def resetArmLocations():
  pc.init()
  pc.executePlan(pc.planMoveToPose(leftArmHomePos, orientation=leftArmHomeOri))
  rospy.sleep(0.1)
  pc.init()
  pc.executePlan(pc.planMoveToPose(rightArmHomePos, orientation=rightArmHomeOri, arm=pc.right_arm))

def drawString(str, size=None):
  if trans is None:
    print 'Can not draw.  Has not recieved the transform of the surface.'

  waypoints = pcreate.strToWaypoints(str,trans,size)
  print 'Waypoints:'
  print waypoints

  if len(waypoints) > 0:
    pc.executePlan(pc.planMoveToPose(waypoints[0],orientation))
    pc.executePlan(pc.planPath(waypoints, orientation, holdOrientation=True))

def callback(data):
  global trans
  trans = np.array(data.transform.flat).reshape((4,4))
  print 'Transform =\n',trans

  msg = DrawResponse()
  msg.id = data.id
  msg.msg = 'working'
  pub.publish(msg)


  drawString(data.text, data.size if data.size > 0 else None)


  msg = DrawResponse()
  msg.id = data.id
  msg.msg = 'done'
  pub.publish(msg)

def callback2(data):
  pc.init()

  print 'Moving arm to position:', data.position
  pos = data.position
  pc.executePlan(pc.planMoveToPose(pos,orientation))

def drawString(str, size=None):
  if trans is None:
    print 'Can not draw.  Has not recieved the transform of the surface.'
    return

  waypoints = pcreate.strToWaypoints(str,trans,size=size)
  print 'Waypoints:'
  print waypoints

  if len(waypoints) > 0:
    pc.init()
    pc.executePlan(pc.planMoveToPose(rightArmDrawPos, orientation=rightArmDrawOri, arm=pc.right_arm))
    rospy.sleep(0.1)
    pc.executePlan(pc.planMoveToPose(waypoints[0],orientation))
    rospy.sleep(0.1)
    pc.executePlan(pc.planPath(waypoints, orientation, holdOrientation=True))
    rospy.sleep(0.1)
    resetArmLocations()

def main():
  global pub
  #Initialize the node

  rospy.init_node('draw_node')

  pub = rospy.Publisher('draw_response', DrawResponse)

  rospy.Subscriber('draw_command', DrawCommand, callback)
  rospy.Subscriber('draw_position', DrawPosition, callback2)

  resetArmLocations()

  print "Ready."

  rospy.spin()


if __name__ == '__main__':
    main()

