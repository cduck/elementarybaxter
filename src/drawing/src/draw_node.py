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
    pc.executePlan(pc.planMoveToPose(waypoints[0],orientation))
    pc.executePlan(pc.planPath(waypoints, orientation, holdOrientation=True))

def main():
  global pub
  #Initialize the node

  rospy.init_node('draw_node')

  pc.init()

  pub = rospy.Publisher('draw_response', DrawResponse)

  rospy.Subscriber('draw_command', DrawCommand, callback)
  rospy.Subscriber('draw_position', DrawPosition, callback2)
  rospy.spin()


if __name__ == '__main__':
    main()

