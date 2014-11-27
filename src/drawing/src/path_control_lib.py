#!/usr/bin/env python

import sys, copy
import numpy as np
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped, Pose
from baxter_interface import gripper as baxter_gripper

robot = None
scene = None
left_arm = None
right_arm = None
left_gripper = None
right_gripper = None

#left_arm_default_state  = ((0.45, 0.45, 1.3),
#  (.01, .17, .64, .75))
left_arm_default_state  = ((0.2, 0.7, 0.4),
  (0, 1, 0, 1))
right_arm_default_state = ((0.2, -0.7, 0.4),
  (0, 1, 0, 1))

def init():
  global robot
  global scene
  global left_arm
  global right_arm
  global left_gripper
  global right_gripper
  #Initialize moveit_commander
  moveit_commander.roscpp_initialize(sys.argv)

  #Initialize both arms
  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  left_arm = moveit_commander.MoveGroupCommander('left_arm')
  right_arm = moveit_commander.MoveGroupCommander('right_arm')
  left_arm.set_planner_id('RRTConnectkConfigDefault')
  left_arm.set_planning_time(20)
  right_arm.set_planner_id('RRTConnectkConfigDefault')
  right_arm.set_planning_time(20)

def planMoveToPose(pos=None, orientation=None, arm=None):
  arm = arm or left_arm
  if arm is left_arm:
    pos = left_arm_default_state[0] if pos is None else pos
    orientation = orientation or left_arm_default_state[1]
  if arm is right_arm:
    pos = right_arm_default_state[0] if pos is None else pos
    orientation = orientation or right_arm_default_state[1]

  #First goal pose ------------------------------------------------------
  goal_1 = PoseStamped()
  goal_1.header.frame_id = "base"

  #x, y, and z position
  goal_1.pose.position.x = pos[0]
  goal_1.pose.position.y = pos[1]
  goal_1.pose.position.z = pos[2]
  
  #Orientation as a quaternion
  orientation = orientation/np.linalg.norm(orientation)
  goal_1.pose.orientation.x = orientation[0]
  goal_1.pose.orientation.y = orientation[1]
  goal_1.pose.orientation.z = orientation[2]
  goal_1.pose.orientation.w = orientation[3]

  #Set the goal state to the pose you just defined
  arm.set_pose_target(goal_1)

  #Set the start state for the left arm
  arm.set_start_state_to_current_state()

  #Plan a path
  return (arm, arm.plan())

def planMoveToPosHoldOrientation(pos=None, orientation=None, arm=None):
  arm = arm or left_arm
  if arm is left_arm:
    pos = pos or left_arm_default_state[0]
    orientation = orientation or left_arm_default_state[1]
  if arm is right_arm:
    pos = pos or right_arm_default_state[0]
    orientation = orientation or right_arm_default_state[1]

  #Second goal pose -----------------------------------------------------
  goal_2 = PoseStamped()
  goal_2.header.frame_id = "base"

  #x, y, and z position
  goal_2.pose.position.x = pos[0]
  goal_2.pose.position.y = pos[1]
  goal_2.pose.position.z = pos[2]
  
  #Orientation as a quaternion
  orientation = orientation/np.linalg.norm(orientation)
  goal_2.pose.orientation.x = orientation[0]
  goal_2.pose.orientation.y = orientation[1]
  goal_2.pose.orientation.z = orientation[2]
  goal_2.pose.orientation.w = orientation[3]

  #Set the goal state to the pose you just defined
  arm.set_pose_target(goal_2)

  #Set the start state for the left arm
  arm.set_start_state_to_current_state()

  # #Create a path constraint for the arm
  # #UNCOMMENT TO ENABLE ORIENTATION CONSTRAINTS
  orien_const = OrientationConstraint()
  orien_const.link_name = "left_gripper";
  orien_const.header.frame_id = "base";
  orien_const.orientation.x = orientation[0]
  orien_const.orientation.y = orientation[1]
  orien_const.orientation.z = orientation[2]
  orien_const.orientation.w = orientation[3]
  orien_const.absolute_x_axis_tolerance = 0.1;
  orien_const.absolute_y_axis_tolerance = 0.1;
  orien_const.absolute_z_axis_tolerance = 0.1;
  orien_const.weight = 1.0;
  consts = Constraints()
  consts.orientation_constraints = [orien_const]
  arm.set_path_constraints(consts)

  #Plan a path
  return (arm, arm.plan())

def planPath(positions, orientation=None, arm=None, holdOrientation=False, step=0.01, threshold=1000):
  arm = arm or left_arm
  if arm is left_arm:
    positions = (left_arm_default_state[0],) if positions is None else positions
    orientation = left_arm_default_state[1] if orientation is None else orientation
  if arm is right_arm:
    positions = (right_arm_default_state[0],) if positions is None else positions
    orientation = right_arm_default_state[1] if orientation is None else orientation

  # Compute path
  waypoints = []
  # start with the current pose
  #waypoints.append(arm.get_current_pose().pose)
  wpose = Pose()
  orientation = orientation/np.linalg.norm(orientation)
  wpose.orientation.x = orientation[0]
  wpose.orientation.y = orientation[1]
  wpose.orientation.z = orientation[2]
  wpose.orientation.w = orientation[3]

  for pos in positions:
    wpose.position.x = pos[0]
    wpose.position.y = pos[1]
    wpose.position.z = pos[2]
    waypoints.append(copy.deepcopy(wpose))


  if holdOrientation:
    # #Create a path constraint for the arm
    # #UNCOMMENT TO ENABLE ORIENTATION CONSTRAINTS
    orien_const = OrientationConstraint()
    orien_const.link_name = "left_gripper";
    orien_const.header.frame_id = "base";
    orien_const.orientation.x = orientation[0]
    orien_const.orientation.y = orientation[1]
    orien_const.orientation.z = orientation[2]
    orien_const.orientation.w = orientation[3]
    orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.1;
    orien_const.absolute_z_axis_tolerance = 0.1;
    orien_const.weight = 1.0;
    consts = Constraints()
    consts.orientation_constraints = [orien_const]
    arm.set_path_constraints(consts)

  #Plan a path
  (plan, fraction) = arm.compute_cartesian_path(waypoints, step, threshold)
  return (arm, plan, fraction)

def initLeftGripper():
  global left_gripper
  #Set up the left gripper
  left_gripper = baxter_gripper.Gripper('left')

  #Calibrate the gripper (other commands won't work unless you do this first)
  print('Calibrating...')
  left_gripper.calibrate()

def initRightGripper():
  global right_gripper
  #Set up the left gripper
  right_gripper = baxter_gripper.Gripper('right')

  #Calibrate the gripper (other commands won't work unless you do this first)
  print('Calibrating...')
  right_gripper.calibrate()

def closeGripper(gripper):  # Blocking
  print('Closing...')
  gripper.close(block=True)
  rospy.sleep(1.0)

def openGripper(gripper):  # Blocking
  print('Opening...')
  gripper.open(block=True)
  rospy.sleep(1.0)

def executePlan(plan):
  plan[0].execute(plan[1])



def main():
  pass


if __name__ == '__main__':
  main()
