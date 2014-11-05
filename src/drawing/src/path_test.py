import path_control_lib as pc
import rospy

pc.init()

#pc.executePlan(pc.planMoveToPose())
pc.executePlan(pc.planMoveToPose(arm=pc.right_arm))
pc.initLeftGripper()
#pc.initRightGripper()


waypoints = (
(.5, .4, .2),
(.7, .4, .2),
(.7, .2, .2),
(.7, .2, .4),
(.7, .4, .4),
(.7, .4, .2),
(.5, .4, .2))
pc.executePlan(pc.planMoveToPose(waypoints[0],(0,1,0,1)))
raw_input('Press enter to load pen:')
rospy.sleep(3)
pc.closeGripper(pc.left_gripper)
rospy.sleep(1)
pc.executePlan(pc.planPath(waypoints,(0,1,0,1), holdOrientation=True))

