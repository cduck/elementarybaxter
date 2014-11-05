#!/bin/bash

# Disable Baxter's sonar
rostopic pub /robot/sonar/headonar/set_sonars_enabled std_msgs/UInt16 0 >/dev/null 2>/dev/null &

# Path planning setup
echo "\nEnable Baxter\n\n"
rosrun baxter_tools enable_robot.py -e

echo "\n\nStart trajectory controller\n\n"
rosrun baxter_interface joint_trajectory_action_server.py &

echo "\n\nLaunch moveit\n\n"
roslaunch baxter_moveit_config move_group.launch &
