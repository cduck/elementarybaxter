#!/bin/bash

# Disable Baxter's sonar
rostopic pub /robot/sonar/head_sonar/set_sonars_enabled std_msgs/UInt16 0x0000 -r 1 &

# Path planning setup
echo "\nEnable Baxter\n\n"
rosrun baxter_tools enable_robot.py -e

echo "\n\nStart trajectory controller\n\n"
rosrun baxter_interface joint_trajectory_action_server.py &

echo "\n\nLaunch moveit\n\n"
while [ 1 ]; do
  roslaunch baxter_moveit_config move_group.launch
done
