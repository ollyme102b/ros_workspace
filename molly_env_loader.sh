#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/ubuntu/Project_Olly/ros_workspace/devel/setup.bash
export ROS_IP=10.42.0.29
export ROS_MASTER_URI=http://10.42.0.187:11311

exec "$@"
