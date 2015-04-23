#!/bin/bash

export ROS_MASTER_URI=http://omnimaxbot:11311
export ROS_HOSTNAME=omnimaxbot
export ROS_IP=192.168.1.101
export ROSLAUNCH_SSH_UNKNOWN=1

. /home/marslab/catkin_ws/devel/setup.sh
exec "$@"
