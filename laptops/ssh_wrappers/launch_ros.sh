#!/usr/bin/env bash

# CC BY-NC
# Alex Lucas & Tom Howard, University of Sheffield
# Copyright (c) 2022

waffle_id=$(hostname)

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=http://$(hostname):11311
export ROS_HOSTNAME=$(hostname)

echo "[$waffle_id]: Launching ROS..."
# roslaunch tb3_tools tb3.launch > /dev/null 2>&1
# roslaunch tb3_tools tb3.launch >/dev/null
roslaunch tb3_tools tb3.launch 2>/dev/null