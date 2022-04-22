#!/usr/bin/env bash
# CC BY-NC
# Alex Lucas & Tom Howard, University of Sheffield
# Copyright (c) 2022

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=http://$(hostname):11311
export ROS_HOSTNAME=$(hostname)

echo "launching ros..."
roslaunch turtlebot3_bringup turtlebot3_robot.launch