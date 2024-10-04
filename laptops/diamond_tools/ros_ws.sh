#!/usr/bin/env bash

ROS2_WS="$HOME/ros2_ws"

rm -rf $ROS2_WS

# Make a new workspace:
mkdir -p $ROS2_WS/src

# Clone the tuos_ros repo
cd $ROS2_WS/src/
git clone -b humble https://github.com/tom-howard/tuos_ros.git
cd $ROS2_WS/src/tuos_ros/

cd $ROS2_WS/ && colcon build --symlink-install