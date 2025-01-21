#!/usr/bin/env bash

ROS2_WS="$HOME/ros2_ws"
SHARE_DIR="/home/laptop"

# remove tuos_ros (if it exists)
rm -rf $ROS2_WS/src/tuos_ros

# Make a new workspace (if necessary):
mkdir -p $ROS2_WS/src

# Copy the tuos_ros repo
cd $SHARE_DIR/repos/tuos_ros && git pull
cp -r $SHARE_DIR/repos/tuos_ros $ROS2_WS/src/

cd $ROS2_WS/ && colcon --log-level ERROR build --packages-up-to tuos_ros