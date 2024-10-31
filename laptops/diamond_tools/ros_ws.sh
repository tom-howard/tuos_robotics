#!/usr/bin/env bash

ROS2_WS="$HOME/ros2_ws"
SHARE_DIR="/home/laptop"

rm -rf $ROS2_WS

# Make a new workspace:
mkdir -p $ROS2_WS/src

# Copy the tuos_ros repo
cd $SHARE_DIR/repos/tuos_ros && git pull
cp -r $SHARE_DIR/repos/tuos_ros $ROS2_WS/src/

cd $ROS2_WS/ && colcon --log-level ERROR build --symlink-install