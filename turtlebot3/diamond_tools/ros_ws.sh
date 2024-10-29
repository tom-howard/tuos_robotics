#!/usr/bin/env bash

ROS2_WS="/home/ros/tb3_ws"

rm -rf $ROS2_WS

# Make a new workspace:
mkdir -p $ROS2_WS/src

# Clone the TB3 repo:
cd $ROS2_WS/src
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
cd $ROS2_WS/src/turtlebot3
rm -rf turtlebot3_cartographer turtlebot3_navigation2 turtlebot3_example

# Clone the tuos_ros repo
cd $ROS2_WS/src/
git clone -b humble https://github.com/tom-howard/tuos_ros.git
cd $ROS2_WS/src/tuos_ros/
rm -rf tuos_examples/ com2009_simulations/ tuos_simulations/

cd $ROS2_WS/ && colcon build --symlink-install
