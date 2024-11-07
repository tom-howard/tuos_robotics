#!/usr/bin/env bash

ROS_WS="/home/ros/tb3_ws"
REPO_DIR="/home/ros/repos"

rm -rf $ROS_WS
# Make a new workspace:
mkdir -p $ROS_WS/src

cd $REPO_DIR/turtlebot3/
# git pull --quiet
tb3_pkgs=("bringup" "description" "node" "teleop")
for tb3_pkg in ${tb3_pkgs[@]}; do
  cp -r turtlebot3_$tb3_pkg $ROS_WS/src/
done

cd $REPO_DIR/tuos_ros/
git pull --quiet
tuos_pkgs=("tb3_tools")
for tuos_pkg in ${tuos_pkgs[@]}; do
  rm -rf $ROS_WS/src/tuos_$tuos_pkg
  cp -r tuos_$tuos_pkg $ROS_WS/src/
done

cd $ROS_WS/ && colcon --log-level ERROR build --symlink-install
# cd $ROS_WS/ && colcon --log-level ERROR build --packages-select tuos_tb3_tools --symlink-install