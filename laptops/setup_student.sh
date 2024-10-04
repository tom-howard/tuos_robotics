#!/usr/bin/env bash

GRN='\033[0;32m'
NC='\033[0m'

name_ros_version=$1
name_ros2_workspace=$2

echo -e "\n${GRN}[Setting up the environment]"
echo "source /opt/ros/$name_ros_version/setup.bash" >> $HOME/.bashrc
source $HOME/.bashrc

echo -e "\n${GRN}[Create and build the ROS2 workspace]${NC}"
mkdir -p $HOME/$name_ros2_workspace/src
cd $HOME/$name_ros2_workspace
colcon build --symlink-install

/tmp/profile_updates.sh
source $HOME/.bashrc

diamond_tools workspace