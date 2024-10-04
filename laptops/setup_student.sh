#!/usr/bin/env bash

GRN='\033[0;32m'
NC='\033[0m'

name_ros_version="humble"
name_ros2_workspace="ros2_ws"

echo -e "\n${GRN}[Setting up the environment]"
echo "source /opt/ros/$name_ros_version/setup.bash" >> $HOME/.bashrc
source $HOME/.bashrc

/tmp/profile_updates.sh

diamond_tools workspace

source $HOME/.bashrc
