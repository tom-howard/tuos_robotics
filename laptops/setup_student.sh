#!/usr/bin/env bash

GRN='\033[0;32m'
NC='\033[0m'

name_ros_version="humble"
name_ros2_workspace="ros2_ws"

/tmp/profile_updates.sh

source $HOME/.bashrc

diamond_tools workspace
