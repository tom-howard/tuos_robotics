#!/usr/bin/env bash

# Delete the old catkin workspace:
rm -rf ~/catkin_ws/

# Then, make a new one:
mkdir -p ~/catkin_ws/src

# Clone the COM2009 repo and build:
cd ~/catkin_ws/src/
git clone https://github.com/tom-howard/tuos_ros.git
cd ~/catkin_ws/ && catkin build
