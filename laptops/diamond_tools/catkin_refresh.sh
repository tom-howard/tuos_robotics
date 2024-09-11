#!/usr/bin/env bash

# Delete the old catkin workspace:
rm -rf ~/tb3_ws/

# Then, make a new one:
mkdir -p ~/tb3_ws/src

# Clone the COM2009 repo and build:
cd ~/tb3_ws/src/
git clone https://github.com/tom-howard/tuos_ros.git
cd ~/tb3_ws/ && catkin build
