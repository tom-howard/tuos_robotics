#!/usr/bin/env bash

rm -rf ~/catkin_ws/

# Make a new catkin workspace:
mkdir -p ~/catkin_ws/src

# Clone the TB3 repo:
cd ~/catkin_ws/src
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
cd ~/catkin_ws/src/turtlebot3
rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/

# Clone the COM2009 repo
cd ~/catkin_ws/src/
git clone https://github.com/tom-howard/COM2009.git
cd ~/catkin_ws/src/COM2009/
rm -rf com2009_assignment2/ com2009_examples/ com2009_simulations/
cd ~/catkin_ws/ && catkin build
