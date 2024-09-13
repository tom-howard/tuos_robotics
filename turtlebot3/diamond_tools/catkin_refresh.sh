#!/usr/bin/env bash

rm -rf ~/tb3_ws/

# Make a new catkin workspace:
mkdir -p ~/tb3_ws/src

# Clone the TB3 repo:
cd ~/tb3_ws/src
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
cd ~/tb3_ws/src/turtlebot3
rm -r turtlebot3_description/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/

# Clone the tuos_ros repo
cd ~/tb3_ws/src/
git clone https://github.com/tom-howard/tuos_ros.git
cd ~/tb3_ws/src/tuos_ros/
rm -rf tuos_examples/ com2009_simulations/ tuos_simulations/
cd ~/tb3_ws/ && catkin build

# copy the pre-built Catkin Workspace over for use by 'robot':
sudo rm -rf /home/robot/tb3_ws/
sudo cp -r ~/tb3_ws/ /home/robot/
sudo chmod -R 777 /home/robot/tb3_ws/
