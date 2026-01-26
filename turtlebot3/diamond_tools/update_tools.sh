#!/usr/bin/env bash

ROS2_WS="/home/ros/tb3_ws"
ROS_VER="jazzy"
SRC_DIR="/home/ros/repos/tuos_robotics/turtlebot3"

echo "$(hostname | tr -d -c 0-9)" > /home/ros/waffle_number

echo "Updating system..."
sleep 4
sudo apt update && sudo apt upgrade -y
sudo apt install -y \
    wavemon \
    avahi-daemon

echo "Updating TUoS Scripts..."
sleep 4

cd $SRC_DIR
sudo install waffle /usr/local/bin/
sudo install wsl_ros /usr/local/bin/
sudo cp eth-cfg.yaml /etc/netplan/99-eth-cfg.yaml
sudo chmod 600 /etc/netplan/99-eth-cfg.yaml

echo "Applying network configuration updates..."
sleep 4
sudo netplan apply
sudo systemctl enable --now avahi-daemon

cd $SRC_DIR/diamond_tools
sudo install diamond_tools /usr/local/bin/
cp profile_updates.sh /tmp/
cp /tmp/profile_updates.sh $HOME/.diamond/diamond_tools/profile_updates-$(date +'%Y%m%d%H%M%S')
chmod +x /tmp/profile_updates.sh

echo "Updating user profiles..."
sleep 2

cd $HOME
# run as admin
/tmp/profile_updates.sh
source $HOME/.bashrc
diamond_tools workspace full

# run as 'robot'
sudo -i -u robot "/tmp/profile_updates.sh"

echo "Updates are complete."