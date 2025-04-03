#!/usr/bin/env bash

ROS_VER="humble"
SRC_DIR="/home/laptop/repos/tuos_robotics/laptops"

echo "$(hostname | tr -d -c 0-9)" > /home/laptop/laptop_number
cp /home/laptop/laptop_number /home/laptop/waffle_number

echo "Updating system..."
sleep 4

sudo apt update # && sudo apt upgrade -y
sudo apt install -y python3-pandas
sudo apt autoremove

echo "Enabling multicast on loopback..."
sleep 2
sudo ip link set lo multicast on

echo "Updating TUoS Scripts..."
sleep 4

cd $SRC_DIR
sudo install robot_mode /usr/local/bin/
sudo install wsl_ros /usr/local/bin/

cd $SRC_DIR/diamond_tools
sudo install diamond_tools /usr/local/bin/
cp profile_updates.sh /tmp/
cp /tmp/profile_updates.sh $HOME/.tuos/diamond_tools/profile_updates-$(date +'%Y%m%d%H%M%S')
chmod +x /tmp/profile_updates.sh

cd $SRC_DIR/waffle_cli
sudo install waffle /usr/local/bin/
sudo cp robot_pair_check.sh /usr/local/bin/
sudo cp robot_pairing.sh /usr/local/bin/
sudo cp robot_sync.sh /usr/local/bin/ 

echo "Updating user profiles..."
sleep 2

cd $HOME
# run as admin
/tmp/profile_updates.sh
# run as user
sudo -i -u student "/tmp/profile_updates.sh"

echo "Updates are complete."
