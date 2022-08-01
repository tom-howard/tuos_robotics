#!/usr/bin/env bash

cd /usr/local/bin/
sudo rm -f diamond_tools

sudo wget https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/turtlebot3/diamond_tools
sudo chmod +x diamond_tools

rm -f /tmp/profile_updates.sh
wget -O /tmp/profile_updates.sh https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/turtlebot3/profile_updates.sh
chmod +x /tmp/profile_updates.sh

# run as admin
/tmp/profile_updates.sh

# run as user
sudo -i -u robot "/tmp/profile_updates.sh"
