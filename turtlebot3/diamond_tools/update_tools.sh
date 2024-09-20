#!/usr/bin/env bash

# sudo apt update && sudo apt upgrade -y

files="/usr/local/bin/diamond_tools /usr/local/bin/wsl_ros /usr/local/bin/waffle"
sudo rm -f $files

sudo wget -O /usr/local/bin/diamond_tools https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/turtlebot3/diamond_tools/diamond_tools
sudo wget -O /usr/local/bin/waffle https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/turtlebot3/waffle
sudo wget -O /usr/local/bin/wsl_ros https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/turtlebot3/wsl_ros
sudo chmod +x $files

sudo wget -O /etc/systemd/system/fastdds.service https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/turtlebot3/startup_service/fastdds.service
sudo systemctl enable fastdds.service

rm -f /tmp/profile_updates.sh
wget -O /tmp/profile_updates.sh https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/turtlebot3/diamond_tools/profile_updates.sh
chmod +x /tmp/profile_updates.sh

# run as admin
/tmp/profile_updates.sh
diamond_tools workspace

# run as 'robot'
sudo -i -u robot "/tmp/profile_updates.sh"
