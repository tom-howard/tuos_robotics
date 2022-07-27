#!/usr/bin/env bash

cd /usr/local/bin/
sudo rm -f diamond_tools

sudo wget https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/turtlebot3/diamond_tools
sudo chmod +x diamond_tools

rm -rf /tmp/profile_updates.sh
wget -O /tmp/profile_updates.sh https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/turtlebot3/profile_updates.sh
chmod +x /tmp/profile_updates.sh

cd ~
rm -f .bashrc .bash_aliases
wget -O .bashrc https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/turtlebot3/.bashrc
wget -O .bash_aliases https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/turtlebot3/.bash_aliases

# run as admin
/tmp/profile_updates.sh

# run as user
sudo -i -u robot "/tmp/profile_updates.sh"
