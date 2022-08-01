#!/usr/bin/env bash

sudo rm -f /usr/local/bin/diamond_tools

sudo wget -O /usr/local/bin/diamond_tools https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/turtlebot3/diamond_tools/diamond_tools
sudo chmod +x /usr/local/bin/diamond_tools

rm -f /tmp/profile_updates.sh
wget -O /tmp/profile_updates.sh https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/turtlebot3/diamond_tools/profile_updates.sh
chmod +x /tmp/profile_updates.sh

# run as admin
/tmp/profile_updates.sh

# run as user
sudo -i -u robot "/tmp/profile_updates.sh"
