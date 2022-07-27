#!/usr/bin/env bash

cd /usr/local/bin/
sudo rm -f diamond_tools

sudo wget https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/turtlebot3/diamond_tools
sudo chmod +x diamond_tools

cd ~
rm -f .bashrc .bash_aliases
wget -O .bashrc https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/turtlebot3/.bashrc
wget -O .bash_aliases https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/turtlebot3/.bash_aliases

sudo -i -u robot
cd ~
rm -f .bashrc .bash_aliases
wget -O .bashrc https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/turtlebot3/.bashrc
wget -O .bash_aliases https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/turtlebot3/.bash_aliases
exit
