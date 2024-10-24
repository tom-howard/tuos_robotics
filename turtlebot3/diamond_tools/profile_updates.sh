#!/usr/bin/env bash

mkdir -p $HOME/.tuos/diamond_tools/

rm -f $HOME/.bash_aliases $HOME/.bashrc $HOME/.tuos/ros_setup.sh
wget -qO $HOME/.bash_aliases https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/turtlebot3/.bash_aliases
wget -qO $HOME/.tuos/ros_setup.sh https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/turtlebot3/ros_setup.sh
cp /etc/skel/.bashrc $HOME/

echo "" >> $HOME/.bashrc
echo "source $HOME/.tuos/ros_setup.sh" >> $HOME/.bashrc
echo "" >> $HOME/.bashrc