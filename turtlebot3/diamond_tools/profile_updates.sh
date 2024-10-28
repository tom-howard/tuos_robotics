#!/usr/bin/env bash

mkdir -p $HOME/.tuos/diamond_tools/

rm -f $HOME/.bash_aliases $HOME/.bashrc $HOME/.tuos/tuos_robot_setup.sh
wget -qO $HOME/.bash_aliases https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/turtlebot3/.bash_aliases
wget -qO $HOME/.tuos/tuos_robot_setup.sh https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/turtlebot3/tuos_robot_setup.sh
cp /etc/skel/.bashrc $HOME/

echo "" >> $HOME/.bashrc
echo "source $HOME/.tuos/tuos_robot_setup.sh" >> $HOME/.bashrc
echo "" >> $HOME/.bashrc