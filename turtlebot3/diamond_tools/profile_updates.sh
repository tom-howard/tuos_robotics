#!/usr/bin/env bash

SRC_DIR="/home/ros/repos/tuos_robotics/turtlebot3"

mkdir -p $HOME/.tuos/diamond_tools/

rm -f $HOME/.bash_aliases $HOME/.bashrc $HOME/.tuos/dia-waffle-config.sh
cd $SRC_DIR
cp bash_aliases $HOME/.bash_aliases 
cp dia-waffle-config.sh $HOME/.tuos/
cp /etc/skel/.bashrc $HOME/

echo "" >> $HOME/.bashrc
echo "source $HOME/.tuos/dia-waffle-config.sh" >> $HOME/.bashrc
echo "" >> $HOME/.bashrc