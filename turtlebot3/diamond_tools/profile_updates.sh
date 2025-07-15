#!/usr/bin/env bash

SRC_DIR="/home/ros/repos/tuos_robotics/turtlebot3"

mkdir -p $HOME/.diamond/diamond_tools/

rm -f $HOME/.bash_aliases $HOME/.bashrc $HOME/.diamond/dia-waffle-config.sh
cd $SRC_DIR
cp bash_aliases $HOME/.bash_aliases 
cp dia-waffle-config.sh $HOME/.diamond/
cp /etc/skel/.bashrc $HOME/

echo "" >> $HOME/.bashrc
echo "source $HOME/.diamond/dia-waffle-config.sh" >> $HOME/.bashrc
echo "" >> $HOME/.bashrc