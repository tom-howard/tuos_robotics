#!/usr/bin/env bash

SRC_DIR="/home/ros/repos/tuos_robotics/laptops"

mkdir -p $HOME/.diamond/diamond_tools/

rm -f $HOME/.bash_aliases $HOME/.bashrc $HOME/.diamond/tuos_laptop_setup.sh
cd $SRC_DIR
cp bash_aliases $HOME/.bash_aliases
cp tuos_laptop_setup.sh $HOME/.diamond/
cp /etc/skel/.bashrc $HOME/

echo "" >> $HOME/.bashrc
echo "source $HOME/.diamond/tuos_laptop_setup.sh" >> $HOME/.bashrc
echo "" >> $HOME/.bashrc
