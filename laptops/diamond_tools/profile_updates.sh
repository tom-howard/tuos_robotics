#!/usr/bin/env bash

mkdir -p $HOME/.tuos/diamond_tools/

rm -f $HOME/.bash_aliases $HOME/.bashrc $HOME/.tuos/waffle_setup.sh
wget -O $HOME/.bash_aliases https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/turtlebot3/.bash_aliases
wget -O $HOME/.tuos/waffle_setup.sh https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/waffle_setup.sh
cp /etc/skel/.bashrc $HOME/

echo "" >> $HOME/.bashrc
echo "source $HOME/.tuos/waffle_setup.sh" >> $HOME/.bashrc
echo "" >> $HOME/.bashrc