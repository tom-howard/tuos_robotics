#!/usr/bin/env bash

SRC_DIR="/home/laptop/repos/tuos_robotics/laptops"

mkdir -p $HOME/.tuos/diamond_tools/

rm -f $HOME/.bash_aliases $HOME/.bashrc $HOME/.tuos/tuos_laptop_setup.sh
cd $SRC_DIR
cp bash_aliases $HOME/.bash_aliases
cp tuos_laptop_setup.sh $HOME/.tuos/
cp /etc/skel/.bashrc $HOME/

echo "" >> $HOME/.bashrc
echo "source $HOME/.tuos/tuos_laptop_setup.sh" >> $HOME/.bashrc
echo "" >> $HOME/.bashrc
