#!/usr/bin/env bash

mkdir -p $HOME/.tuos/diamond_tools/

rm -f $HOME/.bash_aliases $HOME/.bashrc $HOME/.tuos/tuos_laptop_setup.sh
wget -qO $HOME/.bash_aliases https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/.bash_aliases
wget -qO $HOME/.tuos/tuos_laptop_setup.sh https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/tuos_laptop_setup.sh
cp /etc/skel/.bashrc $HOME/

echo "" >> $HOME/.bashrc
echo "source $HOME/.tuos/tuos_laptop_setup.sh" >> $HOME/.bashrc
echo "" >> $HOME/.bashrc