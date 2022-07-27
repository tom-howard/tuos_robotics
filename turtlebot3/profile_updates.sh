#!/usr/bin/env bash

cd ~
rm -f .bashrc .bash_aliases
wget -O .bashrc https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/turtlebot3/.bashrc
wget -O .bash_aliases https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/turtlebot3/.bash_aliases