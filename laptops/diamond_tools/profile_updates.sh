#!/usr/bin/env bash

cd ~
rm -f ~/.bash_aliases ~/.bashrc
wget -O ~/.bash_aliases https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/.bash_aliases
wget -O ~/.bashrc https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/.bashrc # DOESN'T EXIST YET!

cd ~/.tuos
rm -f bashrc_miro bashrc_turtlebot3 bashrc_robot_switch
wget -O ~/.tuos/bashrc_miro https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/bashrc_miro
wget -O ~/.tuos/bashrc_robot_switch https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/bashrc_robot_switch
wget -O ~/.tuos/bashrc_turtlebot3 https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/bashrc_turtlebot3
