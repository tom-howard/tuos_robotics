#!/usr/bin/env bash

cd ~
rm -f ~/.bash_aliases ~/.bashrc
wget -O ~/.bash_aliases https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/.bash_aliases
cp /etc/skel/.bashrc ~/

wget -O /tmp/.bashrc_extras https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/.bashrc_extras
tmp_file=/tmp/.bashrc_extras
while IFS= read -r line; do
  echo "$line" >> ~/.bashrc
done < "$tmp_file"

cd ~/.tuos
rm -f bashrc_miro bashrc_turtlebot3 bashrc_robot_switch bashrc_conda
wget -O ~/.tuos/bashrc_miro https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/bashrc_miro
wget -O ~/.tuos/bashrc_robot_switch https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/bashrc_robot_switch
wget -O ~/.tuos/bashrc_turtlebot3 https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/bashrc_turtlebot3
wget -O ~/.tuos/bashrc_conda https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/bashrc_conda
