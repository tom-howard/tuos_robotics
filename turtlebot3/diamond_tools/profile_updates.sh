#!/usr/bin/env bash

rm -f ~/.bash_aliases ~/.bashrc
wget -O ~/.bash_aliases https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/turtlebot3/.bash_aliases
cp /etc/skel/.bashrc ~/

wget -O /tmp/.bashrc_extras https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/turtlebot3/.bashrc_extras
tmp_file=/tmp/.bashrc_extras
while IFS= read -r line; do
  echo "$line" >> ~/.bashrc
done < "$tmp_file"