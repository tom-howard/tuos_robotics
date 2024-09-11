#!/usr/bin/env bash

rm -f ~/.bash_aliases ~/.bashrc
wget -O ~/.bash_aliases https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/turtlebot3/.bash_aliases
cp /etc/skel/.bashrc ~/

echo "" >> ~/.bashrc
tmp_file=~/.bashrc_extras
wget -O $tmp_file https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/turtlebot3/.bashrc_extras
while IFS= read -r line; do
  echo "$line" >> ~/.bashrc
done < "$tmp_file"
rm -f $tmp_file
echo "" >> ~/.bashrc