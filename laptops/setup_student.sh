#!/usr/bin/env bash
cd $HOME
# TB3
source /opt/ros/humble/setup.sh
mkdir -p $HOME/tb3_ws/src && cd "$_"
colcon build --symlink-install

# TUoS Robotics scripts
source $HOME/.bashrc
mkdir -p $HOME/.tuos
cd $HOME/.tuos
wget -O $HOME/.tuos/bashrc_turtlebot3 https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/bashrc_turtlebot3

tmp_file=/tmp/.bash_aliases_student
wget -O $tmp_file https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/.bash_aliases
touch $HOME/.bash_aliases
while IFS= read -r line; do
    grep -qxF "$line" $HOME/.bash_aliases || echo "$line" >> $HOME/.bash_aliases
done < "$tmp_file"

# COM2009 and COM3528
source $HOME/.bashrc
cd $HOME/tb3_ws/src
git clone https://github.com/tom-howard/COM2009.git
cd $HOME/tb3_ws
colcon build --symlink-install
