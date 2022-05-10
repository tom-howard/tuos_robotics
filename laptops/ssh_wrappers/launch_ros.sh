#!/usr/bin/env bash

# CC BY-NC
# Alex Lucas & Tom Howard, University of Sheffield
# Copyright (c) 2022

YELLOW='\033[1;33m'
NC='\033[0m'

ask() {
    local reply prompt
    prompt='y/n'
    echo -e -n "$1 ${YELLOW} [$prompt] ${NC}>> "
    read -r reply </dev/tty
    if [[ -z $reply ]]; then
        return 1;
    elif [ "$reply" == "y" ] || [ "$reply" == "Y" ]; then
        return 0;
    else
        return 1;
    fi
}

waffle_id=$(hostname)

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=http://$(hostname):11311
export ROS_HOSTNAME=$(hostname)

if [ ! $(rosnode list >/dev/null 2>&1) ] || [ $(rosnode list | grep -v rosout -c) == 0 ]; then
    true # ROS isn't running, so OK to continue... 
else
    if ask "It looks like ROS is already running, do you want to stop all processes and launch it again?"; then
        rosnode kill -a
    else
        exit 0
    fi
fi

echo "[$waffle_id]: Launching ROS... (you can close down this window now)."
roslaunch tb3_tools tb3.launch 2>/dev/null