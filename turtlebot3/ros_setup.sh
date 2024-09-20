#!/usr/bin/env bash

export TURTLEBOT3_MODEL=waffle
source /opt/ros/humble/setup.bash
source $HOME/tb3_ws/install/local_setup.bash
export WAFFLE_NO=$(cat ~/.tuos/waffle_number 2>/dev/null)

source /usr/share/colcon_cd/function/colcon_cd.sh
source /usr/share/colcon_cd/function/colcon_cd-argcomplete.bash
