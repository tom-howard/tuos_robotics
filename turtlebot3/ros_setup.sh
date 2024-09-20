#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source $HOME/tb3_ws/install/local_setup.bash

export TURTLEBOT3_MODEL=waffle
export LDS_MODEL=LDS-01
export WAFFLE_NO=$(cat ~/.tuos/waffle_number 2>/dev/null)

source /usr/share/colcon_cd/function/colcon_cd.sh
source /usr/share/colcon_cd/function/colcon_cd-argcomplete.bash
