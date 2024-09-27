#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source /home/ros/tb3_ws/install/local_setup.bash

export TURTLEBOT3_MODEL=waffle
export LDS_MODEL=LDS-01
export WAFFLE_NO=$(cat ~/.tuos/waffle_number 2>/dev/null)
export ROS_DOMAIN_ID=$WAFFLE_NO

source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/humble/
source /usr/share/colcon_cd/function/colcon_cd-argcomplete.bash

export ROS_DISCOVERY_SERVER=127.0.0.1:11811