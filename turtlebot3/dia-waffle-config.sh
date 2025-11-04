#!/usr/bin/env bash

ROS_VER=jazzy

source /opt/ros/${ROS_VER}/setup.bash
source /home/ros/tb3_ws/install/local_setup.bash

LOCAL_WS_SETUP=${HOME}/ros2_ws/install/local_setup.bash
if [ -f "${LOCAL_WS_SETUP}" ]; then
  source ${LOCAL_WS_SETUP}
fi

export TURTLEBOT3_MODEL=waffle
export LDS_MODEL=LDS-01
export WAFFLE_NO=$(hostname | tr -d -c 0-9)
export ROS_DOMAIN_ID=$WAFFLE_NO

source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/${ROS_VER}/
source /usr/share/colcon_cd/function/colcon_cd-argcomplete.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
