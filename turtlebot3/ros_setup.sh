#!/usr/bin/env bash

ROS2_WS="/home/ros/tb3_ws"

source /opt/ros/humble/setup.bash
source $ROS2_WS/install/local_setup.bash

export TURTLEBOT3_MODEL=waffle
export LDS_MODEL=LDS-01
export WAFFLE_NO=$(hostname | tr -d -c 0-9)
export ROS_DOMAIN_ID=$WAFFLE_NO

source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/humble/
source /usr/share/colcon_cd/function/colcon_cd-argcomplete.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# Zenoh related mods:
# export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export PATH=$PATH:$ROS2_WS/src/zenoh-plugin-ros2dds/target/release
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp