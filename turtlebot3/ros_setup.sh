#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source /home/ros/tb3_ws/install/local_setup.bash

export TURTLEBOT3_MODEL=waffle
export LDS_MODEL=LDS-01
if [ ! -f /home/ros/waffle_number ]; then
    # Hostname is 'dia-waffleX', so save X (a unique number) to file:
    echo "$(hostname | tr -d -c 0-9)" > /home/ros/waffle_number 
fi
export WAFFLE_NO=$(cat /home/ros/waffle_number)
export ROS_DOMAIN_ID=$WAFFLE_NO

source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/humble/
source /usr/share/colcon_cd/function/colcon_cd-argcomplete.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

export ROS_DISCOVERY_SERVER=127.0.0.1:11811