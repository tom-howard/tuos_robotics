#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source /home/ros/tb3_ws/install/local_setup.bash

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
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

tb3_bringup() {
  ZENOH_ID=$(pgrep zenoh)
  if [[ $ZENOH_ID ]]; then
    if [[ "$1" == "restart" || "$1" == "r" ]]; then
      echo "Killing an existing Zenoh bridge (PID: $ZENOH_ID)."
      pkill zenoh
      echo "Launching a Zenoh Bridge..."
      sleep 3
      zenoh-bridge-ros2dds &
    else
      echo "A Zenoh bridge is already running (PID: $ZENOH_ID)."
    fi
  else
    echo "Launching a Zenoh Bridge..."
    sleep 3
    zenoh-bridge-ros2dds &
  fi
  echo "Launching ROS [ros2 launch tuos_tb3_tools ros.launch.py]"
  sleep 3
  ros2 launch tuos_tb3_tools ros.launch.py
}
