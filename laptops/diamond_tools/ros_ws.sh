#!/usr/bin/env bash

ROS2_WS="$HOME/ros2_ws"
SHARE_DIR="/home/laptop"

# remove tuos_ros (if it exists)
rm -rf $ROS2_WS

# Make a new workspace (if necessary):
mkdir -p $ROS2_WS/src

if [[ "${USER}" == "diamond" ]]; then
  echo "Updating 'tuos_ros'..."
  cd $SHARE_DIR/repos/tuos_ros && git pull --quiet
else
  echo "Skipped 'tuos_ros' updates."
fi

# Copy the tuos_ros repo
cp -r $SHARE_DIR/repos/tuos_ros $ROS2_WS/src/

cd $ROS2_WS/ && colcon --log-level ERROR build --packages-up-to tuos_ros