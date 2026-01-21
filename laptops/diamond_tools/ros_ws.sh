#!/usr/bin/env bash

SHARE_DIR="/home/ros"
ROS2_WS="${SHARE_DIR}/ros2_ws"

if id -nG "${USER}" | grep -qw "sudo"; then
  echo "Updating 'tuos_ros'..."
  # remove tuos_ros (if it exists)
  rm -rf ${ROS2_WS}
  # Re-make the workspace:
  mkdir -p ${ROS2_WS}/src
  cd ${SHARE_DIR}/repos/tuos_ros && git pull --quiet
  # Copy the tuos_ros repo
  cp -r ${SHARE_DIR}/repos/tuos_ros ${ROS2_WS}/src/

  cd ${ROS2_WS}/ && colcon --log-level ERROR build --packages-up-to tuos_ros
else
  echo "Skipped 'tuos_ros' updates."
  mkdir -p ${HOME}/ros2_ws/src/
  rm -rf ${HOME}/ros2_ws/src/tuos_ros
  rm -rf ${HOME}/ros2_ws/build ${HOME}/ros2_ws/install ${HOME}/ros2_ws/log
fi
