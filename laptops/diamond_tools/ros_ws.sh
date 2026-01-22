#!/usr/bin/env bash

SHARE_DIR="/home/ros"
ROS2_WS="${SHARE_DIR}/ros2_ws"

if id -nG "${USER}" | grep -qw "sudo"; then
  
  # remove tuos_ros (if it exists)
  rm -rf ${ROS2_WS}
  # Re-make the workspace:
  mkdir -p ${ROS2_WS}/src
  
  # TUOS_ROS
  if [ ! -d ${SHARE_DIR}/repos/tuos_ros ]; then
    git clone -b jazzy https://github.com/tom-howard/tuos_ros.git ${SHARE_DIR}/repos/tuos_ros
  else
    echo "Updating 'tuos_ros'..."
    cd ${SHARE_DIR}/repos/tuos_ros && git pull --quiet
  fi
  # Copy tuos_ros to the ros workspace
  cp -r ${SHARE_DIR}/repos/tuos_ros ${ROS2_WS}/src/
  
  # com_offer_holder_days
  if [ ! -d ${SHARE_DIR}/repos/com_offer_holder_days ]; then
    git clone https://github.com/tom-howard/com_offer_holder_days.git ${SHARE_DIR}/repos/com_offer_holder_days
  else
    echo "Updating 'com_offer_holder_days'..."
    cd ${SHARE_DIR}/repos/com_offer_holder_days && git pull --quiet
  fi
  cp -r ${SHARE_DIR}/repos/com_offer_holder_days ${ROS2_WS}/src/
  
  # Build the workspace
  cd ${ROS2_WS}/ && colcon --log-level ERROR build 
else
  echo "Skipped updates, cleaning up instead."
  mkdir -p ${HOME}/ros2_ws/src/
  rm -rf ${HOME}/ros2_ws/src/tuos_ros
  rm -rf ${HOME}/ros2_ws/src/com_offer_holder_days
  rm -rf ${HOME}/ros2_ws/build ${HOME}/ros2_ws/install ${HOME}/ros2_ws/log
fi
