#!/usr/bin/env bash

cd ${HOME} && rm -rf tuos_robotics
git clone -qb humble https://github.com/tom-howard/tuos_robotics.git tuos_robotics

SRC_DIR=${HOME}/tuos_robotics/wsl/source

cd ${SRC_DIR}

sudo install waffle /usr/local/bin/
sudo install wsl_ros /usr/local/bin/
sudo install diamond_tools /usr/local/bin/

SCRIPTS_PATH=${HOME}/.diamond
cd ${SCRIPTS_PATH}
rm -f bash_aliases wsl_ros_setup.sh
cp ${SRC_DIR}/bash_aliases ./
cp ${SRC_DIR}/wsl_ros_setup.sh ./

cd ${HOME} && rm -rf tuos_robotics

echo "[INFO] Update complete."

