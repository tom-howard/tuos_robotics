# Custom bashrc settings for wsl_ros

source ${HOME}/.tuos/bash_aliases

source /opt/ros/humble/setup.bash
WS_INSTALL_DIR=$HOME/ros2_ws/install/local_setup.bash
if [ -f "${WS_INSTALL_DIR}" ]; then
  source ${WS_INSTALL_DIR}
fi
source /usr/share/gazebo/setup.bash

export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=1
export TURTLEBOT3_MODEL=waffle

source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/humble/
source /usr/share/colcon_cd/function/colcon_cd-argcomplete.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

export WSL_ROS_VER=$(cat /home/tuos/wsl_ros_ver)
# Change terminal prompt:
PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@WSL-ROS2($WSL_ROS_VER)\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '

# # GUI/graphics:
source $HOME/.tuos/xserver.sh

if [ "${XSERVER}" = true ]; then
  ## Configuring DISPLAY for X-Server GUI apps
  ipconfig.exe | grep 'IPv4' | awk {'print $NF'} > $HOME/.tuos/ipv4s && dos2unix -q $HOME/.tuos/ipv4s
  read -r line < $HOME/.tuos/ipv4s 
  export DISPLAY=$line:0.0 && rm $HOME/.tuos/ipv4s
  
  export LIBGL_ALWAYS_INDIRECT=
  export GAZEBO_IP=127.0.0.1
fi
export LIBGL_ALWAYS_SOFTWARE=true

# WSL Ops:
export WINUSER=$(wslvar USERNAME 2>/dev/null)
export WINHOMEDRIVE=$(wslvar HOMEDRIVE 2>/dev/null)
if [ "${WINHOMEDRIVE}" == "U:" ]; then
  export MANWIN=true
  sudo mkdir -p /mnt/u
  sudo mount -t drvfs U: /mnt/u 2>/dev/null
else
  export MANWIN=false
fi

# display a wsl_ros restore prompt to the user
# if this is the first launch of WSL-ROS:
if [[ ! -f ~/.tuos/no_welcome ]]; then
  wsl_ros first-launch
fi