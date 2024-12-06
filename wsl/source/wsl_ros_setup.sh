# Custom bashrc settings for wsl_ros

echo -e "Ahem... Hello world."

source /opt/ros/humble/setup.bash
source $HOME/ros2_ws/install/local_setup.bash
source /usr/share/gazebo/setup.bash

export ROS_LOCALHOST_ONLY=1

export TURTLEBOT3_MODEL=waffle
export ROS_DOMAIN_ID=$WAFFLE_NO

source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/humble/
source /usr/share/colcon_cd/function/colcon_cd-argcomplete.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

export WSL_ROS_VER=$(cat $HOME/.tuos/wsl_ros_ver)
# Change terminal prompt:
PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@WSL-ROS2($WSL_ROS_VER)\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '

# # GUI/graphics:
source $HOME/.tuos/xserver.sh

if [ "$XSERVER" = true ]; then
  ## Configuring DISPLAY for X-Server GUI apps
  ipconfig.exe | grep 'IPv4' | awk {'print $NF'} > $HOME/.ipv4s && dos2unix -q $HOME/.ipv4s
  read -r line < $HOME/.ipv4s 
  export DISPLAY=$line:0.0 && rm $HOME/.ipv4s
  
  export LIBGL_ALWAYS_INDIRECT=
  export GAZEBO_IP=127.0.0.1
fi
export LIBGL_ALWAYS_SOFTWARE=true

# # display a rosrestore prompt to the user
# # if this is the first launch of WSL-ROS:
# if [ ! -f ~/.wsl-ros/no_welcome ]; then
#   touch ~/.wsl-ros/no_welcome
#   wsl_ros first-launch
# fi
