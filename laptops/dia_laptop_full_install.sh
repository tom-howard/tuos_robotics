#!/bin/bash
# CC BY-NC
# Alex Lucas & Tom Howard, University of Sheffield
# Copyright (c) 2022

ask() {
    local reply prompt
    prompt='y/n'
    echo -n "$1 [$prompt] >> "
    read -r reply </dev/tty
    if [[ -z $reply ]]; then
        return 1;
    elif [ "$reply" == "y" ] || [ "$reply" == "Y" ]; then
        return 0;
    else
        return 1;
    fi
}

echo ""
echo "[Note] Target OS version  >>> Ubuntu 20.04.x (Focal Fossa) or Linux Mint 21.x"
echo "[Note] Target ROS version >>> ROS Noetic Ninjemys"
echo "[Note] Catkin workspace   >>> $HOME/catkin_ws"
echo ""
echo

if ! ask "[OK to continue with installation?]"; then
  exit 130
fi
########## Part I. ROS, TB3, catkin_ws and dependencies ################

echo "[Set the target OS, ROS version and the name of catkin workspace]"
name_os_version=${name_os_version:="focal"}
name_ros_version=${name_ros_version:="noetic"}
name_catkin_workspace=${name_catkin_workspace:="catkin_ws"}

echo ""
echo "[Update & Upgrade]"
sudo apt update -y
sudo apt upgrade -y

echo "[NTP: update time]"
sudo apt install -y chrony ntpdate curl build-essential git
sudo ntpdate ntp.ubuntu.com

echo "[Source .bashrc]"
source $HOME/.bashrc

echo "[Add the ROS repository]"
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
  sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu ${name_os_version} main\" > /etc/apt/sources.list.d/ros-latest.list"
fi

echo "[Download the ROS keys]"
roskey=`apt-key list | grep "Open Robotics"`
if [ -z "$roskey" ]; then
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
fi

echo "[Check the ROS keys]"
roskey=`apt-key list | grep "Open Robotics"`
if [ -n "$roskey" ]; then
  echo "[ROS key exists in the list]"
else
  echo "[Failed to receive the ROS key, aborts the installation]"
  exit 0
fi

echo "[Source .bashrc]"
source $HOME/.bashrc

echo "[Update & Upgrade]"
sudo apt update -y
sudo apt upgrade -y

echo "[Install all the ROS and TB3 packages]"
sudo apt install -y ros-noetic-desktop-full ros-noetic-rqt-* ros-noetic-gazebo-* ros-noetic-joy ros-noetic-teleop-twist-joy ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc ros-noetic-rgbd-launch ros-noetic-rosserial-arduino ros-noetic-amcl ros-noetic-map-server ros-noetic-move-base ros-noetic-rqt* ros-noetic-gmapping ros-noetic-navigation python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential ros-noetic-dynamixel-sdk ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations python3-pip python3-catkin-tools ffmpeg

echo "[Initialise rosdep and update]"
sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
sudo sh -c "rosdep init"
rosdep update

echo "[Environment setup]"
source /opt/ros/$name_ros_version/setup.sh
source $HOME/.bashrc

echo "[Create the catkin workspace and do catkin build]"
mkdir -p $HOME/$name_catkin_workspace/src
cd $HOME/$name_catkin_workspace/src
catkin_init_workspace
cd $HOME/$name_catkin_workspace
catkin build

source $HOME/.bashrc

##################### Part II. MDK ######################
if ask "[Do you want to set up MiRo Developer Kit?]"; then
  echo "[Install AprilTag with Pip3]"
  sudo pip3 install apriltag

  echo "[Setting up MDK]"
  mkdir -p ~/pkgs
  cd ~/pkgs/
  wget --no-check-certificate 'https://docs.google.com/uc?export=download&id=1V8pNrwcMY7ucjEzf6NzoAuLK3GORac1k' -O mdk_2-210921.tgz
  tar -xvzf mdk_2-210921.tgz
  cd mdk-210921/bin/script

  # Remove old entries
  rm ~/mdk
  sed -i '/# MDK/d' ~/.bashrc
  sed -i '/source ~\/mdk\/setup.bash/d' ~/.bashrc

  ./install_mdk.sh
  cd ~/mdk/catkin_ws/
  catkin build

  echo "[Adding MDK extras]"
  wget -O ~/mdk/sim/launch_full.sh https://gist.githubusercontent.com/AlexandrLucas/703831843f9b46edc2e2032bcd08651f/raw/launch_full.sh
  chmod +x ~/mdk/sim/launch_full.sh
else
  echo "[Skipping MDK...]"
fi

########## Part III. TUoS Robotics scripts ################
if ask "[Do you want to also set up TUoS Robot Switch scripts?]"; then

  echo "[The following prompt is only relevant for DIA-LAB laptops when connecting to real robots]"
  echo "[Simply hit ENTER if you're installing this on your own PC]"
  echo "[Please enter the dia_laptop number (between 1 and 45)]"
  read -r reply </dev/tty
  if [[ -z $reply ]]; then
    echo "[Ignoring]"
    PC_NO=''
  elif (( $reply >= 1 && $reply <= 45 )); then
      PC_NO=$reply
  else
    echo "[Defaulting to '1']"
    PC_NO=1
  fi

  # Remove the new MDK ~/.bashrc entries (will be done separately in a different way)
  sed -i '/# MDK/d' ~/.bashrc
  sed -i '/source ~\/mdk\/setup.bash/d' ~/.bashrc

  echo "[Setting up /usr/local/bin/ scripts]"
  cd /usr/local/bin/
  sudo wget -O /usr/local/bin/robot_switch https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/robot_switch
  sudo wget -O /usr/local/bin/robot_mode https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/robot_mode
  sudo wget -O /usr/local/bin/pair_with_miro https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/pair_with_miro
  sudo wget -O /usr/local/bin/pair_with_waffle https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/pair_with_waffle
  sudo chmod +x *

  echo "[Setting up ~/.tuos scripts]"
  mkdir -p ~/.tuos
  cd ~/.tuos
  wget -O ~/.tuos/bashrc_miro https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/bashrc_miro
  wget -O ~/.tuos/bashrc_turtlebot3 https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/bashrc_turtlebot3
  wget -O ~/.tuos/bashrc_robot_switch https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/bashrc_robot_switch

  echo "[Setting up 'Shared' group]"
  sudo mkdir -p /home/Shared/
  sudo addgroup sharegroup
  sudo chown :sharegroup /home/Shared
  sudo adduser "$USER" sharegroup

  echo "[Setting device numbers]"
  cd /home/Shared
  sudo touch laptop_number miro_number waffle_number
  sudo chown "$USER" *
  sudo chgrp sharegroup *
  echo "$PC_NO" > laptop_number
  echo "$PC_NO" > miro_number
  echo "$PC_NO" > waffle_number

  echo "[Adding the Robotics Kit switch to ~/.bashrc, if needed]"
  wget -O /tmp/.bashrc_extras https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/.bashrc_extras

  tmp_file=/tmp/.bashrc_extras
  while IFS= read -r line; do
    grep -qxF "$line" ~/.bashrc || echo "$line" >> ~/.bashrc
  done < "$tmp_file"

  echo "[Adding bash aliases, if needed]"
  cd ~
  wget -O /tmp/.bash_aliases https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/.bash_aliases
  touch ~/.bash_aliases
  tmp_file=/tmp/.bash_aliases
  while IFS= read -r line; do
    grep -qxF "$line" ~/.bash_aliases || echo "$line" >> ~/.bash_aliases
  done < "$tmp_file"

else
  echo "[Skipping TUoS Robotics scripts...]"
fi

##################### Part IV. Teaching materials ######################
if ask "[Do you want to download COM2009 and COM3528 teaching materials?]"; then
  cd $HOME/$name_catkin_workspace/src
  git clone https://github.com/tom-howard/COM2009
  catkin build

  cd ~/mdk/catkin_ws/src
  git clone https://github.com/AlexandrLucas/COM3528
  catkin build
fi
##################### Part V. 'Student' profile ######################
#TODO

##################### Part VI. Clean up ######################

echo "[Clean-up]"
sudo apt update -y
sudo apt upgrade -y
sudo apt autoremove -y
sudo apt autoclean -y

echo "[COMPLETE]: Don't forget to reboot ASAP."
exit 0
