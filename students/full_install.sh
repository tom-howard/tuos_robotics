#!/bin/bash
# CC BY-NC
# Alex Lucas & Tom Howard, University of Sheffield
# Copyright (c) 2022

# This script instals the ROS Noetic environment with extra packages
# to support developing code for the MiRo and Turtlebot 3 robots.
# The script sets up the ROS network for work in simulation (localhost).

RED='\033[0;31m'
YELLOW='\033[1;33m'
GREEN='\033[0;32m'
NC='\033[0m'

ask() {
    local reply prompt
    prompt='y/n'
    echo -e -n "$1 ${YELLOW} [$prompt] ${NC}>> "
    read -r reply </dev/tty
    if [[ -z $reply ]]; then
        return 1;
    elif [ "$reply" == "y" ] || [ "$reply" == "Y" ]; then
        return 0;
    else
        return 1;
    fi
}

echo -e "${YELLOW}[Note] Target OS version >>> Ubuntu 22.04.x (Jammy Jellyfish)${NC}"
echo -e "${YELLOW}[Note] Target ROS version >>> ROS2 Humble Hawksbill${NC}"

if ! ask "[OK to continue with installation?]"; then
  echo -e "${YELLOW}Exiting.${NC}"
  exit 130
fi


#################  Part I. ROS2, TB3, ros2 workspace and dependencies ################
echo -e "\n${YELLOW}[Set the target OS, ROS version and the name of catkin workspace]${NC}"
name_os_version=${name_os_version:="jammy"}
name_ros_version=${name_ros_version:="humble"}
name_ros2_workspace=${name_ros2_workspace:="tb3_ws"}

echo -e "\n${YELLOW}[Update & Upgrade]${NC}"
sudo apt update -y
sudo apt upgrade -y

echo -e "\n${YELLOW}[NTP: update time]${NC}"
sudo apt install -y chrony ntpdate curl build-essential git
sudo ntpdate ntp.ubuntu.com
sleep 2

# Add universe repo
sudo apt install software-properties-common
sudo add-apt-repository universe


# Adding the ROS 2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Adding repo to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt -y update
sudo apt -y upgrade

echo -e "\n${YELLOW}[Source .bashrc]${NC}"
source $HOME/.bashrc

echo -e "\n${YELLOW}[Install all the necessary ROS and TB3 packages]${NC}"
sudo apt install -y build-essential ros-humble-desktop ros-dev-tools ros-humble-gazebo-* \
    ros-humble-cartographer ros-humble-cartographer-ros ros-humble-navigation2 \
    ros-humble-nav2-bringup ros-humble-turtlebot3 ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3-simulations ros-humble-turtlebot3-gazebo \
    python3-rosdep python3-colcon-common-extensions ros-humble-rqt* \
    ros-humble-librealsense2* ros-humble-realsense2-* ffmpeg \
    ros-humble-dynamixel-sdk

sudo apt-get install python3-pip python3-numpy python3-scipy

pip install setuptools==58.2.0


echo -e "\n${YELLOW}[Setting up the environment]"
echo "source /opt/ros/$name_ros_version/setup.bash" >> $HOME/.bashrc
source $HOME/.bashrc

echo -e "\n${YELLOW}[Create and build the ROS2 workspace]${NC}"
mkdir -p $HOME/$name_ros2_workspace/src
cd $HOME/$name_ros2_workspace/src
colcon build --symlink-install

source $HOME/.bashrc

################################# Part II. VS Code #################################

if ask "[Install VS Code?]"; then

  sudo apt update -y
  sudo apt install -y software-properties-common apt-transport-https wget
  # Import the Microsoft GPG key:
  wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
  # Enable the Visual Studio Code repository:
  sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
  # Install Visual Studio Code:
  sudo apt install -y code
  # Update everything for good measure:
  sudo apt update -y
  sudo apt upgrade -y
else
  echo -e "\n${YELLOW}[Skipping VS Code Install...]${NC}"
fi

####################### Part III. TUoS Robotics scripts ########################
if ask "[Do you want to also set up TUoS Robot Switch scripts?]"; then

  echo -e "\n${YELLOW}[Setting up scripts in /usr/local/bin/]${NC}"
  cd /usr/local/bin/
  sudo wget -O /usr/local/bin/robot_switch https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/students/robot_switch
  sudo wget -O /usr/local/bin/robot_mode https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/students/robot_mode
  sudo chmod +x *

  echo -e "\n${YELLOW}[Setting up scripts in  ~/.tuos]${NC}"
  mkdir -p ~/.tuos
  cd ~/.tuos
  wget -O ~/.tuos/bashrc_turtlebot3 https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/students/bashrc_turtlebot3

  echo -e "\n${YELLOW}[Adding the Robotics Kit switch to ~/.bashrc]${NC}"
  wget -O /tmp/.bashrc_extras https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/students/.bashrc_extras

  tmp_file=/tmp/.bashrc_extras
  while IFS= read -r line; do
    grep -qxF "$line" ~/.bashrc || echo "$line" >> ~/.bashrc
  done < "$tmp_file"

  echo -e "\n${YELLOW}[Adding bash aliases]${NC}"
  cd ~
  wget -O /tmp/.bash_aliases https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/students/.bash_aliases
  touch ~/.bash_aliases
  tmp_file=/tmp/.bash_aliases
  while IFS= read -r line; do
    grep -qxF "$line" ~/.bash_aliases || echo "$line" >> ~/.bash_aliases
  done < "$tmp_file"

else
  echo -e "\n${YELLOW}[Skipping TUoS Robotics scripts...]${NC}"
fi

############################ Part IV. VcXsrv for WSL ###########################
echo -e "\n${YELLOW}[NOTE: The following is only required if you are installing on WSL]${NC}"
if ask "[Do you want to set up VcXsrv in .bashrc for running graphical applications in WSL?]"; then
  echo -e "\n${YELLOW}[Setting up VcXsrv in .bashrc]${NC}"
  wget -O /tmp/wsl_bashrc https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/students/wsl_bashrc
  tmp_file=/tmp/wsl_bashrc
  while IFS= read -r line; do
    grep -qxF "$line" ~/.bashrc || echo "$line" >> ~/.bashrc
  done < "$tmp_file"

else
  echo -e "\n${YELLOW}[Skipping VcXsrv settings...]${NC}"
fi

########################## Part V. Teaching materials ##########################
if ask "[Do you want to download COM2009 and COM3528 teaching materials?]"; then
  cd $HOME/$name_ros2_workspace/src
  git clone https://github.com/tom-howard/COM2009
  cd $HOME/$name_ros2_workspace
  colcon build --symlink-install
fi

############################## Part VI. Clean up ###############################
echo -e "\n${YELLOW}[Clean-up]${NC}"
sudo apt update -y
sudo apt upgrade -y
sudo apt autoremove -y
sudo apt autoclean -y

echo -e "\n${GREEN}[COMPLETE]: Don't forget to reboot ASAP.${NC}"
exit 0
