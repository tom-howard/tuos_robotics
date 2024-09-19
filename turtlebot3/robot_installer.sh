#!/bin/bash

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

echo -e "\n${YELLOW}[Set the target OS, ROS version and the name of catkin workspace]${NC}"
name_os_version=${name_os_version:="jammy"}
name_ros_version=${name_ros_version:="humble"}
name_ros2_workspace=${name_ros2_workspace:="tb3_ws"}

# # Disable wait for network during bootup:
# systemctl mask systemd-networkd-wait-online.service

# # Set up a "robot" user
# username="robot"
# pass="panQJvEl/BD/g"
# sudo useradd -s /bin/bash -m -p "$pass" "$username"

# echo -e "\n${YELLOW}[Update & Upgrade]${NC}"
# sudo apt update && sudo apt upgrade -y

# echo -e "\n${YELLOW}[Installing Misc Tools]${NC}"
# sudo apt install -y chrony ntpdate curl build-essential net-tools

# # update git:
# sudo add-apt-repository ppa:git-core/ppa
# sudo apt update -y
# sudo apt install -y git

# # Set locales
# locale  # check for UTF-8
# sudo apt update && sudo apt install locales
# sudo locale-gen en_GB en_GB.UTF-8
# sudo update-locale LC_ALL=en_GB.UTF-8 LANG=en_GB.UTF-8
# locale  # verify settings

# echo -e "\n${YELLOW}[NTP: update time]${NC}"
# sudo ntpdate ntp.ubuntu.com
# sleep 2

# # Make poweroff and ntpdate NO PASSWORD-able
# sudo wget -O /etc/sudoers.d/nopwds https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/turtlebot3/nopwds

# echo "Basic system setup complete (CHECKPOINT 1)."
# sleep 2

### INSTALLING ROS ###

# # Add universe repo
# sudo apt install software-properties-common
# sudo add-apt-repository universe

# # Adding the ROS 2 GPG key
# sudo apt update && sudo apt install curl -y
# sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# # Adding repo to sources list
# echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# sudo apt update && sudo apt upgrade -y

# echo -e "\n${YELLOW}[Source .bashrc]${NC}"
# source $HOME/.bashrc

# echo -e "\n${YELLOW}[Install all the necessary ROS and TB3 packages]${NC}"
# sudo apt install -y ros-humble-ros-base \
#                     ros-dev-tools \
#                     python3-argcomplete \
#                     python3-rosdep \
#                     python3-colcon-common-extensions \
#                     libboost-system-dev \
#                     ros-humble-hls-lfcd-lds-driver \
#                     ros-humble-turtlebot3-msgs \
#                     ros-humble-dynamixel-sdk \
#                     libudev-dev \
#                     python3-pip

# pip install setuptools==58.2.0

# echo -e "\n${YELLOW}[Setting up the environment]${NC}"
# echo "source /opt/ros/$name_ros_version/setup.bash" >> $HOME/.bashrc
# source $HOME/.bashrc
# mkdir -p $HOME/$name_ros2_workspace/src && cd $HOME/$name_ros2_workspace/src
# git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
# cd $HOME/$name_ros2_workspace/src/turtlebot3
# rm -r turtlebot3_cartographer turtlebot3_navigation2
# cd $HOME/$name_ros2_workspace
# colcon build --symlink-install
# echo "source $HOME/$name_ros2_workspace/install/local_setup.bash" >> $HOME/.bashrc
# source $HOME/.bashrc

# echo "ROS installation complete (CHECKPOINT 2)."
# sleep 2

### OpenCR & other TB3 Configs ###

sudo cp `ros2 pkg prefix turtlebot3_bringup`/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc
echo 'export LDS_MODEL=LDS-01' >> ~/.bashrc

mkdir -p ~/firmware/opencr/
cd ~/firmware/opencr/
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
tar -xvf ./opencr_update.tar.bz2
rm opencr_update.tar.bz2

cd opencr_update/
./update.sh /dev/ttyACM0 waffle.opencr

echo "OpenCR configs complete (CHECKPOINT 3)."
sleep 2

### Intel RealSense ###

mkdir -p ~/firmware/realsense/


### Custom TUoS Scripts ###





# ####################### Part IV. TUosimS Robotics scripts ########################
# if ask "[Do you want to also set up TUoS scripts?]"; then

#   echo -e "\n${YELLOW}[The following prompt is only relevant for DIA-LAB laptops which connect to real robots]${NC}"
#   echo -e "\n${YELLOW}[Simply hit ENTER if you're installing this on your own PC]${NC}\n"
#   echo -e "\n${YELLOW}[Please enter the dia_laptop number (between 1 and 45)]${NC}"
#   echo -e "\n${YELLOW}[This will also be used as the robot numbers]${NC}"
#   read -r reply </dev/tty
#   if [[ -z $reply ]]; then
#     echo -e "\n${YELLOW}[Ignoring...]${NC}"
#     PC_NO=''
#   elif (( $reply >= 1 && $reply <= 45 )); then
#       PC_NO=$reply
#   else
#     echo -e "\n${YELLOW}[Defaulting to '1']${NC}"
#     PC_NO=1
#   fi

#     echo -e "\n${YELLOW}[Setting up /usr/local/bin/ scripts]${NC}"
#   cd /usr/local/bin/
#   sudo wget -O /usr/local/bin/robot_mode https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/robot_mode
#   sudo wget -O /usr/local/bin/pair_with_waffle https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/pair_with_waffle
#   sudo wget -O /usr/local/bin/waffle https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/waffle_cli/waffle
#   sudo wget -O /usr/local/bin/diamond_tools https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/diamond_tools/diamond_tools
#   sudo chmod +x *

#   sudo wget -O /usr/local/bin/robot_pair_check.sh https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/waffle_cli/robot_pair_check.sh
#   sudo wget -O /usr/local/bin/robot_pairing.sh https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/waffle_cli/robot_pairing.sh
#   sudo wget -O /usr/local/bin/robot_sync.sh https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/waffle_cli/robot_sync.sh

#   echo -e "\n${YELLOW}[Setting up ~/.tuos scripts]${NC}"
#   mkdir -p ~/.tuos
#   cd ~/.tuos
#   wget -O ~/.tuos/bashrc_miro https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/bashrc_miro
#   wget -O ~/.tuos/bashrc_turtlebot3 https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/bashrc_turtlebot3

#   echo -e "\n${YELLOW}[Setting up 'Shared' group]${NC}"
#   sudo mkdir -p /home/Shared/
#   sudo addgroup sharegroup
#   sudo chown :sharegroup /home/Shared
#   sudo adduser "$USER" sharegroup

#     # set selected sudo commands to require no password input
#   sudo wget -O /etc/sudoers.d/nopwds https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/nopwds

#   echo -e "\n${YELLOW}[Setting device numbers]${NC}"
#   cd /home/Shared
#   sudo touch laptop_number waffle_number
#   sudo chown "$USER" *
#   sudo chgrp sharegroup *
#   echo "$PC_NO" > laptop_number
#   echo "$PC_NO" > waffle_number

#   echo -e "\n${YELLOW}[Adding bash aliases, if not found]${NC}"
#   cd ~
#   wget -O /tmp/.bash_aliases https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/.bash_aliases
#   touch ~/.bash_aliases
#   tmp_file=/tmp/.bash_aliases
#   while IFS= read -r line; do
#     grep -qxF "$line" ~/.bash_aliases || echo "$line" >> ~/.bash_aliases
#   done < "$tmp_file"

#   wget -O $HOME/.tuos/.bashrc_extras https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/.bashrc_extras
#   echo "source $HOME/.tuos/.bashrc_extras" >> $HOME/.bashrc
# else
#   echo -e "\n${YELLOW}[Skipping TUoS Robotics scripts...]${NC}"
# fi

# ######################### Part V. Teaching materials ##########################
# if ask "[Do you want to download the COM2009 teaching materials?]"; then
#   cd $HOME/$name_ros2_workspace/src
#   git clone https://github.com/tom-howard/COM2009
#   cd $HOME/$name_ros2_workspace
#   colcon build --symlink-install
# fi




# ############################## Part VII. Clean up ###############################
# echo -e "\n${YELLOW}[Clean-up]${NC}"
# sudo apt update -y
# sudo apt upgrade -y
# sudo apt autoremove -y
# sudo apt autoclean -y

# echo -e "\n${GREEN}[INITIAL INSTALL COMPLETE]: Next Steps:"
# echo -e "   * Install VS Code Extensions (Python, Remote - SSH)"
# echo -e "   * Set up the Student account (VS Code, auto login etc)"
# echo -e "   * Reboot ASAP.${NC}"
# exit 0
