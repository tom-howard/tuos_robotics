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

cleanup() {
    echo -e "\n${YELLOW}[Clean-up]${NC}"
    sudo apt update -y
    sudo apt upgrade -y
    sudo apt autoremove -y
    sudo apt autoclean -y

    echo "Cleanup Done."
}

echo -e "${YELLOW}[Note] Target OS version >>> Ubuntu 22.04.x (Jammy Jellyfish)${NC}"
echo -e "${YELLOW}[Note] Target ROS version >>> ROS2 Humble Hawksbill${NC}"
echo -e "\n${YELLOW}[Set the target OS, ROS version and the name of catkin workspace]${NC}"
name_os_version=${name_os_version:="jammy"}
name_ros_version=${name_ros_version:="humble"}
name_ros2_workspace=${name_ros2_workspace:="/home/ros/tb3_ws"}

if ! ask "[OK to continue with installation?]"; then
  echo -e "${YELLOW}Exiting.${NC}"
  exit 130
fi

if [ ! -f $HOME/checkpoint0 ]; then
    echo -e "### CHECKPOINT 0 (Fresh install) ###"
    if ask "Ok to continue?"; then
        # Disable wait for network during bootup:
        systemctl mask systemd-networkd-wait-online.service
        touch $HOME/checkpoint0
        echo "### CHECKPOINT 0 (Fresh install) COMPLETE ###"
    fi
elif [ ! -f $HOME/checkpoint1 ]; then
    echo -e "### CHECKPOINT 1 (Basic Setup) ###"
    if ask "Ok to continue?"; then

        # Setup additional users
        echo -e "\n${YELLOW}Creating user 'robot'${NC}"
        sudo useradd -s /bin/bash -m -p panQJvEl/BD/g robot
        echo -e "\n${YELLOW}Creating user 'fastdds'${NC}"
        sudo useradd -M fastdds
        sudo passwd fastdds

        # Create a new dir in /home/
        sudo mkdir -p /home/ros/
        # Create a new group called rosgrp and add users to it:
        sudo addgroup rosgrp
        sudo adduser waffle rosgrp
        sudo adduser robot rosgrp
        # Change ownership of /home/ros/ to waffle and change its group to rosgrp:
        sudo chown waffle:rosgrp /home/ros/

        sleep 5

        echo -e "\n${YELLOW}[Update & Upgrade]${NC}"
        sudo apt update && sudo apt upgrade -y

        echo -e "\n${YELLOW}[Installing Misc Tools]${NC}"
        sudo apt install -y chrony ntpdate curl build-essential net-tools unzip

        # update git:
        sudo add-apt-repository ppa:git-core/ppa
        sudo apt update -y
        sudo apt install -y git

        # Set locales
        locale  # check for UTF-8
        sudo apt update && sudo apt install locales
        sudo locale-gen en_GB en_GB.UTF-8
        sudo update-locale LC_ALL=en_GB.UTF-8 LANG=en_GB.UTF-8
        locale  # verify settings

        echo -e "\n${YELLOW}[Update system time]${NC}"
        timedatectl set-timezone Europe/London
        sudo ntpdate ntp.ubuntu.com
        sleep 5

        # Make poweroff and ntpdate NO PASSWORD-able
        sudo wget -O /etc/sudoers.d/nopwds https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/turtlebot3/nopwds

        touch $HOME/checkpoint1
        cleanup

        echo "### CHECKPOINT 1 (Basic Setup) COMPLETE ###"
    fi
elif [ ! -f $HOME/checkpoint2 ]; then
    echo -e "### CHECKPOINT 2 (Installing ROS) ###" 
    if ask "Ok to continue?"; then
        ## INSTALLING ROS ###

        # configure Ubuntu repositories to allow "main" "restricted" "universe" and "multiverse"
        sudo apt install software-properties-common
        sudo add-apt-repository main universe multiverse restricted

        # Adding the ROS 2 GPG key
        sudo apt update && sudo apt install curl -y
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

        # Adding repo to sources list
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

        sudo apt update && sudo apt upgrade -y

        echo -e "\n${YELLOW}[Source .bashrc]${NC}"
        source $HOME/.bashrc

        echo -e "\n${YELLOW}[Install all the necessary ROS and TB3 packages]${NC}"
        sudo apt install -y ros-humble-ros-base \
                            ros-dev-tools \
                            python3-argcomplete \
                            python3-rosdep \
                            python3-colcon-common-extensions \
                            libboost-system-dev \
                            ros-humble-hls-lfcd-lds-driver \
                            ros-humble-turtlebot3-msgs \
                            ros-humble-dynamixel-sdk \
                            libudev-dev \
                            python3-pip

        pip install setuptools==58.2.0

        touch $HOME/checkpoint2
        cleanup

        echo "### CHECKPOINT 2 (Installing ROS) COMPLETE ###"
    fi
elif [ ! -f $HOME/checkpoint3 ]; then
    echo -e "### CHECKPOINT 3 (Configuring Devices) ###" 
    if ask "Ok to continue?"; then

        echo -e "\n${YELLOW}[Setting up the ROS workspace ($name_ros2_workspace)]${NC}"
        echo "source /opt/ros/$name_ros_version/setup.bash" >> $HOME/.bashrc
        source $HOME/.bashrc
        # Make a workspace:
        mkdir -p $name_ros2_workspace/src && cd $name_ros2_workspace/src
        git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
        cd $name_ros2_workspace/src/turtlebot3
        rm -r turtlebot3_cartographer turtlebot3_navigation2
        cd $name_ros2_workspace
        colcon build --symlink-install
        echo "source $name_ros2_workspace/install/local_setup.bash" >> $HOME/.bashrc
        source $HOME/.bashrc

        ### OpenCR & other TB3 Configs ###

        sudo wget -O /etc/udev/rules.d/99-turtlebot3-cdc.rules https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/refs/heads/humble-devel/turtlebot3_bringup/script/99-turtlebot3-cdc.rules
        sudo udevadm control --reload-rules
        sudo udevadm trigger

        echo 'export TURTLEBOT3_MODEL=waffle' >> $HOME/.bashrc
        echo 'export LDS_MODEL=LDS-01' >> $HOME/.bashrc

        mkdir -p ~/firmware/opencr/
        cd ~/firmware/opencr/
        wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
        tar -xvf ./opencr_update.tar.bz2
        rm opencr_update.tar.bz2

        cd opencr_update/
        ./update.sh /dev/ttyACM0 waffle.opencr

        echo "OpenCR configs complete."
        sleep 5

        ### Intel RealSense ###

        mkdir -p $HOME/firmware/realsense/
        cd $HOME/firmware/realsense/
        echo "### Downloading Intel RealSense D435 Firmware (Version 5_16_0_1) ###"
        wget https://downloadmirror.intel.com/821320/d400_series_fw_5_16_0_1.zip
        unzip d400_series_fw_5_16_0_1.zip
        rm d400_series_fw_5_16_0_1.zip
        SIGNED_IMAGE="Signed_Image_UVC_5_16_0_1"
        mv $SIGNED_IMAGE/$SIGNED_IMAGE.bin ./
        rm -r $SIGNED_IMAGE

        echo "Installing Realsense ROS Libraries"

        sleep 5

        sudo apt install -y ros-humble-librealsense2* \
                            ros-humble-realsense2-*

        # to fix permission issues:
        sudo wget -O /etc/udev/rules.d/99-realsense-libusb.rules https://raw.githubusercontent.com/IntelRealSense/librealsense/refs/heads/master/config/99-realsense-libusb.rules
        sudo udevadm control --reload-rules
        sudo udevadm trigger

        touch $HOME/checkpoint3
        cleanup

        echo "### CHECKPOINT 3 (Configuring Devices) COMPLETE ###"
    fi
else
    echo -e "### CHECKPOINT 4 (Setting up TUoS Scripts) ###" 
    if ask "Ok to continue?"; then
        ### Custom TUoS Scripts ###

        echo -e "\n${YELLOW}[Setting up fastdds Service]${NC}"
        sudo wget -O /etc/systemd/system/fastdds.service https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/turtlebot3/startup_service/fastdds.service
        sudo systemctl enable fastdds.service

        echo -e "\n${YELLOW}[Setting up /usr/local/bin/ scripts]${NC}"
        scripts="diamond_tools wsl_ros waffle"
        sudo wget -O /usr/local/bin/diamond_tools https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/turtlebot3/diamond_tools/diamond_tools
        sudo wget -O /usr/local/bin/waffle https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/turtlebot3/waffle
        sudo wget -O /usr/local/bin/wsl_ros https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/turtlebot3/wsl_ros
        pushd /usr/local/bin/ 
        sudo chmod +x $scripts
        popd

        echo -e "\n${YELLOW}Setting up user profiles${NC}"

        mkdir -p $HOME/.tuos/diamond_tools/
        echo "[$(date +'%Y%m%d')_$(date +'%H%M%S')] 2024-09 ROS2 Humble ($(hostname))" > $HOME/.tuos/base_image

        rm -f /tmp/profile_updates.sh
        wget -O /tmp/profile_updates.sh https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/turtlebot3/diamond_tools/profile_updates.sh
        chmod +x /tmp/profile_updates.sh
        # run in current profile:
        /tmp/profile_updates.sh
        diamond_tools workspace

        # run as 'robot'
        sudo -i -u robot "/tmp/profile_updates.sh"

        rm -f $HOME/checkpoint*

        cleanup

        echo "### CHECKPOINT 4 (Setting up TUoS Scripts) COMPLETE ###"
    fi
fi
