#!/bin/bash

RED='\033[0;31m'
YELLOW='\033[1;33m'
GREEN='\033[0;32m'
NC='\033[0m'

cleanup() {
    echo -e "\n${YELLOW}[Clean-up]${NC}"
    sudo apt update -y
    sudo apt upgrade -y
    sudo apt autoremove -y
    sudo apt autoclean -y

    echo "Cleanup Done."
}

OS_VER=${OS_VER:="noble"}
ROS_VER=${ROS_VER:="jazzy"}
ROS_WS=${ROS_WS:="tb3_ws"}
echo -e "${YELLOW}Target OS version >>> '$OS_VER'${NC}"
echo -e "\n${YELLOW}Target ROS version >>> ROS2 '$ROS_VER'${NC}"
echo -e "\n${YELLOW}Workspace Name >>> '$ROS_WS'${NC}"

SHARE_DIR="/home/ros"
STANDARD_USER="robot"

if [ ! -f $HOME/checkpoint0 ]; then
    echo -e "### CHECKPOINT 0 (Fresh install) ###"
    
    # Disable wait for network during bootup:
    systemctl mask systemd-networkd-wait-online.service
    touch ${HOME}/checkpoint0
    echo "### CHECKPOINT 0 (Fresh install) COMPLETE ###"
    
elif [ ! -f $HOME/checkpoint1 ]; then
    echo -e "### CHECKPOINT 1 (Basic Setup) ###"
    
    # Setup additional users
    echo -e "\n${YELLOW}Creating user '${STANDARD_USER}'${NC}"
    sudo useradd -s /bin/bash -m ${STANDARD_USER}
    
    # Create a new dir in /home/
    sudo mkdir -p $SHARE_DIR/
    # Create a new group called rosgrp and add users to it:
    sudo addgroup rosgrp
    sudo adduser "$USER" rosgrp
    sudo adduser ${STANDARD_USER} rosgrp
    # Change ownership of SHARE_DIR and change its group to rosgrp:
    sudo chown $USER:rosgrp $SHARE_DIR

    sleep 5

    echo -e "\n${YELLOW}[Update & Upgrade]${NC}"
    sudo apt update && sudo apt upgrade -y

    echo -e "\n${YELLOW}[Installing Misc Tools]${NC}"
    sudo apt install -y chrony \
                        ntpdate \
                        curl \
                        build-essential \
                        software-properties-common \
                        net-tools \
                        unzip \
                        tree \
                        llvm-dev \
                        libclang-dev \
                        wavemon \
                        avahi-daemon

    # update git:
    echo -e "\n${YELLOW}[Updating Git]${NC}"
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

    mkdir -p $SHARE_DIR/repos/
    cd $SHARE_DIR/repos/
    git clone -b ${ROS_VER} https://github.com/tom-howard/tuos_robotics.git
    cd $HOME

    # Make poweroff and ntpdate NO PASSWORD-able
    sudo cp $SHARE_DIR/repos/tuos_robotics/turtlebot3/nopwds /etc/sudoers.d/
    sudo cp $SHARE_DIR/repos/tuos_robotics/turtlebot3/eth-cfg.yaml /etc/netplan/99-eth-cfg.yaml
    sudo netplan apply

    # Enable multicast on loopback (via a startup service)...
    sudo cp $SHARE_DIR/repos/tuos_robotics/turtlebot3/startup_servce/multicast-lo.service /etc/systemd/system/
    sudo systemctl enable multicast-lo.service

    sudo systemctl enable --now avahi-daemon

    touch $HOME/checkpoint1
    cleanup

    echo "### CHECKPOINT 1 (Basic Setup) COMPLETE ###"

elif [ ! -f $HOME/checkpoint2 ]; then
    echo -e "### CHECKPOINT 2 (Installing ROS) ###" 
    
    ## INSTALLING ROS ###

    # configure Ubuntu repositories to allow "main" "restricted" "universe" and "multiverse"
    sudo add-apt-repository main universe multiverse restricted

    # Adding the ROS 2 GPG key
    sudo apt update
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
    sudo dpkg -i /tmp/ros2-apt-source.deb

    sudo apt update

    echo -e "\n${YELLOW}[Source .bashrc]${NC}"
    source $HOME/.bashrc

    echo -e "\n${YELLOW}[Install all the necessary ROS and TB3 packages]${NC}"
    sudo apt install -y ros-$ROS_VER-ros-base \
                        ros-dev-tools \
                        python3-argcomplete \
                        python3-rosdep \
                        python3-colcon-common-extensions \
                        libboost-system-dev \
                        ros-$ROS_VER-hls-lfcd-lds-driver \
                        ros-$ROS_VER-turtlebot3-msgs \
                        ros-$ROS_VER-dynamixel-sdk \
                        libudev-dev \
                        python3-pip \
                        ros-$ROS_VER-rmw-cyclonedds-cpp \
                        ros-$ROS_VER-rmw-zenoh-cpp

    source /opt/ros/$ROS_VER/setup.bash

    touch $HOME/checkpoint2
    cleanup

    echo "### CHECKPOINT 2 (Installing ROS) COMPLETE ###"
    
elif [ ! -f $HOME/checkpoint3 ]; then
    echo -e "### CHECKPOINT 3 (Configuring Devices) ###" 

    echo -e "\n${YELLOW}[Setting up the ROS workspace ($ROS_WS)]${NC}"
    echo "source /opt/ros/$ROS_VER/setup.bash" >> $HOME/.bashrc
    source $HOME/.bashrc

    cd $SHARE_DIR/repos/
    git clone -b ${ROS_VER} https://github.com/ROBOTIS-GIT/turtlebot3.git
    
    # Make a workspace:
    mkdir -p $SHARE_DIR/$ROS_WS/src 
    cd $SHARE_DIR/$ROS_WS && colcon build
    
    ### OpenCR & other TB3 Configs ###

    sudo wget -O /etc/udev/rules.d/98-turtlebot3-cdc.rules \
        https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/refs/heads/${ROS_VER}/turtlebot3_bringup/script/99-turtlebot3-cdc.rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger

    mkdir -p $HOME/firmware/ && cd $HOME/firmware/
    wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
    tar -xvf ./opencr_update.tar.bz2
    rm opencr_update.tar.bz2

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

    sudo apt install -y ros-$ROS_VER-librealsense2* \
                        ros-$ROS_VER-realsense2-*

    # to fix permission issues:
    sudo wget -O /etc/udev/rules.d/99-realsense-libusb.rules https://raw.githubusercontent.com/IntelRealSense/librealsense/refs/heads/master/config/99-realsense-libusb.rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger

    touch $HOME/checkpoint3
    cleanup

    echo "### CHECKPOINT 3 (Configuring Devices) COMPLETE ###"
    
else
    echo -e "### CHECKPOINT 4 (Setting up TUoS Scripts) ###" 

    ### Custom TUoS Scripts ###

    cd $SHARE_DIR/repos/
    git clone -b ${ROS_VER} https://github.com/tom-howard/tuos_ros.git

    SCRIPTS_DIR=$SHARE_DIR/repos/tuos_robotics/turtlebot3
    cd $SCRIPTS_DIR && cd .. && git pull

    echo -e "\n${YELLOW}[Setting up /usr/local/bin/ scripts]${NC}"
    cd $SCRIPTS_DIR/
    sudo install waffle /usr/local/bin/
    sudo install wsl_ros /usr/local/bin/
    
    cd $SCRIPTS_DIR/diamond_tools/
    sudo install diamond_tools /usr/local/bin/
    
    WAFFLE_NO=$(hostname | tr -d -c 0-9)
    cd $SHARE_DIR
    touch waffle_number
    echo "$WAFFLE_NO" > waffle_number
    chown $USER:rosgrp waffle_number

    echo -e "\n${YELLOW}Setting up user profiles${NC}"

    mkdir -p $HOME/.diamond/diamond_tools/
    echo "[$(date +'%Y%m%d_%H%M%S')] $(date +'%Y-%m') ROS 2 ${ROS_VER} ($(hostname))" > $HOME/.diamond/base_image

    cp $SCRIPTS_DIR/diamond_tools/profile_updates.sh /tmp/
    cp /tmp/profile_updates.sh $HOME/.diamond/diamond_tools/profile_updates-$(date +'%Y%m%d%H%M%S')
    chmod +x /tmp/profile_updates.sh
    # run in current profile:
    /tmp/profile_updates.sh
    source $HOME/.bashrc
    diamond_tools workspace

    # run as STANDARD_USER
    sudo -i -u ${STANDARD_USER} "/tmp/profile_updates.sh"

    rm -f $HOME/checkpoint*

    cleanup

    echo "### CHECKPOINT 4 (Setting up TUoS Scripts) COMPLETE ###"
    
fi
