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

OS_VER=${OS_VER:="jammy"}
ROS_VER=${ROS_VER:="humble"}
ROS_WS=${ROS_WS:="tb3_ws"}
echo -e "${YELLOW}Target OS version >>> '$OS_VER'${NC}"
echo -e "\n${YELLOW}Target ROS version >>> ROS2 '$ROS_VER'${NC}"
echo -e "\n${YELLOW}Workspace Name >>> '$ROS_WS'${NC}"

SHARE_DIR="/home/ros"

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
        echo -e "\n${YELLOW}Creating user 'xdds'${NC}"
        sudo useradd -M xdds
        sudo passwd xdds

        # Create a new dir in /home/
        sudo mkdir -p $SHARE_DIR/
        # Create a new group called rosgrp and add users to it:
        sudo addgroup rosgrp
        sudo adduser "$USER" rosgrp
        sudo adduser robot rosgrp
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
                            libclang-dev

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
        git clone -b humble https://github.com/tom-howard/tuos_robotics.git
        cd $HOME

        # Make poweroff and ntpdate NO PASSWORD-able
        sudo cp $SHARE_DIR/repos/tuos_robotics/turtlebot3/nopwds /etc/sudoers.d/

        # Enable multicast on loopback (via a startup service)...
        sudo cp $SHARE_DIR/repos/tuos_robotics/turtlebot3/startup_servce/multicast-lo.service /etc/systemd/system/
        sudo systemctl enable multicast-lo.service

        touch $HOME/checkpoint1
        cleanup

        echo "### CHECKPOINT 1 (Basic Setup) COMPLETE ###"
    fi
elif [ ! -f $HOME/checkpoint2 ]; then
    echo -e "### CHECKPOINT 2 (Installing ROS) ###" 
    if ask "Ok to continue?"; then
        ## INSTALLING ROS ###

        # configure Ubuntu repositories to allow "main" "restricted" "universe" and "multiverse"
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
                            ros-$ROS_VER-rmw-cyclonedds-cpp

        pip install setuptools==58.2.0

        source /opt/ros/$ROS_VER/setup.bash

        echo "Installing Zenoh..."
        sleep 4

        DDS_WS="$SHARE_DIR/dds_ws"
        mkdir -p $DDS_WS/src/
        cd $DDS_WS/src/
        git clone https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds.git
        cd $DDS_WS

        sudo rosdep init; rosdep update
        echo "using 'rosdep' to install dependencies..."
        sleep 4
        rosdep install --from-paths . --ignore-src -r -y

        cd $DDS_WS/src/zenoh-plugin-ros2dds
        echo "Building zenoh plugin with cargo..."
        sleep 4
        cargo build --release
        sudo install $DDS_WS/src/zenoh-plugin-ros2dds/target/release/zenoh-bridge-ros2dds /usr/local/bin/

        touch $HOME/checkpoint2
        cleanup

        echo "### CHECKPOINT 2 (Installing ROS) COMPLETE ###"
    fi
elif [ ! -f $HOME/checkpoint3 ]; then
    echo -e "### CHECKPOINT 3 (Configuring Devices) ###" 
    if ask "Ok to continue?"; then

        echo -e "\n${YELLOW}[Setting up the ROS workspace ($ROS_WS)]${NC}"
        echo "source /opt/ros/$ROS_VER/setup.bash" >> $HOME/.bashrc
        source $HOME/.bashrc

        cd $SHARE_DIR/repos/
        git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
        
        # Make a workspace:
        mkdir -p $SHARE_DIR/$ROS_WS/src 
        cd $SHARE_DIR/$ROS_WS && colcon build
        
        ### OpenCR & other TB3 Configs ###

        sudo wget -O /etc/udev/rules.d/98-turtlebot3-cdc.rules https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/refs/heads/humble-devel/turtlebot3_bringup/script/99-turtlebot3-cdc.rules
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
    fi
else
    echo -e "### CHECKPOINT 4 (Setting up TUoS Scripts) ###" 
    if ask "Ok to continue?"; then
        ### Custom TUoS Scripts ###

        cd $SHARE_DIR/repos/
        git clone -b humble https://github.com/tom-howard/tuos_ros.git

        SCRIPTS_DIR=$SHARE_DIR/repos/tuos_robotics/turtlebot3
        cd $SCRIPTS_DIR && cd .. && git pull

        # echo -e "\n${YELLOW}[Setting up DDS Service]${NC}"
        # sudo cp $SCRIPTS_DIR/startup_service/zdds.service /etc/systemd/system/
        # sudo systemctl enable zdds.service

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

        mkdir -p $HOME/.tuos/diamond_tools/
        echo "[$(date +'%Y%m%d_%H%M%S')] $(date +'%Y-%m') ROS2 Humble ($(hostname))" > $HOME/.tuos/base_image

        cp $SCRIPTS_DIR/diamond_tools/profile_updates.sh /tmp/
        cp /tmp/profile_updates.sh $HOME/.tuos/diamond_tools/profile_updates-$(date +'%Y%m%d%H%M%S')
        chmod +x /tmp/profile_updates.sh
        # run in current profile:
        /tmp/profile_updates.sh
        source $HOME/.bashrc
        diamond_tools workspace

        # run as 'robot'
        sudo -i -u robot "/tmp/profile_updates.sh"

        rm -f $HOME/checkpoint*

        cleanup

        echo "### CHECKPOINT 4 (Setting up TUoS Scripts) COMPLETE ###"
    fi
fi
