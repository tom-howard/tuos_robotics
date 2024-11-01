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
ROS_WS=${ROS_WS:="ros2_ws"}
echo -e "${YELLOW}Target OS version >>> '$OS_VER'${NC}"
echo -e "\n${YELLOW}Target ROS version >>> ROS2 '$ROS_VER'${NC}"
echo -e "\n${YELLOW}Workspace Name >>> '$ROS_WS'${NC}"

SHARE_DIR="/home/laptop"

if ! ask "[OK to continue with installation?]"; then
  echo -e "${YELLOW}Exiting.${NC}"
  exit 130
fi

if [ ! -f $HOME/checkpoint1 ]; then
    echo -e "### CHECKPOINT 1 (Basic Setup) ###"
    if ask "Ok to continue?"; then
        echo -e "\n${YELLOW}Creating user 'student'${NC}"
        username="student"
        pass="panQJvEl/BD/g"
        sudo useradd -s /bin/bash -m -p "$pass" "$username"
        
        echo -e "\n${YELLOW}[Setting up a shared space]${NC}"
        sudo mkdir -p $SHARE_DIR/
        sudo addgroup laptopgrp
        sudo adduser "$USER" laptopgrp
        sudo adduser student laptopgrp
        sudo chown $USER:laptopgrp $SHARE_DIR

        echo -e "\n${YELLOW}[Update & Upgrade]${NC}"
        sudo apt update && sudo apt upgrade -y

        echo -e "\n${YELLOW}[Installing Essential Tools]${NC}"
        sudo apt install -y chrony \
                            ntpdate \
                            curl \
                            build-essential \
                            net-tools \
                            vlc \
                            gnome-clocks \
                            software-properties-common \
                            apt-transport-https \
                            wget \
                            gpg \
                            tmux \
                            tree \
                            llvm-dev \
                            libclang-dev

        echo -e "\n${YELLOW}[Installing VS Code]${NC}"
        wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
        sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
        echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" |sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null
        rm -f packages.microsoft.gpg
        sudo apt update
        sudo apt install -y code 

        # install OBS Studio:
        echo -e "\n${YELLOW}[Installing OBS Studio]${NC}"
        sudo add-apt-repository ppa:obsproject/obs-studio
        sudo apt install -y obs-studio

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

        echo -e "\n${YELLOW}[NTP: update time]${NC}"
        sudo ntpdate ntp.ubuntu.com
        sleep 2

        mkdir -p $SHARE_DIR/repos/
        cd $SHARE_DIR/repos/
        git clone -b humble https://github.com/tom-howard/tuos_robotics.git
        cd ~

        # set selected sudo commands to require no password input
        sudo cp $SHARE_DIR/repos/tuos_robotics/laptops/nopwds /etc/sudoers.d/

        echo -e "\n${YELLOW}[Connecting to DIA-LAB]${NC}"
        SSID_CURRENT=$(iwgetid -r)
        sudo nmcli --ask dev wifi connect DIA-LAB
        echo -e "\n${YELLOW}Connected to: $(iwgetid -r)"
        echo -e "Connecting back to '$SSID_CURRENT' for the final part of this setup...${NC}"
        sudo nmcli dev wifi connect $SSID_CURRENT

        touch $HOME/checkpoint1
        cleanup

        echo "### CHECKPOINT 1 (Basic Setup) COMPLETE ###"
    fi
elif [ ! -f $HOME/checkpoint2 ]; then
    echo -e "### CHECKPOINT 2 (Installing ROS) ###" 
    if ask "Ok to continue?"; then
        ## INSTALLING ROS ###
        # Add universe repo
        sudo add-apt-repository universe

        # Adding the ROS 2 GPG key
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

        # Adding repo to sources list
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

        sudo apt update && sudo apt upgrade -y

        echo -e "\n${YELLOW}[Source .bashrc]${NC}"
        source $HOME/.bashrc

        echo -e "\n${YELLOW}[Install all the necessary ROS and TB3 packages]${NC}"
        sudo apt install -y ros-$ROS_VER-desktop \
                            ros-dev-tools \
                            ros-$ROS_VER-gazebo-* \
                            ros-$ROS_VER-cartographer \
                            ros-$ROS_VER-cartographer-ros \
                            ros-$ROS_VER-navigation2 \
                            ros-$ROS_VER-nav2-bringup \
                            ros-$ROS_VER-turtlebot3 \
                            ros-$ROS_VER-turtlebot3-msgs \
                            ros-$ROS_VER-turtlebot3-simulations \
                            ros-$ROS_VER-turtlebot3-gazebo \
                            python3-rosdep \
                            python3-colcon-common-extensions \
                            ros-$ROS_VER-rqt* \
                            ffmpeg \
                            python3-pip \
                            python3-numpy \
                            python3-scipy \
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
else
    echo -e "### CHECKPOINT 3 (Setting up TUoS Scripts) ###" 
    if ask "Ok to continue?"; then

        echo -e "\n${YELLOW}[Setting up the environment]"
        echo "source /opt/ros/$ROS_VER/setup.bash" >> $HOME/.bashrc

        source $HOME/.bashrc

        echo -e "\n${YELLOW}[Installing TUoS Scripts]${NC}"
        
        cd $SHARE_DIR/repos
        git clone -b humble https://github.com/tom-howard/tuos_ros.git

        LAPTOP_NO=$(hostname | tr -d -c 0-9)
        echo "configuring for dia-laptop$LAPTOP_NO..."
        sleep 4

        echo -e "\n${YELLOW}[Setting up /usr/local/bin/ scripts]${NC}"
        cd $SHARE_DIR/repos/tuos_robotics/laptops/
        sudo install robot_mode /usr/local/bin/
        
        cd $SHARE_DIR/repos/tuos_robotics/laptops/diamond_tools/
        sudo install diamond_tools /usr/local/bin/
        
        cd $SHARE_DIR/repos/tuos_robotics/laptops/waffle_cli/
        sudo install waffle /usr/local/bin/
        sudo cp robot_pair_check.sh /usr/local/bin/
        sudo cp robot_pairing.sh /usr/local/bin/
        sudo cp robot_sync.sh /usr/local/bin/
        
        echo -e "\n${YELLOW}[Setting device numbers]${NC}"
        cd $SHARE_DIR
        touch laptop_number waffle_number
        echo "$LAPTOP_NO" > laptop_number
        echo "$LAPTOP_NO" > waffle_number
        chown $USER:laptopgrp laptop_number waffle_number

        echo -e "\n${YELLOW}Setting up user profiles${NC}"

        mkdir -p $HOME/.tuos/diamond_tools/
        echo "[$(date +'%Y%m%d_%H%M%S')] $(date +'%Y-%m') ROS2 Humble ($(hostname))" > $HOME/.tuos/base_image

        cd $SHARE_DIR/repos/tuos_robotics/laptops/diamond_tools/
        cp profile_updates.sh /tmp/ 
        cd ~
        chmod +x /tmp/profile_updates.sh
        chown $USER:laptopgrp /tmp/profile_updates.sh
        # run as current user:
        /tmp/profile_updates.sh
        source $HOME/.bashrc
        diamond_tools workspace

        # setting up 'student' profile
        echo -e "\n${YELLOW}[Setting up the same environment for 'student' account]${NC}"
        cp $SHARE_DIR/repos/tuos_robotics/laptops/setup_student.sh /tmp/
        chmod +x /tmp/setup_student.sh
        chown $USER:laptopgrp /tmp/setup_student.sh
        sudo -i -u student "/tmp/setup_student.sh"

        rm -f $HOME/checkpoint*

        echo "### CHECKPOINT 3 (Setting up TUoS Scripts) COMPLETE ###"
        
        echo -e "\n${GREEN}[LAPTOP INSTALL COMPLETE] Next Steps:"
        echo -e "   * Install VS Code Extensions (Python, Remote - SSH)"
        echo -e "   * Set up the Student account (VS Code, auto login etc)"
        echo -e "   * Power settings (don't sleep etc.)"
        echo -e "   * Reboot ASAP.${NC}"

    fi
fi
