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

OS_VER=${OS_VER:="noble"}
ROS_VER=${ROS_VER:="jazzy"}
ROS_WS=${ROS_WS:="ros2_ws"}
echo -e "${YELLOW}Target OS version >>> '$OS_VER'${NC}"
echo -e "\n${YELLOW}Target ROS version >>> ROS2 '$ROS_VER'${NC}"
echo -e "\n${YELLOW}Workspace Name >>> '$ROS_WS'${NC}"

SHARE_DIR="/home/ros"
STUDENT_USER="student"

if ! ask "[OK to continue with installation?]"; then
  echo -e "${YELLOW}Exiting.${NC}"
  exit 130
fi

if [ ! -f $HOME/checkpoint1 ]; then
    echo -e "### CHECKPOINT 1 (Basic Setup) ###"
    if ask "Ok to continue?"; then
        echo -e "\n${YELLOW}Creating user '${STUDENT_USER}'${NC}"
        username="${STUDENT_USER}"
        pass="panQJvEl/BD/g"
        sudo useradd -s /bin/bash -m -p "$pass" "$username"
        
        echo -e "\n${YELLOW}[Setting up a shared space]${NC}"
        sudo mkdir -p $SHARE_DIR/
        sudo addgroup laptopgrp
        sudo adduser "$USER" laptopgrp
        sudo adduser ${STUDENT_USER} laptopgrp
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

        echo -e "\n### Installing Docker ###\n"
        # Add Docker's official GPG key:
        sudo install -m 0755 -d /etc/apt/keyrings
        sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
        sudo chmod a+r /etc/apt/keyrings/docker.asc
        # Add the repository to Apt sources:
        echo \
        "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
        $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
        sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
        sudo apt update
        sudo apt install -y docker-ce \
                            docker-ce-cli \
                            containerd.io \
                            docker-buildx-plugin \
                            docker-compose-plugin
        sudo groupadd docker
        sudo usermod -aG docker ${USER}
        sudo usermod -aG docker ${STUDENT_USER}

        echo -e "\n### Installing NVIDIA Drivers and Container Toolkit ###\n"
        sudo apt install -y nvidia-driver-570
        curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
        && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
            sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
            sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
        sudo apt update
        export NVIDIA_CONTAINER_TOOLKIT_VERSION=1.17.8-1
        sudo apt install -y nvidia-container-toolkit=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
                            nvidia-container-toolkit-base=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
                            libnvidia-container-tools=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
                            libnvidia-container1=${NVIDIA_CONTAINER_TOOLKIT_VERSION}
        sudo nvidia-ctk runtime configure --runtime=docker
        sudo systemctl restart docker

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
        git clone -b ${ROS_VER} https://github.com/tom-howard/tuos_robotics.git
        cd ~

        # set selected sudo commands to require no password input
        sudo cp $SHARE_DIR/repos/tuos_robotics/laptops/nopwds /etc/sudoers.d/

        # Enable multicast on loopback (via a startup service)
        sudo cp ${SHARE_DIR}/repos/tuos_robotics/laptops/startup_service/multicast-lo.service /etc/systemd/system/
        sudo systemctl enable multicast-lo.service

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

        export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
        curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
        sudo dpkg -i /tmp/ros2-apt-source.deb

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
                            python3-venv \
                            python3-pandas \
                            python3-scipy \
                            python3-venv \
                            ros-$ROS_VER-rmw-cyclonedds-cpp \
                            ros-$ROS_VER-rmw-zenoh-cpp 

        source /opt/ros/$ROS_VER/setup.bash

        touch $HOME/checkpoint2
        cleanup

        echo "### CHECKPOINT 2 (Installing ROS) COMPLETE ###"
    fi
else
    echo -e "### CHECKPOINT 3 (Setting up TUoS Scripts) ###" 
    if ask "Ok to continue?"; then

        echo -e "\n${YELLOW}[Setting up the environment]"
        echo "source /opt/ros/${ROS_VER}/setup.bash" >> $HOME/.bashrc

        source $HOME/.bashrc

        echo -e "\n${YELLOW}[Installing TUoS Scripts]${NC}"
        
        cd $SHARE_DIR/repos
        git clone -b ${ROS_VER} https://github.com/tom-howard/tuos_ros.git

        LAPTOP_NO=$(hostname | tr -d -c 0-9)
        echo "configuring for dia-laptop$LAPTOP_NO..."
        sleep 4

        echo -e "\n${YELLOW}[Setting up /usr/local/bin/ scripts]${NC}"
        cd $SHARE_DIR/repos/tuos_robotics/laptops/
        sudo install ros_mode /usr/local/bin/
        
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

        mkdir -p $HOME/.diamond/diamond_tools/
        echo "[$(date +'%Y%m%d_%H%M%S')] $(date +'%Y-%m') ROS 2 ${ROS_VER} ($(hostname))" > $HOME/.diamond/base_image

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
        echo -e "\n${YELLOW}[Setting up the same environment for '${STUDENT_USER}' account]${NC}"
        cp $SHARE_DIR/repos/tuos_robotics/laptops/setup_student.sh /tmp/
        chmod +x /tmp/setup_student.sh
        chown $USER:laptopgrp /tmp/setup_student.sh
        sudo -i -u ${STUDENT_USER} "/tmp/setup_student.sh"

        echo "### CHECKPOINT 3 (Setting up TUoS Scripts) COMPLETE ###"
        
        echo -e "\n${GREEN}[LAPTOP INSTALL COMPLETE] Next Steps:"
        echo -e "   * Install VS Code Extensions (Python, Remote - SSH)"
        echo -e "   * Set up the Student account (VS Code, auto login etc)"
        echo -e "   * Power settings (don't sleep etc.)"
        echo -e "   * Reboot ASAP.${NC}"

    fi
fi
