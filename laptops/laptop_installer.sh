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
name_ros2_workspace=${name_ros2_workspace:="ros2_ws"}

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
        sudo mkdir -p /home/laptop/
        sudo addgroup laptopgrp
        sudo adduser "$USER" laptopgrp
        sudo adduser student laptopgrp
        sudo chown $USER:laptopgrp /home/laptop

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
                            gpg

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

        # set selected sudo commands to require no password input
        sudo wget -O /etc/sudoers.d/nopwds https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/nopwds

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
        sudo apt install -y tree
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
        sudo apt install -y ros-humble-desktop \
                            ros-dev-tools \
                            ros-humble-gazebo-* \
                            ros-humble-cartographer \
                            ros-humble-cartographer-ros \
                            ros-humble-navigation2 \
                            ros-humble-nav2-bringup \
                            ros-humble-turtlebot3 \
                            ros-humble-turtlebot3-msgs \
                            ros-humble-turtlebot3-simulations \
                            ros-humble-turtlebot3-gazebo \
                            python3-rosdep \
                            python3-colcon-common-extensions \
                            ros-humble-rqt* \
                            ffmpeg \
                            python3-pip \
                            python3-numpy \
                            python3-scipy

        pip install setuptools==58.2.0

        sudo rosdep init; rosdep update

        touch $HOME/checkpoint2
        cleanup

        echo "### CHECKPOINT 2 (Installing ROS) COMPLETE ###"
    fi
else
    echo -e "### CHECKPOINT 3 (Setting up TUoS Scripts) ###" 
    if ask "Ok to continue?"; then

        echo -e "\n${YELLOW}[Setting up the environment]"
        echo "source /opt/ros/$name_ros_version/setup.bash" >> $HOME/.bashrc

        source $HOME/.bashrc

        echo -e "\n${YELLOW}[Installing TUoS Scripts]${NC}"
        echo "Connecting to DIA-LAB..."
        nmcli c up DIA-LAB
        sleep 4
        # get the last part of the IP address:
        LAPTOP_NO=$(ip -o addr show dev "wlo1" | awk '$3 == "inet" {print $4}' | sed -r 's!/.*!!; s!.*\.!!')

        echo "Connecting to eduroam..."
        nmcli c up eduroam
        sleep 4
        
        echo -e "\n${YELLOW}[Setting up /usr/local/bin/ scripts]${NC}"
        sudo wget -O /usr/local/bin/robot_mode https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/robot_mode
        sudo wget -O /usr/local/bin/waffle https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/waffle_cli/waffle
        sudo wget -O /usr/local/bin/diamond_tools https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/diamond_tools/diamond_tools
        sudo chmod +x /usr/local/bin/*

        sudo wget -O /usr/local/bin/robot_pair_check.sh https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/waffle_cli/robot_pair_check.sh
        sudo wget -O /usr/local/bin/robot_pairing.sh https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/waffle_cli/robot_pairing.sh
        sudo wget -O /usr/local/bin/robot_sync.sh https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/waffle_cli/robot_sync.sh
        
        echo -e "\n${YELLOW}[Setting device numbers]${NC}"
        cd /home/laptop
        sudo touch laptop_number waffle_number
        echo "$LAPTOP_NO" > laptop_number
        echo "$LAPTOP_NO" > waffle_number

        echo -e "\n${YELLOW}Setting up user profiles${NC}"

        mkdir -p $HOME/.tuos/diamond_tools/
        echo "[$(date +'%Y%m%d')_$(date +'%H%M%S')] 2024-09 ROS2 Humble ($(hostname))" > $HOME/.tuos/base_image

        rm -f /tmp/profile_updates.sh
        wget -O /tmp/profile_updates.sh https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/diamond_tools/profile_updates.sh
        chmod +x /tmp/profile_updates.sh
        sudo chown $USER:laptopgrp /tmp/profile_updates.sh
        # run as current user:
        /tmp/profile_updates.sh
        diamond_tools workspace
        source $HOME/.bashrc

        # setting up 'student' profile
        echo -e "\n${YELLOW}[Setting up the same environment for 'student' account]${NC}"
        wget -O /tmp/setup_student.sh https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/setup_student.sh
        chmod +x /tmp/setup_student.sh
        sudo chown $USER:laptopgrp /tmp/setup_student.sh
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
