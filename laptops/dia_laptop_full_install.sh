#!/bin/bash
# CC BY-NC
# Alex Lucas & Tom Howard, University of Sheffield
# Copyright (c) 2022

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


echo -e "${YELLOW}[Note] Target OS version  >>> Ubuntu 20.04.x (Focal Fossa) or Linux Mint 21.x${NC}"
echo -e "${YELLOW}[Note] Target ROS version >>> ROS Noetic Ninjemys${NC}"
echo -e "${YELLOW}[Note] Catkin workspace   >>> $HOME/catkin_ws${NC}\n"


if ! ask "[OK to continue with installation?]"; then
  echo -e "${YELLOW}Exiting.${NC}"
  exit 130
fi

#################  Part I. ROS, TB3, catkin_ws and dependencies ################
echo -e "\n${YELLOW}[Set the target OS, ROS version and the name of catkin workspace]${NC}"
name_os_version=${name_os_version:="focal"}
name_ros_version=${name_ros_version:="noetic"}
name_catkin_workspace=${name_catkin_workspace:="catkin_ws"}

echo -e "\n${YELLOW}[Update & Upgrade]${NC}"
sudo apt update -y
sudo apt upgrade -y

echo -e "\n${YELLOW}[Installing Misc Tools]${NC}"
sudo apt install -y chrony ntpdate curl build-essential git net-tools
# anything else...?

echo -e "\n${YELLOW}[NTP: update time]${NC}"
sudo ntpdate ntp.ubuntu.com
sleep 2

echo -e "\n${YELLOW}[Add the ROS repository]${NC}"
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
  sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu ${name_os_version} main\" > /etc/apt/sources.list.d/ros-latest.list"
fi

echo -e "\n${YELLOW}[Download the ROS keys]${NC}"
roskey=`apt-key list | grep "Open Robotics"`
if [ -z "$roskey" ]; then
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
fi

echo -e "\n${YELLOW}[Check the ROS keys]${NC}"
roskey=`apt-key list | grep "Open Robotics"`
if [ -n "$roskey" ]; then
  echo -e "\n${YELLOW}[ROS key exists in the list]${NC}"
else
  echo -e "\n${RED}[Failed to receive the ROS key, aborts the installation]${NC}"
  exit 0
fi

echo -e "\n${YELLOW}[Update & Upgrade]${NC}"
sudo apt update -y
sudo apt upgrade -y

echo -e "\n${YELLOW}[Source .bashrc]${NC}"
source $HOME/.bashrc

echo -e "\n${YELLOW}[Install all the necessary ROS and TB3 packages]${NC}"
sudo apt install -y ros-noetic-desktop-full ros-noetic-rqt-* ros-noetic-gazebo-* \
ros-noetic-joy ros-noetic-teleop-twist-joy ros-noetic-teleop-twist-keyboard \
ros-noetic-laser-proc ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
ros-noetic-amcl ros-noetic-map-server ros-noetic-move-base ros-noetic-rqt* \
ros-noetic-gmapping ros-noetic-navigation python3-rosdep python3-rosinstall \
python3-rosinstall-generator python3-wstool build-essential ros-noetic-dynamixel-sdk \
ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations \
python3-pip python3-catkin-tools ffmpeg

echo -e "\n${YELLOW}[Initialise rosdep and update]${NC}"
sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
sudo sh -c "rosdep init"
echo "Running rosdep update now..."
sleep 2
rosdep update

echo -e "\n${YELLOW}[Environment setup]${NC}"
source /opt/ros/$name_ros_version/setup.sh
source $HOME/.bashrc

echo -e "\n${YELLOW}[Create and build the TB3 catkin workspace]${NC}"
mkdir -p $HOME/$name_catkin_workspace/src
cd $HOME/$name_catkin_workspace/src
catkin_init_workspace
cd $HOME/$name_catkin_workspace
catkin build

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

################################# Part III. MDK #################################
if ask "[Do you want to set up MiRo Developer Kit?]"; then
  echo -e "\n${YELLOW}[Install AprilTags with Pip3]${NC}"
  sudo pip3 install apriltag

  echo -e "\n${YELLOW}[Download and unpack MDK into ~/pkgs]${NC}"
  mkdir -p ~/pkgs
  cd ~/pkgs/
  wget --no-check-certificate 'https://docs.google.com/uc?export=download&id=1V8pNrwcMY7ucjEzf6NzoAuLK3GORac1k' -O mdk_2-210921.tgz
  tar -xvzf mdk_2-210921.tgz
  cd mdk-210921/bin/script

  # Remove MDK .bashrc entries
  rm ~/mdk
  sed -i '/# MDK/d' ~/.bashrc
  sed -i '/source ~\/mdk\/setup.bash/d' ~/.bashrc

  echo -e "\n${YELLOW}[Install MDK]${NC}"
  ./install_mdk.sh
  cd ~/mdk/catkin_ws/
  catkin build

  echo -e "\n${YELLOW}[Add MDK extras]${NC}"
  wget -O ~/mdk/sim/launch_full.sh https://gist.githubusercontent.com/AlexandrLucas/703831843f9b46edc2e2032bcd08651f/raw/launch_full.sh
  chmod +x ~/mdk/sim/launch_full.sh
else
  echo -e "\n${YELLOW}[Skipping MDK...]${NC}"
fi

####################### Part IV. TUoS Robotics scripts ########################
if ask "[Do you want to also set up TUoS Robot Switch scripts?]"; then

  echo -e "\n${YELLOW}[The following prompt is only relevant for DIA-LAB laptops which connect to real robots]${NC}"
  echo -e "\n${YELLOW}[Simply hit ENTER if you're installing this on your own PC]${NC}\n"
  echo -e "\n${YELLOW}[Please enter the dia_laptop number (between 1 and 45)]${NC}"
  echo -e "\n${YELLOW}[This will also be used as the robot numbers]${NC}"
  read -r reply </dev/tty
  if [[ -z $reply ]]; then
    echo -e "\n${YELLOW}[Ignoring...]${NC}"
    PC_NO=''
  elif (( $reply >= 1 && $reply <= 45 )); then
      PC_NO=$reply
  else
    echo -e "\n${YELLOW}[Defaulting to '1']${NC}"
    PC_NO=1
  fi

  # Remove the new MDK ~/.bashrc entries (will be done separately in a different way)
  sed -i '/# MDK/d' ~/.bashrc
  sed -i '/source ~\/mdk\/setup.bash/d' ~/.bashrc

  echo -e "\n${YELLOW}[Setting up /usr/local/bin/ scripts]${NC}"
  cd /usr/local/bin/
  sudo wget -O /usr/local/bin/robot_switch https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/robot_switch
  sudo wget -O /usr/local/bin/robot_mode https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/robot_mode
  sudo wget -O /usr/local/bin/pair_with_miro https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/pair_with_miro
  sudo wget -O /usr/local/bin/pair_with_waffle https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/pair_with_waffle
  sudo chmod +x *

  echo -e "\n${YELLOW}[Setting up ~/.tuos scripts]${NC}"
  mkdir -p ~/.tuos
  cd ~/.tuos
  wget -O ~/.tuos/bashrc_miro https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/bashrc_miro
  wget -O ~/.tuos/bashrc_turtlebot3 https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/bashrc_turtlebot3
  wget -O ~/.tuos/bashrc_robot_switch https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/bashrc_robot_switch

  echo -e "\n${YELLOW}[Setting up 'Shared' group]${NC}"
  sudo mkdir -p /home/Shared/
  sudo addgroup sharegroup
  sudo chown :sharegroup /home/Shared
  sudo adduser "$USER" sharegroup

  echo -e "\n${YELLOW}[Setting device numbers]${NC}"
  cd /home/Shared
  sudo touch laptop_number miro_number waffle_number
  sudo chown "$USER" *
  sudo chgrp sharegroup *
  echo "$PC_NO" > laptop_number
  echo "$PC_NO" > miro_number
  echo "$PC_NO" > waffle_number

  echo -e "\n${YELLOW}[Adding the Robotics Kit switch to ~/.bashrc, if not found]${NC}"
  wget -O /tmp/.bashrc_extras https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/.bashrc_extras

  tmp_file=/tmp/.bashrc_extras
  while IFS= read -r line; do
    grep -qxF "$line" ~/.bashrc || echo "$line" >> ~/.bashrc
  done < "$tmp_file"

  echo -e "\n${YELLOW}[Adding bash aliases, if not found]${NC}"
  cd ~
  wget -O /tmp/.bash_aliases https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/.bash_aliases
  touch ~/.bash_aliases
  tmp_file=/tmp/.bash_aliases
  while IFS= read -r line; do
    grep -qxF "$line" ~/.bash_aliases || echo "$line" >> ~/.bash_aliases
  done < "$tmp_file"

else
  echo -e "\n${YELLOW}[Skipping TUoS Robotics scripts...]${NC}"
fi

######################### Part V. Teaching materials ##########################
if ask "[Do you want to download COM2009 and COM3528 teaching materials?]"; then
  cd $HOME/$name_catkin_workspace/src
  git clone https://github.com/tom-howard/COM2009
  catkin build

  cd ~/mdk/catkin_ws/src
  git clone https://github.com/AlexandrLucas/COM3528
  catkin build

  sudo apt install -y python3-pandas
fi

########################## Part VI. 'Student' profile ###########################
if ask "[Do you want to set up a 'student' profile?]"; then
  username="student"
  pass="panQJvEl/BD/g"
  sudo useradd -s /bin/bash -m -p "$pass" "$username"
  if [ $? -eq 0 ]; then
    echo -e "\n${YELLOW}[User 'student' has been added to system]${NC}"
    sudo adduser student sharegroup

    # Most of the commands above are now simply copied over
    # TODO: There must be a more clever way of doing this
    echo -e "\n${YELLOW}[Setting up the same environment for 'student' account]${NC}"
    wget -O /tmp/setup_student.sh https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/setup_student.sh
    chmod +x /tmp/setup_student.sh
    sudo -i -u student "/tmp/setup_student.sh"
  else
    echo -e "\n${RED}[Failed to add user 'student']${NC}"
  fi
fi

############################## Part VII. DIA-LAB ###############################

if ask "[Do you want to connect this device to DIA-LAB now?]"; then
  SSID_CURRENT=$(iwgetid -r)
  sudo nmcli --ask dev wifi connect DIA-LAB
  echo -e "\n${YELLOW}Connected to: $(iwgetid -r)"
  echo -e "Connecting back to '$SSID_CURRENT' for the final part of this setup...${NC}"
  sudo nmcli dev wifi connect $SSID_CURRENT
  sleep 4
else
  echo -e "\n${YELLOW}[SKIPPED the DIA-LAB connection step]${NC}"
fi

############################## Part VII. Clean up ###############################
echo -e "\n${YELLOW}[Clean-up]${NC}"
sudo apt update -y
sudo apt upgrade -y
sudo apt autoremove -y
sudo apt autoclean -y

echo -e "\n${GREEN}[INITIAL INSTALL COMPLETE]: Next Steps:"
echo -e "   * Install VS Code Extensions (Python, Remote - SSH)"
echo -e "   * Set up the Student account"
echo -e "   * Don't forget to reboot ASAP.${NC}"
exit 0
