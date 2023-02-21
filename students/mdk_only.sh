#!/bin/bash
# CC BY-NC
# Alex Lucas, University of Sheffield
# Copyright (c) 2023

RED='\033[0;31m'
YELLOW='\033[1;33m'
GREEN='\033[0;32m'
NC='\033[0m'

echo -e "\n${GREEN}[This is a helper script to automate the installation 
of the latest version of the MiRo Develeper Kit]${NC}"

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

##############

echo -e "\n${YELLOW}[Downloading and unpacking MDK into ~/pkgs]${NC}"
mkdir -p ~/pkgs
cd ~/pkgs/
wget --no-check-certificate 'https://docs.google.com/uc?export=download&id=1vNODaenljocVWalM4cOW4Kax-RB4U3nh' -O mdk_2-230105.tgz
tar -xvzf mdk_2-230105.tgz

echo -e "\n${YELLOW}[Remove MDK soft link]${NC}"
rm ~/mdk
echo -e "\n${YELLOW}[Remove MDK .bashrc entries]${NC}"
sed -i '/# MDK/d' ~/.bashrc
sed -i '/source ~\/mdk\/setup.bash/d' ~/.bashrc
unset MIRO_DIR_MDK
unset MIRO_SYSTEM

echo -e "\n${YELLOW}[Install MDK]${NC}"
source ~/.bashrc
cd ~/pkgs/mdk-230105/bin/deb64
./install_mdk.sh

if ask "[Do you want to remove the newly added MDK ~/.bashrc entries?]\n Select 'Yes' if using TUoS robot switching scripts"; then
    echo -e "\n${YELLOW}[Removing MDK ~/.bashrc entries]${NC}"
    sed -i '/# MDK/d' ~/.bashrc
    sed -i '/source ~\/mdk\/setup.bash/d' ~/.bashrc
fi

echo -e "\n${YELLOW}[Add the 'launch_full' script]${NC}"
wget -O ~/mdk/sim/launch_full.sh https://gist.githubusercontent.com/AlexandrLucas/703831843f9b46edc2e2032bcd08651f/raw/launch_full.sh
chmod +x ~/mdk/sim/launch_full.sh

if ask "[Do you want to build the catkin workspace for MDK?]"; then
    echo -e "\n${YELLOW}[Cleaning and building MDK workspace]${NC}"
    source ~/.bashrc
    cd ~/mdk/catkin_ws/
    catkin clean
    catkin build
    cd ~/mdk/catkin_ws/build/miro2_msg/
    make install
fi

if ask "[Do you want to remove the MDK archive?]"; then
    echo -e "\n${YELLOW}[Removing MDK archive]${NC}"
    rm ~/pkgs/mdk_2-230105.tgz
fi

echo -e "\n${GREEN}[COMPLETE]: MDK installed.${NC}"
