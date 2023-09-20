#!/usr/bin/env bash

if [[ ! $(sudo echo 0) ]]; then
    echo "Invalid credentials. Exiting."
    exit
fi

cd ~

rm -rf ~/tuos_robotics

if ! git clone -q https://github.com/tom-howard/tuos_robotics.git; then
    echo "Error."
else
    # /usr/local/bin
    cd /usr/local/bin
    files="robot_switch wsl_ros"
    sudo rm -f $files
    cd ~/tuos_robotics/wsl/
    sudo cp $files /usr/local/bin/
    cd /usr/local/bin
    sudo chmod +x $files
    sudo rm -f diamond_tools
    cd ~/tuos_robotics/wsl/diamond_tools/
    sudo cp diamond_tools /usr/local/bin/
    sudo chmod +x /usr/local/bin/diamond_tools

    # ~/.wsl-ros/
    files="bashrc_miro bashrc_turtlebot3 bashrc_wsl_ros default_backup_exclusions get_tuos_user.sh set_display.sh"
    cd ~/.wsl-ros/
    rm -f $files
    cd ~/tuos_robotics/wsl/
    cp $files ~/.wsl-ros/
    
    # ~
    rm -f ~/.bash_aliases
    cd ~/tuos_robotics/wsl/
    cp .bash_aliases ~/
    
    rm -rf ~/tuos_robotics
    
    cd ~
    echo "Done."
fi
