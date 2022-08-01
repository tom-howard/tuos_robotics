#!/usr/bin/env bash

# Clean up the WSL environment, prior to creating an image of it...

if [[ ! $(sudo echo 0) ]]; then
    echo "Invalid credentials. Exiting."
    exit
fi

source /usr/local/bin/ask_user.sh

echo -n "Enter the new wsl-ros version number: "
read -r reply </dev/tty
echo "Setting the new wsl-ros version to '$reply'."
echo "$reply" > ~/.wsl-ros/wsl_ros_ver

if ask "Do you want to update all the custom wsl-ros scripts?"; then
    diamond_tools update
else
    echo "No custom scripts were updated."
fi

rm -f ~/.wsl-ros/no_welcome ~/.current_robot
rm -rf ~/.gazebo/ ~/.ignition/ ~/.rviz/ ~/.ros/
rm -f ~/.python_history
rm -rf ~/tuos_robotics
rm -f ~/.wsl-ros/update.sh ~/.wsl-ros/cleanup.sh

echo -e "All done."
echo -e "Now run the following command to complete the process:\n"
echo -e "    history -c && history -w && logout\n"
