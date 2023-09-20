#!/usr/bin/env bash

# Clean up the WSL environment, prior to creating an image of it...

if [[ ! $(sudo echo 0) ]]; then
    echo "Invalid credentials. Exiting."
    exit
fi

ask() {
  local prompt default reply
  prompt='y/n'
  default=''
  while true; do
    # Ask the question (not using "read -p" as it uses stderr not stdout)
    echo -n "$1 [$prompt] "
    # Read the answer (use /dev/tty in case stdin is redirected from somewhere else)
    read -r reply </dev/tty
    # Default?
    if [[ -z $reply ]]; then
        reply=$default
    fi
    # Check if the reply is valid
    case "$reply" in
        Y*|y*) return 0 ;;
        N*|n*) return 1 ;;
    esac
  done
}

echo -n "Enter the new wsl-ros version number: "
read -r reply </dev/tty
echo "Setting the new wsl-ros version to '$reply'."
echo "$reply" > ~/.wsl-ros/wsl_ros_ver
echo "1" > ~/.wsl-ros/local_ver

if ask "Do you want to update all the custom wsl-ros scripts?"; then
    diamond_tools update
else
    echo "No custom scripts were updated."
fi

rm -f ~/.wsl-ros/no_welcome ~/.current_robot ~/.backup_exclusions
rm -rf ~/.gazebo/ ~/.ignition/ ~/.rviz/ ~/.ros/ ~/.dbus ~/.landscape
rm -f ~/.python_history ~/.wget-hsts ~/.backup_exclusions
rm -rf ~/tuos_robotics
rm -f ~/.wsl-ros/update.sh ~/.wsl-ros/cleanup.sh
echo "export XSERVER=true" > ~/.wsl-ros/xserver.sh

echo -e "All done."
echo -e "Now run the following command to complete the process:\n"
echo -e "    history -c && history -w && logout\n"
