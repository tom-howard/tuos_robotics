#!/usr/bin/env bash

ROS_VER="humble"

# sudo apt update && sudo apt upgrade -y
echo "$(hostname | tr -d -c 0-9)" > /home/ros/waffle_number

echo "Installing Zenoh related components..."
sleep 4

sudo apt install ros-$ROS_VER-rmw-cyclonedds-cpp \
                 llvm-dev \
                 libclang-dev

DDS_WS="$HOME/dds_ws"

mkdir -p $DDS_WS/src/
cd $DDS_WS/src/
git clone https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds.git
cd $DDS_WS
sudo rosdep init; rosdep update
# use `rosdep` to install dependencies
rosdep install --from-paths . --ignore-src -r -y
cd $DDS_WS/src/zenoh-plugin-ros2dds
# this bit takes a while...
cargo build --release

echo "Finished installing Zenoh and dependencies."
sleep 4

echo "Updating TUoS Scripts..."
sleep 4

cd /usr/local/bin/
sudo rm -f diamond_tools robot_switch robot_mode pair_with_miro pair_with_waffle waffle wsl_ros
sudo wget -qO /usr/local/bin/diamond_tools https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/diamond_tools/diamond_tools
sudo wget -qO /usr/local/bin/robot_mode https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/robot_mode
sudo wget -qO /usr/local/bin/pair_with_waffle https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/pair_with_waffle
sudo wget -qO /usr/local/bin/waffle https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/waffle_cli/waffle
sudo wget -qO /usr/local/bin/wsl_ros https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/wsl_ros

sudo chmod +x diamond_tools robot_switch robot_mode pair_with_miro pair_with_waffle waffle wsl_ros

sudo rm -f robot_pair_check.sh robot_pairing.sh robot_sync.sh
sudo wget -qO /usr/local/bin/robot_pair_check.sh https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/waffle_cli/robot_pair_check.sh
sudo wget -qO /usr/local/bin/robot_pairing.sh https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/waffle_cli/robot_pairing.sh
sudo wget -qO /usr/local/bin/robot_sync.sh https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/waffle_cli/robot_sync.sh

echo "Updating user profiles..."
sleep 2

cd /tmp/
rm -f /tmp/profile_updates.sh
wget -qO /tmp/profile_updates.sh https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/diamond_tools/profile_updates.sh
chmod +x /tmp/profile_updates.sh

cd ~
# run as admin
/tmp/profile_updates.sh

# run as user
sudo -i -u student "/tmp/profile_updates.sh"

echo "Updates are complete."
