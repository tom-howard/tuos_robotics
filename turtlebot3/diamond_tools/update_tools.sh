#!/usr/bin/env bash

ROS2_WS="/home/ros/tb3_ws"
ROS_VER="humble"

# sudo apt update && sudo apt upgrade -y
echo "$(hostname | tr -d -c 0-9)" > /home/ros/waffle_number

echo "Installing Zenoh related components..."
sleep 4

sudo apt install ros-$ROS_VER-rmw-cyclonedds-cpp \
                 llvm-dev \
                 libclang-dev

cd $ROS2_WS/src/
git clone https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds.git
cd $ROS2_WS
sudo rosdep init; rosdep update
# use `rosdep` to install dependencies
rosdep install --from-paths . --ignore-src -r -y
cd $ROS2_WS/src/zenoh-plugin-ros2dds
# this bit takes a while...
cargo build --release

echo "Finished installing Zenoh and dependencies."
sleep 4

# sudo wget -qO /etc/systemd/system/fastdds.service https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/turtlebot3/startup_service/fastdds.service
# sudo systemctl enable fastdds.service
sudo systemctl stop fastdds.service
sudo systemctl disable fastdds.service

echo "Updating TUoS Scripts..."
sleep 4

files="/usr/local/bin/diamond_tools /usr/local/bin/wsl_ros /usr/local/bin/waffle"
sudo rm -f $files

sudo wget -qO /usr/local/bin/diamond_tools https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/turtlebot3/diamond_tools/diamond_tools
sudo wget -qO /usr/local/bin/waffle https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/turtlebot3/waffle
sudo wget -qO /usr/local/bin/wsl_ros https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/turtlebot3/wsl_ros
sudo chmod +x $files

echo "Updating user profiles..."
sleep 2

rm -f /tmp/profile_updates.sh
wget -qO /tmp/profile_updates.sh https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/turtlebot3/diamond_tools/profile_updates.sh
chmod +x /tmp/profile_updates.sh

# run as admin
/tmp/profile_updates.sh
diamond_tools workspace

# run as 'robot'
sudo -i -u robot "/tmp/profile_updates.sh"

echo "Updates are complete."