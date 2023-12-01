#!/usr/bin/env bash

cd /usr/local/bin/
sudo rm -f diamond_tools robot_switch robot_mode pair_with_miro pair_with_waffle waffle wsl_ros
sudo wget -O /usr/local/bin/diamond_tools https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/diamond_tools/diamond_tools
sudo wget -O /usr/local/bin/robot_switch https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/robot_switch
sudo wget -O /usr/local/bin/robot_mode https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/robot_mode
sudo wget -O /usr/local/bin/pair_with_miro https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/pair_with_miro
sudo wget -O /usr/local/bin/pair_with_waffle https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/pair_with_waffle
sudo wget -O /usr/local/bin/waffle https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/waffle_cli/waffle
sudo wget -O /usr/local/bin/wsl_ros https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/wsl_ros

sudo chmod +x diamond_tools robot_switch robot_mode pair_with_miro pair_with_waffle waffle wsl_ros

sudo rm -f robot_pair_check.sh robot_pairing.sh robot_sync.sh
sudo wget -O /usr/local/bin/robot_pair_check.sh https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/waffle_cli/robot_pair_check.sh
sudo wget -O /usr/local/bin/robot_pairing.sh https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/waffle_cli/robot_pairing.sh
sudo wget -O /usr/local/bin/robot_sync.sh https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/waffle_cli/robot_sync.sh

cd /tmp/
rm -f /tmp/profile_updates.sh
wget -O /tmp/profile_updates.sh https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/diamond_tools/profile_updates.sh
chmod +x /tmp/profile_updates.sh

cd ~
# run as admin
/tmp/profile_updates.sh

# run as user
sudo -i -u student "/tmp/profile_updates.sh"
