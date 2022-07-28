#!/usr/bin/env bash

cd /usr/local/bin/
sudo rm -f diamond_tools robot_switch robot_mode pair_with_miro pair_with_waffle waffle
sudo wget https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/diamond_tools/diamond_tools
sudo wget https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/robot_switch
sudo wget https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/robot_mode
sudo wget https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/pair_with_miro
sudo wget https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/pair_with_waffle
sudo wget https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/waffle_cli/waffle

sudo chmod +x diamond_tools robot_switch robot_mode pair_with_miro pair_with_waffle waffle

sudo rm -f robot_pair_check.sh robot_pairing.sh robot_sync.sh
sudo wget https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/waffle_cli/robot_pair_check.sh
sudo wget https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/waffle_cli/robot_pairing.sh
sudo wget https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/waffle_cli/robot_sync.sh

cd /tmp/
rm -f /tmp/profile_updates.sh
wget -O /tmp/profile_updates.sh https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/diamond_tools/profile_updates.sh
chmod +x /tmp/profile_updates.sh

cd ~
# run as admin
/tmp/profile_updates.sh

# run as user
sudo -i -u robot "/tmp/profile_updates.sh"
