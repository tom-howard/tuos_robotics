#!/usr/bin/env bash
cd $HOME
# TB3
source /opt/ros/noetic/setup.sh
mkdir -p $HOME/catkin_ws/src
cd $HOME/catkin_ws/src
catkin_init_workspace
cd $HOME/catkin_ws
catkin build
# MDK
source $HOME/.bashrc
mkdir -p $HOME/pkgs
cd $HOME/pkgs/
wget --no-check-certificate 'https://docs.google.com/uc?export=download&id=1V8pNrwcMY7ucjEzf6NzoAuLK3GORac1k' -O mdk_2-210921.tgz
tar -xvzf mdk_2-210921.tgz
cd $HOME/pkgs/mdk-210921/bin/script
rm $HOME/mdk
./install_mdk.sh
source $HOME/.bashrc
cd $HOME/mdk/catkin_ws/
catkin build
wget -O $HOME/mdk/sim/launch_full.sh https://gist.githubusercontent.com/AlexandrLucas/703831843f9b46edc2e2032bcd08651f/raw/launch_full.sh
chmod +x $HOME/mdk/sim/launch_full.sh
# TUoS Robotics scripts
sed -i '/# MDK/d' $HOME/.bashrc
sed -i '/source ~\/mdk\/setup.bash/d' $HOME/.bashrc
source $HOME/.bashrc
mkdir -p $HOME/.tuos
cd $HOME/.tuos
wget -O $HOME/.tuos/bashrc_miro https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/bashrc_miro
wget -O $HOME/.tuos/bashrc_turtlebot3 https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/bashrc_turtlebot3
wget -O $HOME/.tuos/bashrc_robot_switch https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/bashrc_robot_switch
cp /home/Shared/bashrc_conda ~/.tuos/
echo "auto_activate_base: false" > $HOME/.condarc

tmp_file=/tmp/.bashrc_extras_student
wget -O $tmp_file https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/.bashrc_extras
while IFS= read -r line; do
    grep -qxF "$line" $HOME/.bashrc || echo "$line" >> $HOME/.bashrc
done < "$tmp_file"

tmp_file=/tmp/.bash_aliases_student
wget -O $tmp_file https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/.bash_aliases
touch $HOME/.bash_aliases
while IFS= read -r line; do
    grep -qxF "$line" $HOME/.bash_aliases || echo "$line" >> $HOME/.bash_aliases
done < "$tmp_file"

# COM2009 and COM3528
source $HOME/.bashrc
cd $HOME/catkin_ws/src
git clone https://github.com/tom-howard/COM2009
catkin build
cd $HOME/mdk/catkin_ws/src
git clone https://github.com/AlexandrLucas/COM3528
catkin build
