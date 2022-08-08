#!/usr/bin/env bash

sudo apt install -y libgl1-mesa-glx
wget -O ~/anaconda.sh https://repo.anaconda.com/archive/Anaconda3-2022.05-Linux-x86_64.sh
bash ~/anaconda.sh
rm ~/anaconda.sh

sudo chgrp -R sharegroup $HOME/anaconda3
sudo chmod 770 -R $HOME/anaconda3

rc_file=/home/Shared/bashrc_conda
sudo touch $rc_file
sudo chown "$USER":sharegroup $rc_file
echo "$(grep -A 14 "# >>> conda" ~/.bashrc)" > $rc_file

wget -O /tmp/.bashrc_extras https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/.bashrc_extras

configs="#!/usr/bin/env bash

echo \"auto_activate_base: false\" > ~/.condarc

rm -f ~/.bashrc
cp /etc/skel/.bashrc ~/

echo \"\" >> ~/.bashrc
while read line; do
    echo \"\$line\" >> ~/.bashrc
done < /tmp/.bashrc_extras
"
echo -e "$configs" > /tmp/conda_configs.sh

chmod +x /tmp/conda_configs.sh
cd ~
# run as admin
echo "Running conda configs as admin..."
/tmp/conda_configs.sh
# run as user
echo "Running conda configs as 'student'..."
sudo -i -u student "/tmp/conda_configs.sh"
