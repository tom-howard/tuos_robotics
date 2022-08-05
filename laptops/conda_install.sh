#!/usr/bin/env bash

# sudo apt install -y libgl1-mesa-glx
# wget -O ~/anaconda.sh https://repo.anaconda.com/archive/Anaconda3-2022.05-Linux-x86_64.sh
# bash ~/anaconda.sh
# rm ~/anaconda.sh

# echo "auto_activate_base: false" > $HOME/.condarc

# sudo chgrp -R sharegroup $HOME/anaconda3
# sudo chmod 770 -R $HOME/anaconda3
# echo "$(grep -A 14  "# >>> conda" ~/.bashrc)" > /home/Shared/bashrc_conda

content="#!/usr/bin/env bash

cp /home/Shared/bashrc_conda ~/.tuos/ 
rm -f ~/.bashrc
cp /etc/skel/.bashrc ~/

wget -O /tmp/.bashrc_extras https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/.bashrc_extras
tmp_file=/tmp/.bashrc_extras
while read line; do
    echo \"\$line\" >> ~/.bashrc
done < \"\$tmp_file\"
"
echo -e "$content" > /tmp/conda_configs.sh
