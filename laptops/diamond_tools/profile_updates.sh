#!/usr/bin/env bash

SRC_DIR="/home/ros/repos/tuos_robotics/laptops"

mkdir -p $HOME/.diamond/diamond_tools/

rm -f $HOME/.bash_aliases $HOME/.bashrc $HOME/.diamond/dia-laptop-config.sh
cd $SRC_DIR
cp bash_aliases $HOME/.bash_aliases
cp dia-laptop-config.sh $HOME/.diamond/
cp /etc/skel/.bashrc $HOME/

if [ "$USER" = "offer_holders" ]; then
    cd $HOME/opendaycybersecurity && git pull --quiet
fi

echo "" >> $HOME/.bashrc
echo "source $HOME/.diamond/dia-laptop-config.sh" >> $HOME/.bashrc
echo "" >> $HOME/.bashrc

echo "Profile updates applied for ${USER}. Please restart the terminal or run 'source ~/.bashrc' to apply the changes."
