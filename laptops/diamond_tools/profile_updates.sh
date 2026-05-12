#!/usr/bin/env bash

SRC_DIR="/home/ros/repos/tuos_robotics/laptops"

mkdir -p $HOME/.diamond/diamond_tools/

rm -f $HOME/.bash_aliases $HOME/.bashrc $HOME/.diamond/dia-laptop-config.sh
cd $SRC_DIR
cp bash_aliases $HOME/.bash_aliases
cp dia-laptop-config.sh $HOME/.diamond/
cp /etc/skel/.bashrc $HOME/

rm -f $HOME/.ssh/waffle_rsa*
rm -f $HOME/.ssh/known_hosts*

OD_REPO=${HOME}/opendaycybersecurity
if [ -d "${OD_REPO}" ]; then
  cd ${OD_REPO} && git pull --quiet
fi

echo "" >> $HOME/.bashrc
echo "source $HOME/.diamond/dia-laptop-config.sh" >> $HOME/.bashrc
echo "" >> $HOME/.bashrc

echo "Profile updates applied to '${USER}'. Please restart the terminal or run 'source ~/.bashrc' to apply the changes."
