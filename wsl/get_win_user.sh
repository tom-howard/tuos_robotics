#!/bin/bash

cur_dir=$(pwd)
cd /mnt/c/ && cmd.exe /c "echo %USERNAME%" > ~/.wsl-ros/win_user
dos2unix -q ~/.wsl-ros/win_user
cd $cur_dir
read -r first_line < ~/.wsl-ros/win_user
export WIN_USER=$first_line
rm ~/.wsl-ros/win_user
