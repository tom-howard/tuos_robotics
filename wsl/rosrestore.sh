#!/usr/bin/env bash

rbpth=$rbname'.tar.gz'
rbwpth=$(sed "s/\/mnt\/u/U\:/;s/\//\\\/g" <<< $rbpth)

echo "Restoring your home directory from '$rbwpth', please wait..."

rblog=$rbname'-log.txt'

echo "$(date): Restore to $(hostname) [wsl-ros version: $WSL_ROS_VER]" >> $rblog

# to extract a single file:
tar -xvf /mnt/u/wsl-ros/ros-backup-2201.tar.gz -C / home/student/wsl_ros_backup_manifest
# then delete everything in WSL before extracting...?

if tar --checkpoint=.200 -xjf $rbpth -C / ; then
  echo -e ".\nRestore complete."
fi