#!/usr/bin/env bash

if [[ "$rbname" == "" ]]; then
  echo "You can't run this script standalone. Use 'wsl_ros backup' instead."
  exit 1
fi

if [ ! -f ~/wsl_ros_backup_manifest ]; then
  # no backup manifest exists in ~, so load a default:
  cp ~/.wsl-ros/wsl_ros_backup_manifest ~
fi

rbpth=$rbname'.tar.gz'
rbwpth=$(sed "s/\/mnt\/u/U\:/;s/\//\\\/g" <<< $rbpth)

echo "Backing up your WSL-ROS environment to '$rbwpth', please wait..."

rbdir=$(sed "s/\/[^/]*$//" <<< $rbname)

mkdir -p $rbdir

rblog=$rbname'-log.txt'
if ! touch $rblog; then
  echo "Restore failed (cannot access the backup file)."
  exit 1
fi

flist=$(awk '!/^ *#/ && NF' ~/wsl_ros_backup_manifest)

if tar --checkpoint=.200 --ignore-failed-read -cjf $rbpth -C / $flist home/student/wsl_ros_backup_manifest ; then
  echo "$(date): Backup from $(hostname) [wsl-ros version: $WSL_ROS_VER]" >> $rblog
  echo -e ".\nBackup complete."
fi
