#!/usr/bin/env bash

if [[ "$rbname" == "" ]]; then
  echo "You can't run this script standalone. Use 'wsl_ros backup' instead."
  exit 1
fi

if [ ! -f ~/.backup_exclusions ]; then
  # no exclusions file exists in ~, so load a default:
  cp ~/.wsl-ros/default_backup_exclusions ~/.backup_exclusions
fi

rbpth=$rbname'.tar.gz'
rbwpth=$(sed "s/\/mnt\/u/U\:/;s/\//\\\/g" <<< $rbpth)

echo "Backing up your WSL-ROS environment to '$rbwpth', please wait..."

rbdir=$(sed "s/\/[^/]*$//" <<< $rbname)
rblog=$rbname'-log.txt'

mkdir -p $rbdir

if ! touch $rblog; then
  echo "Restore failed (cannot access the backup file)."
  exit 1
fi

if tar -X $HOME/.backup_exclusions --exclude='home/student/.wsl-ros' --checkpoint=.200 -cjf $rbpth -C / home/student ; then
  echo "$(date): Backup from $(hostname) [wsl-ros version: $WSL_ROS_VER]" >> $rblog
  echo -e ".\nBackup complete."
else
  echo -e "An error occurred during the backup process...\nYOUR DATA MAY NOT HAVE BEEN BACKED UP CORRECTLY."
fi
