#!/usr/bin/env bash

rbpth=$rbname'.tar.gz'
rbwpth=$(sed "s/\/mnt\/u/U\:/;s/\//\\\/g" <<< $rbpth)

echo "Restoring your home directory from '$rbwpth', please wait..."

rblog=$rbname'-log.txt'

if ! touch $rblog; then
  echo "Restore failed (cannot access the backup file)."
  exit 1
fi

if tar --checkpoint=.200 -xjf $rbpth -C / ; then
  echo "$(date): Restore to $(hostname) [wsl-ros version: $WSL_ROS_VER]" >> $rblog
  echo -e ".\nRestore complete."
else
  echo -e "An error occurred during the restore process...\nYOUR DATA MAY NOT HAVE BEEN RECOVERED CORRECTLY."
fi