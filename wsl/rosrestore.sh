#!/usr/bin/env bash

rbpth=$rbname'.tar.gz'
rbwpth=$(sed "s/\/mnt\/u/U\:/;s/\//\\\/g" <<< $rbpth)

echo "Restoring your home directory from '$rbwpth', please wait..."

rblog=$rbname'-log.txt'

if ! touch $rblog; then
  echo "Restore failed (cannot access the backup file)."
  exit 1
fi

echo "$(date): Restore to $(hostname) [wsl-ros version: $WSL_ROS_VER]" >> $rblog

# extract the backup manifest from the archive:
if ! tar -xjf $rbpth -C / home/student/wsl_ros_backup_manifest; then
  flist=$(awk '!/^ *#/ && NF' ~/wsl_ros_backup_manifest)

  while IFS= read -r line ; do 
    line="/$line"
    if [ -d "$line" ]; then
      # this is a directory, so try to delete it:
      rm -rf $line
    fi
  done <<< "$flist"
fi

if tar --checkpoint=.200 -xjf $rbpth -C / ; then
  echo -e ".\nRestore complete."
fi