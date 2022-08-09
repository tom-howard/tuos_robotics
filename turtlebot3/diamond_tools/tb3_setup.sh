#!/usr/bin/env bash

OLD_HOST=$(hostname)

ask() {
    local reply prompt
    prompt='y/n'
    echo -n "$1 [$prompt] >> "
    read -r reply </dev/tty
    if [[ -z $reply ]]; then
        return 1;
    elif [ "$reply" == "y" ] || [ "$reply" == "Y" ]; then
        return 0;
    else
        return 1;
    fi
}

# get the last part of the IP address:
ip_sfx=$(ip -o addr show dev "wlp4s0" | awk '$3 == "inet" {print $4}' | sed -r 's!/.*!!; s!.*\.!!')
# subtract 100 to get the device number:
WAFFLE_NO=$(($ip_sfx-100))

if ask "[INPUT] Preparing to configure for robot: dia-waffle$WAFFLE_NO. IS THIS CORRECT??"; then
    echo "[INFO] Updating hostname..."
    sudo hostnamectl set-hostname dia-waffle$WAFFLE_NO
    sudo sed -i 's/'$OLD_HOST'/dia-waffle'$WAFFLE_NO'/g' /etc/hosts
    echo "[INFO] Checking the RealSense Camera Firmware..."
    rs-fw-update -l
    echo "[INFO] Camera firmware version should be displayed above as 05.12.14.50... (hopefully!)"
    sleep 5
    echo "[COMPLETE] Robot configuration is now complete."
    if ask "[INPUT] Do you want to shutdown the robot now?"; then
        echo "[INFO] Powering off..."
        sudo poweroff
    else
        echo "[INFO] Don't forget to reboot/poweroff ASAP."
    fi
else
    echo "[INFO] CANCELLED."
fi
