#!/usr/bin/env bash

if ! sudo -v; then
    exit 0
else
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

    nmcli c up DIA-LAB

    # get the last part of the IP address:
    ip_sfx=$(ip -o addr show dev "wlo1" | awk '$3 == "inet" {print $4}' | sed -r 's!/.*!!; s!.*\.!!')
    # subtract 100 to get the device number:
    LAPTOP_NO=$ip_sfx

    nmcli c up eduroam

    if ask "[INPUT]: Preparing to configure for device: dia-laptop$LAPTOP_NO. IS THIS CORRECT??"; then
        echo "[INFO]: Updating hostname..."
        sudo hostnamectl set-hostname dia-laptop$LAPTOP_NO
        sudo sed -i 's/'$OLD_HOST'/dia-laptop'$LAPTOP_NO'/g' /etc/hosts
        sudo sed -i 's/'$OLD_HOST'/dia-laptop'$LAPTOP_NO'/g' ~/.bashrc
        echo "[INFO]: Robot configuration is now complete."
        echo "Restart the device ASAP."
    else
        echo "[INFO]: CANCELLED."
    fi
fi
