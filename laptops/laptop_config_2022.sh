#!/bin/bash

LAPTOP_NO=$1
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

if ask "[INPUT]: Preparing to configure for laptop: dia-waffle$LAPTOP_NO. OK to continue?"; then
    echo "[INFO]: Updating hostname..."
    sudo hostnamectl set-hostname dia-laptop$LAPTOP_NO
    sudo sed -i 's/'$OLD_HOST'/dia-laptop'$LAPTOP_NO'/g' /etc/hosts
    echo "$LAPTOP_NO" > /home/Shared/laptop_number
    echo "$LAPTOP_NO" > /home/Shared/miro_number
    sleep 2
    echo "[INFO]: Robot configuration is now complete. The system now needs to be restarted."
    if ask "[INPUT]: Do you want to do this now?"; then
        echo "[COMPLETE]: Initiating reboot..."
        reboot
    else
        echo "[COMPLETE]: Don't forget to reboot ASAP."
    fi
else
    echo "[EXIT]: Nothing was modified."
fi
