#!/bin/bash

WAFFLE_NO=$1
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

if ask "[INPUT]: Preparing to configure for robot: dia-waffle$WAFFLE_NO. OK to continue?"; then
    echo "[INFO]: Updating hostname..."
    sudo hostnamectl set-hostname dia-waffle$WAFFLE_NO
    sudo sed -i 's/'$OLD_HOST'/dia-waffle'$WAFFLE_NO'/g' /etc/hosts
    sudo sed -i 's/'$OLD_HOST'/dia-waffle'$WAFFLE_NO'/g' ~/.bashrc
    if ask "[INPUT]: Do you want to update the OpenCR board firmware?"; then
        if ask "[INPUT]: Is the OpenCR board connected?"; then
            export OPENCR_PORT=/dev/ttyACM0
            export OPENCR_MODEL=waffle_noetic
            cd ~/device_firmware/opencr/opencr_update/
            ./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
            cd ~
            echo "[INFO]: OpenCR firmware (should) now be updated!"
            sleep 5
        else
            echo "[INFO] Couldn't update the OpenCR board."
        fi
    fi
    if ask "[INPUT]: Do you want to update the RealSense Camera firmware?"; then
        if ask "[INPUT]: Is the camera connected?"; then
            echo "[INFO]: Looking for available devices..."
            rs-fw-update -l
            if ask "[INPUT]: Is the device visible in the list above?"; then
                echo "[INFO]: Attempting to update the camera firmware..."
                rs-fw-update -f ~/device_firmware/realsense_d435/Signed_Image_UVC_5_12_14_50.bin
                sleep 5
                rs-fw-update -l
                echo "[INFO]: The device firmware should now be listed (above) as 05.12.14.50..."
                sleep 5
            else
                echo "[INFO]: Couldn't connect to the camera."
            fi
        else
            echo "[INFO]: Couldn't update the camera."
        fi
    fi
    echo "[INFO]: Robot configuration is now complete. The system now needs to be restarted."
    if ask "[INPUT]: Do you want to do this now?"; then
        echo "[COMPLETE]: Initiating reboot..."
        sudo reboot
    else
        echo "[COMPLETE]: Don't forget to reboot ASAP."
    fi
fi
