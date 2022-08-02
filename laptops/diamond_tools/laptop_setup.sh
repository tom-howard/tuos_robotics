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

SSID_NOW=$(iwgetid -r)
if [[ $SSID_NOW != "DIA-LAB" ]]; then
    echo "Connecting to DIA-LAB..."
    nmcli c up DIA-LAB
fi

# get the last part of the IP address:
ip_sfx=$(ip -o addr show dev "wlo1" | awk '$3 == "inet" {print $4}' | sed -r 's!/.*!!; s!.*\.!!')
# subtract 100 to get the device number:
LAPTOP_NO=$ip_sfx

SSID_NOW=$(iwgetid -r)
if [[ $SSID_NOW != "eduroam" ]]; then
    echo "Connecting to eduroam..."
    nmcli c up eduroam
fi

echo "Preparing to configure this device as dia-laptop$LAPTOP_NO."
if ask "[INPUT] IS THIS CORRECT??"; then
    echo "[INFO] Updating hostname..."
    sudo hostnamectl set-hostname dia-laptop$LAPTOP_NO
    sudo sed -i 's/'$OLD_HOST'/dia-laptop'$LAPTOP_NO'/g' /etc/hosts
    sudo sed -i 's/'$OLD_HOST'/dia-laptop'$LAPTOP_NO'/g' ~/.bashrc

    echo "[INFO] Removing '~/.tuos/waffle_number' from both user profiles..."
    
    tmp_file=/tmp/remove_waffle_no.sh
    rm -f $tmp_file
    touch $tmp_file
    echo -e "#!/usr/bin/env bash\n" >> $tmp_file
    echo -e "rm -f ~/.tuos/waffle_number" >> $tmp_file
    # run as current user:
    chmod +x $tmp_file
    # run as 'student':
    sudo -i -u student "$tmp_file"
    rm -f $tmp_file

    echo "[INFO] Robot configuration is now complete."
    echo "Restart the device ASAP."
else
    echo "[INFO] CANCELLED."
fi
