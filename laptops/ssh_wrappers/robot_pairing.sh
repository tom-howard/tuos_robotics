#!/usr/bin/env bash

# CC BY-NC
# Alex Lucas & Tom Howard, University of Sheffield
# Copyright (c) 2022

GREEN='\033[0;32m'
NC='\033[0m'

waffle_id=$(hostname)
laptop_id=$1

echo -e "${GREEN}[$waffle_id]${NC}: Pairing Robot to dia-laptop$laptop_id..."
echo "$laptop_id" > ~/.tb3_tools/dia_laptop

# remove all ssh keys other than the one that has just been created:
ssh_key_file=~/.ssh/authorized_keys
new_key=$(grep "." $ssh_key_file | tail -1)
rm $ssh_key_file
echo "$new_key" > $ssh_key_file

echo -e "${GREEN}[$waffle_id]${NC}: Syncronising with NTP server..."
sudo ntpdate ntp.ubuntu.com

echo -e "${GREEN}[$waffle_id]${NC}: Pairing complete."