#!/usr/bin/env bash

# construct hostname from IP address

# get the last part of the IP address:
ip_sfx=$(ip -o addr show dev "wlp4s0" | awk '$3 == "inet" {print $4}' | sed -r 's!/.*!!; s!.*\.!!')
# subtract 100 to get the device number:
waffle_no=$(($ipnum-100))
# construct the hostname:
echo "dia-waffle$waffle_no"

# tbc...
