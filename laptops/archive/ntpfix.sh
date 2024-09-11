#!/usr/bin/env bash

# 11th May 2022

sudo apt install -y chrony ntpdate net-tools

sudo ntpdate ntp.ubuntu.com

sudo groupadd timesyncers
sudo adduser student timesyncers

sudo wget -O /etc/sudoers.d/rossync https://raw.githubusercontent.com/tom-howard/tuos_robotics/humble/laptops/rossync

sudo reboot
