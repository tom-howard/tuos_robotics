#!/usr/bin/env bash

# 11th May 2022

sudo apt install -y chrony ntpdate net-tools

sudo wget -O /usr/local/bin/laptop_tsync https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/laptop_tsync
sudo chmod +x /usr/local/bin/laptop_tsync

sudo groupadd timesyncers
sudo adduser student timesyncers

sudo wget -O etc/sudoers.d/rossync https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/laptops/rossync
