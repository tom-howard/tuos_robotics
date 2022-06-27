#!/usr/bin/env bash

# CC BY-NC
# Alex Lucas & Tom Howard, University of Sheffield
# Copyright (c) 2022

GREEN='\033[0;32m'
NC='\033[0m'

waffle_id=$(hostname)

echo -e "${GREEN}[$waffle_id]${NC} Syncronising with NTP Server..."
sudo ntpdate ntp.ubuntu.com > /dev/null
echo -e "${GREEN}[$waffle_id]${NC} Time syncronisation complete."