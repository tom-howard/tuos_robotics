#!/usr/bin/env bash

# CC BY-NC
# Alex Lucas & Tom Howard, University of Sheffield
# Copyright (c) 2022

waffle_id=$(hostname)
laptop_id=$1

ln=~/.diamond/dia_laptop
touch $ln
if grep -qi "$laptop_id" $ln; then
    # robot-side pairing looks OK...
    echo "confirm"
else 
    echo "deny"
fi