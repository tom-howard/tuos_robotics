#!/usr/bin/env bash

LOG_DIR="${HOME}/.diamond/diamond_tools/"

FILES=$(ls -la "$LOG_DIR")
LATEST=$(ls "$LOG_DIR"/ros_ws-*.sh 2>/dev/null | sort | tail -n 1)

FILENAME=$(basename "${LATEST}")

TIMESTAMP=$(stat -c %y "$LATEST" | awk '{print $1, $2}')

