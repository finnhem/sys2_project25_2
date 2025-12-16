#!/bin/bash
# Fix USB serial device permissions for Arduino access
# This script ensures /dev/ttyACM* and /dev/ttyUSB* devices are accessible

# Find all USB serial devices and fix their permissions
for device in /dev/ttyACM* /dev/ttyUSB*; do
    if [ -e "$device" ]; then
        sudo chgrp dialout "$device" 2>/dev/null || true
        sudo chmod 0660 "$device" 2>/dev/null || true
        echo "Fixed permissions for $device"
    fi
done

