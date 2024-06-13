#!/bin/bash

# Run the first command
python scripts/gello_get_offset.py --start-joints 0 0 0 -1.571 0 1.571 0 --joint-signs 1 -1 1 1 1 -1 1 --port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISXHW-if00-port0

# Run the second command in a loop until it succeeds
while true; do
    output=$(python3 experiments/run_env.py --agent=gello)
    echo "$output"
    if echo "$output" | grep -q "joint\[7\]:"; then
        echo "Adjust the gripper trigger position and press [Enter] to retry."
        read -p ""
    else
        echo "Executed successfully."
        break
    fi
done

