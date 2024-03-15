#!/bin/bash -e

# This script is used to flash the cannode firmware to the cannode board
BOARD_NAME=ark_cannode_default

# Run the build script
make "${BOARD_NAME}"

# Set the path to the firmware
ELF_FILE=./build/"${BOARD_NAME}"/"${BOARD_NAME}".elf

# Start a JLinkGDBServer
JLinkGDBServer \
	-if SWD \
	-port 2331 \
	-speed 4000 \
	-device STM32F103C8 \
	-singlerun -nogui &

# Start a GDB session
arm-none-eabi-gdb "${ELF_FILE}" \
	-ex "target remote localhost:2331" \
	-ex "load" \
	-ex "detach" \
	-ex "quit"

# Wait for the JLinkGDBServer to finish
sleep 1

# Done!
echo -e "\n\nDone!"
