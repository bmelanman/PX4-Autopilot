#!/bin/bash

function print_info {
	printf "\n${BLUE}Info:${RESET} $@\n\n" | tee -a ${LOG_FILE}
}

function print_error {
	printf "\n${RED}Error:${RESET} $@\n\n" | tee -a ${LOG_FILE}
	exit 1
}

function do_task {
	# Print the task header
	echo "#======================================#" | tee -a ${LOG_FILE}

	# Print the info message
	print_info "$1"

	# Do the task
	printf "${GREEN}Running \"$2\"...${RESET}\n" | tee -a ${LOG_FILE}
	script -F -q -e -a ${LOG_FILE} $2

	# Check if the task was successful
	if [ $? -ne 0 ]; then
		# Print the error message
		print_error "$3"
		exit 1
	else
		echo ""
	fi

	# Print the task footer
	echo "#======================================#" | tee -a ${LOG_FILE}
}

# Colors
RED="\x1b[31m"
GREEN="\x1b[32m"
BLUE="\x1b[34m"
RESET="\x1b[0m"

SD_CARD_DIR="/Volumes/PIXHAWK"
LOG_DIR="./build_logs"

ARK_BOOTLOADER="ark_cannode_canbootloader"
ADDR_BOOTLOADER="0x08000000"

ARK_DEFAULT="ark_cannode_default"
ADDR_APPLICATION="0x08010000"

MAKE_TARGET=${ARK_DEFAULT}
#MAKE_TARGET=${ARK_BOOTLOADER}
FLASH_ADDR=${ADDR_APPLICATION}


# Get the current time
TIME_START=$(date +%s)

# Check for input arguments
if [ $# -ge 1 ]; then
	# Use the first argument as the target and remove it from the list
	MAKE_TARGET=$1
	shift
fi

BIN_DIR="./build/${MAKE_TARGET}"
OLD_BINS="${BIN_DIR}/old_bins"

LOG_FILE="${LOG_DIR}/${MAKE_TARGET}-$(date +%Y%m%d-%H%M%S).log"
mkdir -p ${LOG_DIR} && touch ${LOG_FILE}

# Set the flash address based on the selected target
if [ "${MAKE_TARGET}" == "${ARK_BOOTLOADER}" ]; then
	FLASH_ADDR=${ADDR_BOOTLOADER}
	BIN_FILE="${BIN_DIR}/${MAKE_TARGET}.bin"

elif [ "${MAKE_TARGET}" == "${ARK_DEFAULT}" ]; then
	FLASH_ADDR=${ADDR_APPLICATION}
	BIN_FILE="${BIN_DIR}/83*.uavcan.bin"

else
	# Print an error message
	print_error "Invalid target: ${MAKE_TARGET}"
	exit 1
fi

# Move any old binary files
mkdir -p ${OLD_BINS} && mv ${BIN_FILE} ${OLD_BINS} >/dev/null 2>&1

# Create the build log directory if it doesn't exist
if [ ! -d ${BUILD_LOG_DIR} ] && [ ! $(mkdir -p ${BUILD_LOG_DIR}) ]; then
	print_error "Failed to create build log directory: ${BUILD_LOG_DIR}"
	exit 1
fi

print_info "Making target \"${MAKE_TARGET}\" and attempting to upload to loacal hardware..."

# Call ulimit to make sure we have enough file descriptors
ulimit -S -n 2048

# Make the target
do_task \
	"Building target: ${MAKE_TARGET}..." \
	"make ${MAKE_TARGET}" \
	"Failed to build target: ${MAKE_TARGET}"

# Get the path to the binary file
BIN_FILE="$(realpath ${BIN_FILE})"

if [ ! -f "${BIN_FILE}" ]; then
	print_error "Binary file not found: ${BIN_FILE}"
	exit 1
fi

# If an ST-LINK is connected, flash the binary file, or check if we can copy it to an SD card
if [ "$(st-info --descr)" ]; then

	# Flash the binary file
	do_task \
		"Flashing ${BIN_FILE}..." \
		"st-flash write ${BIN_FILE} ${FLASH_ADDR}" \
		"Failed to flash ${BIN_FILE}"

	# Reset the board
	do_task \
		"Resetting the board..." \
		"st-flash reset" \
		"Failed to reset the board?"

# Check if an SD card exists
elif [ -d "${SD_CARD_DIR}" ]; then

	# Copy the binary file to the SD card
	do_task \
		"Copying ${BIN_FILE} to the SD card..." \
		"mv ${BIN_FILE} ${SD_CARD_DIR}" \
		"Failed to copy ${BIN_FILE} to the SD card"

	diskutil eject ${SD_CARD_DIR}

else
	BIN_FILE_RELATIVE=$(python3 -c "import os.path; print(os.path.relpath('${BIN_FILE}', '$(pwd)'))")
	# Print a warning message
	printf "\n${RED}Error:${RESET} No ST-LINK or SD card found!\n"
	printf "${BLUE}\t- Binary file:   ${GREEN}./${BIN_FILE_RELATIVE}${RESET}\n"
	printf "${BLUE}\t- Flash address: ${GREEN}${FLASH_ADDR}${RESET}\n\n"
fi

# Get the end time
TIME_END=$(date +%s)

# Calculate the elapsed time
ELAPSED_TIME=$(($TIME_END - $TIME_START))

# Print the elapsed time
print_info "Elapsed time: Took ${ELAPSED_TIME} seconds"

# Done!
print_info "Done!"
