#!/bin/bash

#? Usage:
#?    ${SCRIPT_NAME} <target> <flash_addr>
#?
#? Options:
#?    <target>      The target to build and flash.
#?    <flash_addr>  The flash address to use when flashing.
#?
#? Available Targets:
#?    1. ark_cannode_canbootloader ... 0x08000000 (Default)
#?    2. ark_cannode_default ......... 0x08010000
#?

SCRIPT_NAME="$(basename ${0})"
HEADER_LEN=20
HELP_FILTER="^#?"

# Check for help
if [ "$#" -eq 3 ] || [ [ "$#" -ge 1 ] && [ [ "$1" == "-h" ] || [ "$1" == "--help" ] ] ]; then
	# Print the help message
	head -${HEADER_LEN} ${SCRIPT_NAME} | grep -e "${HELP_FILTER}" | sed -e "s/${HELP_FILTER}//g" -e "s/\${SCRIPT_NAME}/${SCRIPT_NAME}/g" -e "s/^ //g"

	echo "---"
	echo "Num args: ${#}"
	echo "Args: ${@}"
	echo "---"
	exit 0
fi

##### Config

# Colors
RED="\x1b[31m"
GREEN="\x1b[32m"
YELLOW="\x1b[33m"
BLUE="\x1b[34m"
RESET="\x1b[0m"

SD_CARD_DIR="/Volumes/PIXHAWK"
LOG_DIR="./build_logs"

ARK_BOOTLOADER="ark_cannode_canbootloader"
ADDR_BOOTLOADER="0x08000000"

ARK_DEFAULT="ark_cannode_default"
ADDR_APPLICATION="0x08010000"

MAKE_TARGET=${1:-${ARK_DEFAULT}}
FLASH_ADDR=${2:-${ADDR_APPLICATION}}
DO_FLASH=1

##### Functions

function print_with_log {
	printf  "$1" | tee -a ${LOG_FILE}
}

function print_info {
	print_with_log "${BLUE}Info:${RESET} $@\n"
}

function print_warning {
	print_with_log "${YELLOW}Warning:${RESET} $@\n"
}

function print_error {
	print_with_log "${RED}Error:${RESET} $@\n"
	exit 1
}

# Arg 1: Task description message
# Arg 2: Task command
# Arg 3: Error message if `task command` fails
function do_task {
	# Check the number of args
	if [ $# -ne 3 ]; then
		print_error "do_task() requires 3 arguments but got $# instead." | tee -a ${LOG_FILE}
		exit 1
	fi

	# Task header
	print_with_log "\n#======================================#\n\n"

	# Info message
	print_info "$1"

	# Do the task
	print_with_log "      Running \"$2\"...\n\n" | tee -a ${LOG_FILE}

	# Breakdown:
	# -a: Append the output to `LOG_FILE`
	# -e: Return the command's exit status
	# -F: Immediately flush output after each write
	# -q: Run in quiet mode, omiting the start, stop, and command status messages
	script -F -q -e -a ${LOG_FILE} $2

	# Check if the task was successful
	if [ $? -ne 0 ]; then
		# Print the error message
		print_error "\n$3\n"
		exit 1
	fi

	# Task footer
	print_with_log "\n#======================================#\n\n"
}

##### Script Start

# Get the current time
TIME_START=$(date +%s)

# Setup the log file
LOG_FILE="${LOG_DIR}/${MAKE_TARGET}-$(date +%Y%m%d-%H%M%S).log"
mkdir -p ${LOG_DIR} && touch ${LOG_FILE}

print_with_log "\n#======================================#\n\n"

BUILD_DIR="./build/${MAKE_TARGET}"
OLD_BINS="${BUILD_DIR}/old_bins"

# Set the flash address based on the selected target
if [ "${MAKE_TARGET}" == "${ARK_BOOTLOADER}" ]; then
	FLASH_ADDR=${ADDR_BOOTLOADER}
	BIN_FILE="${BUILD_DIR}/${MAKE_TARGET}*.bin"

elif [ "${MAKE_TARGET}" == "${ARK_DEFAULT}" ]; then
	FLASH_ADDR=${ADDR_APPLICATION}
	BIN_FILE="${BUILD_DIR}/83*.uavcan.bin"

else
	# Print an error message
	print_warning "Unknown target: ${MAKE_TARGET}"
	print_warning "Build will continue, but the binary file will not be flashed :)"
	DO_FLASH=0
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

# Check if we should flash the binary file or not
if [ ${DO_FLASH} -eq 1 ]; then

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

	# Check if an we can copy the binary file to an SD card
	elif [ -d "${SD_CARD_DIR}" ]; then

		# Copy the binary file to the SD card
		do_task \
			"Copying ${BIN_FILE} to the SD card..." \
			"mv ${BIN_FILE} ${SD_CARD_DIR}" \
			"Failed to copy ${BIN_FILE} to the SD card"

		diskutil eject ${SD_CARD_DIR}

	# Otherwise, print an info message
	else
		BIN_FILE_RELATIVE=$(python3 -c "import os.path; print(os.path.relpath('${BIN_FILE}', '$(pwd)'))")
		# Print a warning message
		print_warning "No ST-LINK or SD card found!"
	fi
else
	# Print an info message
	print_info "Build is complete! Binary file not flashed."
fi

# Print the binary file path
if [ -f "${BIN_FILE}" ]; then
	print_info "Binary file:   ${GREEN}${BIN_FILE}${RESET}"
fi

# Print the flash address if we have one
if [ ${DO_FLASH} -eq 1 ]; then
	print_info "Flash address: ${GREEN}${FLASH_ADDR}${RESET}"
fi

print_with_log "\n#======================================#\n\n"

# Print the elapsed time, and we're done!
print_info "Took $(($(date +%s) - $TIME_START)) seconds. Done!\n"
