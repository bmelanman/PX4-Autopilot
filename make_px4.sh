#!/bin/bash

# Call ulimit to make sure we have enough file descriptors
ulimit -S -n 2048

# Run make with the arguments passed to this script
make $@

# Done!
echo -e "\n\nDone!"
