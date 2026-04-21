#!/bin/bash
BUILD_DIR=$(dirname "$0")/build/


# spawn logger
"$BUILD_DIR/command-recorder" log.csv &
LOGGER_PID=$!

# run process
$@

#kill logger
pkill -SIGINT -f command-recorder