#!/bin/bash
set -e
# setup ros2 environment
#source "/ros2_ws/install/setup.bash"
source "/opt/rosbook/install/setup.bash"

# debug output formatting
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{time}]: {message}"
exec "$@"
