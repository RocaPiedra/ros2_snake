#!/bin/bash
set -e

source /opt/ros/foxy/setup.bash # ros reference
# source /acciona_challenge/foxy/dev_ws/install/setup.bash # package reference

exec "$@"