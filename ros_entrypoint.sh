#!/bin/bash
set -e

. /opt/ros/foxy/setup.bash # ros reference
. /ros2_snake/dev_ws/install/setup.bash # package reference

exec "$@"