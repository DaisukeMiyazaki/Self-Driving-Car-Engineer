#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
source /opt/carla-ros-bridge/install/setup.bash
exec "$@"
