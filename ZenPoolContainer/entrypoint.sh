#!/bin/bash

set -e

source /opt/ros/humble/setup.bash
cd ~

echo "Provided arguments: $@"


exec $@