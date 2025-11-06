#!/bin/bash

source /opt/ros/humble/setup.bash
if [ -f install/setup.bash ]; then
    source $(pwd)/install/setup.bash
fi

export GAZEBO_MODEL_DATABASE_URI=""