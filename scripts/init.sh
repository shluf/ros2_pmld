#!/bin/bash

source /opt/ros/$(echo $ROS_DISTRO)/setup.bash
if [ -f install/setup.bash ]; then
    source $(pwd)/install/setup.bash
fi

export GAZEBO_MODEL_DATABASE_URI=""