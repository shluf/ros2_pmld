source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=30 #TURTLEBOT3
export TURTLEBOT3_MODEL=waffle

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_pmld/src/follower/models
export GAZEBO_MODEL_DATABASE_URI=""