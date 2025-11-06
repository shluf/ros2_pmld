#!/bin/bash

echo "Installing ROS2 humble packages..."
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-cv-bridge \
    ros-humble-camera-calibration-parsers

echo ""
echo "Installing system dependencies..."
sudo apt install -y \
    libasio-dev \
    python3-opencv \
    python3-numpy \
    python3-pyqt5

echo ""
echo "Installing Python packages..."
pip3 install mediapipe tflite-runtime

# echo ""
# echo "Fixing Python build dependencies..."
# pip3 install 'setuptools>=30.3.0,<80' --force-reinstall --quiet
# pip3 install --upgrade packaging wheel --quiet

echo ""
echo "All Dependencies Installed!"