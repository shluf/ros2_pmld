from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'tello_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'model/keypoint_classifier'), glob('model/keypoint_classifier/*')),
        (os.path.join('share', package_name, 'model/point_history_classifier'), glob('model/point_history_classifier/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Control nodes for Tello drone',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mode_manager_node = tello_control.mode_manager_node:main',
            'control_arbitrator_node = tello_control.control_arbitrator_node:main',
            'tracking_controller_node = tello_control.control_modes.tracking_controller_node:main',
            'gesture_control_node = tello_control.control_modes.gesture_control_node:main',
            'keyboard_controller = tello_control.control_modes.keyboard_controller:main',
            'joy_controller_node = tello_control.control_modes.joy_controller_node:main',
        ],
    },
)
