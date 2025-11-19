from setuptools import setup
import os
from glob import glob

package_name = 'tello_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
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
            'tracking_controller_node = tello_control.tracking_controller_node:main',
            'control_arbitrator_node = tello_control.control_arbitrator_node:main',
        ],
    },
)
