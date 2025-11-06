from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'gesture_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Model files
        (os.path.join('share', package_name, 'model/keypoint_classifier'),
            glob('model/keypoint_classifier/*')),
        (os.path.join('share', package_name, 'model/point_history_classifier'),
            glob('model/point_history_classifier/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shluf',
    maintainer_email='luthfisalis09@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gesture_controller = gesture_control.gesture_controller:main',
            'test_gesture = gesture_control.gesture_recognition:main',
        ],
    },
)
