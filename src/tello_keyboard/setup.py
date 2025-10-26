from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tello_keyboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shluf',
    maintainer_email='luthfisalis09@gmail.com',
    description='Keyboard controller for Tello drone',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'keyboard_controller = tello_keyboard.keyboard_controller:main',
        ],
    },
)
