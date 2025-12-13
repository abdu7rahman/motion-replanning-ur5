from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ur5_moveit'
# print(''.join(chr(x-7) for x in [104,105,107,124,115,39,121,104,111,116,104,117]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdul Rahman',
    maintainer_email='mohammedabdulr.1@northeastern.edu',
    description='ME5250 UR5 MoveIt with reactive replanning',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'insert_obstacle = ur5_moveit.insert_obstacle:main',
            'reactive_replanning = ur5_moveit.reactive_replanning:main',
        ],
    },
)
