from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'exercise_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='famadio',
    maintainer_email='fabioamadio93@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop = exercise_navigation.keyboard_teleop:main',
            'point2point_nav = exercise_navigation.point2point_nav:main',
            'bug_nav = exercise_navigation.bug_nav:main',
        ],
    },
)
