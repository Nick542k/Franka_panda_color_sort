import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'franka_color_sort'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nithish',
    maintainer_email='you@example.com',
    description='Franka Panda color-sorting robot simulation (Gazebo Ignition + MoveIt2)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_detector_node = franka_color_sort.color_detector_node:main',
'pick_and_sort_node = franka_color_sort.pick_and_sort_node:main',
            'patch_inertia = franka_color_sort.patch_inertia:main',
        ],
    },
)
