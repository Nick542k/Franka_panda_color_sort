from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ridgeback_sim_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nick542k',
    maintainer_email='your_email@example.com',
    description='Ridgeback simulation control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'autonomous_navigator = ridgeback_sim_control.autonomous_navigator:main',
        'obstacle_avoider = ridgeback_sim_control.obstacle_avoider:main',
        'sensor_listener = ridgeback_sim_control.sensor_listener:main',
        'velocity_commander = ridgeback_sim_control.velocity_commander:main',
    ], 
  },
)
