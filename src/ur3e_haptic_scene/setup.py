from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ur3e_haptic_scene'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    # Allow ROS 2 to discover this package
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
    # Install package.xml
        ('share/' + package_name, ['package.xml']),
    # Install launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    # Install Gazebo world files
        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*.sdf')),
    # Install YAML configs
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rookie',
    maintainer_email='rookie@todo.todo',
    description='UR3e haptic scene configuration (Gazebo world + MoveIt obstacles).',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Note: scripts are provided from the scripts subpackage
            'add_obstacles = ur3e_haptic_scene.scripts.add_obstacles:main',
            'virtual_force = ur3e_haptic_scene.scripts.virtual_force_node:main',
        ],
    },
)

