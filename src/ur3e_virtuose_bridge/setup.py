from setuptools import setup

package_name = 'ur3e_virtuose_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/virtuose_bridge.yaml', 'config/virtuose_force_bridge.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rookie',
    maintainer_email='you@example.com',
    description='Bridge Virtuose ROS2 output to UR3e MoveIt Servo Twist commands.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'virtuose_bridge = ur3e_virtuose_bridge.virtuose_bridge:main',
            'virtuose_force_bridge = ur3e_virtuose_bridge.virtuose_force_bridge:main',
        ],
    },
)
