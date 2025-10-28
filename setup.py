from setuptools import find_packages, setup

package_name = 'ur3_bridge'

setup(
    name='ur3_bridge',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rookie',
    maintainer_email='rookie@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bridge_node = ur3_bridge.bridge_node:main',
            'gamepad_to_ur3_cmd = ur3_bridge.gamepad_to_ur3_cmd:main',

        ],
    },
)
