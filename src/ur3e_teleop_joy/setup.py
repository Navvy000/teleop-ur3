from setuptools import setup

package_name = 'ur3e_teleop_joy'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rookie',
    maintainer_email='you@example.com',
    description='Joy-based teleoperation nodes for UR3e (dummy + joint teleop).',
    license='Apache-2.0',
    # Avoid tests_require to suppress legacy warnings
    entry_points={
        'console_scripts': [
            'joy_dummy = ur3e_teleop_joy.joy_dummy:main',
            'joint_teleop = ur3e_teleop_joy.joint_teleop:main',
            'cartesian_teleop = ur3e_teleop_joy.cartesian_teleop_improved:main',
        ],
    },
)
