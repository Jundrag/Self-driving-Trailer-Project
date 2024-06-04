from setuptools import setup
import os

package_name = 'motor_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/dual_motor_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jundrag',
    maintainer_email='jundrag@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'controller1 = motor_control.controller1:main',
            'controller2 = motor_control.controller2:main',
            'serial_bridge = motor_control.serial_bridge:main',
            'tankturn = motor_control.tankturn:main',
        ],
    },
)
