from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_control',
            executable='controller1',
            name='motor_controller1',
            output='screen'
        ),
        Node(
            package='motor_control',
            executable='controller2',
            name='motor_controller2',
            output='screen'
        ),
        Node(
            package='motor_control',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen'
        ),
        Node(
            package='motor_control',
            executable='tankturn',
            name='tankturn',
            output='screen'
        )
    ])

