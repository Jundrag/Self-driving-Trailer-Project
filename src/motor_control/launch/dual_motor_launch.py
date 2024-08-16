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
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='motor_control',
            executable='joystick',
            name='joystick',
            output='screen'
        ),
        Node(
            package='motor_control',
            executable='monitor_motor',
            name='monitor_motor',
            output='screen'
        ),
        Node(
            package='motor_control',
            executable='actuator_control',
            name='actuator_control',
            output='screen'
        ),
        Node(
            package='motor_control',
            executable='realsense_camera_node',
            name='realsense_camera_node',
            output='screen'
        )
    ])

