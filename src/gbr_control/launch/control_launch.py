from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gbr_control',
            executable='gbr_direct_interface',
            name='gbr_keylogger_node',
            output='screen'
        ),
        Node(
            package='gbr_control',
            executable='dynsys_program',
            name='dynsys_node',
            output='screen'
        ),
    ])

