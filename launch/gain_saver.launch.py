"""Vehicle-side gain persistence.

Run this ON THE VEHICLE alongside the controller. A parameter write from a
ground station changes only the running node's memory; this node writes the
live values to override YAML that the controller launch files load at the
next boot.

    ros2 launch mav_controllers_ros gain_saver.launch.py
    ros2 launch mav_controllers_ros gain_saver.launch.py controller_ns:=interceptor
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'controller_ns', default_value='',
            description="Namespace of the controller stack (e.g. 'interceptor')."),
        DeclareLaunchArgument(
            'output_dir', default_value='',
            description='Where the override YAML is written. Empty resolves '
                        'the same way the controller launch files do (shared '
                        'volume on the vehicle); see launch/config_dir.py.'),
        Node(
            package='mav_controllers_ros',
            executable='gain_saver.py',
            name='gain_saver',
            namespace=LaunchConfiguration('controller_ns'),
            output='screen',
            parameters=[{
                'controller_node': [LaunchConfiguration('controller_ns'),
                                    '/geometric_controller_node'],
                'mavros_node': [LaunchConfiguration('controller_ns'),
                                '/geometric_mavros_node'],
                'output_dir': LaunchConfiguration('output_dir'),
            }],
        ),
    ])
