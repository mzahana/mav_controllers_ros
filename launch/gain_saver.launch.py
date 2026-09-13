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
                        'volume on the vehicle); see launch/config_dir.py. '
                        'Also settable at runtime -- the RViz gain panel sets '
                        'it from its override dir box.'),
        DeclareLaunchArgument(
            'controller_config_file', default_value='',
            description='Shipped config the controller was launched with.'),
        DeclareLaunchArgument(
            'mavros_config_file', default_value='',
            description='Shipped config geometric_mavros_node was launched with.'),
        DeclareLaunchArgument(
            'controller_session_file', default_value='',
            description='Per-launch pin file loaded after the controller config '
                        '(its keys are saved at their configured values).'),
        DeclareLaunchArgument(
            'mavros_session_file', default_value='',
            description='Per-launch pin file loaded after the mavros config, e.g. '
                        'the tuner forcing enable_thrust_estimator false.'),
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
                'controller_config_file': LaunchConfiguration('controller_config_file'),
                'mavros_config_file': LaunchConfiguration('mavros_config_file'),
                'controller_session_file': LaunchConfiguration('controller_session_file'),
                'mavros_session_file': LaunchConfiguration('mavros_session_file'),
            }],
        ),
    ])
