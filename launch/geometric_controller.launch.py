import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import sys

from ament_index_python.packages import get_package_share_directory

# The override-directory resolver is shared with gain_saver and the other
# launch files; see launch/config_dir.py.
sys.path.insert(0, os.path.join(get_package_share_directory('mav_controllers_ros'), 'launch'))
from config_dir import override_path  # noqa: E402

def generate_launch_description():

    # Define the path to the default YAML file
    default_yaml_path = os.path.join(
        get_package_share_directory('mav_controllers_ros'),
        'config',
        'geometric_controller.yaml'
    )

    # Declare the argument for the YAML file path
    yaml_path_arg = DeclareLaunchArgument(
        'yaml_path',
        default_value=default_yaml_path,
        description='Path to the YAML file with parameters for the controller'
    )

    # Namespace
    # If the namespace is not '', you need to adjust the yaml file to have the same namespace defined here
    controller_ns_arg = DeclareLaunchArgument(
        'controller_ns',
        default_value='',
        description='Namespace of the controller node'
    )

    # Define the node
    geometric_controller_node = Node(
        package='mav_controllers_ros',
        executable='geometric_controller_node',
        name='geometric_controller_node',
        namespace=LaunchConfiguration('controller_ns'),
        output='screen',
        # The override file, when present, is listed AFTER the shipped
        # config so its values win. It is written on the vehicle by
        # gain_saver; delete it to fall back to the package defaults.
        parameters=([LaunchConfiguration('yaml_path')] +
                    ([override_path('geometric_controller.override.yaml')]
                     if override_path('geometric_controller.override.yaml') else [])),
        remappings=[
            ('geometric_controller/setpoint', 'geometric_controller/setpoint'), # sub
            ('geometric_controller/odom', 'geometric_controller/odom'), # sub
            ('geometric_controller/enable_motors', 'geometric_controller/enable_motors'), # sub
            ('geometric_controller/multi_dof_setpoint', 'geometric_controller/multi_dof_setpoint'), # sub
        ]
    )

    return LaunchDescription([
        yaml_path_arg,
        controller_ns_arg,
        geometric_controller_node,
        LogInfo(msg=["Launching GeometricAttitudeControl with parameters from: ", LaunchConfiguration('yaml_path')]),
    ])
