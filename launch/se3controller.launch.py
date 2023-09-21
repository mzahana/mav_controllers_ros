import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Define the path to the default YAML file
    default_yaml_path = os.path.join(
        get_package_share_directory('px4_geometric_controller'),
        'config',
        'se3controller.yaml'
    )

    # Declare the argument for the YAML file path
    yaml_path_arg = DeclareLaunchArgument(
        'yaml_path',
        default_value=default_yaml_path,
        description='Path to the YAML file with parameters for the SE3ControllerNode'
    )

    # Define the node
    se3controller_node = Node(
        package='px4_geometric_controller',
        executable='se3controller_node',
        name='se3controller_node',
        output='screen',
        parameters=[LaunchConfiguration('yaml_path')],
        remappings=[
            ('se3controller/setpoint', 'se3controller/setpoint'),
            ('se3controller/odom', 'mavros/local_position/odom'),
            ('se3controller/enable_motors', 'se3controller/enable_motors')
        ]
    )

    return LaunchDescription([
        yaml_path_arg,
        se3controller_node,
        LogInfo(msg=["Launching SE3ControllerNode with parameters from: ", LaunchConfiguration('yaml_path')]),
    ])
