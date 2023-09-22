import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the default YAML config file
    default_param_file = os.path.join(
        get_package_share_directory('geometric_controller_ros'),
        'config',
        'se3controller_mavros.yaml'
    )

    # Declare an argument for the YAML file path
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_param_file,
        description='Full path to the YAML parameter file to use'
    )

    # Node configuration
    se3controller_to_mavros_node = Node(
        package='geometric_controller_ros',
        executable='se3controller_mavros_node',
        name='se3controller_mavros_node',
        namespace='',
        output='screen',
        parameters=[LaunchConfiguration('param_file')],
        remappings=[
            ('mavros/attitude_target', 'mavros/setpoint_raw/attitude'),
            ('se3controller_mavros/odom', 'mavros/local_position/odom'),
            ('se3controller_mavros/imu', 'mavros/imu/data'),
            ('mavros/state', 'mavros/state')
        ]
    )

    return LaunchDescription([
        param_file_arg,
        se3controller_to_mavros_node,
        LogInfo(msg=["Launching se3controller_mavros_node with parameters from: ", LaunchConfiguration('param_file')]),
    ])
