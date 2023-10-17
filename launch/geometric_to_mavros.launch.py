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
        get_package_share_directory('mav_controllers_ros'),
        'config',
        'geometric_mavros.yaml'
    )

    # Declare an argument for the YAML file path
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_param_file,
        description='Full path to the YAML parameter file to use'
    )

    # Node configuration
    geometric_to_mavros_node = Node(
        package='mav_controllers_ros',
        executable='geometric_mavros_node',
        name='geometric_mavros_node',
        namespace='',
        output='screen',
        parameters=[LaunchConfiguration('param_file')],
        remappings=[
            ('mavros/attitude_target', 'mavros/setpoint_raw/attitude'), # pub
            ('geometric_mavros/combined_odometry', 'geometric_controller/odom'), # pub
            ('geometric_mavros/odom', 'geometric_mavros/odom'), # sub
            ('geometric_mavros/imu', 'mavros/imu/data'), # sub
            ('mavros/state', 'mavros/state'), # sub
            ('geometric_mavros/pose', 'mavros/local_position/pose'), # sub
            ('geometric_mavros/twist', 'mavros/local_position/velocity_local'), # sub
        ]
    )

    return LaunchDescription([
        param_file_arg,
        geometric_to_mavros_node,
        LogInfo(msg=["Launching geometric_mavros_node with parameters from: ", LaunchConfiguration('param_file')]),
    ])
