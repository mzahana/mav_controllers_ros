import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths to the default YAML config files
    default_se3controller_yaml = os.path.join(
        get_package_share_directory('mav_controllers_ros'),
        'config',
        'se3controller.yaml'
    )
    
    default_se3controller_mavros_yaml = os.path.join(
        get_package_share_directory('mav_controllers_ros'),
        'config',
        'se3controller_mavros.yaml'
    )

    # Declare launch arguments
    yaml_path_arg = DeclareLaunchArgument(
        'se3controller_yaml',
        default_value=default_se3controller_yaml,
        description='Path to the YAML file with parameters for the SE3ControllerNode'
    )

    param_file_arg = DeclareLaunchArgument(
        'se3controller_mavros_yaml',
        default_value=default_se3controller_mavros_yaml,
        description='Full path to the YAML parameter file to use for se3controller_mavros_node'
    )

    # Include mavros.launch.py
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/mavros.launch.py']),
        launch_arguments={'fcu_url': 'udp://:14540@127.0.0.1:14557'}.items()
    )

    # Include se3controller.launch.py
    se3controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/se3controller.launch.py']),
        launch_arguments={'yaml_path': LaunchConfiguration('se3controller_yaml')}.items()
    )

    # Include se3controller_to_mavros.launch.py
    se3controller_to_mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/se3controller_to_mavros.launch.py']),
        launch_arguments={'param_file': LaunchConfiguration('se3controller_mavros_yaml')}.items()
    )

    return LaunchDescription([
        yaml_path_arg,
        param_file_arg,
        mavros_launch,
        se3controller_launch,
        se3controller_to_mavros_launch,
        LogInfo(msg=["Combined launch file initiated."]),
    ])

