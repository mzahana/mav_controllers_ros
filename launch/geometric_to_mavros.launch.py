import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import sys

from ament_index_python.packages import get_package_share_directory

# The override-directory resolver is shared with gain_saver and the other
# launch files; see launch/config_dir.py.
sys.path.insert(0, os.path.join(get_package_share_directory('mav_controllers_ros'), 'launch'))
from config_dir import override_path  # noqa: E402

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

    # Namespace
    # If the namespace is not '', you need to adjust the yaml file to have the same namespace defined here
    ns_arg = DeclareLaunchArgument(
        'mavros_ns',
        default_value='',
        description='Namespace of the mavros node'
    )

    # Declared here, resolved inside make_node: an empty value must mean
    # "no file" rather than a parameter file named ''.
    session_params_arg = DeclareLaunchArgument(
        'session_params',
        default_value='',
        description='Optional YAML loaded AFTER the shipped config and the '
                    'persisted override, for parameters one launch needs to '
                    'pin for the duration of a flight (e.g. the tuner '
                    'forcing enable_thrust_estimator off). Never persisted.'
    )

    def make_node(context):
        session = LaunchConfiguration('session_params').perform(context).strip()
        # Load order is precedence: shipped config, then the persisted
        # override, then the session file -- so a session pin wins over
        # both without touching what is persisted on the vehicle.
        params = [LaunchConfiguration('param_file')]
        if override_path('geometric_mavros.override.yaml'):
            params.append(override_path('geometric_mavros.override.yaml'))
        if session:
            params.append(session)
        return [Node(
            package='mav_controllers_ros',
            executable='geometric_mavros_node',
            name='geometric_mavros_node',
            namespace=LaunchConfiguration('mavros_ns'),
            output='screen',
            parameters=params,
            remappings=[
                ('mavros/attitude_target', 'mavros/setpoint_raw/attitude'), # pub
                ('geometric_mavros/combined_odometry', 'geometric_controller/odom'), # pub
                ('geometric_mavros/odom', 'geometric_mavros/odom'), # sub
                ('geometric_mavros/imu', 'mavros/imu/data'), # sub
                ('mavros/state', 'mavros/state'), # sub
                ('geometric_mavros/pose', 'mavros/local_position/pose'), # sub
                ('geometric_mavros/twist', 'mavros/local_position/velocity_local'), # sub
            ]
        )]

    return LaunchDescription([
        param_file_arg,
        ns_arg,
        session_params_arg,
        OpaqueFunction(function=make_node),
        LogInfo(msg=["Launching geometric_mavros_node with parameters from: ", LaunchConfiguration('param_file')]),
    ])
