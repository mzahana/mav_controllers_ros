"""Launch the trajectory test node (setpoint / circle / lemniscate).

Publishes feasibility-checked TargetCommand references for the geometric
controller. Typical use (SITL, namespace /interceptor):

  ros2 launch mav_controllers_ros trajectory_test.launch.py \
      controller_ns:=interceptor trajectory_type:=lemniscate auto_start:=true

Field use: keep auto_start:=false and trigger with
  ros2 service call /<ns>/trajectory_test/start std_srvs/srv/Trigger
"""

import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    yaml_path = LaunchConfiguration('yaml_path').perform(context)
    ns = LaunchConfiguration('controller_ns').perform(context)

    # CLI overrides applied on top of the YAML (later parameter entries win).
    # Empty strings mean "keep the YAML value".
    overrides = {}
    traj_type = LaunchConfiguration('trajectory_type').perform(context)
    if traj_type:
        overrides['trajectory_type'] = traj_type
    auto_start = LaunchConfiguration('auto_start').perform(context)
    if auto_start:
        overrides['auto_start'] = auto_start.lower() in ('true', '1')

    parameters = [yaml_path]
    if overrides:
        parameters.append(overrides)

    node = Node(
        package='mav_controllers_ros',
        executable='trajectory_test_node',
        name='trajectory_test_node',
        namespace=LaunchConfiguration('controller_ns'),
        output='screen',
        parameters=parameters,
        remappings=[
            # Reference consumed by the geometric controller
            ('geometric_controller/setpoint', 'geometric_controller/setpoint'),
            # Vehicle state inputs
            ('mavros/local_position/odom', 'mavros/local_position/odom'),
            ('mavros/state', 'mavros/state'),
            ('geometric_controller/enable_motors', 'geometric_controller/enable_motors'),
        ]
    )
    actions = [node]

    # Optional RViz with the tracking-quality layout. The shipped config uses
    # a __NS__ placeholder for topic names; fill it with the namespace here.
    if LaunchConfiguration('rviz').perform(context).lower() in ('true', '1'):
        template = os.path.join(
            get_package_share_directory('mav_controllers_ros'),
            'rviz', 'trajectory_test.rviz')
        with open(template, 'r') as f:
            cfg = f.read().replace('__NS__', f'/{ns}' if ns else '')
        out = tempfile.NamedTemporaryFile(
            mode='w', suffix='.rviz', prefix='trajectory_test_', delete=False)
        out.write(cfg)
        out.close()
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='trajectory_test_rviz',
            arguments=['-d', out.name],
            output='screen',
        ))

    return actions


def generate_launch_description():
    default_yaml_path = os.path.join(
        get_package_share_directory('mav_controllers_ros'),
        'config',
        'trajectory_test.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'yaml_path',
            default_value=default_yaml_path,
            description='Path to the trajectory test parameter YAML file'),
        DeclareLaunchArgument(
            'controller_ns',
            default_value='',
            description='Namespace of the controller stack (e.g. interceptor)'),
        DeclareLaunchArgument(
            'trajectory_type',
            default_value='',
            description='Override trajectory type: setpoint | circle | lemniscate '
                        '(empty keeps the YAML value)'),
        DeclareLaunchArgument(
            'auto_start',
            default_value='',
            description='Override auto_start (true only for SITL; empty keeps YAML value)'),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Open RViz with the trajectory tracking layout'),
        OpaqueFunction(function=launch_setup),
    ])
