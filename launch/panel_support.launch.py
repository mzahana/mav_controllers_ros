"""Everything the RViz field panels need, running ON THE VEHICLE.

The panels themselves run on a ground-station laptop
(`ros2 launch geo_tuner_rviz_plugins field_monitor.launch.py`); this launch
starts the two nodes they talk to, which must be on the vehicle:

  gain_saver           persists gains changed from the panel, by writing the
                       override YAML the controller launch files load at boot.
                       A parameter write alone is runtime only.
  trajectory_test_node the excitation source the Trajectory Test panel drives.
                       auto_start is forced off: a trajectory must be started
                       deliberately from the panel, never by a launch.

The auto-tune conductor lives in geo_tuner and is started by that package's
own launch (geo_tuner depends on this one, so this file cannot depend back):

    ros2 launch geo_tuner field_tune.launch.py ns:=interceptor

It starts neither the controller nor mavros -- those come up with your normal
bringup, and this attaches to them.

    ros2 launch mav_controllers_ros panel_support.launch.py
    ros2 launch mav_controllers_ros panel_support.launch.py controller_ns:=interceptor
    ros2 launch mav_controllers_ros panel_support.launch.py trajectory:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('mav_controllers_ros')

    return LaunchDescription([
        DeclareLaunchArgument(
            'controller_ns', default_value='',
            description="Namespace of the controller stack (e.g. 'interceptor' "
                        "in the d2dtracker SITL; empty for a bare field stack)."),
        DeclareLaunchArgument(
            'gain_saver', default_value='true',
            description='Run gain_saver, which the panel Save button calls.'),
        DeclareLaunchArgument(
            'trajectory', default_value='true',
            description='Run trajectory_test_node, which the Trajectory Test '
                        'panel drives.'),
        DeclareLaunchArgument(
            'trajectory_type', default_value='',
            description='Initial shape (empty keeps the YAML value). The panel '
                        'can change it at any time while the node is holding.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gain_saver.launch.py')),
            launch_arguments={'controller_ns': LaunchConfiguration('controller_ns')}.items(),
            condition=IfCondition(LaunchConfiguration('gain_saver')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'trajectory_test.launch.py')),
            launch_arguments={
                'controller_ns': LaunchConfiguration('controller_ns'),
                'trajectory_type': LaunchConfiguration('trajectory_type'),
                # Never auto-start from a launch file: the whole point of the
                # panel is that a human presses START.
                'auto_start': 'false',
                # Explicit, and note that this value also lands in the
                # PARENT scope -- IncludeLaunchDescription does not isolate
                # launch configurations. Nothing above may use the name
                # 'rviz' for its own RViz (sitl_test.launch.py calls its
                # argument open_rviz for exactly this reason).
                'rviz': 'false',
            }.items(),
            condition=IfCondition(LaunchConfiguration('trajectory')),
        ),
    ])
