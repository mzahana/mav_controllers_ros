from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare an argument for the YAML file path
    x_arg = DeclareLaunchArgument(
        'x',
        default_value="0.0",
        description='x setpoints in meter(s)'
    )
    y_arg = DeclareLaunchArgument(
        'y',
        default_value="0.0",
        description='y setpoints in meter(s)'
    )
    z_arg = DeclareLaunchArgument(
        'z',
        default_value="2.0",
        description='z setpoints in meter(s)'
    )
    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value="0.0",
        description='yaw setpoints in radian'
    )
    
    return LaunchDescription([
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        Node(
            package='mav_controllers_ros',  
            executable='static_setpoint_test_node',
            name='static_setpoint_test_node',
            output='screen',
            parameters=[
                {'x': LaunchConfiguration('x')},
                {'y': LaunchConfiguration('y')},
                {'z': LaunchConfiguration('z')},
                {'yaw': LaunchConfiguration('yaw')},
                {'use_multi_dof_msg': True}
            ],
            remappings=[
                ('se3controller/setpoint', 'geometric_controller/setpoint'),
                ('se3controller/multi_dof_setpoint', 'multi_dof_wp') # Goe to the topic in go_to_waypoint_node in mav_trajectory_generation_ros2 pkg
            ]
        )
    ])
