from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ns_arg = DeclareLaunchArgument(
        'circularsetpoin_ns',
        default_value="",
        description='Naemspace'
    )

    speed_arg = DeclareLaunchArgument(
        'speed',
        default_value="2.0",
        description='trajectory speed'
    )

    x_arg = DeclareLaunchArgument(
        'x',
        default_value="0.0",
        description='x coordinate of the center'
    )

    y_arg = DeclareLaunchArgument(
        'y',
        default_value="0.0",
        description='y coordinate of the center'
    )

    z_arg = DeclareLaunchArgument(
        'z',
        default_value="2.0",
        description='height'
    )

    radius_arg = DeclareLaunchArgument(
        'radius',
        default_value="1.0",
        description='radius'
    )

    yawrate_arg = DeclareLaunchArgument(
        'yawrate',
        default_value="20.0",
        description='Maximum yaw rate'
    )

    zeroyaw_arg = DeclareLaunchArgument(
        'zeroyaw',
        default_value='false',
        description='If True, yaw will be fixed at zero'
    )
    return LaunchDescription([
        speed_arg,
        x_arg,
        y_arg,
        z_arg,
        radius_arg,
        yawrate_arg,
        zeroyaw_arg,
        Node(
            package='mav_controllers_ros',
            executable='circular_trajectory_node',
            name='circular_trajectory_node',
            namespace=LaunchConfiguration('circularsetpoin_ns'),
            output='screen',
            parameters=[
                {'x_center': LaunchConfiguration('x')},
                {'y_center': LaunchConfiguration('y')},
                {'z': LaunchConfiguration('z')},
                {'radius': LaunchConfiguration('radius')},
                {'speed': LaunchConfiguration('speed')},
                {'publish_rate_ms': 10},
                {'max_yaw_rate': LaunchConfiguration('yawrate')},
                {'zero_yaw': LaunchConfiguration('zeroyaw')}
            ],
            remappings=[
                ('se3controller/setpoint', 'geometric_controller/setpoint'),  # Replace 'new_topic_name' with your desired topic name
                ('mavros/state', 'mavros/state'),
                ('se3controller/enable_motors', 'geometric_controller/enable_motors'),
                ('mavros/local_position/odom', 'mavros/local_position/odom')
            ]
        )
    ])
