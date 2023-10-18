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
    return LaunchDescription([
        Node(
            package='mav_controllers_ros',
            executable='circular_trajectory_node',
            name='circular_trajectory_node',
            namespace=LaunchConfiguration('circularsetpoin_ns'),
            output='screen',
            parameters=[
                {'x_center': 0.0},
                {'y_center': 0.0},
                {'z': 2.0},
                {'radius': 1.0},
                {'speed': 2.0},
                {'publish_rate_ms': 10},
                {'max_yaw_rate': 20.0}
            ],
            remappings=[
                ('se3controller/setpoint', 'geometric_controller/setpoint'),  # Replace 'new_topic_name' with your desired topic name
                ('mavros/state', 'mavros/state'),
                ('se3controller/enable_motors', 'geometric_controller/enable_motors'),
                ('mavros/local_position/odom', 'mavros/local_position/odom')
            ]
        )
    ])
