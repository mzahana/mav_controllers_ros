from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='geometric_controller_ros',  
            executable='static_setpoint_test_node',
            name='static_setpoint_test_node',
            output='screen',
            parameters=[
                {'x': 0.0},
                {'y': 0.0},
                {'z': 2.0},
                {'yaw': 0.0}
            ],
            remappings=[
                ('se3controller/setpoint', 'geometric_controller/setpoint')  
            ]
        )
    ])
