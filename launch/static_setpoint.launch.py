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
                {'x': 1.0},
                {'y': 2.0},
                {'z': 3.0},
                {'yaw': 0.5}
            ],
            remappings=[
                ('se3controller/setpoint', 'geometric_controller/setpoint')  
            ]
        )
    ])
