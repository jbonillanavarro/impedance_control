from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    pd_controller_node = Node(
        package='impedance_control',
        executable='pd_controller_node',
        name='pd_controller_node',
        output='screen',
        parameters=[{
            'kp': 100.0,
            'kd': 20.0,
            'qd': [1.0, 1.0]  # Posición articular deseada [q1, q2]
        }]
    )
    
    return LaunchDescription([pd_controller_node])