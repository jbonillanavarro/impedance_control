import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('impedance_control'),
        'config',
        'dynamics_params.yaml'
    )

    uma_arm_dynamics_node = Node(
            package='impedance_control',
            executable='uma_arm_dynamics',
            name='uma_arm_dynamics',
            output='screen',
            parameters=[config]
        )

    return LaunchDescription([uma_arm_dynamics_node])