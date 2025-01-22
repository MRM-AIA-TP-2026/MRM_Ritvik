from launch import LaunchDescription
from launch_ros.actions.node import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_params_file = os.path.join(
        get_package_share_directory('my_rover'),  # Replace 'my_rover' with your package name
        'config',
        'nav2_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[nav2_params_file],
            output='screen'
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[nav2_params_file],
            output='screen'
        ),
        Node(
            package='nav2_behavior_tree',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[nav2_params_file],
            output='screen'
        )
    ])

