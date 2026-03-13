from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='world_sim',
            executable='order_cli',
            name='order_cli',
            output='screen'
        )

    ])
