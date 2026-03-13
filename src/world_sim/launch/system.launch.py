from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='world_sim',
            executable='world_node',
            name='world_node',
            output='screen'
        ),

        Node(
            package='world_sim',
            executable='goal_nav',
            name='goal_nav',
            output='screen'
        ),

        Node(
            package='world_sim',
            executable='human_sim',
            name='human_sim',
            output='screen'
        ),

        Node(
            package='world_sim',
            executable='human_predictor',
            name='human_predictor',
            output='screen'
        ),

        Node(
            package='world_sim',
            executable='metrics_logger',
            name='metrics_logger',
            output='screen'
        )

    ])
