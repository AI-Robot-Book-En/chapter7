from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Define the nodes to be launched.
        Node(
            package='pseudo_node_action',
            executable='manipulation_node',
        ),

        Node(
            package='pseudo_node_action',
            executable='navigation_node',
        ),

        Node(
            package='pseudo_node_action',
            executable='vision_node',
        ),

        Node(
            package='pseudo_node_action',
            executable='voice_node',
        )
    ]
)
