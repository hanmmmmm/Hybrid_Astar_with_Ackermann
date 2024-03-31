from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2'
        ),
        Node(
            package='hawa_ackermann_sim',
            executable='akm_sim_node',
            name='akm_sim'
        ),
        Node(
            package='hawa_control',
            executable='node_purepursuit',
            name='node_purepursuit'
        ),
        Node(
            package='hawa_pathplan',
            executable='map_fusion_node',
            name='map_fusion_node'
        ),
        Node(
            package='hawa_pathplan',
            executable='path_plan_node',
            name='path_plan_node'
        )
        # ,
        # Node(
        #     package='hawa_pathplan',
        #     executable='gridmap_publish.py',
        #     name='gridmap_publish'
        # )
    ])