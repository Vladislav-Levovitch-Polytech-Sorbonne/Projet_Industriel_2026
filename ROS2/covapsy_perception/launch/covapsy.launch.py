from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='covapsy_perception',
            executable='lidar_node',
            name='lidar_node',
            output='screen',
        ),

        Node(
            package='covapsy_perception',
            executable='camera_node',
            name='camera_node',
            output='screen',
        ),

        Node(
            package='covapsy_perception',
            executable='control_node',
            name='control_node',
            output='screen',
        ),

        Node(
            package='covapsy_hardware',
            executable='hw_bridge_node',
            name='hw_bridge_node',
            output='screen',
        ),

    ])