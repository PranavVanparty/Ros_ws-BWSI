from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone',
            namespace='drone',
            executable='apriltags',
            name='sim'
        ),
        Node(
            package= 'drone',
            namespace= 'drone',
            executable= 'find_center'
        ),
        Node(
            package= 'webcam',
            namespace= 'webcam',
            executable= 'webcam_node'
        )
    ])