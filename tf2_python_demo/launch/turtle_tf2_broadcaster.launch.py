from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='tf2_python_demo',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),
        Node(
            package='tf2_python_demo',
            executable='turtle_tf2_listener',
            name='listener',
            parameters=[
                {'target_frame': 'turtle1'}
            ]
        ),
    ])