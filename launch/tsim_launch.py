from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tsim',
            namespace='tsim1',
            executable='commander',
            name='commander'
        ),
        Node(
            package='tsim',
            namespace='tsim1',
            executable='motors',
            name='motors'
        ),
        Node(
            package='tsim',
            namespace='tsim1',
            executable='locator',
            name='locator'
        ),
        Node(
            package='tsim',
            namespace='tsim1',
            executable='requester',
            name='requester',
            remappings = [
                ('keys', '/tsim1/keys'),
                ('get_position', '/tsim1/get_position'),
                ('reset', '/tsim1/reset')
            ]
        )
    ])