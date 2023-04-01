from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_pubsub',
            namespace='cpp_pubsub1',
            executable='talker',
            name='pub'
        ),
        Node(
            package='cpp_pubsub',
            namespace='cpp_pubsub2',
            executable='listener',
            name='sub'
        )
    ])