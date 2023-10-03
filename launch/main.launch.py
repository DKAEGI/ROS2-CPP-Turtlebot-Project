from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_ros2ject',
            executable='follow_wall_node',
            output='screen'),

        Node(
            package='my_ros2ject',
            executable='find_wall_node',
            output='screen'),

        Node(
            package='my_ros2ject',
            executable='odom_record_action_server_node',
            output='screen'),
    ])

