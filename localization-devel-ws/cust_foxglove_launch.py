import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[{
                "topic_whitelist": ["^(?!/rosout$).*"]  # Exclude /rosout
            }]
        )
    ])
