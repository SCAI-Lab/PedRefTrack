from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = Path(get_package_share_directory("pedreftrack")) / "config" / "pedreftrack.yaml"
    return LaunchDescription(
        [
            Node(
                package="pedreftrack",
                executable="pedreftrack_node",
                name="pedreftrack",
                output="screen",
                parameters=[str(config)],
            )
        ]
    )
