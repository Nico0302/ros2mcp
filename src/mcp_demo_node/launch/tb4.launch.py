from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [
                        FindPackageShare("nav2_bringup"),
                        "launch",
                        "tb4_simulation_launch.py",
                    ]
                ),
                launch_arguments={"headless": "False"}.items(),
            ),
            Node(
                package="mcp_demo_node",
                executable="mcp_demo_node",
                name="demo",
                output="screen",
                parameters=[
                    {"waypoint_storage_file": "/tmp/waypoints.json"},
                ],
            ),
            Node(
                package="mcp_server",
                executable="mcp_server",
                name="mcp_server",
                output="screen",
                parameters=[
                    {
                        "include_resource_topics": ["*/waypoints"],
                        "include_tool_services": [
                            "*/navigate_to_waypoint",
                            "*/create_waypoint",
                        ],
                        "include_tool_topics": ["/"],
                    },
                ],
            ),
        ]
    )
