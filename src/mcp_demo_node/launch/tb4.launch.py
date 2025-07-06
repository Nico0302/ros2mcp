from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            # IncludeLaunchDescription(
            #     PathJoinSubstitution(
            #         [
            #             FindPackageShare("nav2_bringup"),
            #             "launch",
            #             "tb4_simulation_launch.py",
            #         ]
            #     ),
            #     launch_arguments={"headless": "False"}.items(),
            # ),
            Node(
                package="mcp_demo_node",
                executable="node",
                name="waypoints",
                output="screen",
                parameters=[
                    {"waypoint_storage_file": "./assets/waypoints.json"},
                ],
            ),
            Node(
                package="mcp_server",
                executable="mcp_server",
                name="mcp_server",
                output="screen",
                parameters=[
                    {
                        "included_resource_topics": [
                            "*/waypoints",
                            "/rgbd_camera/image",
                        ],
                        "included_tool_services": [
                            "*/navigate_to_waypoint",
                            "*/create_waypoint",
                        ],
                        "included_tool_topics": ["/cmd_vel"],
                    },
                ],
            ),
        ]
    )
