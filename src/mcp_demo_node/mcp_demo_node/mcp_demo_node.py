#!/usr/bin/env python3

# Copyright 2024 Lucas Butler
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
import rclpy
from rclpy.node import Node
import json
from mcp_demo_msgs.srv import CreateWaypoint, Nav2Waypoint
from mcp_demo_msgs.msg import WaypointList
from rcl_interfaces.msg import ParameterDescriptor
from nav2_simple_commander.robot_navigator import BasicNavigator
import os
from mcp_demo_node.waypoint import Waypoint


class McpDemoNode(Node):
    def __init__(self):
        super().__init__("mcp_demo_node")

        self.waypoint_list: list[Waypoint] = []

        self.nav = BasicNavigator()

        self.declare_parameter(
            "waypoint_storage_file",
            "/tmp/waypoints.json",
            ParameterDescriptor(
                description="Path to the JSON file where waypoints are stored."
            ),
        )
        self.waypoint_storage_file = (
            self.get_parameter("waypoint_storage_file")
            .get_parameter_value()
            .string_value
        )

        if os.path.exists(self.waypoint_storage_file):
            self.waypoint_list = self.load_waypoints_from_file()
            self.get_logger().info(f"Found {len(self.waypoint_list)} waypoints.")

        self.waypoints_publisher_ = self.create_publisher(WaypointList, "waypoints", 10)
        self.navigate_to_waypoint_srv = self.create_service(
            Nav2Waypoint, "navigate_to_waypoint", self.navigate_to_waypoint
        )
        self.create_waypoint_srv = self.create_service(
            CreateWaypoint, "create_waypoint", self.create_waypoint
        )

        # For grabbing positional information
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.pose_listener_callback, 10
        )

        # Stores current position of the robot
        self.position = None

        self.get_logger().info("MCP Demo Node started successfully.")

    def _find_waypoint(self, waypoint_shortname):
        for point in self.waypoint_list:
            if point.name == waypoint_shortname:
                return point

        return None

    def navigate_to_waypoint(
        self, request: Nav2Waypoint.Request, response: Nav2Waypoint.Response
    ):
        """
        Searches the waypoint list for the short_name, if found, navigates to that position.
        """
        waypoint = self._find_waypoint(request.name)
        if not waypoint:
            self.get_logger().info("No waypoint found")
            return response

        # Create the target pose and send it to nav2
        pose_stamped = PoseStamped()
        pose_stamped.pose = waypoint.location
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "map"  # Fixed global point, not changing
        self.nav.goToPose(pose_stamped)

        return response

    def publish_waypoints(self):
        """
        Publishes the string of available waypoints and their distances relative to the robot
            to the /waypoints topic.
        """
        if self.position is not None:
            waypoint_list = WaypointList(
                waypoints=[
                    waypoint.to_message(self.position)
                    for waypoint in self.waypoint_list
                ]
            )
            self.waypoints_publisher_.publish(waypoint_list)
            self.get_logger().info('Publishing: "%s"' % waypoint_list)
        else:
            self.get_logger().info(
                "Position has not be published by amcl_pose. Set starting pose in Nav2."
            )

    def pose_listener_callback(self, msg: PoseWithCovarianceStamped):
        """
        Recieves the position of the robot from nav2
        """
        self.get_logger().info(f"Received pose: {msg.pose.pose}")
        self.position = (
            msg.pose.pose
        )  # Traversal PoseWithCovarienceStamped -> PoseWithCovariance -> Pose
        assert isinstance(self.position, Pose)
        self.publish_waypoints()  # Update /waypoints after a new position is posted

    def create_waypoint(
        self, request: CreateWaypoint.Request, response: CreateWaypoint.Response
    ):
        """
        Creates a new waypoint.
        """
        if self.position is not None:
            self.get_logger().info(
                f"Creating new waypoint: {request.name} {self.position}"
            )
            pose = self.position  # type: Pose
            new_waypoint = Waypoint(request.name, request.description, pose)
            self.waypoint_list.append(new_waypoint)
            self.save_waypoints_to_file()
            self.publish_waypoints()
            return response
        else:
            self.get_logger().info("Cannot generate new waypoint, position not set.")
            return response

    def save_waypoints_to_file(self):
        """
        Serialize Waypoints and save them to a JSON file.
        """
        waypoints_dict = [wp.to_dict() for wp in self.waypoint_list]
        with open(self.waypoint_storage_file, "w") as file:
            json.dump(waypoints_dict, file, indent=4)

    def load_waypoints_from_file(self):
        """
        Load Waypoints from a JSON file and deserialize them into Waypoint objects.
        """
        with open(self.waypoint_storage_file, "r") as file:
            waypoints_dict = json.load(file)

        waypoints = []
        for data in waypoints_dict:
            waypoints.append(Waypoint.from_dict(data))

        return waypoints


def main(args=None):
    rclpy.init(args=args)
    node = McpDemoNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
