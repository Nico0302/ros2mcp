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

import math
from geometry_msgs.msg import Pose, Point
from mcp_demo_interfaces.msg import Waypoint as WaypointMsg


class Waypoint:
    def __init__(self, name: str, description: str, location: Pose):
        self.name = name
        self.description = description
        self.location = location

    def to_dict(self):
        """
        Converts the waypoint to a string for the LLM to consume.
        """
        return {
            "name": self.name,
            "description": self.description,
            "location": self.pose_to_dict(self.location),
        }

    def to_message(self, position: Pose) -> WaypointMsg:
        """
        Converts the waypoint to a message for the LLM to consume.
        Includes the distance from the current position.
        """
        return WaypointMsg(
            name=self.name,
            description=self.description,
            distance=self.calc_distance(position),
        )

    def calc_distance(self, position: Pose):
        """
        Calculates the euclidean distance from the waypoint to the current location of the robot.
        """
        x = abs(position.position.x - self.location.position.x)
        y = abs(position.position.y - self.location.position.y)
        z = abs(position.position.z - self.location.position.z)

        return round(math.sqrt(x * x + y * y + z * z), 2)

    @staticmethod
    def pose_to_dict(pose: Pose):
        """
        Convert a Pose message to a dictionary.
        """
        return {
            "position": {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
            },
            "orientation": {
                "x": pose.orientation.x,
                "y": pose.orientation.y,
                "z": pose.orientation.z,
                "w": pose.orientation.w,
            },
        }

    @staticmethod
    def from_dict(data):
        """
        Creates a Waypoint object from a dictionary.
        """
        location = Pose()
        location.position.x = data["location"]["position"]["x"]
        location.position.y = data["location"]["position"]["y"]
        location.position.z = data["location"]["position"]["z"]
        location.orientation.x = data["location"]["orientation"]["x"]
        location.orientation.y = data["location"]["orientation"]["y"]
        location.orientation.z = data["location"]["orientation"]["z"]
        location.orientation.w = data["location"]["orientation"]["w"]

        return Waypoint(data["name"], data["description"], location)


def test():
    starting_pose = Pose()
    point = Point()
    point.x = 2.0
    point.y = 3.0
    point.z = 4.0
    starting_pose.position = point
    waypoint = Waypoint("test", "test", starting_pose)

    end_pose = Pose()
    point = Point()
    point.x = 1.0
    point.y = 1.0
    point.z = 1.0
    end_pose.position = point

    print(waypoint.to_dict(end_pose))
