# Copyright 2024 Nicolas Gres
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

from abc import ABC, abstractmethod
from typing import Any, List
import mcp.types as types
from mcp_server.subject.subject import Subject
from rclpy.node import Node


ROS2_SCHEMA = "ros2"


class ResourceAdapter(Subject, ABC):
    """
    Base class for resources in the MCP server.
    Resources are entities that can be discovered and interacted with.
    """

    def setup_resource(self, node: Node) -> None:
        """
        Setup the resource for use in the context of the given node.
        (Not every resource needs to implement this method)

        Args:
            node (Node): The ROS 2 node in which the resource will be used.
        """
        pass

    @abstractmethod
    def read_resource(self) -> Any:
        """
        Get the resource definitions.

        Returns:
            Iterable[ReadResourceContents]: An iterable of ReadResourceContents objects containing the resource data.
        """
        pass

    @abstractmethod
    def list_resources(self) -> List[types.Resource]:
        """
        Get the resource definitions.

        Returns:
            List[types.Resource]: A list of resource definitions.
        """
        pass

    def get_uris(self) -> list[str]:
        """
        Returns a list of URIs for the entity.
        The URIs are formatted as 'ros2://namespace/name'.

        Returns:
            list[str]: A list of URIs for the resource.
        """
        path = self.name.replace(" ", "_")
        if self.namespace:
            path = f"{self.namespace}/{path}"
        return [f"{ROS2_SCHEMA}://{path}"]
