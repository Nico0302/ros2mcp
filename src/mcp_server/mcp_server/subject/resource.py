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
from typing import Any
import mcp.types as types
from mcp_server.subject.tool import Tool
from mcp_server.subject.subject import Subject
from rclpy.task import Future


ROS2_SCHEMA = "ros2"


class Resource(ABC):
    def setup_resource(self) -> None:
        """
        Setup the resource for use in the context of the given node.
        (Not every resource needs to implement this method)

        Args:
            node (Node): The ROS 2 node in which the resource will be used.
        """
        pass

    @abstractmethod
    def read_resource(self) -> Any | None:
        """
        Get the resource definitions.

        Returns:
            Iterable[ReadResourceContents]: An iterable of ReadResourceContents objects containing the resource data.
        """
        pass

    @abstractmethod
    def get_resource_metadata(self) -> types.Resource:
        """
        Get the resource definition.

        Returns:
            types.Resource: A resource definition.
        """
        pass

    @abstractmethod
    def get_uri(self) -> str:
        """
        Returns the resource URI for the entity.

        Returns:
            str: URI for the resource.
        """
        pass


class ResourceAdapter(Subject, Resource, ABC):
    """
    Base class for resources in the MCP server.
    Resources are entities that can be discovered and interacted with.
    """

    def get_uri(self) -> str:
        """
        Returns a list of URIs for the entity.
        The URIs are formatted as 'ros2://namespace/name'.

        Returns:
            str: A list of URIs for the resource.
        """
        path = self.name.replace(" ", "_")
        if self.namespace:
            path = f"{self.namespace}/{path}"
        return f"{ROS2_SCHEMA}://{path}"


class ListResourcesTool(Tool):
    """
    Provides a tool for listing resources in the MCP server.

    This can be used for MCP clients that do not support resources natively.
    """

    def __init__(self, resources: dict[str, Resource]) -> None:
        """
        Initialize the ListResourcesTool.
        """
        self.resources = resources

    def get_name(self) -> str:
        """
        Returns a list of names for the entity.
        The name is formatted as 'list_resources'.

        Returns:
            str: A list of names for the tool.
        """
        return "list_resources"

    def get_tool_metadata(self) -> types.Tool:
        """
        Get the tool definition.
        """
        return types.Tool(
            name=self.get_name(),
            inputSchema={},
            outputSchema={
                "type": "array",
                "items": {
                    "type": "object",
                    "properties": {
                        "uri": {"type": "string", "format": "uri"},
                        "name": {"type": "string"},
                        "description": {"type": "string"},
                    },
                    "required": ["uri", "name"],
                },
            },
            description="Lists all resources (ROS topics) available in the MCP server.",
        )

    def call_tool(self, values: dict) -> Future:
        """
        Call the tool with the given values.
        This tool does not require any input values.
        Args:
            values (dict): Input values for the tool (not used in this case).
        """

        resource_list = [resource.get_uri() for resource in self.resources.values()]

        # wrap the resource list in a Future
        future = Future()
        future.set_result(resource_list)
        return future


class ReadResourceTool(Tool):
    """
    Provides a tool for reading a specific resource in the MCP server.

    This can be used for MCP clients that do not support resources natively.
    """

    def __init__(self, resources: dict[str, Resource]) -> None:
        """
        Initialize the ReadResourceTool.
        """
        self.resources = resources

    def get_name(self) -> str:
        """
        Returns a list of names for the entity.
        The name is formatted as 'read_resource'.

        Returns:
            str: A list of names for the tool.
        """
        return "read_resource"

    def get_tool_metadata(self) -> types.Tool:
        """
        Get the tool definition.
        """
        return types.Tool(
            name=self.get_name(),
            inputSchema={
                "type": "object",
                "properties": {
                    "uri": {"type": "string", "format": "uri"},
                },
                "required": ["uri"],
            },
        )

    def call_tool(self, values: dict) -> Future:
        """
        Call the tool with the given values.
        Args:
            values (dict): Input values for the tool (not used in this case).
        """
        uri = values.get("uri")
        if not uri:
            raise ValueError("The 'uri' key is required in the input values.")

        resource = self.resources.get(uri)
        if not resource:
            raise ValueError(f"No resource found for URI: {uri}")

        # Read the resource data
        data = resource.read_resource()

        # wrap the data in a Future
        future = Future()
        future.set_result(data)
        return future
