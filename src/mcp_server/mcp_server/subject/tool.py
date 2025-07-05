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

from typing import List
from rclpy.task import Future
from rclpy.node import Node
from abc import ABC, abstractmethod
import mcp.types as types

from mcp_server.subject.subject import Subject


class ToolAdapter(Subject, ABC):
    def setup_tool(self, node: Node) -> None:
        """
        Setup the tool for use in the context of the given node.
        (Not every tool needs to implement this method)
        """
        pass

    @abstractmethod
    def call_tool(self, node: Node, uri: str, values: dict) -> Future | None:
        """
        Call the tool with the given values from a ROS node.
        """
        pass

    @abstractmethod
    def list_tools(self) -> List[types.Tool]:
        """
        Get the tool definitions. One adapter can provide multiple MCP tools
        """
        pass

    def get_names(self) -> list[str]:
        """
        Returns a list of names for the entity.
        """
        name = self.name[1:60].replace("/", "_").replace(" ", "_")
        if self.namespace:
            name = f"{self.namespace}_{name}"
        return [name]
