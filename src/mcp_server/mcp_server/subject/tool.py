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
import mcp.types as types
from rclpy.task import Future

from mcp_server.subject.subject import Subject


class Tool(ABC):
    def setup_tool(self) -> None:
        """
        Setup the tool for use in the context of the given node.
        (Not every tool needs to implement this method)
        """
        pass

    @abstractmethod
    def call_tool(self, values: dict) -> Future | None:
        """
        Call the tool with the given values.
        """
        pass

    @abstractmethod
    def get_tool_metadata(self) -> types.Tool:
        """
        Get the tool definition.
        """
        pass

    @abstractmethod
    def get_name(self) -> str:
        """
        Returns a list of names for the entity.
        """


class ToolAdapter(Subject, Tool, ABC):
    def get_name(self) -> str:
        """
        Returns a list of names for the entity.
        """
        name = self.name[1:60].replace("/", "_").replace(" ", "_")
        if self.namespace:
            name = f"{self.namespace}_{name}"
        return name
