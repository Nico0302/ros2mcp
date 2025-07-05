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
from mcp.server.lowlevel.helper_types import ReadResourceContents
import mcp.types as types


class ContentAdapter(ABC):
    """
    Base class for content in the MCP server.
    Can either be a tool call result or a resource.
    """

    @abstractmethod
    def get_resource_contents(self) -> list[ReadResourceContents]:
        """
        Get the content of the resource.

        Returns:
            list[ReadResourceContents]: The content of the resource.
        """
        raise NotImplementedError("This method should be implemented by subclasses.")

    @abstractmethod
    def get_content_blocks(self) -> list[types.ContentBlock]:
        """
        Get the content blocks of the tool.

        Returns:
            list[types.ContentBlock]: The content blocks of the tool.
        """
        raise NotImplementedError("This method should be implemented by subclasses.")
