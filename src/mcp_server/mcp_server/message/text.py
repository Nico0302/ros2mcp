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

from typing import Any
from mcp_server.message.content import ContentAdapter
from mcp.server.lowlevel.helper_types import ReadResourceContents
from rosidl_runtime_py import message_to_yaml
import mcp.types as types


class TextAdapter(ContentAdapter):
    """
    Text message adapter for the MCP server.
    This class is used to adapt text messages to the MCP server's requirements.
    """

    def __init__(self, message: Any) -> None:
        self.message = message

    def _content(self) -> str:
        """
        Returns the text content of the message.

        Returns:
            str: The text content of the message.
        """
        if self.message is None:
            return ""

        if isinstance(self.message, str):
            return self.message

        if isinstance(self.message, (int, float, complex)):
            return str(self.message)

        return message_to_yaml(self.message)

    def get_resource_contents(self) -> list[ReadResourceContents]:
        """
        Converts the image to a ReadResourceContents object with base64 encoded content.

        Returns:
            list[ReadResourceContents]: An object containing the text content of the message and its MIME type.
        """
        return [
            ReadResourceContents(
                content=self._content(),
                mime_type="text/plain",
            )
        ]

    def get_content_blocks(self) -> list[types.ContentBlock]:
        """
        Converts the image to a ContentBlock object.

        Returns:
           list[types.ContentBlock]: An object containing the text content of the message.
        """
        return [
            types.TextContent(
                type="text",
                text=self._content(),
            )
        ]
