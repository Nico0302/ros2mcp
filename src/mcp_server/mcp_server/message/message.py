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
from mcp_server.message.empty import EmptyAdapter
from mcp_server.message.text import TextAdapter
from mcp_server.message.image import ImageAdapter
from mcp_server.message.content import ContentAdapter
from mcp.server.lowlevel.helper_types import ReadResourceContents
import mcp.types as types
from sensor_msgs.msg import Image


class MessageAdapter(ContentAdapter):
    """
    Base class for messages in the MCP server.
    This class is used to adapt messages to the MCP server's requirements.
    """

    def __init__(self, message: Any) -> None:
        """
        Initializes the MessageAdapter with a message.

        Args:
            message: The message to adapt.
        """
        self.message = message

    def _content(self) -> ContentAdapter:
        if self.message is None:
            return EmptyAdapter()

        if isinstance(self.message, Image):
            return ImageAdapter(self.message)

        return TextAdapter(self.message)

    def get_resource_contents(self) -> list[ReadResourceContents]:
        """
        Converts the message to a list of ReadResourceContents objects.

        Returns:
            list[ReadResourceContents]: An object containing the text content of the message and its MIME type.
        """
        return self._content().get_resource_contents()

    def get_content_blocks(self) -> list[types.ContentBlock]:
        """
        Converts the message to a list of ContentBlock objects.

        Returns:
           list[types.ContentBlock]: An object containing the text content of the message.
        """
        return self._content().get_content_blocks()
