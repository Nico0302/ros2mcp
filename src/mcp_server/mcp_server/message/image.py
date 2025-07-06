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

from sensor_msgs.msg import Image
import base64
import numpy as np
import cv2
from mcp.server.lowlevel.helper_types import ReadResourceContents
import mcp.types as types
from mcp_server.message.content import ContentAdapter


class ImageAdapter(ContentAdapter):
    """
    An adapter for sensor_msgs/Image to convert images to MCP compatible formats.
    """

    def __init__(self, message: Image, extension: str = "webp") -> None:
        """
        Initializes the ImageAdapter with a sensor_msgs/Image message and an optional file extension.

        Args:
            message (Image): The sensor_msgs/Image message to adapt.
            extension (str): The file extension for the image format (default is "webp").
        """
        self.message = message
        self.extension = extension

    def _mime_type(self) -> str:
        """
        Returns the MIME type of the image based on its extension.

        Returns:
            str: The MIME type of the image.
        """
        return f"image/{self.extension}"

    def _base64(self) -> bytes:
        """
        Converts the image to a base64 encoded string.

        Returns:
            str: The base64 encoded string of the image.
        Raises:
            ValueError: If the image encoding is not supported.
        """
        encoding = self.message.encoding

        match encoding:
            case "bgr8":
                # OpenCV uses BGR format by default
                image = np.frombuffer(self.message.data, dtype=np.uint8).reshape(
                    (self.message.height, self.message.width, 3)
                )
            case "rgb8":
                # Convert RGB to BGR for OpenCV compatibility
                image_data = np.frombuffer(self.message.data, dtype=np.uint8).reshape(
                    (self.message.height, self.message.width, 3)
                )
                image = cv2.cvtColor(image_data, cv2.COLOR_RGB2BGR)
            case "mono8":
                # Single channel grayscale image
                image = np.frombuffer(self.message.data, dtype=np.uint8).reshape(
                    (self.message.height, self.message.width)
                )
            case _:
                raise ValueError(f"Unsupported encoding: {encoding}")

        _, buffer = cv2.imencode(f".{self.extension}", image)
        return buffer.tobytes()

    def get_resource_contents(self) -> list[ReadResourceContents]:
        """
        Converts the image to a ReadResourceContents object with base64 encoded content.

        Returns:
            ReadResourceContents: An object containing the base64 encoded image content and its MIME type.
        """
        return [
            ReadResourceContents(
                content=self._base64(),
                mime_type=self._mime_type(),
            )
        ]

    def get_content_blocks(self) -> list[types.ContentBlock]:
        """
        Converts the image to a ContentBlock object.

        Returns:
            types.ContentBlock: An object containing the base64 encoded image content and its MIME type.
        """
        return [
            types.ImageContent(
                type="image",
                data=base64.b64encode(self._base64()).decode("utf-8"),
                mimeType=self._mime_type(),
            )
        ]
