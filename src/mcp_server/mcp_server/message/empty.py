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

from mcp_server.message.content import ContentAdapter
from mcp.server.lowlevel.helper_types import ReadResourceContents
import mcp.types as types


class EmptyAdapter(ContentAdapter):
    """
    An empty adapter that does not provide any content.
    This is useful for cases where an empty response is returned or no content is available.
    """

    def get_resource_contents(self) -> list[ReadResourceContents]:
        """
        Returns an empty list of resource contents.
        """
        return []

    def get_content_blocks(self) -> list[types.ContentBlock]:
        """
        Returns an empty list of content blocks.
        """
        return []
