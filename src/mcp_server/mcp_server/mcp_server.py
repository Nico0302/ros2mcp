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

import contextlib
import threading
from typing import AsyncIterator, Iterable, Sequence
from pydantic import AnyUrl
from rclpy.executors import MultiThreadedExecutor
import rclpy
from rclpy.node import Node
import mcp.types as types
from mcp.server.lowlevel import Server
from mcp.server.streamable_http_manager import StreamableHTTPSessionManager
from mcp.server.lowlevel.helper_types import ReadResourceContents
from rcl_interfaces.msg import ParameterDescriptor
from mcp_server.subject.resource import ListResourcesTool, ReadResourceTool, Resource
from mcp_server.subject.tool import Tool
from mcp_server.filter import EntityFilter
import asyncio
from starlette.applications import Starlette
from starlette.routing import Mount
from starlette.types import Receive, Scope, Send
import uvicorn
from mcp_server.message import MessageAdapter
from mcp_server.subject.service import Service
from mcp_server.subject.topic import Topic

server = Server("ros2mcp")


class MCPNode(Node):
    """
    A ROS 2 node that provides a service to call tools and a service to get tool descriptions.
    """

    tools: dict[str, Tool]
    resources: dict[str, Resource]

    def __init__(self):
        super().__init__("mcp_server")

        self.tools = {}
        self.resources = {}

        self.declare_parameter(
            "mcp_port",
            "8080",
            ParameterDescriptor(
                description="The port on which the MCP server will listen."
            ),
        )

        self.declare_parameter(
            "included_resource_topics",
            ["/*"],
            ParameterDescriptor(
                description='List of topic names to include in the resource discovery. Use "*" as a wildcard.'
            ),
        )
        self.declare_parameter(
            "excluded_resource_topics",
            ["/"],
            ParameterDescriptor(
                description='List of topic names to exclude from the resource discovery. Use "*" as a wildcard.'
            ),
        )
        self.declare_parameter(
            "included_tool_services",
            ["/*"],
            ParameterDescriptor(
                description='List of service names to include in the tool discovery. Use "*" as a wildcard.'
            ),
        )
        self.declare_parameter(
            "excluded_tool_services",
            ["*parameter*"],
            ParameterDescriptor(
                description='List of service names to exclude from the tool discovery. Use "*" as a wildcard.'
            ),
        )
        self.declare_parameter(
            "included_tool_topics",
            ["/*"],
            ParameterDescriptor(
                description='List of topic names to include in the tool discovery. Use "*" as a wildcard.'
            ),
        )
        self.declare_parameter(
            "excluded_tool_topics",
            ["*parameter*"],
            ParameterDescriptor(
                description='List of topic names to exclude from the tool discovery. Use "*" as a wildcard.'
            ),
        )
        self.declare_parameter(
            "enable_resource_tools",
            False,
            ParameterDescriptor(
                description="Enables the list_resources and read_resource tools."
            ),
        )

    def get_mcp_config(self) -> dict:
        """Get the MCP server configuration."""
        return {
            "port": self.get_parameter("mcp_port").get_parameter_value().string_value,
        }

    async def read_resource(self, uri: AnyUrl) -> Iterable[ReadResourceContents]:
        resource_result = self.resources[str(uri)].read_resource()
        return MessageAdapter(resource_result).get_resource_contents()

    async def list_resources(self) -> list[types.Resource]:
        included_resource_topics = (
            self.get_parameter("included_resource_topics")
            .get_parameter_value()
            .string_array_value
        )
        excluded_resource_topics = (
            self.get_parameter("excluded_resource_topics")
            .get_parameter_value()
            .string_array_value
        )
        enable_resource_tools = (
            self.get_parameter("enable_resource_tools").get_parameter_value().bool_value
        )

        resource_topic_filter = EntityFilter(
            included_resource_topics,  # type: ignore
            excluded_resource_topics,  # type: ignore
        )

        resources: Sequence[Resource] = []

        if enable_resource_tools:
            # Add the resource tools if enabled
            resources.append(ListResourcesTool(self.resources))  # type: ignore
            resources.append(ReadResourceTool(self.resources))  # type: ignore

        for node_name in self.get_node_names():
            if node_name == self.get_name():
                continue
            resources += resource_topic_filter.apply(Topic.discover(self))  # type: ignore

        for resource in resources:
            # check if resource already setup
            if resource.get_uri() in self.resources:
                continue
            resource.setup_resource()
            self.resources[resource.get_uri()] = resource

        definitions = [
            resource.get_resource_metadata() for resource in self.resources.values()
        ]

        return definitions

    async def list_tools(self) -> list[types.Tool]:
        included_tool_services = (
            self.get_parameter("included_tool_services")
            .get_parameter_value()
            .string_array_value
        )
        excluded_tool_services = (
            self.get_parameter("excluded_tool_services")
            .get_parameter_value()
            .string_array_value
        )
        included_tool_topics = (
            self.get_parameter("included_tool_topics")
            .get_parameter_value()
            .string_array_value
        )
        excluded_tool_topics = (
            self.get_parameter("excluded_tool_topics")
            .get_parameter_value()
            .string_array_value
        )

        tool_topic_filter = EntityFilter(
            included_tool_topics,  # type: ignore
            excluded_tool_topics,  # type: ignore
        )
        tool_service_filter = EntityFilter(
            included_tool_services,  # type: ignore
            excluded_tool_services,  # type: ignore
        )

        tools: Sequence[Tool] = []

        for node_name in self.get_node_names():
            if node_name == self.get_name():
                continue
            tools += tool_topic_filter.apply(Topic.discover(self))  # type: ignore
            tools += tool_service_filter.apply(Service.discover(self))  # type: ignore

        for tool in tools:
            # check if tool already setup
            if tool.get_name() in self.tools:
                continue
            tool.setup_tool()
            self.tools[tool.get_name()] = tool

        definitions = [tool.get_tool_metadata() for tool in self.tools.values()]

        return definitions

    async def call_tool(self, name: str, arguments: dict):
        self.get_logger().info(f"Calling tool {name} with arguments {arguments}")
        call_result = None
        try:
            call_result = await self._call_async(name, arguments)
            self.get_logger().info(f"Call result: {call_result}")
        except Exception as e:
            self.get_logger().error(f"Error calling tool {name}: {e}")
            return [types.TextContent(type="text", text=f"Error: {str(e)}")]

        return MessageAdapter(call_result).get_content_blocks()

    async def _call_async(self, uri: str, parameters: dict):
        result = self.tools[uri].call_tool(parameters)
        if result is not None:
            return await self._wait_for_response(result)
        return None

    async def _wait_for_response(self, future):
        """Wait for response asynchronously without blocking the event loop."""
        while rclpy.ok() and not future.done():  # type: ignore
            # [TODO] https://github.com/ros2/rclpy/pull/1399
            await asyncio.sleep(0.01)  # Small delay to prevent busy waiting

        if future.done():
            return future.result()
        return None


def main(args=None):
    rclpy.init()

    node = MCPNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Update the MCP server handlers to use the node instance
    @server.list_tools()
    async def handle_list_tools() -> list[types.Tool]:
        """List available tools."""
        return await node.list_tools()

    @server.call_tool()
    async def handle_call_tool(name: str, arguments: dict | None):
        """Call a tool with given arguments."""
        if arguments is None:
            arguments = {}
        return await node.call_tool(name, arguments)

    @server.list_resources()
    async def handle_list_resources() -> list[types.Resource]:
        return await node.list_resources()

    @server.read_resource()
    async def handle_read_resource(uri: AnyUrl) -> Iterable[ReadResourceContents]:
        return await node.read_resource(uri)

    # Create the session manager with true stateless mode
    session_manager = StreamableHTTPSessionManager(
        app=server,
        event_store=None,
        json_response=True,
        stateless=True,
    )

    async def handle_streamable_http(
        scope: Scope, receive: Receive, send: Send
    ) -> None:
        await session_manager.handle_request(scope, receive, send)

    @contextlib.asynccontextmanager
    async def lifespan(app: Starlette) -> AsyncIterator[None]:
        """Context manager for session manager."""
        async with session_manager.run():
            try:
                yield
            finally:
                rclpy.shutdown()

    # Create an ASGI application using the transport
    starlette_app = Starlette(
        debug=True,
        routes=[
            Mount("/", app=handle_streamable_http),
        ],
        lifespan=lifespan,
    )

    config = node.get_mcp_config()
    port = config.get("port", "8080")

    uvicorn.run(starlette_app, host="localhost", port=int(port))


if __name__ == "__main__":
    main()
