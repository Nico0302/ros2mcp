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
from mcp.types import Tool as MCPTool
from rclpy.task import Future
from rclpy.node import Node
from rosidl_runtime_py import set_message_fields
from rclpy.topic_or_service_is_hidden import topic_or_service_is_hidden
from mcp_server.subject.subject import Subject
from mcp_server.subject.schema import JSONSchema
from mcp_server.subject.tool import ToolAdapter
from rosidl_adapter import parser
import importlib
import mcp.types as types


class Service(ToolAdapter):
    """
    ROS services as an LLM tool
    """

    def __init__(self, name: str, type_name: str, namespace: str = "") -> None:
        super().__init__(name, type_name, namespace)

        self.srv_module = self._get_srv_module()
        self.request = self.srv_module.Request()
        self.json_schema = JSONSchema()

    def call_tool(self, node: Node, uri: str, values: dict) -> Future:
        node.get_logger().info(f"Calling service {self.name} of type {self.type_name}")

        cli = node.create_client(self.srv_module, self.name)

        try:
            set_message_fields(self.request, values)
        except Exception as e:
            raise RuntimeError("Failed to populate field: {0}".format(e))

        while not cli.wait_for_service(timeout_sec=1.0):
            node.get_logger().info("service not available, waiting again...")

        return cli.call_async(self.request)

    @staticmethod
    def discover(node: Node) -> List[Subject]:
        tools = []
        services = []
        try:
            services = node.get_service_names_and_types()
        except Exception as _:
            pass
        for [service_name, type_names] in services:
            if not isinstance(type_names, list):
                type_names = [type_names]
            if topic_or_service_is_hidden(service_name):
                continue
            for service_type in type_names:
                tools.append(Service(service_name, service_type))
        return tools

    def _get_srv_module(self):
        srv_module = None
        try:
            parts = self.type_name.split("/")
            if len(parts) == 2:
                parts = [parts[0], "srv", parts[1]]
            module = importlib.import_module(".".join(parts[:-1]))
            srv_name = parts[-1]
            srv_module = getattr(module, srv_name)
        except (AttributeError, ModuleNotFoundError, ValueError):
            raise RuntimeError("The passed service type is invalid")
        try:
            srv_module.Request
            srv_module.Response
        except AttributeError:
            raise RuntimeError("The passed type is not a service")
        return srv_module

    def list_tools(self) -> List[MCPTool]:
        spec = parser.parse_service_file(
            *self.json_schema.get_interface_path(self.type_name)
        )
        inputSchema = self.json_schema.convert_message(spec.request, False)

        desc = self.json_schema.get_description(spec.request)

        return [
            types.Tool(
                name=self.get_names()[0],
                inputSchema=inputSchema,
                description=desc["description"]
                if "description" in desc
                else "Call ROS2 service",
            )
        ]
