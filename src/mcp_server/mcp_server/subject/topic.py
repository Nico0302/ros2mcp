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

import importlib
from typing import Any, List
from pydantic import AnyUrl
from rclpy.node import Node
import rclpy.publisher
import rclpy.subscription
from rclpy.topic_or_service_is_hidden import topic_or_service_is_hidden
from rosidl_runtime_py import set_message_fields
from mcp_server.subject.subject import Subject
from mcp_server.subject.resource import ResourceAdapter
from mcp_server.subject.schema import JSONSchema
from mcp_server.subject.tool import ToolAdapter
import mcp.types as types


class Topic(ToolAdapter, ResourceAdapter):
    """
    A subscribable topic to be injected into the LLM context.
    """

    publisher: rclpy.publisher.Publisher | None
    subscription: rclpy.subscription.Subscription | None

    def __init__(self, name: str, type_name: str) -> None:
        self.name = name
        self.type_name = type_name
        self.publisher = None
        self.json_schema = JSONSchema()
        self.value = None  # The last received message value

    def setup_tool(self, node: Node) -> None:
        self.publisher = node.create_publisher(self._get_msg_module(), self.name, 10)

    def setup_resource(self, node: Node) -> None:
        self.subscription = node.create_subscription(
            self._get_msg_module(), self.name, self._listener_callback, 10
        )

    def _listener_callback(self, msg) -> None:
        """
        Callback for the subscription.
        """
        self.value = msg

    def call_tool(self, node, uri: str, values):
        if self.publisher is None:
            raise RuntimeError("The publisher is not initialized")
        if "values" not in values:
            raise RuntimeError("The values key is missing")
        msg_module = self._get_msg_module()
        msg = msg_module()
        try:
            set_message_fields(msg, values["values"])
        except Exception as e:
            raise RuntimeError("Failed to populate field: {0}".format(e))
        times = values.get("times", 1)

        count = 0

        def timer_callback():
            if self.publisher is None:
                return
            nonlocal count
            count += 1
            self.publisher.publish(msg)

        timer_callback()
        if times != 1:
            timer = node.create_timer(1, timer_callback)
            while times == 0 or count < times:
                rclpy.spin_once(node)
            node.destroy_timer(timer)

        return None

    def list_resources(self) -> List[types.Resource]:
        return [
            types.Resource(uri=AnyUrl(self.get_uris()[0]), name=f"Topic: {self.name}")
        ]

    def read_resource(self) -> Any:
        return self.value

    def list_tools(self) -> List[types.Tool]:
        inputSchema = {
            "type": "object",
            "properties": {
                "values": self.json_schema.convert_message(self.type_name),
                "times": {
                    "type": "integer",
                    "minimum": 1,
                    "default": 1,
                    "description": "The number of times to publish the message with 1 Hz.",
                },
            },
            "required": ["values"],
        }

        return [
            types.Tool(
                name=self.get_names()[0],
                description="Publishes a message to a ROS2 topic",
                inputSchema=inputSchema,
            )
        ]

    @staticmethod
    def discover(node) -> List[Subject]:
        tools = []
        topics = []
        try:
            topics = node.get_topic_names_and_types()
        except Exception as e:
            print("Failed to get publisher names and types by node: {0}".format(e))
            return []

        for [topic_name, type_names] in topics:
            if not isinstance(type_names, list):
                type_names = [type_names]
            if topic_or_service_is_hidden(topic_name):
                continue
            for topic_type in type_names:
                tools.append(Topic(topic_name, topic_type))
        return tools

    def _get_msg_module(self):
        msg_module = None
        try:
            parts = self.type_name.split("/")
            if len(parts) == 2:
                parts = [parts[0], "msg", parts[1]]
            module = importlib.import_module(".".join(parts[:-1]))
            msg_name = parts[-1]
            msg_module = getattr(module, msg_name)
        except (AttributeError, ModuleNotFoundError, ValueError):
            raise RuntimeError("The passed message type is invalid")
        return msg_module
