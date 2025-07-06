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
from dataclasses import dataclass
from typing import Sequence
from rclpy.node import Node


@dataclass
class Subject(ABC):
    """Base class for ROS entities such as topics, servicers and actions."""

    name: str
    type_name: str
    namespace: str = ""

    @staticmethod
    @abstractmethod
    def discover(node: Node) -> Sequence["Subject"]:
        """
        Discover all entities of this type in the given node.
        This method should be implemented by subclasses to discover specific entities.

        Args:
            node (Node): The ROS 2 node in which to discover entities.
        Returns:
            list[Subject]: A list of discovered entities of this type.
        """
        pass

    def __eq__(self, value: object) -> bool:
        if not isinstance(value, Subject):
            return False
        return (
            self.name == value.name
            and self.type_name == value.type_name
            and self.namespace == value.namespace
        )
