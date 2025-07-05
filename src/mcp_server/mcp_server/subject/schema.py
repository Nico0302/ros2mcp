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

from rosidl_adapter import parser
import copy
from rosidl_runtime_py import get_interface_path


class JSONSchema:
    """
    Converts a ROS messages to JSON schema definitions.
    """

    def convert_type(self, type: parser.Type | parser.MessageSpecification):
        """
        Converts a ROS type into a JSON schema type.
        """
        if isinstance(type, parser.MessageSpecification):
            return self.convert_message(type)
        elif type.is_array:
            return self.convert_array_type(type)
        elif type.is_primitive_type():
            return self.convert_primitive_type(type)
        else:
            # load the external message specification
            return self.convert_message((type.pkg_name or "") + "/msg/" + type.type)

    def convert_primitive_type(self, type: parser.Type):
        """
        Converts a ROS primitive type into a JSON schema type.
        """
        match type.type:
            case "bool":
                return {"type": "boolean"}
            case "int8" | "int16" | "int32" | "int64":
                return {"type": "integer"}
            case "uint8" | "uint16" | "uint32" | "uint64":
                return {"type": "integer", "minimum": 0}
            case "float32" | "float64":
                return {"type": "number"}
            case "string" | "wstring":
                return {"type": "string"}
            case "char":
                return {"type": "string", "minLength": 1, "maxLength": 1}
            case _:
                return {"type": "null"}

    def convert_array_type(self, type: parser.Type):
        """
        Converts a ROS array type into a JSON schema type.
        """
        item_type = copy.deepcopy(type)
        item_type.is_array = False
        definiton = {"type": "array", "items": self.convert_type(item_type)}
        if type.is_fixed_size_array():
            definiton["minItems"] = type.array_size
            definiton["maxItems"] = type.array_size
        if type.is_upper_bound:
            definiton["maxItems"] = type.array_size
        return definiton

    def convert_field(self, field: parser.Field):
        """
        Converts a ROS field into a JSON schema field.
        """
        return [
            field.name,
            self.convert_type(field.type)
            | self.get_description(field)
            | (
                {"default": field.default_value}
                if field.default_value is not None
                else {}
            ),
        ]

    def convert_message(
        self, spec: parser.MessageSpecification | str, parseDescription: bool = True
    ):
        """
        Converts a ROS message specification into a JSON schema definition.
        """
        if isinstance(spec, str):
            spec = parser.parse_message_file(*self.get_interface_path(spec))
        properties = {}
        required = []
        for field in spec.fields:
            [name, definiton] = self.convert_field(field)
            properties[name] = definiton
            # required fields need to be explicitly defined
            if definiton.get("default") is None and "required" in definiton.get(
                "description", ""
            ):
                required.append(name)
        return (
            {"type": "object", "properties": properties}
            | (self.get_description(spec) if parseDescription else {})
            | ({"required": required} if len(required) > 0 else {})
        )

    def get_description(self, spec):
        """
        Get the description from the annotations.
        """
        if "comment" in spec.annotations and len(spec.annotations["comment"]) > 0:
            return {"description": "\n".join(spec.annotations["comment"])}
        return {}

    def get_interface_path(self, interface_name: str):
        [package, _, _] = interface_name.rsplit("/")
        return [package, get_interface_path(interface_name)]
