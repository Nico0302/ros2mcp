# mcp_server

This package provides the ROS2 to MCP bridge, allowing MCP to interact with ROS2 services and topics. It includes tools for managing ROS2 services, topics, and messages, enabling seamless integration between MCP and ROS2 systems.

## Architecture

A `Subject` consisting of a `Topic` or `Service` is mapped to an MCP `ToolAdapter` or `ResourceAdapter`.

A `Subject` can also be discovered. The `JSONSchema` allows the conversion of ROS2 IDL messages to [JSON Schema](https://json-schema.org/) tool parameter descriptions.
Each `Subject` can be filtered via a `SubjectFilter` using wildcards `*` and it topic and type name.

The resulting ROS2 messages are converted using the `MessageAdapter` to a list of MCP `ReadResourceContents` or `ContentBlock` items.
