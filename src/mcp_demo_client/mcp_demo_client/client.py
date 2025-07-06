from functools import reduce
import json
import os
import asyncio
from typing import Iterable, List
from mcp.client.streamable_http import streamablehttp_client
from mcp import ClientSession
from openai import OpenAI
from openai.types.chat.chat_completion_message_param import ChatCompletionMessageParam
from openai.types.chat.chat_completion_tool_param import (
    ChatCompletionToolParam,
)
from openai.types.chat.chat_completion_content_part_param import (
    ChatCompletionContentPartParam,
)
from openai.types.chat.chat_completion_message_tool_call import (
    ChatCompletionMessageToolCall,
)
from mcp.types import TextResourceContents, BlobResourceContents, Tool
from termcolor import colored
from dotenv import load_dotenv

load_dotenv()

MCP_CLIENT_URL = os.getenv("MCP_CLIENT_URL", "http://localhost:8080")
LLM_MODEL = os.getenv("LLM_MODEL", "gpt-4.1-nano-2025-04-14")
MAX_CALL_COUNT = int(os.getenv("MAX_CALL_COUNT", "10"))


class Client:
    def __init__(self, mcp_session: ClientSession):
        self.mcp_session = mcp_session
        self.resources = []
        self.tools: Iterable[ChatCompletionToolParam] = []
        self.history: list[ChatCompletionMessageParam] = [
            {
                "role": "system",
                "content": "You are an embodied AI agent, in control of a ROS 2 robot.",
            }
        ]
        self.client = OpenAI()

    async def initialize(self):
        """
        Initialize the client session and fetch the context prompt.
        This method should be called before using the `ask` method.
        """
        # read all paginated resources from the MCP server
        response = await self.mcp_session.list_resources()
        tools = await self.mcp_session.list_tools()

        self.resources = response.resources
        self.tools = list(map(self._get_tool, tools.tools))

    async def _get_context_prompt(self) -> Iterable[ChatCompletionContentPartParam]:
        """
        Creates a context prompt based on the resources available in the MCP server.
        It combines all resources into a single string that can be used as context for the LLM and attached images.
        """
        text_content = "=== Start of MCP Resources ===\n"
        image_content = []
        for resource in self.resources:
            result = await self.mcp_session.read_resource(resource.uri)
            if len(result.contents) == 0:
                continue
            for content_block in result.contents:
                text_content += f"URI: {resource.uri}\n"
                if isinstance(content_block, TextResourceContents):
                    text_content += content_block.text + "\n"
                elif isinstance(content_block, BlobResourceContents):
                    text_content += f"Blob: {content_block.mimeType}\n"
                    image_content.append(
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:{content_block.mimeType};base64,{content_block.blob}",
                                "detail": "auto",
                            },
                        }
                    )
                text_content += "\n"
        text_content += "=== End of MCP Resources ===\n"
        return [
            {
                "type": "text",
                "text": text_content.strip(),
            },
            *image_content,
        ]

    def _get_tool(self, tool: Tool) -> ChatCompletionToolParam:
        """
        Converts an MCP Tool to a ChatCompletionToolParam.
        """
        return {
            "type": "function",
            "function": {
                "name": tool.name,
                "description": tool.description or "",
                "parameters": tool.inputSchema,
            },
        }

    async def _execute_tool_calls(
        self, tool_calls: List[ChatCompletionMessageToolCall]
    ):
        """
        Executes the tool calls provided by the LLM using the MCP session.
        """
        for tool_call in tool_calls:
            # Call the tool with the arguments provided by the LLM
            args = json.loads(tool_call.function.arguments)
            result = await self.mcp_session.call_tool(tool_call.function.name, args)
            content = reduce(
                # openai does not support multi modal tool call results
                lambda acc, block: acc + block.text if block.type == "text" else acc,
                result.content,
                "",
            )

            # Add the result to the history
            self.history.append(
                {"role": "tool", "tool_call_id": tool_call.id, "content": content}
            )

    async def _create_completion(self, call_count=0):
        """
        Creates a completion using the LLM based on the current history and context.
        """
        messages = self.history
        # Create a copy of the last message and inject context into it
        last_message = messages[-1].copy()
        if last_message["role"] == "user":
            context_content = await self._get_context_prompt()
            # Create new content by combining existing content with context
            existing_content = last_message.get("content", [])
            if isinstance(existing_content, str):
                existing_content = [{"type": "text", "text": existing_content}]
            last_message["content"] = [*context_content, *existing_content]  # type: ignore
            # Create new messages list with the modified last message
            messages = messages[:-1] + [last_message]

        response = self.client.chat.completions.create(
            model=LLM_MODEL,
            messages=messages,
            tools=self.tools,
        )

        response_message = response.choices[0].message
        self.history.append(response_message)  # type: ignore

        call_count += 1

        if call_count > MAX_CALL_COUNT:
            return "I'm sorry, I can't help you with that."
        if response_message.content:
            call_count = 0
            return response_message.content
        if (
            response_message.tool_calls is not None
            and len(response_message.tool_calls) > 0
        ):
            # run the function call
            await self._execute_tool_calls(response_message.tool_calls)
            # pass the result back to the model
            return await self._create_completion(call_count)

    async def ask(self, user_input: str):
        """
        Sends a user input to the LLM and returns the response.
        """
        self.history.append(
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": user_input},
                ],
            }
        )
        response = await self._create_completion()
        return response


async def run():
    async with streamablehttp_client(MCP_CLIENT_URL) as (
        read_stream,
        write_stream,
        _,
    ):
        async with ClientSession(read_stream, write_stream) as session:
            # Initialize the connection between client and server
            await session.initialize()

            client = Client(session)
            await client.initialize()
            user_input = ""
            while user_input != "exit":
                user_input = input(colored("You: ", "green"))
                response = await client.ask(user_input)
                if response is not None:
                    print(colored("AI: ", "cyan"), response)


def main():
    # Start the asyncio event loop and run the main function
    asyncio.run(run())


if __name__ == "__main__":
    main()
