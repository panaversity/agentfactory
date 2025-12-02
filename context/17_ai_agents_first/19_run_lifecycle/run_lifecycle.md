# Install openai-agents SDK

```python
!pip install -Uq openai-agents
```

# Make your Notebook capable of running asynchronous functions.
Both Jupyter notebooks and Python's asyncio library utilize event loops, but they serve different purposes and can sometimes interfere with each other.

The nest_asyncio library allows the existing event loop to accept nested event loops, enabling asyncio code to run within environments that already have an event loop, such as Jupyter notebooks.

In summary, both Jupyter notebooks and Python's asyncio library utilize event loops to manage asynchronous operations. When working within Jupyter notebooks, it's essential to be aware of the existing event loop to effectively run asyncio code without conflicts.

```python
import nest_asyncio
nest_asyncio.apply()
```

# Config

```python
from agents import (
    AsyncOpenAI,
    OpenAIChatCompletionsModel,
)
from google.colab import userdata
```

```python
gemini_api_key = userdata.get("GEMINI_API_KEY")


# Check if the API key is present; if not, raise an error
if not gemini_api_key:
    raise ValueError("GEMINI_API_KEY is not set. Please ensure it is defined in your .env file.")

#Reference: https://ai.google.dev/gemini-api/docs/openai
external_client = AsyncOpenAI(
    api_key=gemini_api_key,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
)

model = OpenAIChatCompletionsModel(
    model="gemini-2.0-flash",
    openai_client=external_client
)
```

```python
from agents import set_default_openai_client, set_tracing_disabled

set_default_openai_client(external_client)
set_tracing_disabled(True)
```

# Learning LifeCycle

```python
# Imports
import asyncio
import random
from typing import Any

from agents import Agent, RunContextWrapper, RunHooks, Runner, Tool, Usage, function_tool
```

### 1. Basic Example (Understand Core Concept)

```python
class TestHooks(RunHooks):
    def __init__(self):
        self.event_counter = 0
        self.name = "TestHooks"

    async def on_agent_start(self, context: RunContextWrapper, agent: Agent) -> None:
        self.event_counter += 1
        print(f"### {self.name} {self.event_counter}: Agent {agent.name} started. Usage: {context.usage}")

    async def on_agent_end(self, context: RunContextWrapper, agent: Agent, output: Any) -> None:
        self.event_counter += 1
        print(f"### {self.name} {self.event_counter}: Agent {agent.name} ended. Usage: {context.usage}, Output: {output}")
```

```python
start_hook = TestHooks()

start_agent = Agent(
    name="Content Moderator Agent",
    instructions="You are content moderation agent. Watch social media content received and flag queries that need help or answer. We will answer anything about AI?",
    model=model
)

async def main():
  result = await Runner.run(
      start_agent,
      hooks=start_hook,
      input=f"<tweet>Will Agentic AI Die at end of 2025?.</tweet>"
  )

  print(result.final_output)

asyncio.run(main())
print("--end--")
```

We can add callbacks on various lifecycle events in an agent run listed here:

https://openai.github.io/openai-agents-python/ref/lifecycle/#agents.lifecycle.RunHooks

### 2. Advanced Example (With Tools and Agents HandOff)

```python
class ExampleHooks(RunHooks):
    def __init__(self):
        self.event_counter = 0

    def _usage_to_str(self, usage: Usage) -> str:
        return f"{usage.requests} requests, {usage.input_tokens} input tokens, {usage.output_tokens} output tokens, {usage.total_tokens} total tokens"

    async def on_agent_start(self, context: RunContextWrapper, agent: Agent) -> None:
        self.event_counter += 1
        print(
            f"### {self.event_counter}: Agent {agent.name} started. Usage: {self._usage_to_str(context.usage)}"
        )

    async def on_agent_end(self, context: RunContextWrapper, agent: Agent, output: Any) -> None:
        self.event_counter += 1
        print(
            f"### {self.event_counter}: Agent {agent.name} ended with output {output}. Usage: {self._usage_to_str(context.usage)}"
        )

    async def on_tool_start(self, context: RunContextWrapper, agent: Agent, tool: Tool) -> None:
        self.event_counter += 1
        print(
            f"### {self.event_counter}: Tool {tool.name} started. Usage: {self._usage_to_str(context.usage)}"
        )

    async def on_tool_end(
        self, context: RunContextWrapper, agent: Agent, tool: Tool, result: str
    ) -> None:
        self.event_counter += 1
        print(
            f"### {self.event_counter}: Tool {tool.name} ended with result {result}. Usage: {self._usage_to_str(context.usage)}"
        )

    async def on_handoff(
        self, context: RunContextWrapper, from_agent: Agent, to_agent: Agent
    ) -> None:
        self.event_counter += 1
        print(
            f"### {self.event_counter}: Handoff from {from_agent.name} to {to_agent.name}. Usage: {self._usage_to_str(context.usage)}"
        )
```

```python
hooks = ExampleHooks()


@function_tool("random_number")
def random_number(max: int) -> int:
    """Generate a random number up to the provided max."""
    return random.randint(0, max)


@function_tool("multiply_by_two")
def multiply_by_two(x: int) -> int:
    """Return x times two."""
    return x * 2


multiply_agent = Agent(
    name="Multiply Agent",
    instructions="Multiply the number by 2 and then return the final result.",
    tools=[multiply_by_two],
    model=model
)

start_agent = Agent(
    name="Start Agent",
    instructions="Generate a random number. If it's even, stop. If it's odd, hand off to the multipler agent.",
    tools=[random_number],
    handoffs=[multiply_agent],
    model=model
)


async def main() -> None:
    user_input = input("Enter a max number: ")
    ans = await Runner.run(
        start_agent,
        hooks=hooks,
        input=f"Generate a random number between 0 and {user_input}.",
    )

    print(ans.final_output)

    print("Done!")

asyncio.run(main())
```
