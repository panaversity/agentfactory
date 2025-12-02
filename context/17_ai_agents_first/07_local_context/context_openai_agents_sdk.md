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
    OpenAIChatCompletionsModel
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
from agents import set_tracing_disabled
set_tracing_disabled(True)
```

# Context Management

Context available locally to your code: this is data and dependencies you might need when tool functions run, during callbacks like on_handoff, in lifecycle hooks, etc.
- You create any Python object you want. A common pattern is to use a dataclass or a Pydantic object.
- You pass that object to the various run methods (e.g. Runner.run(..., **context=whatever**)).
- All your tool calls, lifecycle hooks etc will be passed a wrapper object, - RunContextWrapper[T], where T represents your context object type which you can access via wrapper.context.
- The most important thing to be aware of: every agent, tool function, lifecycle etc for a given agent run must use the same type of context.

You can use the context for things like:
- Contextual data for your run (e.g. things like a username/uid or other information about the user)
- Dependencies (e.g. logger objects, data fetchers, etc)
- Helper functions

[Learning Reference](https://openai.github.io/openai-agents-python/context/)

# Local Context

```python
import asyncio
from dataclasses import dataclass

from agents import Agent, RunContextWrapper, Runner, function_tool

@dataclass
class UserInfo1:
    name: str
    uid: int
    location: str = "Pakistan"

@function_tool
async def fetch_user_age(wrapper: RunContextWrapper[UserInfo1]) -> str:
    '''Returns the age of the user.'''
    return f"User {wrapper.context.name} is 30 years old"

@function_tool
async def fetch_user_location(wrapper: RunContextWrapper[UserInfo1]) -> str:
    '''Returns the location of the user.'''
    return f"User {wrapper.context.name} is from {wrapper.context.location}"

async def main():
    user_info = UserInfo1(name="Muhammad Qasim", uid=123)

    agent = Agent[UserInfo1](
        name="Assistant",
        tools=[fetch_user_age,fetch_user_location],
        model=model
    )

    result = await Runner.run(
        starting_agent=agent,
        input="What is the age of the user? current location of his/her?",
        context=user_info,
    )

    print(result.final_output)
    # The user John is 47 years old.

if __name__ == "__main__":
    asyncio.run(main())
```
