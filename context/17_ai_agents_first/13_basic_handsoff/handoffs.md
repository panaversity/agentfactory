# Handoffs

Handoffs allow an agent to delegate tasks to another agent. This is particularly useful in scenarios where different agents specialize in distinct areas. For example, a customer support app might have agents that each specifically handle tasks like order status, refunds, FAQs, etc.

Handoffs are represented as tools to the LLM. So if there's a handoff to an agent named Refund Agent, the tool would be called transfer_to_refund_agent.

[Learning Reference](https://openai.github.io/openai-agents-python/handoffs/)

## Install openai-agents SDK

```python
!pip install -Uq openai-agents
```

## Make your Notebook capable of running asynchronous functions.
Both Jupyter notebooks and Python's asyncio library utilize event loops, but they serve different purposes and can sometimes interfere with each other.

The nest_asyncio library allows the existing event loop to accept nested event loops, enabling asyncio code to run within environments that already have an event loop, such as Jupyter notebooks.

In summary, both Jupyter notebooks and Python's asyncio library utilize event loops to manage asynchronous operations. When working within Jupyter notebooks, it's essential to be aware of the existing event loop to effectively run asyncio code without conflicts.

```python
import nest_asyncio
nest_asyncio.apply()
```

## Config

```python
from pydantic import BaseModel
from agents import (
    AsyncOpenAI,
    OpenAIChatCompletionsModel,
    RunConfig
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

config = RunConfig(
    model=model,
    model_provider=external_client,
    tracing_disabled=True
)
```

# Creating a handoff

All agents have a handoffs param, which can either take an Agent directly, or a Handoff object that customizes the Handoff.

```python
import asyncio
from agents import Agent, handoff, Runner
```

### 1. Basic Usage

```python
urdu_agent = Agent(
    name="Urdu agent",
    instructions="You only speak Urdu."
)

english_agent = Agent(
    name="English agent",
    instructions="You only speak English"
)

triage_agent = Agent(
    name="Triage agent",
    instructions="Handoff to the appropriate agent based on the language of the request.",
    handoffs=[urdu_agent, english_agent],
)


async def main(input: str):
    result = await Runner.run(triage_agent, input=input, run_config=config)
    print(result.final_output)
```

```python
asyncio.run(main("السلام عليكم"))
```

```python
asyncio.run(main("Hi"))
```

### 2. Customizing handoffs via the handoff() function

```python
from agents import Agent, handoff, RunContextWrapper

urdu_agent = Agent(
    name="Urdu agent",
    instructions="You only speak Urdu."
)

english_agent = Agent(
    name="English agent",
    instructions="You only speak English"
)

def on_handoff(agent: Agent, ctx: RunContextWrapper[None]):
    agent_name = agent.name
    print("--------------------------------")
    print(f"Handing off to {agent_name}...")
    print("--------------------------------")

triage_agent = Agent(
    name="Triage agent",
    instructions="Handoff to the appropriate agent based on the language of the request.",
    handoffs=[
            handoff(urdu_agent, on_handoff=lambda ctx: on_handoff(urdu_agent, ctx)),
            handoff(english_agent, on_handoff=lambda ctx: on_handoff(english_agent, ctx))
    ],
)


async def main(input: str):
    result = await Runner.run(triage_agent, input=input, run_config=config)
    print(result.final_output)
```

```python
asyncio.run(main("السلام عليكم"))
```

```python
asyncio.run(main("Hello, nice to meet you"))
```

### Self Assignment

Implement the following with HandOffs Pattern:
- Handoff Custom Inputs
- Set an input_filter
- Share info about handoffs in your agents
- Implement Streaming With HandOff

Here are some research references to get started:
- https://openai.github.io/openai-agents-python/handoffs/
- https://github.com/openai/openai-agents-python/blob/main/examples/handoffs/message_filter_streaming.py
- https://github.com/openai/openai-agents-python/blob/main/examples/handoffs/message_filter.py
