### Install openai-agents SDK

```python
!pip install -Uq openai-agents
```

### Make your Jupyter Notebook capable of running asynchronous functions.

```python
import nest_asyncio
nest_asyncio.apply()
```

### Get your LLM API Key

```python
from google.colab import userdata
gemini_api_key = userdata.get("GEMINI_API_KEY")
```

## How to configure LLM Providers at different levels (Global, Run and Agent)?

Agents SDK is setup to use OpenAI as default providers. When using other providers you can setup at different levels:
1. Agent Level
2. RUN LEVEL
3. Global Level

We will always your Agent Level Configuration so each agent can use the LLM best fit for it.

### 1. AGENT LEVEL

```python
import asyncio
from openai import AsyncOpenAI
from agents import Agent, OpenAIChatCompletionsModel, Runner


#Reference: https://ai.google.dev/gemini-api/docs/openai
client = AsyncOpenAI(
    api_key=gemini_api_key,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
)

async def main():
    # This agent will use the custom LLM provider
    agent = Agent(
        name="Assistant",
        instructions="You only respond in urdu.",
        model=OpenAIChatCompletionsModel(model="gemini-2.5-flash", openai_client=client),
    )

    result = await Runner.run(
        agent,
        "I am learning Agentic AI with Panaversity Community",
    )
    print(result.final_output)


if __name__ == "__main__":
    asyncio.run(main())
```

### 2. RUN LEVEL

```python
from agents import Agent, Runner, AsyncOpenAI, OpenAIChatCompletionsModel
from agents.run import RunConfig

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

agent: Agent = Agent(name="Assistant", instructions="You are a helpful assistant")

result = Runner.run_sync(agent, "Hello, how are you.", run_config=config)

print(result.final_output)
```

### GLOBAL

```python
from agents import Agent, Runner, AsyncOpenAI, set_default_openai_client, set_tracing_disabled, set_default_openai_api

set_tracing_disabled(True)
set_default_openai_api("chat_completions")

external_client = AsyncOpenAI(
    api_key=gemini_api_key,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
)
set_default_openai_client(external_client)

agent: Agent = Agent(name="Assistant", instructions="You are a helpful assistant", model="gemini-2.0-flash")

result = Runner.run_sync(agent, "Hello")

print(result.final_output)
```

### Set debug mode on (Optional)

```python
from agents import enable_verbose_stdout_logging

enable_verbose_stdout_logging()
```

> This is for Debugging and looking what happens inside of Agent SDK

```python
result = Runner.run_sync(agent, "Hello")
```
