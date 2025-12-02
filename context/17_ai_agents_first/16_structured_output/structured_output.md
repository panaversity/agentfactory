# **Output Types: Structured Output with Pydantic**

Structured outputs are crucial when you need your AI agents to provide data in a predictable and usable format. Instead of free-form text, you can enforce a specific structure, making it easier for your applications to parse and utilize the information. The OpenAI Agents SDK provides mechanisms to achieve this.

**Why Use Structured Outputs?**

**Data Parsing**: Easier to extract specific information without complex text parsing.

**Integration**: Seamless integration with other systems that expect structured data (e.g., databases, APIs).

**Reliability**: Reduces ambiguity and ensures consistent data formatting.

**Automation**: Automate workflows that rely on specific data points.

**Structured Outputs vs. JSON Mode:**

It's important to differentiate between "Structured Outputs" and the simpler "JSON mode."
"JSON mode" ensures that the output is valid JSON, but it doesn't guarantee that it conforms to a specific schema.
"Structured Outputs" goes further, **guaranteeing schema adherence**. This is a very important distinction.

# Install openai-agents SDK

```python
!pip install -Uq openai-agents pydantic
```

# Make your Notebook capable of running asynchronous functions.
Both Jupyter notebooks and Python's asyncio library utilize event loops, but they serve different purposes and can sometimes interfere with each other.

The nest_asyncio library allows the existing event loop to accept nested event loops, enabling asyncio code to run within environments that already have an event loop, such as Jupyter notebooks.

In summary, both Jupyter notebooks and Python's asyncio library utilize event loops to manage asynchronous operations. When working within Jupyter notebooks, it's essential to be aware of the existing event loop to effectively run asyncio code without conflicts.

```python
import nest_asyncio
nest_asyncio.apply()
```

```python
from pydantic import BaseModel
from agents import Agent, Runner, AsyncOpenAI, OpenAIChatCompletionsModel, RunConfig
from google.colab import userdata
```

```python
gemini_api_key = userdata.get("GEMINI_API_KEY")


# Check if the API key is present; if not, raise an error
if not gemini_api_key:
    raise ValueError("GEMINI_API_KEY is not set. Please ensure it is defined in your .env file.")
```

```python
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

```python
class WeatherAnswer(BaseModel):
  location: str
  temperature_c: float
  summary: str
```

```python
agent = Agent(
  name="StructuredWeatherAgent",
  instructions="Use the final_output tool with WeatherAnswer schema.",
  output_type=WeatherAnswer
)
```

```python
out = await Runner.run(agent, "What's the temperature in Karachi?", run_config=config)
print(type(out.final_output))
# <class '__main__.WeatherAnswer'>
print(out.final_output.temperature_c)
# e.g. 22.0
```
