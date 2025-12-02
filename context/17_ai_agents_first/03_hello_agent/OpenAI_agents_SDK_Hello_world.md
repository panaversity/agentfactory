# 🚀 Exploring the openai-agents Library with Gemini
This notebook shows how to use the openai-agents library with Gemini API to build conversational agents—covering two execution methods

# 📦 installation

```python
!pip install -Uq openai-agents
```

# 📦 Imports

```python
import nest_asyncio
nest_asyncio.apply()
```

# 📦 Imports

```python
from agents import Agent, Runner, AsyncOpenAI, OpenAIChatCompletionsModel, set_tracing_disabled
```

# 🔐 Step 1: Setup for Api Keys

```python
from google.colab import userdata
gemini_api_key = userdata.get("GEMINI_API_KEY")
```

# 🌐  Step 2: Client Setup for Connecting to **Gemini**

```python
# Tracing disabled
set_tracing_disabled(disabled=True)

# 1. Which LLM Service?
external_client: AsyncOpenAI = AsyncOpenAI(
    api_key=gemini_api_key,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
)

# 2. Which LLM Model?
llm_model: OpenAIChatCompletionsModel = OpenAIChatCompletionsModel(
    model="gemini-2.5-flash",
    openai_client=external_client
)
```

# 💬  Step 3 Running Agent Synchronously

```python
math_agent: Agent = Agent(name="MathAgent",
                     instructions="You are a helpful math assistant.",
                     model=llm_model) # gemini-2.5 as agent brain - chat completions

result: Runner = Runner.run_sync(math_agent, "why learn math for AI Agents?")

print("\nCALLING AGENT\n")
print(result.final_output)
```

```python
result
```

# 💬 Step 3: Running Agent Asynchronously

```python
import asyncio

async def main():

    result: Runner = await Runner.run(math_agent, "Tell me about recursion in programming.")

    print(result.final_output)


asyncio.run(main())
```

## Example 1:
### 👨‍🍳🍽️ Recipe Bot

```python
set_tracing_disabled(disabled=True)

# Client Setup for Connecting to Gemini
external_client:AsyncOpenAI = AsyncOpenAI(
    api_key=gemini_api_key,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
)

#Initialize model
model:OpenAIChatCompletionsModel = OpenAIChatCompletionsModel(
    model="gemini-2.5-flash",
    openai_client=external_client
)

def main():
  # Create the Recipe Agent
  agent = Agent(
      name="RecipeBot",
      instructions=(
          """You are a helpful recipe assistant. A user will give you a few ingredients
          they have at home, and you will suggest one simple and quick recipe using only those items.
          Keep it short, step-by-step, and easy for beginners to cook."""
      ),
      model=model
  )

  print("\n🍳 What can I cook today?\n")
  ingredients = "eggs, tomatoes, onions, and bread"
  result:Runner = Runner.run_sync(agent, f"I have these at home: {ingredients}. What can I cook?")

  print(result.final_output)
```

```python
if __name__ == "__main__":
    main()
```
