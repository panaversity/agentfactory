```python
!pip install -Uq openai-agents
```

```python
import nest_asyncio
nest_asyncio.apply()
```

```python
from google.colab import userdata
import os

os.environ['OPENAI_API_KEY'] = userdata.get('OPENAI_API_KEY')
```

# Setup vector store on OPENAI dashboard
- create `vector store` in openai panel.
- copy `vector store name`
  - upload pdf file here

```python
from agents import Agent, FileSearchTool, Runner, WebSearchTool

agent = Agent(
    name="Assistant",
    tools=[
        WebSearchTool(),
        FileSearchTool(
            max_num_results=3,
            # vector_store_ids=["VECTOR_STORE_ID"],
            vector_store_ids=["vs_6813268d82a081919782a0990f3a68f9"],

        ),
    ],
)


result =  Runner.run_sync(agent, "Show Muhammad Qasim current organization and job title")
print(result.final_output)
```

# check internet serch tool

```python
result =  Runner.run_sync(agent, "Current Pakistan India News")
print(result.final_output)
```
