# Google ADK API Patterns

## Agent Class Signatures

### Core Agent

```python
from google.adk.agents import Agent

Agent(
    name: str,                    # Required: unique identifier
    model: str,                   # Required: "gemini-2.5-flash", "gemini-2.0-flash"
    instruction: str,             # Required: system prompt
    description: str = None,      # For multi-agent routing
    tools: List = None,           # Function tools, MCP tools, AgentTools
    sub_agents: List = None,      # Sub-agents for delegation
    output_key: str = None,       # Store output in session.state[key]
    before_model_callback = None, # Input guardrail
    before_tool_callback = None,  # Tool guardrail
    code_executor = None,         # BuiltInCodeExecutor for code execution
)
```

### Workflow Agents

```python
from google.adk.agents import SequentialAgent, ParallelAgent, LoopAgent

SequentialAgent(
    name: str,
    description: str,
    sub_agents: List[Agent],      # Executed in order
)

ParallelAgent(
    name: str,
    description: str,
    sub_agents: List[Agent],      # Executed concurrently
)

LoopAgent(
    name: str,
    description: str,
    sub_agents: List[Agent],
    max_iterations: int = 10,     # Stops after N iterations
)
```

## Tool Patterns

### FunctionTool

```python
# ADK auto-wraps functions with docstrings
def my_tool(param: str, count: int = 5) -> dict:
    """Tool description for LLM.

    Args:
        param: Description of param
        count: How many items (default 5)

    Returns:
        Result dictionary
    """
    return {"result": param, "count": count}

# Explicit wrapping
from google.adk.tools import FunctionTool
tool = FunctionTool(func=my_tool)
```

### Tool with Context

```python
from google.adk.tools.tool_context import ToolContext

def stateful_tool(query: str, tool_context: ToolContext) -> dict:
    """Tool with session state access."""
    # Read state
    history = tool_context.state.get("history", [])

    # Modify state
    history.append(query)
    tool_context.state["history"] = history

    return {"history_length": len(history)}
```

### AgentTool

```python
from google.adk.tools.agent_tool import AgentTool

# Wrap agent as callable tool
tool = AgentTool(agent=specialized_agent)

# Use in parent agent
parent = Agent(
    name="parent",
    tools=[AgentTool(child1), AgentTool(child2)]
)
```

### McpToolset

```python
from google.adk.tools.mcp_tool import McpToolset
from google.adk.tools.mcp_tool.mcp_session_manager import StdioConnectionParams
from mcp import StdioServerParameters

# Stdio connection (local MCP server)
toolset = McpToolset(
    connection_params=StdioConnectionParams(
        server_params=StdioServerParameters(
            command='npx',
            args=["-y", "@modelcontextprotocol/server-filesystem", "/path"]
        )
    ),
    tool_filter=['list_directory', 'read_file']  # Optional filter
)

# SSE connection (remote MCP server)
from google.adk.tools.mcp_tool.mcp_session_manager import SseConnectionParams

toolset = McpToolset(
    connection_params=SseConnectionParams(
        url="https://mcp-server.example.com/sse",
        headers={'Authorization': 'Bearer token'}
    )
)
```

## Runner Patterns

### Basic Runner

```python
from google.adk import Runner
from google.adk.sessions import InMemorySessionService

runner = Runner(
    app_name="my_app",
    agent=agent,
    session_service=InMemorySessionService()
)
```

### Full Configuration

```python
from google.adk import Runner
from google.adk.sessions import InMemorySessionService
from google.adk.artifacts import InMemoryArtifactService
from google.adk.memory import InMemoryMemoryService

runner = Runner(
    app_name="my_app",
    agent=agent,
    session_service=InMemorySessionService(),
    artifact_service=InMemoryArtifactService(),
    memory_service=InMemoryMemoryService()
)
```

### Async Execution

```python
from google.genai import types

async def run_agent():
    session = await runner.session_service.create_session(
        app_name="my_app",
        user_id="user123",
        state={"initial": "state"}
    )

    message = types.Content(
        role='user',
        parts=[types.Part(text="Hello!")]
    )

    async for event in runner.run_async(
        user_id=session.user_id,
        session_id=session.id,
        new_message=message
    ):
        if event.content and event.content.parts:
            for part in event.content.parts:
                if part.text:
                    print(part.text)
```

### Quick Debugging

```python
from google.adk.runners import InMemoryRunner

runner = InMemoryRunner(agent=agent)
await runner.run_debug("Test message")
```

## Callback Patterns

### Before Model Callback

```python
from google.adk.agents.callback_context import CallbackContext
from google.adk.models.llm_request import LlmRequest
from google.adk.models.llm_response import LlmResponse
from google.genai import types
from typing import Optional

def input_guardrail(
    callback_context: CallbackContext,
    llm_request: LlmRequest
) -> Optional[LlmResponse]:
    """
    Return LlmResponse to block, None to allow.

    Access:
    - callback_context.agent_name
    - callback_context.state
    - llm_request.contents (list of messages)
    """
    # Check last user message
    for content in reversed(llm_request.contents):
        if content.role == 'user' and content.parts:
            text = content.parts[0].text or ""
            if is_blocked(text):
                return LlmResponse(
                    content=types.Content(
                        role="model",
                        parts=[types.Part(text="Blocked.")]
                    )
                )
    return None
```

### Before Tool Callback

```python
from google.adk.tools.base_tool import BaseTool
from google.adk.tools.tool_context import ToolContext
from typing import Optional, Dict, Any

def tool_guardrail(
    tool: BaseTool,
    args: Dict[str, Any],
    tool_context: ToolContext
) -> Optional[Dict]:
    """
    Return dict to block (becomes tool result), None to allow.

    Access:
    - tool.name
    - args (tool arguments)
    - tool_context.agent_name
    - tool_context.state
    """
    if tool.name == "dangerous_tool":
        return {"error": "Tool blocked"}
    return None
```

## Session Service Patterns

### InMemorySessionService (Development)

```python
from google.adk.sessions import InMemorySessionService

session_service = InMemorySessionService()
```

### FirestoreSessionService (Production)

```python
from google.adk.sessions import FirestoreSessionService

session_service = FirestoreSessionService(
    project="your-project-id",
    database="(default)"
)
```

### VertexAiSessionService (Managed)

```python
from google.adk.sessions import VertexAiSessionService

session_service = VertexAiSessionService(
    project="your-project-id",
    location="us-central1"
)
```

## Evaluation Patterns

### Test File Format

```json
{
  "eval_set_id": "test_set_id",
  "name": "Test Set Name",
  "description": "Description",
  "eval_cases": [
    {
      "eval_id": "case_id",
      "conversation": [
        {
          "invocation_id": "uuid",
          "user_content": {
            "parts": [{"text": "User message"}],
            "role": "user"
          },
          "final_response": {
            "parts": [{"text": "Expected response"}],
            "role": "model"
          },
          "intermediate_data": {
            "tool_uses": [
              {"name": "tool_name", "args": {"key": "value"}}
            ]
          }
        }
      ],
      "session_input": {
        "app_name": "app_name",
        "user_id": "user_id",
        "state": {}
      }
    }
  ]
}
```

### Pytest Integration

```python
from google.adk.evaluation import AgentEvaluator
import pytest

@pytest.mark.asyncio
async def test_agent():
    await AgentEvaluator.evaluate(
        agent_module="my_agent",
        eval_dataset_file_path_or_dir="tests/eval.test.json"
    )
```

## Model Options

| Model ID | Best For | Notes |
|----------|----------|-------|
| `gemini-2.5-flash` | Default, balanced | Fast, capable |
| `gemini-2.5-pro` | Complex reasoning | Higher latency |
| `gemini-2.0-flash` | Multimodal | Image/video support |
| `gemini-1.5-flash` | Legacy compatibility | Older model |
