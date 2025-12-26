# OpenAI Agents SDK API Patterns Reference

Detailed API patterns for advanced use cases.

## Agent Configuration

```python
from agents import Agent, ModelSettings

agent = Agent(
    # Required
    name="AgentName",

    # Instructions (static or dynamic)
    instructions="System prompt...",
    # OR
    instructions=lambda ctx, agent: f"Dynamic for {ctx.context.user}",

    # Model selection
    model="gpt-4o",  # Default OpenAI
    # OR use LiteLLM:
    # model=LitellmModel(model="anthropic/claude-3-5-sonnet-20240620"),

    # Model tuning
    model_settings=ModelSettings(
        temperature=0.7,
        top_p=0.9,
        tool_choice="auto",  # or "required", "none", or specific tool
        include_usage=True,
    ),

    # Tools
    tools=[my_function_tool, other_tool],

    # Handoffs
    handoffs=[specialist_agent, escalation_agent],

    # Structured output
    output_type=MyPydanticModel,

    # Guardrails
    input_guardrails=[pii_check, topic_check],
    output_guardrails=[secret_scan],

    # MCP servers
    mcp_servers=[filesystem_server, api_server],

    # Tool behavior
    tool_use_behavior="run_llm_again",  # Default
    # OR "stop_on_first_tool"
    # OR StopAtTools(["specific_tool"])
)
```

## Runner Configuration

```python
from agents import Runner, RunConfig

config = RunConfig(
    # Model override
    model="gpt-4o-mini",

    # Turn limits
    max_turns=10,

    # Tracing
    tracing_disabled=False,
    trace_include_sensitive_data=False,
    workflow_name="MyWorkflow",

    # Handoff behavior
    nest_handoff_history=True,  # Collapse history on handoffs

    # Global guardrails
    input_guardrails=[global_check],
    output_guardrails=[global_output_check],
)

result = await Runner.run(agent, "Input", run_config=config)
```

## Result Types

```python
# Standard result
result = await Runner.run(agent, input)
result.final_output          # The agent's response
result.last_agent           # Final agent that ran
result.new_items            # All generated items
result.raw_responses        # LLM responses
result.to_input_list()      # Convert to input for next turn

# Streaming result
result = await Runner.run_streamed(agent, input)
async for event in result.stream_events():
    if event.type == "raw_response_event":
        # Token-by-token output
        print(event.data, end="")
```

## Tool Patterns

### Return Types

```python
from agents import ToolOutputImage, ToolOutputFileContent

@function_tool
def generate_chart(data: list) -> ToolOutputImage:
    """Generate chart image."""
    image_bytes = create_chart(data)
    return ToolOutputImage(base64_data=base64.b64encode(image_bytes))

@function_tool
def export_data(query: str) -> ToolOutputFileContent:
    """Export data as file."""
    csv_content = run_query(query)
    return ToolOutputFileContent(
        content=csv_content,
        filename="export.csv",
        mime_type="text/csv"
    )
```

### Context Access

```python
from agents import function_tool, RunContextWrapper

@function_tool
async def user_aware_tool(
    ctx: RunContextWrapper,
    query: str
) -> str:
    """Tool with access to run context."""
    user = ctx.context.user_id
    return f"Results for {user}: {query}"
```

### Conditional Tools

```python
@function_tool(
    is_enabled=lambda ctx: ctx.context.is_admin
)
def admin_only_tool(action: str) -> str:
    """Only available to admins."""
    return perform_admin_action(action)
```

## Handoff Patterns

### Input Filtering

```python
from agents import handoff
from agents.extensions import handoff_filters

# Remove all tool calls from history
clean_handoff = handoff(
    agent=specialist,
    input_filter=handoff_filters.remove_all_tools
)

# Custom filter
def my_filter(data: HandoffInputData) -> HandoffInputData:
    # Modify conversation history before handoff
    filtered_history = [
        msg for msg in data.input_history
        if msg.role != "tool"
    ]
    return HandoffInputData(
        input_history=filtered_history,
        pre_handoff_items=data.pre_handoff_items
    )

custom_handoff = handoff(
    agent=specialist,
    input_filter=my_filter
)
```

## Session Patterns

### Manual History Management

```python
# Without session - manual history
result1 = await Runner.run(agent, "Hello")
history = result1.to_input_list()
history.append({"role": "user", "content": "Follow up"})
result2 = await Runner.run(agent, history)
```

### Custom Session Backend

```python
from agents import SessionABC

class RedisSession(SessionABC):
    async def get_items(self) -> list:
        data = await redis.get(self.session_id)
        return json.loads(data) if data else []

    async def add_items(self, items: list) -> None:
        existing = await self.get_items()
        existing.extend(items)
        await redis.set(self.session_id, json.dumps(existing))

    async def pop_item(self):
        items = await self.get_items()
        if items:
            item = items.pop()
            await redis.set(self.session_id, json.dumps(items))
            return item
        return None

    async def clear_session(self) -> None:
        await redis.delete(self.session_id)
```

## MCP Patterns

### Multiple Servers

```python
from agents.mcp import MCPServerStdio, MCPServerStreamableHttp

async with (
    MCPServerStdio(
        name="Filesystem",
        params={"command": "npx", "args": ["server-filesystem", "/"]}
    ) as fs_server,
    MCPServerStreamableHttp(
        name="API",
        params={"url": "https://api.example.com/mcp"}
    ) as api_server,
):
    agent = Agent(
        name="MultiTool",
        mcp_servers=[fs_server, api_server]
    )
```

### Dynamic Tool Filtering

```python
async def context_aware_filter(ctx, tool) -> bool:
    """Filter tools based on agent context."""
    if ctx.agent.name == "ReadOnlyAgent":
        return not tool.name.startswith("write_")
    return True

server = MCPServerStdio(
    params={...},
    tool_filter=context_aware_filter
)
```

## Tracing Patterns

### External Integrations

```python
# Logfire integration
from agents.tracing.logfire_processor import LogfireTracingProcessor
from agents import add_trace_processor

add_trace_processor(LogfireTracingProcessor())

# AgentOps integration
from agents.tracing.agentops_processor import AgentOpsTracingProcessor
add_trace_processor(AgentOpsTracingProcessor())

# Custom processor
from agents import TracingProcessor, Span

class MyProcessor(TracingProcessor):
    def on_span_start(self, span: Span):
        print(f"Started: {span.name}")

    def on_span_end(self, span: Span):
        print(f"Ended: {span.name} ({span.duration_ms}ms)")

add_trace_processor(MyProcessor())
```

## LiteLLM Model Variants

```python
from agents.extensions.models.litellm_model import LitellmModel

# Anthropic
model = LitellmModel(
    model="anthropic/claude-3-5-sonnet-20240620",
    api_key=os.environ["ANTHROPIC_API_KEY"]
)

# Google Gemini
model = LitellmModel(
    model="gemini/gemini-2.5-flash-preview-04-17",
    api_key=os.environ["GOOGLE_API_KEY"]
)

# Azure OpenAI
model = LitellmModel(
    model="azure/gpt-4o",
    api_key=os.environ["AZURE_API_KEY"],
    api_base=os.environ["AZURE_API_BASE"]
)

# AWS Bedrock
model = LitellmModel(
    model="bedrock/anthropic.claude-v2"
    # Uses AWS credentials from environment
)
```

## Error Handling

```python
from agents import (
    AgentsException,
    MaxTurnsExceeded,
    ModelBehaviorError,
    UserError,
    InputGuardrailTripwireTriggered,
    OutputGuardrailTripwireTriggered,
)

try:
    result = await Runner.run(agent, input)
except MaxTurnsExceeded:
    # Agent exceeded turn limit
    print("Agent took too many turns")
except ModelBehaviorError as e:
    # Model returned invalid output
    print(f"Model error: {e}")
except InputGuardrailTripwireTriggered as e:
    # Input blocked by guardrail
    print(f"Input blocked: {e.guardrail_result.output_info}")
except OutputGuardrailTripwireTriggered as e:
    # Output blocked by guardrail
    print(f"Output blocked: {e.guardrail_result.output_info}")
except AgentsException as e:
    # Catch-all for SDK errors
    print(f"Agent error: {e}")
```
