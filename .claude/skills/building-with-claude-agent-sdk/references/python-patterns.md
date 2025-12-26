# Python SDK Patterns

## query() vs ClaudeSDKClient

| Feature | `query()` | `ClaudeSDKClient` |
|---------|-----------|-------------------|
| Session | Creates new each time | Reuses same session |
| Conversation | Single exchange | Multiple exchanges |
| Interrupts | Not supported | Supported |
| Hooks | Not supported | Supported |
| Custom Tools | Not supported | Supported |
| Use Case | One-off tasks | Continuous conversations |

## Complete ClaudeAgentOptions

```python
@dataclass
class ClaudeAgentOptions:
    allowed_tools: list[str] = field(default_factory=list)
    system_prompt: str | SystemPromptPreset | None = None
    mcp_servers: dict[str, McpServerConfig] | str | Path = field(default_factory=dict)
    permission_mode: PermissionMode | None = None
    continue_conversation: bool = False
    resume: str | None = None
    max_turns: int | None = None
    disallowed_tools: list[str] = field(default_factory=list)
    model: str | None = None
    output_format: OutputFormat | None = None
    cwd: str | Path | None = None
    settings: str | None = None
    add_dirs: list[str | Path] = field(default_factory=list)
    env: dict[str, str] = field(default_factory=dict)
    can_use_tool: CanUseTool | None = None
    hooks: dict[HookEvent, list[HookMatcher]] | None = None
    agents: dict[str, AgentDefinition] | None = None
    setting_sources: list[SettingSource] | None = None
    fork_session: bool = False
```

## Message Types

```python
Message = UserMessage | AssistantMessage | SystemMessage | ResultMessage

@dataclass
class ResultMessage:
    subtype: str  # 'success' | 'error_max_turns' | 'error_during_execution'
    duration_ms: int
    is_error: bool
    num_turns: int
    session_id: str
    total_cost_usd: float | None = None
    result: str | None = None
```

## Content Block Types

```python
ContentBlock = TextBlock | ThinkingBlock | ToolUseBlock | ToolResultBlock

@dataclass
class ToolUseBlock:
    id: str
    name: str
    input: dict[str, Any]
```

## Custom Permission Handler

```python
async def can_use_tool(tool: str, input: dict, context: dict):
    if tool == "Bash" and "rm -rf" in str(input.get("command", "")):
        return {
            "behavior": "deny",
            "message": "Dangerous command blocked"
        }

    if tool == "Write" and "config" in input.get("file_path", ""):
        return {
            "behavior": "allow",
            "updatedInput": {
                **input,
                "file_path": f"./sandbox/{input['file_path']}"
            }
        }

    return {"behavior": "allow", "updatedInput": input}
```

## Streaming Input Pattern

```python
async def message_generator():
    yield {
        "type": "user",
        "message": {
            "role": "user",
            "content": "First message"
        }
    }

    await asyncio.sleep(1)

    # With image
    import base64
    with open("diagram.png", "rb") as f:
        image_data = base64.b64encode(f.read()).decode()

    yield {
        "type": "user",
        "message": {
            "role": "user",
            "content": [
                {"type": "text", "text": "Analyze this"},
                {
                    "type": "image",
                    "source": {
                        "type": "base64",
                        "media_type": "image/png",
                        "data": image_data
                    }
                }
            ]
        }
    }

async with ClaudeSDKClient(options) as client:
    await client.query(message_generator())
    async for msg in client.receive_response():
        print(msg)
```

## Complete Hook Example

```python
from claude_agent_sdk import query, ClaudeAgentOptions, HookMatcher, HookContext
from datetime import datetime

async def audit_logger(input_data, tool_use_id, context: HookContext):
    with open("audit.log", "a") as f:
        f.write(f"{datetime.now()}: {input_data['tool_name']}\n")
    return {}

async def security_check(input_data, tool_use_id, context: HookContext):
    if input_data['tool_name'] == 'Bash':
        command = input_data['tool_input'].get('command', '')
        dangerous = ['rm -rf', 'sudo', 'chmod 777', '> /dev/sda']
        if any(d in command for d in dangerous):
            return {
                'hookSpecificOutput': {
                    'hookEventName': 'PreToolUse',
                    'permissionDecision': 'deny',
                    'permissionDecisionReason': f'Dangerous: {command[:50]}'
                }
            }
    return {}

async def sandbox_writes(input_data, tool_use_id, context: HookContext):
    if input_data['tool_name'] in ['Write', 'Edit']:
        file_path = input_data['tool_input'].get('file_path', '')
        return {
            'hookSpecificOutput': {
                'hookEventName': 'PreToolUse',
                'permissionDecision': 'allow',
                'updatedInput': {
                    **input_data['tool_input'],
                    'file_path': f'/sandbox{file_path}'
                }
            }
        }
    return {}

options = ClaudeAgentOptions(
    hooks={
        'PreToolUse': [
            HookMatcher(hooks=[audit_logger]),  # All tools
            HookMatcher(matcher='Bash', hooks=[security_check]),
            HookMatcher(matcher='Write|Edit', hooks=[sandbox_writes])
        ],
        'PostToolUse': [
            HookMatcher(hooks=[audit_logger])
        ]
    }
)
```

## MCP Server Types

```python
# Stdio (local processes)
mcp_servers = {
    "filesystem": {
        "command": "python",
        "args": ["-m", "mcp_server_filesystem"],
        "env": {"ALLOWED_PATHS": "/home/user"}
    }
}

# SSE (remote)
mcp_servers = {
    "remote": {
        "type": "sse",
        "url": "https://api.example.com/mcp/sse",
        "headers": {"Authorization": "Bearer ${API_TOKEN}"}
    }
}

# SDK (in-process)
from claude_agent_sdk import tool, create_sdk_mcp_server

@tool("my_tool", "Description", {"param": str})
async def my_tool(args):
    return {"content": [{"type": "text", "text": "Result"}]}

mcp_servers = {"custom": create_sdk_mcp_server(name="custom", tools=[my_tool])}
```

## Subagent Definition

```python
from claude_agent_sdk import AgentDefinition

agents = {
    "researcher": AgentDefinition(
        description="Deep research and analysis specialist",
        prompt="""You are a research specialist.

        When researching:
        - Search thoroughly using Grep and Glob
        - Read relevant files completely
        - Synthesize findings into clear reports""",
        tools=["Read", "Grep", "Glob", "WebSearch"],
        model="sonnet"
    ),

    "coder": AgentDefinition(
        description="Implementation and coding tasks",
        prompt="You implement code following best practices.",
        tools=["Read", "Edit", "Write", "Bash"]
        # model inherits from parent if not specified
    )
}
```

## Error Types

```python
from claude_agent_sdk import (
    ClaudeSDKError,      # Base error
    CLINotFoundError,    # Claude Code not installed
    CLIConnectionError,  # Connection failed
    ProcessError,        # Process failed (has exit_code, stderr)
    CLIJSONDecodeError   # JSON parsing failed (has line, original_error)
)
```

## Sandbox Configuration

```python
options = ClaudeAgentOptions(
    sandbox={
        "enabled": True,
        "autoAllowBashIfSandboxed": True,
        "excludedCommands": ["docker"],  # Always bypass sandbox
        "allowUnsandboxedCommands": False,
        "network": {
            "allowLocalBinding": True,
            "allowUnixSockets": ["/var/run/docker.sock"]
        }
    }
)
```
