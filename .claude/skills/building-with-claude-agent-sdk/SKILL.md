---
name: building-with-claude-agent-sdk
description: |
  Use when building AI agents with Anthropic's Claude Agent SDK (formerly Claude Code SDK).
  Triggers: Creating autonomous agents, building agentic applications, SDK-based automation,
  implementing hooks/subagents/MCP servers, session management, or agent tool usage.
  NOT when using Claude API directly (use anthropic-sdk) or building MCP servers from scratch (use mcp-builder).
---

# Claude Agent SDK Development Guide

Build production AI agents that autonomously read files, run commands, search the web, edit code, and more.

## Overview

The Claude Agent SDK gives you the same tools, agent loop, and context management that power Claude Code, programmable in Python and TypeScript. It differs from the Anthropic Client SDK: with the Client SDK you implement the tool loop yourself; with the Agent SDK, Claude handles tool execution autonomously.

## Quick Reference

### Installation

```bash
# Step 1: Install Claude Code (required runtime)
curl -fsSL https://claude.ai/install.sh | bash  # macOS/Linux/WSL
# or: npm install -g @anthropic-ai/claude-code

# Step 2: Install SDK
pip install claude-agent-sdk        # Python
npm install @anthropic-ai/claude-agent-sdk  # TypeScript
```

### Authentication

```bash
export ANTHROPIC_API_KEY=your-api-key
# Alternative providers:
# export CLAUDE_CODE_USE_BEDROCK=1  (Amazon Bedrock)
# export CLAUDE_CODE_USE_VERTEX=1  (Google Vertex AI)
```

### Basic Usage

```python
# Python
import asyncio
from claude_agent_sdk import query, ClaudeAgentOptions

async def main():
    async for message in query(
        prompt="Find and fix the bug in auth.py",
        options=ClaudeAgentOptions(
            allowed_tools=["Read", "Edit", "Bash"],
            permission_mode="acceptEdits"
        )
    ):
        if hasattr(message, "result"):
            print(message.result)

asyncio.run(main())
```

```typescript
// TypeScript
import { query } from "@anthropic-ai/claude-agent-sdk";

for await (const message of query({
  prompt: "Find and fix the bug in auth.py",
  options: {
    allowedTools: ["Read", "Edit", "Bash"],
    permissionMode: "acceptEdits"
  }
})) {
  if ("result" in message) console.log(message.result);
}
```

## Built-in Tools

| Tool | Description |
|------|-------------|
| `Read` | Read any file (text, images, PDFs, notebooks) |
| `Write` | Create new files |
| `Edit` | Make precise edits to existing files |
| `Bash` | Run terminal commands, scripts, git |
| `Glob` | Find files by pattern (`**/*.ts`) |
| `Grep` | Search file contents with regex |
| `WebSearch` | Search the web |
| `WebFetch` | Fetch and parse web pages |
| `Task` | Spawn subagents for parallel work |

## Permission Modes

| Mode | Behavior |
|------|----------|
| `default` | Standard permission checks |
| `acceptEdits` | Auto-approve file edits |
| `bypassPermissions` | No permission prompts (use cautiously) |
| `plan` | Planning mode, no execution |

## Key Configuration Options

```python
ClaudeAgentOptions(
    allowed_tools=["Read", "Edit", "Bash"],  # Tool access
    disallowed_tools=["WebSearch"],           # Block specific tools
    system_prompt="You are a Python expert",  # Custom instructions
    permission_mode="acceptEdits",            # Auto-approve edits
    max_turns=10,                             # Limit iterations
    cwd="/path/to/project",                   # Working directory
    mcp_servers={"db": db_server},            # MCP integrations
    hooks={"PreToolUse": [...]},              # Lifecycle hooks
    agents={"reviewer": agent_def},           # Define subagents
)
```

## Core Patterns

### Pattern 1: Streaming vs Single-Turn

**Streaming (recommended)**: Persistent session, supports images, hooks, interrupts:

```python
async def message_generator():
    yield {"type": "user", "message": {"role": "user", "content": "Analyze this"}}

async for msg in query(prompt=message_generator(), options=opts):
    print(msg)
```

**Single-turn**: One-shot queries, simpler but limited:

```python
async for msg in query(prompt="Explain the auth flow", options=opts):
    if hasattr(msg, "result"):
        print(msg.result)
```

### Pattern 2: Session Management

```python
# Capture session ID
async for message in query(prompt="Start task", options=opts):
    if hasattr(message, 'subtype') and message.subtype == 'init':
        session_id = message.session_id

# Resume later
async for msg in query(
    prompt="Continue from where we left off",
    options=ClaudeAgentOptions(resume=session_id)
):
    print(msg)
```

### Pattern 3: Subagents

```python
from claude_agent_sdk import AgentDefinition

options = ClaudeAgentOptions(
    allowed_tools=["Read", "Grep", "Task"],  # Task required for subagents
    agents={
        "code-reviewer": AgentDefinition(
            description="Expert code reviewer for security and quality",
            prompt="Analyze code quality and suggest improvements.",
            tools=["Read", "Grep", "Glob"],
            model="sonnet"  # Optional model override
        )
    }
)
```

### Pattern 4: Hooks

```python
from claude_agent_sdk import HookMatcher

async def block_dangerous(input_data, tool_use_id, context):
    if 'rm -rf /' in input_data.get('tool_input', {}).get('command', ''):
        return {
            'hookSpecificOutput': {
                'hookEventName': 'PreToolUse',
                'permissionDecision': 'deny',
                'permissionDecisionReason': 'Dangerous command blocked'
            }
        }
    return {}

options = ClaudeAgentOptions(
    hooks={
        'PreToolUse': [HookMatcher(matcher='Bash', hooks=[block_dangerous])]
    }
)
```

### Pattern 5: Custom MCP Tools

```python
from claude_agent_sdk import tool, create_sdk_mcp_server

@tool("get_weather", "Get temperature for location", {"lat": float, "lon": float})
async def get_weather(args):
    # Your API call here
    return {"content": [{"type": "text", "text": f"Temperature: 72°F"}]}

weather_server = create_sdk_mcp_server(
    name="weather",
    tools=[get_weather]
)

options = ClaudeAgentOptions(
    mcp_servers={"weather": weather_server},
    allowed_tools=["mcp__weather__get_weather"]
)
```

## ClaudeSDKClient (Python Multi-Turn)

For continuous conversations with session continuity:

```python
from claude_agent_sdk import ClaudeSDKClient, ClaudeAgentOptions

async with ClaudeSDKClient(options) as client:
    await client.query("What's the capital of France?")
    async for msg in client.receive_response():
        print(msg)

    # Follow-up - Claude remembers context
    await client.query("What's the population of that city?")
    async for msg in client.receive_response():
        print(msg)
```

## Hook Events

| Event | When Triggered | Use Case |
|-------|---------------|----------|
| `PreToolUse` | Before tool executes | Block/modify operations |
| `PostToolUse` | After tool completes | Logging, auditing |
| `UserPromptSubmit` | Prompt submitted | Add context |
| `Stop` | Agent stops | Cleanup |
| `SubagentStop` | Subagent finishes | Aggregate results |
| `PreCompact` | Before compaction | Archive transcript |

## Best Practices

### Context Management
- Isolate per-subagent context
- Use CLAUDE.md for project conventions
- Periodically prune context in long sessions

### Security
- Start from deny-all; allowlist needed tools only
- Require confirmations for sensitive actions
- Block dangerous commands (rm -rf, sudo)

### Tool Design
- Keep tools small with clear schemas
- Tools are prominent in Claude's context
- Design for composability

### Error Handling
```python
from claude_agent_sdk import CLINotFoundError, ProcessError

try:
    async for msg in query(prompt="Task", options=opts):
        print(msg)
except CLINotFoundError:
    print("Install Claude Code: npm install -g @anthropic-ai/claude-code")
except ProcessError as e:
    print(f"Failed with exit code: {e.exit_code}")
```

## Hosting Patterns

| Pattern | Description | Use Case |
|---------|-------------|----------|
| Ephemeral | New container per task | Bug fixes, one-off processing |
| Long-Running | Persistent containers | Email agents, chatbots |
| Hybrid | Hydrated with history | Project managers, deep research |

## Official Documentation

For complete API reference:
- Python: https://platform.claude.com/docs/en/agent-sdk/python
- TypeScript: https://platform.claude.com/docs/en/agent-sdk/typescript
- Overview: https://platform.claude.com/docs/en/agent-sdk/overview

## References

Load these for detailed patterns:
- [Python API Patterns](references/python-patterns.md)
- [TypeScript API Patterns](references/typescript-patterns.md)
- [Hosting Guide](references/hosting.md)
