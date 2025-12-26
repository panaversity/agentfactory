# Hosting & Deployment Guide

## System Requirements

| Component | Requirement |
|-----------|-------------|
| Runtime | Python 3.10+ or Node.js 18+ |
| Claude CLI | `npm install -g @anthropic-ai/claude-code` |
| Memory | Recommended 1GiB RAM minimum |
| Disk | 5GiB for CLI and dependencies |
| Network | Outbound HTTPS to `api.anthropic.com` |

## Deployment Patterns

### Pattern 1: Ephemeral Sessions

Create container per task, destroy when complete.

**Use Cases:**
- Bug investigation and fixing
- Document processing
- Translation tasks
- One-off code generation

**Implementation:**
```python
import asyncio
from claude_agent_sdk import query, ClaudeAgentOptions

async def process_task(task: str):
    """Process single task in ephemeral context"""
    result = None
    async for message in query(
        prompt=task,
        options=ClaudeAgentOptions(
            max_turns=20,
            allowed_tools=["Read", "Edit", "Bash"]
        )
    ):
        if hasattr(message, 'result'):
            result = message.result
    return result
```

### Pattern 2: Long-Running Sessions

Persistent containers for continuous operation.

**Use Cases:**
- Email monitoring agents
- Chat assistants
- Site builders
- High-frequency message handlers

**Implementation:**
```python
from claude_agent_sdk import ClaudeSDKClient, ClaudeAgentOptions

class PersistentAgent:
    def __init__(self):
        self.client = None

    async def start(self):
        self.client = ClaudeSDKClient(ClaudeAgentOptions(
            allowed_tools=["Read", "Write", "Bash", "WebSearch"],
            permission_mode="acceptEdits"
        ))
        await self.client.__aenter__()

    async def handle_message(self, message: str):
        await self.client.query(message)
        async for msg in self.client.receive_response():
            yield msg

    async def stop(self):
        if self.client:
            await self.client.__aexit__(None, None, None)
```

### Pattern 3: Hybrid Sessions

Ephemeral containers hydrated with session history.

**Use Cases:**
- Project management assistants
- Deep research agents
- Customer support with ticket history

**Implementation:**
```python
async def resume_work(session_id: str, new_task: str):
    """Resume previous session with new task"""
    async for message in query(
        prompt=new_task,
        options=ClaudeAgentOptions(
            resume=session_id,
            max_turns=30
        )
    ):
        if hasattr(message, 'result'):
            return message.result
```

## Sandbox Providers

| Provider | Type | Best For |
|----------|------|----------|
| [Cloudflare Sandboxes](https://github.com/cloudflare/sandbox-sdk) | Edge | Global distribution |
| [Modal Sandboxes](https://modal.com/docs/guide/sandbox) | Cloud | GPU workloads |
| [Daytona](https://www.daytona.io/) | Dev Environments | Development setups |
| [E2B](https://e2b.dev/) | Code Execution | Secure code running |
| [Fly Machines](https://fly.io/docs/machines/) | VMs | Full isolation |
| [Vercel Sandbox](https://vercel.com/docs/functions/sandbox) | Serverless | Next.js integration |

## Docker Deployment

```dockerfile
FROM python:3.11-slim

# Install Node.js for Claude CLI
RUN apt-get update && apt-get install -y nodejs npm

# Install Claude Code CLI
RUN npm install -g @anthropic-ai/claude-code

# Install Python SDK
COPY requirements.txt .
RUN pip install -r requirements.txt

# Set API key via environment
ENV ANTHROPIC_API_KEY=""

COPY . /app
WORKDIR /app

CMD ["python", "agent.py"]
```

```yaml
# docker-compose.yml
version: '3.8'
services:
  agent:
    build: .
    environment:
      - ANTHROPIC_API_KEY=${ANTHROPIC_API_KEY}
    volumes:
      - ./workspace:/app/workspace
    deploy:
      resources:
        limits:
          memory: 2G
          cpus: '1'
```

## Security Hardening

### Permission Mode Selection

| Mode | Security | Use Case |
|------|----------|----------|
| `default` | High | Production with user confirmation |
| `acceptEdits` | Medium | Trusted file operations |
| `bypassPermissions` | Low | Sandboxed containers only |

### Tool Restrictions

```python
# Minimal toolset for security
options = ClaudeAgentOptions(
    allowed_tools=["Read", "Grep", "Glob"],  # Read-only
    disallowed_tools=["Bash"],  # No shell access
    hooks={
        'PreToolUse': [
            HookMatcher(matcher='Write|Edit', hooks=[sandbox_check])
        ]
    }
)
```

### Network Controls

```python
# Container-level network isolation
sandbox = {
    "enabled": True,
    "network": {
        "allowLocalBinding": False,
        "allowUnixSockets": [],
        # Whitelist specific hosts
        "allowedHosts": ["api.anthropic.com"]
    }
}
```

## Monitoring & Observability

### Cost Tracking

```python
async for message in query(prompt="Task", options=opts):
    if message.type == 'result':
        print(f"Cost: ${message.total_cost_usd:.4f}")
        print(f"Turns: {message.num_turns}")
        print(f"Duration: {message.duration_ms}ms")
```

### Logging Hook

```python
import logging
logger = logging.getLogger(__name__)

async def log_all_tools(input_data, tool_use_id, context):
    logger.info(f"Tool: {input_data['tool_name']}")
    logger.debug(f"Input: {input_data['tool_input']}")
    return {}

options = ClaudeAgentOptions(
    hooks={
        'PreToolUse': [HookMatcher(hooks=[log_all_tools])],
        'PostToolUse': [HookMatcher(hooks=[log_all_tools])]
    }
)
```

## Scaling Considerations

### Session Affinity

When scaling horizontally, ensure session continuity:

```python
# Store session IDs in Redis/database
import redis

r = redis.Redis()

async def get_or_create_session(user_id: str, task: str):
    session_id = r.get(f"session:{user_id}")

    if session_id:
        # Resume existing session
        return query(prompt=task, options=ClaudeAgentOptions(
            resume=session_id.decode()
        ))
    else:
        # New session, capture ID
        async for message in query(prompt=task, options=opts):
            if hasattr(message, 'session_id'):
                r.set(f"session:{user_id}", message.session_id)
            yield message
```

### Rate Limiting

```python
from asyncio import Semaphore

# Limit concurrent agent sessions
agent_semaphore = Semaphore(10)

async def rate_limited_query(prompt: str, options):
    async with agent_semaphore:
        async for message in query(prompt, options):
            yield message
```

## Health Checks

```python
from claude_agent_sdk import query

async def health_check():
    """Verify SDK connectivity"""
    try:
        async for message in query(
            prompt="Say 'healthy'",
            options=ClaudeAgentOptions(max_turns=1)
        ):
            if hasattr(message, 'result') and 'healthy' in message.result.lower():
                return True
        return False
    except Exception:
        return False
```

## Cost Optimization

| Strategy | Implementation |
|----------|----------------|
| Model selection | Use `haiku` for simple tasks, `sonnet` for complex |
| Max turns | Set `max_turns` to prevent runaway loops |
| Tool restrictions | Limit tools to reduce context overhead |
| Session reuse | Resume sessions instead of starting fresh |
