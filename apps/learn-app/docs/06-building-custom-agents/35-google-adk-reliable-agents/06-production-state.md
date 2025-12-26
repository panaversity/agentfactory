---
sidebar_position: 6
title: "Production State"
description: "Manage agent state with SessionService and integrate MCP tools for persistent, reliable agents"
keywords: [google adk, session service, firestore, tool context, mcp toolset, state persistence, production agents]
chapter: 35
lesson: 6
duration_minutes: 50

skills:
  - name: "SessionService Abstraction"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student can switch between InMemory and Firestore session services and observe persistence behavior"

  - name: "ToolContext State Access"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student can implement stateful tools that read and write session state within agent workflows"

  - name: "MCP Integration Architecture"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student can connect MCP servers to ADK agents and understand tool discovery flow"

learning_objectives:
  - objective: "Configure SessionService for development and production environments"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student implements agent with switchable session backends"

  - objective: "Access and modify session state from within tools using ToolContext"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student implements tool that persists user preferences across sessions"

  - objective: "Integrate MCP servers as agent toolsets"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student connects filesystem MCP server and executes file operations through agent"

cognitive_load:
  new_concepts: 3
  assessment: "3 infrastructure concepts (SessionService abstraction, ToolContext pattern, McpToolset integration) within B1 limit (7-10 concepts)"

differentiation:
  extension_for_advanced: "Implement custom SessionService backend (Redis, PostgreSQL) that extends production options"
  remedial_for_struggling: "Focus on InMemory→Firestore transition first; understand state persistence concept before MCP complexity"
---

# Production State

Your task manager agent works perfectly during development. Users add tasks, the agent lists them back, conversations make sense. But what happens when you restart the agent? All tasks disappear. All conversation history evaporates. The agent has no memory of anything that happened before.

This is the gap between demo agents and production systems.

Production agents need production state: persistent memory that survives restarts, deployments, and scaling. ADK provides a clean abstraction layer that lets you swap state backends without changing agent code. Start with in-memory during development, switch to Firestore for production, scale to managed services—all by changing configuration, not reimplementing logic.

This lesson teaches three interconnected patterns that separate reliable agents from fragile ones.

## The State Problem: Memory Loss in Restarts

Let's be concrete about what happens without persistent state.

### Development Agents (In-Memory)

```python
from google.adk.agents import Agent
from google.adk.runners import InMemoryRunner

tasks = []  # Lives in process memory

def add_task(name: str) -> dict:
    """Add task to in-memory list."""
    tasks.append({"id": len(tasks) + 1, "name": name})
    return {"status": "added", "total_tasks": len(tasks)}

root_agent = Agent(
    name="task_agent",
    model="gemini-2.5-flash",
    instruction="Manage tasks for the user.",
    tools=[add_task]
)

runner = InMemoryRunner(agent=root_agent)
```

**Output:**
```
User: Add task "Finish report"
Agent: Added task. Total: 1

# ... agent restarts ...

User: List my tasks
Agent: I don't have access to that information.
```

The `tasks` list lived in process memory. When the process exits, it's gone. No persistence layer exists.

### Production Challenge

Real systems restart constantly:
- Deployments (new versions)
- Scaling (load balancers spin up new instances)
- Failures (unexpected crashes)
- Maintenance (scheduled updates)

Without persistent state, each restart is amnesia. Users lose data. Trust erodes.

## SessionService: The Abstraction Layer

ADK's `SessionService` is a pluggable abstraction that separates "where state lives" from "how agents use state."

### The Contract

Every `SessionService` implements the same interface:

```python
class SessionService:
    """Abstract session storage."""

    async def create_session(self, app_name: str, user_id: str) -> Session:
        """Create or retrieve session."""
        pass

    async def update_session(self, session: Session) -> None:
        """Persist session state."""
        pass
```

This means your agent code never depends on "Firestore" or "Redis" or "in-memory." It depends on `SessionService`, which is abstract. You inject the concrete implementation.

### Development: InMemorySessionService

For local development and testing, state lives in process RAM:

```python
from google.adk.sessions import InMemorySessionService
from google.adk.runners import Runner
from google.adk.agents import Agent

session_service = InMemorySessionService()

root_agent = Agent(
    name="task_agent",
    model="gemini-2.5-flash",
    instruction="Help manage tasks."
)

runner = Runner(
    app_name="task_manager",
    agent=root_agent,
    session_service=session_service
)

# Run locally
# $ adk run task_manager
```

**Benefits:**
- Zero setup (no database, no credentials)
- Fast (in-process RAM)
- Perfect for rapid iteration

**Drawbacks:**
- Data lost on restart
- Not suitable for production
- Can't be scaled across multiple processes

### Production: FirestoreSessionService

For production, state persists in Google Cloud Firestore:

```python
from google.adk.sessions import FirestoreSessionService
from google.adk.runners import Runner
from google.cloud.firestore import Client

# Connect to Firestore
firestore_client = Client(project="my-ai-agents")

session_service = FirestoreSessionService(
    database=firestore_client
)

root_agent = Agent(
    name="task_agent",
    model="gemini-2.5-flash",
    instruction="Help manage tasks."
)

runner = Runner(
    app_name="task_manager",
    agent=root_agent,
    session_service=session_service
)

# Deploy to production
# $ adk deploy cloud_run --project=my-ai-agents
```

**Output:**
```
User: Add task "Finish report"
Agent: Added task. Total: 1

# ... agent restarts (new deployment) ...

User: List my tasks
Agent: I found 1 task:
  - Finish report
```

The session persists across restarts because it's stored in Firestore, not process memory.

**Benefits:**
- Survives restarts and deployments
- Scales across multiple instances
- Accessible from anywhere (cloud-based)

**Drawbacks:**
- Requires GCP project and credentials
- Slightly higher latency (network calls)
- Costs money (but minimal for most workloads)

### Scaling: VertexAiSessionService

For enterprise scale, use Vertex AI's managed session service:

```python
from google.adk.sessions import VertexAiSessionService
from google.adk.runners import Runner

session_service = VertexAiSessionService(
    project="my-ai-agents",
    location="us-central1"
)

root_agent = Agent(
    name="task_agent",
    model="gemini-2.5-flash",
    instruction="Help manage tasks."
)

runner = Runner(
    app_name="task_manager",
    agent=root_agent,
    session_service=session_service
)
```

Vertex AI handles everything: replication, backup, scaling, monitoring. You focus on agent logic, not infrastructure.

## ToolContext: State Inside Tools

Now you've chosen a session backend. The next pattern is accessing state from within tools.

Problem: Your tools need to read and write session data, but how? A tool is just a Python function.

Solution: ADK injects a special `ToolContext` parameter that tools can use to access session state.

### Reading State

```python
from google.adk.tools.tool_context import ToolContext

def get_user_preference(tool_context: ToolContext) -> str:
    """Read user preference from session."""
    # Session state is a dictionary accessible via tool_context
    preference = tool_context.state.get("user_preference", "light")
    return preference

root_agent = Agent(
    name="settings_agent",
    model="gemini-2.5-flash",
    instruction="Manage user settings.",
    tools=[get_user_preference]
)
```

**Output:**
```
User: What's my theme preference?
Agent: Your theme is set to "light".
```

When the agent calls `get_user_preference`, ADK automatically injects `tool_context` containing the current session state.

### Writing State

```python
from google.adk.tools.tool_context import ToolContext

def set_user_preference(preference: str, tool_context: ToolContext) -> dict:
    """Set user preference and persist to session."""
    # Write to session state
    tool_context.state["user_preference"] = preference

    # Return confirmation
    return {
        "status": "saved",
        "preference": preference
    }

root_agent = Agent(
    name="settings_agent",
    model="gemini-2.5-flash",
    instruction="Manage user settings. Call set_user_preference to save preferences.",
    tools=[set_user_preference]
)
```

**Output:**
```
User: Change my theme to dark
Agent: I've saved your preference to dark.

# ... restart agent ...

User: What's my theme?
Agent: Your theme is set to dark.
```

This works because:

1. `set_user_preference` writes to `tool_context.state`
2. ADK persists that state to the SessionService
3. When the agent restarts, the SessionService loads that state
4. The next `get_user_preference` call reads the persisted value

### Stateful Workflow: Task Management

Here's a complete example combining state reading and writing:

```python
from google.adk.tools.tool_context import ToolContext
from typing import List, Dict

def add_task(task_name: str, tool_context: ToolContext) -> dict:
    """Add task and persist to session state."""
    # Read current tasks from session
    tasks = tool_context.state.get("tasks", [])

    # Create new task
    new_task = {
        "id": len(tasks) + 1,
        "name": task_name,
        "completed": False,
        "created_at": "2025-01-15"
    }

    # Append and persist
    tasks.append(new_task)
    tool_context.state["tasks"] = tasks

    return {
        "status": "created",
        "task": new_task,
        "total_tasks": len(tasks)
    }

def complete_task(task_id: int, tool_context: ToolContext) -> dict:
    """Mark task complete."""
    tasks = tool_context.state.get("tasks", [])

    for task in tasks:
        if task["id"] == task_id:
            task["completed"] = True
            tool_context.state["tasks"] = tasks
            return {"status": "completed", "task": task}

    return {"status": "error", "message": f"Task {task_id} not found"}

def list_tasks(tool_context: ToolContext) -> dict:
    """List all tasks."""
    tasks = tool_context.state.get("tasks", [])

    pending = [t for t in tasks if not t["completed"]]
    completed = [t for t in tasks if t["completed"]]

    return {
        "pending": pending,
        "completed": completed,
        "total": len(tasks)
    }

root_agent = Agent(
    name="task_manager",
    model="gemini-2.5-flash",
    instruction="""Manage user tasks.
Use add_task to create tasks.
Use complete_task to mark tasks done.
Use list_tasks to show current tasks.
Always show confirmation after changes.""",
    tools=[add_task, complete_task, list_tasks]
)
```

**Execution:**
```
User: Add "Write documentation"
Agent: Created task 1: Write documentation. Total: 1

User: Add "Review PR"
Agent: Created task 2: Review PR. Total: 2

User: List my tasks
Agent: Pending (2):
  - Write documentation
  - Review PR

User: Complete task 1
Agent: Marked "Write documentation" complete.

# ... agent restarts ...

User: What tasks are left?
Agent: Pending (1):
  - Review PR
  Completed (1):
  - Write documentation
```

The key insight: State changes inside tools are automatically persisted. You don't call "save" explicitly. `tool_context.state` is automatically synced to your SessionService.

## MCP Integration: Pluggable Tool Ecosystems

So far, you've built tools directly in Python. But many common capabilities—filesystem access, database queries, API calls—are already implemented as MCP servers.

Instead of reimplementing these, you can connect existing MCP servers to your agent through `McpToolset`.

### What Is MCP?

Model Context Protocol is a standard for expressing tools and capabilities. Think of it like USB: once you understand the standard, any device just works.

An MCP server is a process that exposes tools through a standard interface. Your agent doesn't care what's inside—it just calls tools by name.

### Connecting an MCP Server

Here's how to add the filesystem MCP server (lets your agent read/write files):

```python
from google.adk.agents import Agent
from google.adk.tools.mcp_tool import McpToolset, StdioConnectionParams
from mcp import StdioServerParameters

# Configure filesystem MCP server
mcp_toolset = McpToolset(
    connection_params=StdioConnectionParams(
        server_params=StdioServerParameters(
            command="npx",
            args=["-y", "@modelcontextprotocol/server-filesystem", "/tmp"]
        )
    )
)

root_agent = Agent(
    name="file_manager",
    model="gemini-2.5-flash",
    instruction="Help users work with files. Use available tools.",
    tools=[mcp_toolset]  # Attach MCP toolset
)
```

**Output:**
```
User: Create a file called "notes.txt" with my task list
Agent: I've created notes.txt with your tasks.

User: Read the file back to me
Agent: [reads file from filesystem via MCP]
```

The filesystem MCP server provides:
- `read_file(path)` - Read file contents
- `write_file(path, content)` - Write files
- `list_directory(path)` - List files
- `delete_file(path)` - Remove files

All available to your agent automatically through the McpToolset.

### Architecture: How MCP Integration Works

```
Your Agent (ADK)
    |
    v
McpToolset (Tool Wrapper)
    |
    v
MCP Protocol (Stdio, HTTP, SSE)
    |
    v
MCP Server (Filesystem, Database, API)
    |
    v
External Capability (Filesystem, Database, API)
```

When your agent calls a tool:

1. **Tool Discovery**: McpToolset queries the MCP server for available tools
2. **Tool Execution**: Agent calls tool → ADK routes to McpToolset
3. **MCP Protocol**: McpToolset sends request to MCP server via standard protocol
4. **Execution**: MCP server executes capability
5. **Result Return**: Server returns result to ADK → Agent receives response

### Combining SessionService + MCP

Here's a realistic workflow: Agent manages a task list (state) and persists it to a file (MCP):

```python
from google.adk.agents import Agent
from google.adk.tools.mcp_tool import McpToolset, StdioConnectionParams
from google.adk.tools.tool_context import ToolContext
from mcp import StdioServerParameters
import json

# MCP filesystem server
mcp_toolset = McpToolset(
    connection_params=StdioConnectionParams(
        server_params=StdioServerParameters(
            command="npx",
            args=["-y", "@modelcontextprotocol/server-filesystem", "/tmp"]
        )
    )
)

def add_task(task_name: str, tool_context: ToolContext) -> dict:
    """Add task to session state."""
    tasks = tool_context.state.get("tasks", [])
    new_task = {"id": len(tasks) + 1, "name": task_name, "completed": False}
    tasks.append(new_task)
    tool_context.state["tasks"] = tasks
    return {"status": "created", "task": new_task}

def export_tasks(tool_context: ToolContext) -> dict:
    """Export tasks to file via MCP."""
    # Read tasks from session state
    tasks = tool_context.state.get("tasks", [])

    # This would trigger MCP tools indirectly through agent instruction
    return {
        "status": "ready_to_export",
        "task_count": len(tasks),
        "instruction": "Use filesystem tools to save tasks.json"
    }

root_agent = Agent(
    name="task_manager_with_export",
    model="gemini-2.5-flash",
    instruction="""Manage tasks and export to files.

Available:
- add_task: Add new task to session
- export_tasks: Prepare export
- Filesystem tools (via MCP): write_file, read_file

Workflow:
1. User adds tasks
2. Agent calls add_task to persist to session
3. User requests export
4. Agent calls write_file (via MCP) to save tasks.json""",
    tools=[add_task, export_tasks, mcp_toolset]
)
```

### MCP Server Options

Common MCP servers you can integrate:

| Server | Enables | Command |
|--------|---------|---------|
| **Filesystem** | File operations | `@modelcontextprotocol/server-filesystem` |
| **PostgreSQL** | Database queries | `@modelcontextprotocol/server-postgres` |
| **Google Drive** | Drive file access | `@modelcontextprotocol/server-google-drive` |
| **Slack** | Slack operations | `@modelcontextprotocol/server-slack` |
| **Git** | Git operations | `@modelcontextprotocol/server-git` |

You can also write custom MCP servers, but that's advanced.

## Choosing Your Architecture

How do you decide between session backends? How do you mix SessionService with MCP?

Here's a decision framework:

### Session Backend Selection

| Environment | Use | Why |
|-------------|-----|-----|
| **Local Development** | `InMemorySessionService` | No setup, fast iteration |
| **Testing** | `InMemorySessionService` | Isolated tests, no database |
| **Small Production** | `FirestoreSessionService` | Persistent, scalable, low cost |
| **Enterprise/Scale** | `VertexAiSessionService` | Fully managed, no ops |
| **Custom Backend** | Implement `SessionService` | If you need Redis, DynamoDB, etc. |

### MCP Integration Decision

| Need | Use MCP | Why |
|------|---------|-----|
| **File operations** | Yes (filesystem server) | Standard, proven, secure |
| **Database queries** | Yes (postgres server) | Avoid embedding credentials |
| **Simple Python functions** | No | Direct tools faster |
| **Third-party APIs** | Maybe | MCP server if available; use tools otherwise |
| **Complex custom logic** | No | Implement as Python tools |

### Common Pattern: Development to Production

```python
# config.py
import os
from google.adk.sessions import InMemorySessionService, FirestoreSessionService

def get_session_service():
    """Choose session backend based on environment."""
    env = os.getenv("ENVIRONMENT", "development")

    if env == "development":
        # Fast, zero setup
        return InMemorySessionService()
    elif env == "production":
        # Persistent, scalable
        from google.cloud.firestore import Client
        firestore = Client(project=os.getenv("GCP_PROJECT"))
        return FirestoreSessionService(database=firestore)
    else:
        raise ValueError(f"Unknown environment: {env}")
```

Then in your agent:

```python
from google.adk.runners import Runner
from config import get_session_service

session_service = get_session_service()

runner = Runner(
    app_name="my_agent",
    agent=root_agent,
    session_service=session_service
)
```

Run locally with development defaults:
```bash
python agent.py
```

Deploy to production:
```bash
ENVIRONMENT=production GCP_PROJECT=my-project python agent.py
```

Same code. Different backends based on configuration.

## Try With AI

### Prompt 1: Session Backend Trade-offs

```
I'm building an agent for customer support that needs to remember conversation
history and user preferences. Talk me through the trade-offs:

- In-memory during development?
- Firestore for production?
- When would I need Vertex AI?

What are the actual latency, cost, and operational differences?
```

**What you're learning**: Storage architecture thinking—understanding that state backends are pluggable, not architectural constraints.

### Prompt 2: Implement Stateful Tool

```
I want a tool that tracks "conversation turns" for rate limiting.

Requirement: Count how many times a user has called tool X in the last hour.
Reset the counter when an hour passes.

Help me implement this using ToolContext.state. Should I store timestamps?
What's the simplest way to detect "hour boundary"?
```

**What you're learning**: State design patterns—thinking about what to persist, how to structure it, when to update it.

### Prompt 3: MCP Server Connection

```
I want to let my agent create and read files to summarize conversations.

There's an MCP filesystem server I can use. Help me:

1. Connect the MCP server to my agent
2. Add an instruction for the agent to save conversation summaries to a file
3. What security considerations exist? (What if an agent tries to read /etc/passwd?)
```

**What you're learning**: Tool ecosystem integration—understanding that MCP provides standard capabilities rather than implementing everything custom.

Safety note: When agents access external systems (filesystems, databases) via MCP, implement guardrails. Use `before_tool_callback` to validate requests. Never give agents more permissions than necessary.

