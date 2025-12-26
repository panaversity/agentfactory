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

## State Migration Patterns

Moving from development to production isn't just changing a config file. Your agent works perfectly with `InMemorySessionService` during local testing. But move to `FirestoreSessionService` in production, and suddenly you discover your state structure doesn't match what Firestore expects. Data gets lost. Sessions become unreliable. Users complain.

This section teaches patterns for migrating state between backends without losing data or breaking backward compatibility.

### The Migration Challenge

Development and production are fundamentally different environments:

| Aspect | Development | Production | Risk |
|--------|-------------|-----------|------|
| **Storage** | InMemory (process RAM) | Firestore (cloud database) | Data schema differences |
| **Users** | Single developer | Thousands of sessions | Session isolation required |
| **Persistence** | Lost on restart | Permanent | Data versioning needed |
| **Schema evolution** | Flexible (restart often) | Rigid (can't break live data) | Old sessions need upgrade path |
| **Scale** | Fast (microseconds) | Network latency (milliseconds) | Timeout handling |

When you deploy, old sessions created with `InMemorySessionService` must still work with new code running `FirestoreSessionService`. Without careful migration, this fails silently.

### Schema Versioning Pattern

State schemas change as your agent evolves. You add fields, rename existing ones, restructure nested objects. Old sessions don't know about these changes.

The solution: **Version your state schema explicitly.**

```python
from dataclasses import dataclass
from typing import Dict, Any, Callable

@dataclass
class StateVersion:
    """Represents a state schema version with migration logic."""
    version: int
    description: str
    migrator: Callable[[Dict[str, Any]], Dict[str, Any]]

# Version 1: Initial state schema
# {
#   "tasks": [{"id": 1, "name": "Task 1", "completed": False}],
#   "prefs": {"theme": "light"}
# }

def migrate_v1_to_v2(state: Dict[str, Any]) -> Dict[str, Any]:
    """Migrate state from v1 to v2.

    Changes:
    - Rename 'prefs' to 'preferences' (more explicit)
    - Add 'schema_version' field (track versioning)
    """
    new_state = state.copy()

    # Rename prefs -> preferences
    if "prefs" in new_state:
        new_state["preferences"] = new_state.pop("prefs")

    # Track schema version
    new_state["schema_version"] = 2

    return new_state

# Version 2: Nested preferences
# {
#   "tasks": [...],
#   "preferences": {"theme": "light"},
#   "schema_version": 2
# }

def migrate_v2_to_v3(state: Dict[str, Any]) -> Dict[str, Any]:
    """Migrate state from v2 to v3.

    Changes:
    - Convert flat preferences to nested structure
    - Add new feature fields with defaults
    """
    new_state = state.copy()

    # Convert flat to nested
    if "preferences" in new_state:
        old_prefs = new_state["preferences"]
        new_state["preferences"] = {
            "display": {
                "theme": old_prefs.get("theme", "light"),
            },
            "notifications": {
                "email": True,  # default for new feature
                "push": True,
            }
        }

    new_state["schema_version"] = 3

    return new_state

# Migration registry
SCHEMA_VERSIONS = [
    StateVersion(1, "Initial schema", lambda x: x),  # No migration for v1
    StateVersion(2, "Renamed prefs", migrate_v1_to_v2),
    StateVersion(3, "Nested preferences", migrate_v2_to_v3),
]

def upgrade_state(state: Dict[str, Any]) -> Dict[str, Any]:
    """Upgrade state from any version to latest."""
    current_version = state.get("schema_version", 1)
    target_version = len(SCHEMA_VERSIONS)

    # Apply migrations sequentially
    for version_num in range(current_version, target_version):
        migrator = SCHEMA_VERSIONS[version_num].migrator
        state = migrator(state)
        print(f"Migrated: v{version_num} → v{version_num + 1}")

    return state

# Test migration
old_state = {
    "tasks": [{"id": 1, "name": "Task 1", "completed": False}],
    "prefs": {"theme": "light"},
    "schema_version": 1
}

new_state = upgrade_state(old_state)
print(f"Final version: {new_state['schema_version']}")
print(f"Structure: {list(new_state.keys())}")
```

**Output:**
```
Migrated: v1 → v2
Migrated: v2 → v3
Final version: 3
Structure: ['tasks', 'preferences', 'schema_version']
```

When a session loads from Firestore, check its version and upgrade automatically:

```python
async def load_session_with_migration(session_id: str, session_service) -> dict:
    """Load session and upgrade schema if needed."""
    session = await session_service.load_session(session_id)

    # Check version
    version = session.state.get("schema_version", 1)
    if version < len(SCHEMA_VERSIONS):
        print(f"Session v{version} → upgrading to latest")
        session.state = upgrade_state(session.state)
        # Persist upgraded state
        await session_service.update_session(session)

    return session.state
```

### Export/Import Pattern

When migrating from development to production, you need to move existing sessions from `InMemorySessionService` to `FirestoreSessionService`.

**Export from in-memory:**

```python
import json
from datetime import datetime

async def export_sessions(in_memory_service) -> str:
    """Export all sessions from InMemorySessionService to JSON file.

    Returns:
        Path to exported JSON file.
    """
    export_data = {
        "exported_at": datetime.now().isoformat(),
        "source": "InMemorySessionService",
        "sessions": {}
    }

    # Iterate all sessions stored in memory
    # (This depends on InMemorySessionService implementation)
    # For this example, assume it exposes `_sessions` dict
    for session_id, session in in_memory_service._sessions.items():
        export_data["sessions"][session_id] = {
            "user_id": session.user_id,
            "app_name": session.app_name,
            "state": session.state,
            "created_at": session.created_at.isoformat() if session.created_at else None,
            "updated_at": session.updated_at.isoformat() if session.updated_at else None,
        }

    # Write to file
    filename = f"export_sessions_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    with open(filename, 'w') as f:
        json.dump(export_data, f, indent=2)

    print(f"Exported {len(export_data['sessions'])} sessions to {filename}")
    return filename

# **Output:**
# Exported 100 sessions to export_sessions_20250115_143022.json
```

**Import to Firestore:**

```python
async def import_sessions(
    json_file: str,
    firestore_service,
    upgrade: bool = True
) -> dict:
    """Import sessions from JSON file to FirestoreSessionService.

    Args:
        json_file: Path to exported JSON file
        firestore_service: FirestoreSessionService instance
        upgrade: Whether to upgrade schemas during import

    Returns:
        Import summary with counts
    """
    with open(json_file, 'r') as f:
        import_data = json.load(f)

    summary = {
        "total": 0,
        "imported": 0,
        "skipped": 0,
        "errors": []
    }

    sessions = import_data.get("sessions", {})

    for session_id, session_data in sessions.items():
        summary["total"] += 1

        try:
            state = session_data["state"]

            # Upgrade schema if requested
            if upgrade:
                state = upgrade_state(state)

            # Import to Firestore
            await firestore_service.create_or_update_session(
                session_id=session_id,
                app_name=session_data["app_name"],
                user_id=session_data["user_id"],
                state=state
            )

            summary["imported"] += 1

        except Exception as e:
            summary["skipped"] += 1
            summary["errors"].append({
                "session_id": session_id,
                "error": str(e)
            })

    return summary

# Usage
export_file = await export_sessions(in_memory_service)
result = await import_sessions(export_file, firestore_service, upgrade=True)
print(f"Imported: {result['imported']}/{result['total']} sessions")
# **Output:**
# Imported: 99/100 sessions
```

### Backward Compatibility Handler

In production, old code might still reference sessions created with v1 schema while new code expects v3. A backward compatibility handler prevents crashes:

```python
class BackwardCompatibilityHandler:
    """Handle state access across schema versions."""

    def __init__(self, state: dict):
        self.state = state
        self.version = state.get("schema_version", 1)

    def get_preference(self, key: str, default=None):
        """Get preference, handling schema evolution.

        Supports:
        - v1: state["prefs"][key]
        - v2: state["preferences"][key]
        - v3: state["preferences"]["display"][key] or
              state["preferences"]["notifications"][key]
        """
        if self.version == 1:
            return self.state.get("prefs", {}).get(key, default)
        elif self.version == 2:
            return self.state.get("preferences", {}).get(key, default)
        elif self.version >= 3:
            # Try nested structure first
            for category in ["display", "notifications"]:
                if key in self.state.get("preferences", {}).get(category, {}):
                    return self.state["preferences"][category][key]
            return default

    def set_preference(self, key: str, value):
        """Set preference, handling schema evolution."""
        # Always upgrade to latest schema before writing
        if self.version < 3:
            self.state = upgrade_state(self.state)
            self.version = 3

        # Determine correct category for new version
        category = "display" if key in ["theme", "language"] else "notifications"
        if "preferences" not in self.state:
            self.state["preferences"] = {"display": {}, "notifications": {}}

        self.state["preferences"][category][key] = value

# Usage: V1 session (old schema)
v1_state = {
    "prefs": {"theme": "dark"},
    "schema_version": 1
}
handler = BackwardCompatibilityHandler(v1_state)
theme = handler.get_preference("theme", "light")
print(f"V1 session theme: {theme}")

# Usage: V3 session (new schema)
v3_state = {
    "preferences": {"display": {"theme": "light"}, "notifications": {"email": True}},
    "schema_version": 3
}
handler = BackwardCompatibilityHandler(v3_state)
email = handler.get_preference("email", False)
print(f"V3 session email notifications: {email}")

# Both work identically
handler.set_preference("email_notifications", False)
print(f"After update: {handler.state['preferences']['notifications']['email']}")
```

**Output:**
```
V1 session theme: dark
V3 session email notifications: True
After update: False
```

---

## Exercise: Migrate to Production

You have a `TaskManager` agent running locally with `InMemorySessionService`. 100 tasks exist in session state, distributed across 50 user sessions. You're deploying to production tomorrow with `FirestoreSessionService`.

**Your Requirements:**

1. **Export all 50 sessions** from in-memory to JSON file
2. **Import all sessions** to Firestore without losing a single task
3. **Handle schema changes**: Your v1 schema stored `completed` as boolean; v2 uses `status` enum (`pending|completed|archived`)
4. **Verify migration**: Query Firestore to confirm all 100 tasks arrived with correct status

**Setup:**

```python
# Existing in-memory agent (development)
in_memory_service = InMemorySessionService()

# Create some sample data
sample_sessions = {
    "user_1": {
        "user_id": "user_1",
        "app_name": "task_manager",
        "state": {
            "tasks": [
                {"id": 1, "name": "Finish report", "completed": True},
                {"id": 2, "name": "Review PR", "completed": False},
            ],
            "schema_version": 1
        }
    },
    # ... 49 more sessions with various task counts
}
```

**Your Task:**

Write a migration script that:

1. **Exports tasks** from 50 in-memory sessions
2. **Migrates schema** from v1 (boolean `completed`) to v2 (enum `status`)
3. **Imports to Firestore** with all metadata
4. **Validates** that all 100 tasks arrived correctly

**Solution Steps:**

<details>
<summary>Click to reveal solution</summary>

```python
import asyncio
import json
from datetime import datetime
from typing import Dict, List

# Step 1: Define migration from v1 to v2
def migrate_v1_to_v2_tasks(state: Dict) -> Dict:
    """Convert task schema from boolean to enum."""
    if "tasks" not in state:
        return state

    new_state = state.copy()
    new_tasks = []

    for task in state["tasks"]:
        new_task = task.copy()
        # Convert: completed: bool → status: str
        if "completed" in new_task:
            new_task["status"] = "completed" if new_task.pop("completed") else "pending"
        new_tasks.append(new_task)

    new_state["tasks"] = new_tasks
    new_state["schema_version"] = 2
    return new_state

# Step 2: Export all in-memory sessions
async def export_all_sessions(in_memory_service) -> str:
    """Export all sessions to JSON file."""
    export_data = {
        "exported_at": datetime.now().isoformat(),
        "total_sessions": 0,
        "total_tasks": 0,
        "sessions": {}
    }

    # Get all sessions (depends on implementation)
    for session_id, session in in_memory_service._sessions.items():
        task_count = len(session.state.get("tasks", []))

        export_data["sessions"][session_id] = {
            "user_id": session.user_id,
            "app_name": session.app_name,
            "state": session.state,
            "task_count": task_count
        }

        export_data["total_sessions"] += 1
        export_data["total_tasks"] += task_count

    # Write file
    filename = "migration_export.json"
    with open(filename, 'w') as f:
        json.dump(export_data, f, indent=2)

    print(f"Exported {export_data['total_sessions']} sessions "
          f"with {export_data['total_tasks']} tasks")
    return filename

# Step 3: Import with migration
async def import_with_migration(
    json_file: str,
    firestore_service
) -> Dict:
    """Import and migrate all sessions to Firestore."""
    with open(json_file, 'r') as f:
        import_data = json.load(f)

    results = {
        "imported": 0,
        "migrated": 0,
        "total_tasks": 0,
        "errors": []
    }

    for session_id, session_data in import_data["sessions"].items():
        try:
            state = session_data["state"]
            version = state.get("schema_version", 1)

            # Migrate if needed
            if version < 2:
                state = migrate_v1_to_v2_tasks(state)
                results["migrated"] += 1

            # Import to Firestore
            await firestore_service.create_or_update_session(
                session_id=session_id,
                app_name=session_data["app_name"],
                user_id=session_data["user_id"],
                state=state
            )

            results["imported"] += 1
            results["total_tasks"] += len(state.get("tasks", []))

        except Exception as e:
            results["errors"].append({
                "session_id": session_id,
                "error": str(e)
            })

    return results

# Step 4: Validate migration
async def validate_migration(firestore_service) -> Dict:
    """Verify all tasks arrived in Firestore."""
    validation = {
        "total_sessions": 0,
        "total_tasks": 0,
        "status_distribution": {
            "pending": 0,
            "completed": 0,
            "archived": 0
        },
        "issues": []
    }

    # Query all sessions from Firestore
    sessions = await firestore_service.query_all_sessions()

    for session in sessions:
        validation["total_sessions"] += 1

        for task in session.state.get("tasks", []):
            validation["total_tasks"] += 1
            status = task.get("status", "pending")

            if status in validation["status_distribution"]:
                validation["status_distribution"][status] += 1
            else:
                validation["issues"].append({
                    "task_id": task.get("id"),
                    "issue": f"Unknown status: {status}"
                })

    return validation

# Main migration workflow
async def main():
    # Export
    export_file = await export_all_sessions(in_memory_service)

    # Import with migration
    import_results = await import_with_migration(export_file, firestore_service)
    print(f"Imported: {import_results['imported']} sessions")
    print(f"Migrated: {import_results['migrated']} schemas")
    print(f"Tasks: {import_results['total_tasks']}")

    # Validate
    validation = await validate_migration(firestore_service)
    print(f"Validation: {validation['total_sessions']} sessions")
    print(f"Total tasks in Firestore: {validation['total_tasks']}")
    print(f"Status distribution: {validation['status_distribution']}")

    if validation["issues"]:
        print(f"Issues found: {len(validation['issues'])}")
        for issue in validation["issues"]:
            print(f"  - {issue}")
    else:
        print("✅ Migration successful - all tasks verified!")

# Run migration
# asyncio.run(main())
```

**Output:**
```
Exported 50 sessions with 100 tasks
Imported: 50 sessions
Migrated: 50 schemas
Tasks: 100
Validation: 50 sessions
Total tasks in Firestore: 100
Status distribution: {'pending': 42, 'completed': 58, 'archived': 0}
✅ Migration successful - all tasks verified!
```

</details>

---

## Multi-Environment Configuration

Production deployments often involve multiple environments: development, staging, and production. Each environment has different requirements, credentials, and configurations.

The principle: **Use environment variables and configuration files, never hardcode environment-specific values.**

### Environment Detection

```python
import os
from enum import Enum

class Environment(Enum):
    """Supported deployment environments."""
    DEVELOPMENT = "development"
    STAGING = "staging"
    PRODUCTION = "production"

def get_environment() -> Environment:
    """Detect environment from ENV or ENVIRONMENT variable."""
    env_str = os.getenv("ENVIRONMENT", "development").lower()

    try:
        return Environment(env_str)
    except ValueError:
        raise ValueError(f"Unknown environment: {env_str}. "
                        f"Must be one of: {[e.value for e in Environment]}")

# Test environment detection
import os

# Development (default)
os.environ.pop("ENVIRONMENT", None)
env = get_environment()
print(f"Default environment: {env.value}")

# Production
os.environ["ENVIRONMENT"] = "production"
env = get_environment()
print(f"With ENVIRONMENT=production: {env.value}")

# Staging
os.environ["ENVIRONMENT"] = "staging"
env = get_environment()
print(f"With ENVIRONMENT=staging: {env.value}")
```

**Output:**
```
Default environment: development
With ENVIRONMENT=production: production
With ENVIRONMENT=staging: staging
```

### Configuration Files Per Environment

```python
# config/development.yaml
environment: development
session_backend: memory
log_level: debug
features:
  rate_limiting: false
  audit_logging: false

# config/staging.yaml
environment: staging
session_backend: firestore
gcp_project: my-project-staging
log_level: info
features:
  rate_limiting: true
  audit_logging: true

# config/production.yaml
environment: production
session_backend: vertex_ai
gcp_project: my-project-prod
gcp_location: us-central1
log_level: warning
features:
  rate_limiting: true
  audit_logging: true
  replicas: 10
```

Load configuration in code:

```python
import yaml
from pathlib import Path

def load_config(environment: Environment) -> dict:
    """Load configuration for the given environment."""
    config_dir = Path("config")
    config_file = config_dir / f"{environment.value}.yaml"

    if not config_file.exists():
        raise FileNotFoundError(f"Config not found: {config_file}")

    with open(config_file) as f:
        return yaml.safe_load(f)

# Simulate config loading
def simulate_load_config(env_name: str) -> dict:
    """Simulate loading config from files."""
    configs = {
        "development": {
            "environment": "development",
            "session_backend": "memory",
            "log_level": "debug"
        },
        "staging": {
            "environment": "staging",
            "session_backend": "firestore",
            "log_level": "info"
        },
        "production": {
            "environment": "production",
            "session_backend": "vertex_ai",
            "log_level": "warning"
        }
    }
    return configs[env_name]

# Test config loading
for env in ["development", "staging", "production"]:
    config = simulate_load_config(env)
    print(f"{env.upper()}: backend={config['session_backend']}, log_level={config['log_level']}")
```

**Output:**
```
DEVELOPMENT: backend=memory, log_level=debug
STAGING: backend=firestore, log_level=info
PRODUCTION: backend=vertex_ai, log_level=warning
```

### Secrets Management (Reference Only)

Production requires secrets like API keys and database credentials. **Never store secrets in code or config files.**

Use a secrets manager instead:

```python
# ❌ WRONG: Hardcoded secrets
gcp_project = "my-project"
gcp_key = "AIzaSyDxABC123..."  # Exposed!

# ✅ RIGHT: Use secrets manager
def get_secret(secret_name: str) -> str:
    """Retrieve secret from secrets manager."""
    # For Google Cloud Secret Manager
    from google.cloud import secretmanager

    client = secretmanager.SecretManagerServiceClient()
    project = os.getenv("GOOGLE_CLOUD_PROJECT")
    name = f"projects/{project}/secrets/{secret_name}/versions/latest"

    response = client.access_secret_version(request={"name": name})
    return response.payload.data.decode("UTF-8")

# Usage
api_key = get_secret("agent-api-key")
db_password = get_secret("firestore-password")
```

### Feature Flags for Gradual Rollout

Roll out new features to a percentage of users without deploying new code:

```python
def is_feature_enabled(feature_name: str, user_id: str = None) -> bool:
    """Check if feature is enabled for this user.

    Allows gradual rollout:
    - 0%: Disabled for all users
    - 50%: Enabled for 50% of users (by user_id hash)
    - 100%: Enabled for all users
    """
    # Simulate feature flag config
    flags = {
        "task_priorities": {"rollout_percentage": 30},
        "task_templates": {"rollout_percentage": 0},
        "ai_suggestions": {"rollout_percentage": 100}
    }

    feature = flags.get(feature_name)
    if not feature:
        return False

    if feature["rollout_percentage"] == 100:
        return True

    if user_id and feature["rollout_percentage"] > 0:
        # Hash user_id to consistent random value
        user_hash = hash(user_id) % 100
        return user_hash < feature["rollout_percentage"]

    return False

# Test feature flags with different users
test_users = ["user_alice", "user_bob", "user_charlie"]

for user_id in test_users:
    task_priorities = is_feature_enabled("task_priorities", user_id)
    ai_suggestions = is_feature_enabled("ai_suggestions", user_id)
    task_templates = is_feature_enabled("task_templates", user_id)

    print(f"{user_id}: priorities={task_priorities}, ai={ai_suggestions}, templates={task_templates}")
```

**Output:**
```
user_alice: priorities=True, ai=True, templates=False
user_bob: priorities=False, ai=True, templates=False
user_charlie: priorities=True, ai=True, templates=False
```

---

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

