---
sidebar_position: 12
title: "Agent Integration"
description: "Transform your API endpoints into agent tools—the bridge between FastAPI and AI agents"
keywords: [agent-tools, openai-agents, function-calling, fastapi, ai-integration]
chapter: 40
lesson: 12
duration_minutes: 50

skills:
  - name: "API-to-Function Conversion"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "3.4 Programming"
    measurable_at_this_level: "Student converts API endpoint to callable function"

  - name: "Agent Tool Creation"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "3.4 Programming"
    measurable_at_this_level: "Student creates agent with API-backed tools"

  - name: "Streaming Agent Responses"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "3.4 Programming"
    measurable_at_this_level: "Student streams agent response via SSE"

learning_objectives:
  - objective: "Convert API endpoints to agent tools"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Agent successfully calls task CRUD via tools"

  - objective: "Create an agent endpoint that uses your API"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "POST /agent/chat endpoint responds using tools"

  - objective: "Stream agent responses to clients"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "SSE endpoint streams agent tokens"

cognitive_load:
  new_concepts: 4
  assessment: "function_tool wrapper, Agent class, Runner, SSE streaming"

differentiation:
  extension_for_advanced: "Add multiple specialized agents with handoffs"
  remedial_for_struggling: "Focus on single tool before adding agent"
---

# Agent Integration

You've built a complete API: CRUD, authentication, database, streaming. Now the payoff—turning those endpoints into tools that AI agents can use. This is where FastAPI meets AI agents.

The pattern is straightforward:
1. Your API endpoints define capabilities
2. Wrap them as functions agents can call
3. Create an agent that orchestrates the tools
4. Expose the agent via an SSE endpoint

After this lesson, natural language requests become API operations.

## The Pattern: APIs → Functions → Tools

Your Task API has these operations:

| Endpoint | Operation |
|----------|-----------|
| POST /tasks | Create task |
| GET /tasks | List tasks |
| GET /tasks/{id} | Get task |
| PUT /tasks/{id} | Update task |
| DELETE /tasks/{id} | Delete task |

Each becomes a function an agent can call. The agent decides WHEN to call them based on natural language requests.

## Creating Tool Functions

Start with simple wrappers around your existing code:

```python
# tools.py
from sqlmodel import Session, select
from models import Task, TaskCreate
from database import engine


def create_task(title: str, description: str | None = None) -> dict:
    """Create a new task with the given title and optional description."""
    with Session(engine) as session:
        task = Task(title=title, description=description)
        session.add(task)
        session.commit()
        session.refresh(task)
        return {"id": task.id, "title": task.title, "status": task.status}


def list_tasks() -> list[dict]:
    """List all tasks."""
    with Session(engine) as session:
        tasks = session.exec(select(Task)).all()
        return [{"id": t.id, "title": t.title, "status": t.status} for t in tasks]


def get_task(task_id: int) -> dict | None:
    """Get a specific task by ID."""
    with Session(engine) as session:
        task = session.get(Task, task_id)
        if task:
            return {"id": task.id, "title": task.title, "description": task.description, "status": task.status}
        return None


def update_task_status(task_id: int, status: str) -> dict | None:
    """Update a task's status. Status must be: pending, in_progress, or completed."""
    with Session(engine) as session:
        task = session.get(Task, task_id)
        if not task:
            return None
        task.status = status
        session.add(task)
        session.commit()
        session.refresh(task)
        return {"id": task.id, "title": task.title, "status": task.status}


def delete_task(task_id: int) -> bool:
    """Delete a task by ID. Returns True if deleted, False if not found."""
    with Session(engine) as session:
        task = session.get(Task, task_id)
        if not task:
            return False
        session.delete(task)
        session.commit()
        return True
```

**Key points:**
- Each function has a clear docstring (becomes tool description)
- Parameters have type hints (agents use these)
- Returns simple dicts (JSON-serializable)
- Manages its own session (independent of HTTP request)

## Creating the Agent

Using OpenAI Agents SDK:

```bash
uv add openai-agents
```

```python
# agent.py
from agents import Agent, Runner, function_tool
from tools import create_task, list_tasks, get_task, update_task_status, delete_task


# Wrap functions as tools
@function_tool
def tool_create_task(title: str, description: str | None = None) -> str:
    """Create a new task with the given title and optional description."""
    result = create_task(title, description)
    return f"Created task {result['id']}: {result['title']}"


@function_tool
def tool_list_tasks() -> str:
    """List all tasks."""
    tasks = list_tasks()
    if not tasks:
        return "No tasks found."
    return "\n".join([f"[{t['id']}] {t['title']} ({t['status']})" for t in tasks])


@function_tool
def tool_get_task(task_id: int) -> str:
    """Get details of a specific task by ID."""
    task = get_task(task_id)
    if not task:
        return f"Task {task_id} not found."
    return f"Task {task['id']}: {task['title']}\nDescription: {task['description']}\nStatus: {task['status']}"


@function_tool
def tool_update_status(task_id: int, status: str) -> str:
    """Update a task's status. Status must be: pending, in_progress, or completed."""
    result = update_task_status(task_id, status)
    if not result:
        return f"Task {task_id} not found."
    return f"Updated task {result['id']} to {result['status']}"


@function_tool
def tool_delete_task(task_id: int) -> str:
    """Delete a task by ID."""
    if delete_task(task_id):
        return f"Task {task_id} deleted."
    return f"Task {task_id} not found."


# Create the agent
task_agent = Agent(
    name="Task Manager",
    instructions="""You are a task management assistant. Help users manage their tasks.

You can:
- Create new tasks
- List all tasks
- Get details of specific tasks
- Update task status (pending, in_progress, completed)
- Delete tasks

Be helpful and concise. When creating tasks, confirm what you created.
When listing tasks, format them clearly.""",
    tools=[
        tool_create_task,
        tool_list_tasks,
        tool_get_task,
        tool_update_status,
        tool_delete_task,
    ],
)
```

## Non-Streaming Agent Endpoint

Start with a simple endpoint that waits for the complete response:

```python
# main.py
from fastapi import FastAPI
from pydantic import BaseModel
from agents import Runner
from agent import task_agent

app = FastAPI()


class ChatRequest(BaseModel):
    message: str


class ChatResponse(BaseModel):
    response: str


@app.post("/agent/chat", response_model=ChatResponse)
async def chat_with_agent(request: ChatRequest):
    """Send a message to the task agent and get a response."""
    result = await Runner.run(task_agent, request.message)
    return ChatResponse(response=result.final_output)
```

**Testing:**

```bash
curl -X POST http://localhost:8000/agent/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "Create a task called Learn FastAPI"}'

{"response": "Created task 1: Learn FastAPI"}
```

```bash
curl -X POST http://localhost:8000/agent/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "List all my tasks"}'

{"response": "[1] Learn FastAPI (pending)"}
```

## Streaming Agent Endpoint

For better UX, stream the response as it generates:

```python
# main.py
from fastapi.responses import StreamingResponse
from sse_starlette.sse import EventSourceResponse
import json


async def agent_stream_generator(message: str):
    """Generate SSE events from agent response."""
    result = Runner.run_streamed(task_agent, message)

    async for event in result.stream_events():
        if event.type == "raw_response_event":
            # Extract text from the response
            if hasattr(event.data, 'delta') and hasattr(event.data.delta, 'text'):
                text = event.data.delta.text
                if text:
                    yield {
                        "event": "token",
                        "data": json.dumps({"text": text})
                    }

    # Final event with complete response
    yield {
        "event": "complete",
        "data": json.dumps({"response": result.final_output})
    }


@app.post("/agent/chat/stream")
async def chat_with_agent_stream(request: ChatRequest):
    """Stream agent response via SSE."""
    return EventSourceResponse(agent_stream_generator(request.message))
```

**Testing in browser console:**

```javascript
const response = await fetch('http://localhost:8000/agent/chat/stream', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({message: 'Create a task called Test streaming'})
});

const reader = response.body.getReader();
const decoder = new TextDecoder();

while (true) {
    const {done, value} = await reader.read();
    if (done) break;
    console.log(decoder.decode(value));
}
```

## Complete Agent Integration

Here's everything together:

```python
# tools.py
from sqlmodel import Session, select
from models import Task
from database import engine


def create_task(title: str, description: str | None = None) -> dict:
    with Session(engine) as session:
        task = Task(title=title, description=description)
        session.add(task)
        session.commit()
        session.refresh(task)
        return {"id": task.id, "title": task.title, "status": task.status}


def list_tasks() -> list[dict]:
    with Session(engine) as session:
        tasks = session.exec(select(Task)).all()
        return [{"id": t.id, "title": t.title, "status": t.status} for t in tasks]


def get_task(task_id: int) -> dict | None:
    with Session(engine) as session:
        task = session.get(Task, task_id)
        if task:
            return {"id": task.id, "title": task.title, "description": task.description, "status": task.status}
        return None


def update_task_status(task_id: int, status: str) -> dict | None:
    with Session(engine) as session:
        task = session.get(Task, task_id)
        if not task:
            return None
        task.status = status
        session.add(task)
        session.commit()
        session.refresh(task)
        return {"id": task.id, "title": task.title, "status": task.status}


def delete_task(task_id: int) -> bool:
    with Session(engine) as session:
        task = session.get(Task, task_id)
        if not task:
            return False
        session.delete(task)
        session.commit()
        return True
```

```python
# agent.py
from agents import Agent, function_tool
from tools import create_task, list_tasks, get_task, update_task_status, delete_task


@function_tool
def tool_create_task(title: str, description: str | None = None) -> str:
    """Create a new task."""
    result = create_task(title, description)
    return f"Created task {result['id']}: {result['title']}"


@function_tool
def tool_list_tasks() -> str:
    """List all tasks."""
    tasks = list_tasks()
    if not tasks:
        return "No tasks found."
    return "\n".join([f"[{t['id']}] {t['title']} ({t['status']})" for t in tasks])


@function_tool
def tool_get_task(task_id: int) -> str:
    """Get a specific task."""
    task = get_task(task_id)
    if not task:
        return f"Task {task_id} not found."
    return f"Task {task['id']}: {task['title']}\nStatus: {task['status']}"


@function_tool
def tool_update_status(task_id: int, status: str) -> str:
    """Update task status (pending/in_progress/completed)."""
    result = update_task_status(task_id, status)
    if not result:
        return f"Task {task_id} not found."
    return f"Updated task {result['id']} to {result['status']}"


@function_tool
def tool_delete_task(task_id: int) -> str:
    """Delete a task."""
    if delete_task(task_id):
        return f"Task {task_id} deleted."
    return f"Task {task_id} not found."


task_agent = Agent(
    name="Task Manager",
    instructions="You help manage tasks. Create, list, update, and delete tasks as requested.",
    tools=[tool_create_task, tool_list_tasks, tool_get_task, tool_update_status, tool_delete_task],
)
```

```python
# main.py (agent routes)
from fastapi import FastAPI
from pydantic import BaseModel
from sse_starlette.sse import EventSourceResponse
from agents import Runner
from agent import task_agent
import json

app = FastAPI()


class ChatRequest(BaseModel):
    message: str


@app.post("/agent/chat")
async def chat_with_agent(request: ChatRequest):
    result = await Runner.run(task_agent, request.message)
    return {"response": result.final_output}


async def agent_stream_generator(message: str):
    result = Runner.run_streamed(task_agent, message)
    async for event in result.stream_events():
        if event.type == "raw_response_event":
            if hasattr(event.data, 'delta') and hasattr(event.data.delta, 'text'):
                text = event.data.delta.text
                if text:
                    yield {"event": "token", "data": json.dumps({"text": text})}
    yield {"event": "complete", "data": json.dumps({"response": result.final_output})}


@app.post("/agent/chat/stream")
async def chat_with_agent_stream(request: ChatRequest):
    return EventSourceResponse(agent_stream_generator(request.message))
```

## Hands-On Exercise

**Step 1:** Create tools.py with CRUD wrapper functions

**Step 2:** Create agent.py with tool decorators and agent definition

**Step 3:** Add /agent/chat endpoint to main.py

**Step 4:** Test with natural language:

```bash
# Create
curl -X POST http://localhost:8000/agent/chat \
  -d '{"message": "Add a task: Review pull request"}'

# List
curl -X POST http://localhost:8000/agent/chat \
  -d '{"message": "What tasks do I have?"}'

# Update
curl -X POST http://localhost:8000/agent/chat \
  -d '{"message": "Mark task 1 as completed"}'

# Delete
curl -X POST http://localhost:8000/agent/chat \
  -d '{"message": "Remove task 1"}'
```

**Step 5:** Add the streaming endpoint and test in browser

## Common Mistakes

**Mistake 1:** Forgetting docstrings on tools

```python
# Wrong - agent doesn't know what this does
@function_tool
def tool_create_task(title: str) -> str:
    ...

# Correct - docstring becomes tool description
@function_tool
def tool_create_task(title: str) -> str:
    """Create a new task with the given title."""
    ...
```

**Mistake 2:** Returning complex objects

```python
# Wrong - returns SQLModel object
def get_task(task_id: int) -> Task:
    ...

# Correct - returns serializable dict
def get_task(task_id: int) -> dict:
    ...
```

**Mistake 3:** Not handling missing resources

```python
# Wrong - crashes on missing task
def get_task(task_id: int) -> dict:
    task = session.get(Task, task_id)
    return {"id": task.id}  # AttributeError if None!

# Correct - handle not found
def get_task(task_id: int) -> dict | None:
    task = session.get(Task, task_id)
    if task:
        return {"id": task.id}
    return None
```

## What You've Achieved

You've completed the core loop:

1. **L1-L5**: Built REST API with CRUD
2. **L6-L9**: Added configuration, database, authentication
3. **L10**: Organized with dependency injection
4. **L11**: Added streaming
5. **L12**: Integrated AI agent

Your API is now both:
- **Machine-callable** (REST endpoints for direct integration)
- **Natural language accessible** (agent endpoint for conversational use)

This is the foundation of a Digital FTE service.

## Try With AI

After completing the exercise, explore these scenarios.

**Prompt 1: Adding Context to Tools**

```text
My tools work, but the agent doesn't know about the current user.
How do I pass user context to tools so the agent only manages
THEIR tasks, not everyone's?
```

**What you're learning:** Tool functions can receive context beyond their explicit parameters. Understanding this enables multi-user agent systems.

**Prompt 2: Error Handling in Tools**

```text
When a tool fails (database down, invalid input), what should it
return? Should I raise an exception or return an error string?
How does the agent handle tool failures?
```

**What you're learning:** Agents react to tool outputs. Clear error messages help agents recover gracefully.

**Prompt 3: Adding More Agents**

```text
I want to add a second agent that helps with scheduling tasks
(setting due dates, suggesting priorities). How do I structure
multiple specialized agents that can work together?
```

**What you're learning:** Multi-agent architectures enable specialization. This extends to triage agents, handoffs, and agent collaboration—covered in the capstone.
