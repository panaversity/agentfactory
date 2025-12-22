---
title: "Capstone: Agent-Powered Task Service"
sidebar_position: 8
chapter: 40
lesson: 8
duration_minutes: 90

skills:
  - name: "Full Agent System Design"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Create"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student builds complete multi-agent API from specification"

  - name: "Pattern Integration"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student combines CRUD, streaming, agent handoffs in single system"

  - name: "Production API Design"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student implements proper error handling, streaming, and documentation"

learning_objectives:
  - objective: "Build complete multi-agent task service"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "All specified endpoints work correctly"

  - objective: "Integrate all chapter patterns in single project"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Project uses CRUD, DI, streaming, agent handoffs"

  - objective: "Implement production-ready streaming and error handling"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "All endpoints stream correctly and handle errors gracefully"

cognitive_load:
  new_concepts: 0
  assessment: "Synthesis lesson - no new concepts, combines all previous learning"

differentiation:
  extension_for_advanced: "Add authentication; implement rate limiting; add WebSocket support"
  remedial_for_struggling: "Complete CRUD first, then add agents incrementally"

generated_by: "content-implementer"
source_spec: "specs/040-chapter-40-fastapi-for-agents/spec.md"
created: "2025-12-22"
---

# Capstone: Agent-Powered Task Service

This chapter taught you seven interconnected skills. Now you'll synthesize them into something real.

This capstone isn't about following instructions—it's about demonstrating that you can work from a specification and deliver a functioning system. The specification below defines what to build. How you build it is up to you.

## What This Capstone Tests

Before diving in, understand what you're demonstrating:

| Lesson | Skill Being Applied |
|--------|-------------------|
| Lesson 1 | Creating FastAPI app, async endpoints, Swagger documentation |
| Lesson 2 | Pydantic models for request/response validation |
| Lesson 3 | CRUD operations with proper HTTP methods |
| Lesson 4 | Error handling with correct status codes |
| Lesson 5 | Dependency injection with repository pattern |
| Lesson 6 | SSE streaming with async generators |
| Lesson 7 | Multi-agent routing with handoff visibility |

If you struggle with any part, that's diagnostic. It tells you which lesson to revisit.

## The Specification

Build a Task Management API with these capabilities:

### CRUD Endpoints

| Method | Path | Description | Success Code | Error Codes |
|--------|------|-------------|--------------|-------------|
| POST | /tasks | Create a new task | 201 | 400 (invalid input) |
| GET | /tasks | List all tasks (optional status filter) | 200 | 400 (invalid filter) |
| GET | `/tasks/{task_id}` | Get a single task | 200 | 404 (not found) |
| PUT | `/tasks/{task_id}` | Update a task | 200 | 400, 404 |
| DELETE | `/tasks/{task_id}` | Delete a task | 200 | 404 |

### Agent Endpoints

| Method | Path | Description | Streaming |
|--------|------|-------------|-----------|
| POST | `/tasks/{task_id}/help` | Triage → Specialist routing | Yes |
| POST | `/tasks/{task_id}/schedule` | Direct to scheduler agent | Yes |
| POST | `/tasks/{task_id}/collaborate` | Direct to collaboration agent | Yes |

### System Endpoints

| Method | Path | Description |
|--------|------|-------------|
| GET | /agents/status | List available agents and capabilities |
| GET | / | Health check with service info |

### Business Rules

1. Task titles cannot be empty or whitespace-only
2. Valid status values: `pending`, `in_progress`, `completed`
3. New tasks start with status `pending`
4. Agent endpoints must validate task exists before running agent
5. All agent responses stream with handoff and tool visibility

## Project Structure

```
task-service/
├── pyproject.toml
├── main.py              # FastAPI app and routes
├── models.py            # Pydantic models
├── repository.py        # TaskRepository with DI
└── agents/
    ├── __init__.py
    ├── triage.py        # Triage agent with handoffs
    ├── scheduler.py     # Scheduler specialist
    └── collaboration.py # Collaboration specialist
```

## Your Approach

**Don't just copy code.** Work through this systematically:

### Phase 1: Data Layer (30 min)
Build `models.py` and `repository.py` first. These are the foundation. Test them independently before adding FastAPI.

Questions to answer:
- What fields does TaskCreate need vs TaskResponse?
- How do you make status filtering optional?
- How do you handle the ID counter across requests?

### Phase 2: CRUD Endpoints (30 min)
Implement all CRUD endpoints. Test each one in Swagger UI before moving on.

Questions to answer:
- Why use `status_code=status.HTTP_201_CREATED` for POST?
- Where does business validation (empty title) go?
- How do you make the status filter query parameter optional?

### Phase 3: Agent Integration (30 min)
Create the three agents and wire them into endpoints with streaming.

Questions to answer:
- What context does the agent need about the task?
- How do you structure SSE events for routing visibility?
- When does the endpoint return an error vs when does the stream?

## Reference Implementation

Use this as a reference after attempting each phase yourself. Don't read ahead—try first, then compare.

### models.py

```python
from pydantic import BaseModel

class TaskCreate(BaseModel):
    title: str
    description: str | None = None

class TaskUpdate(BaseModel):
    title: str
    description: str | None = None
    status: str | None = None

class TaskResponse(BaseModel):
    id: int
    title: str
    description: str | None
    status: str

class HelpRequest(BaseModel):
    question: str

class ToolCallInfo(BaseModel):
    name: str
    arguments: dict
    result: dict | None = None

class AgentResponse(BaseModel):
    task_id: int
    question: str
    response: str
    handled_by: str
    handoff_chain: list[str]
    tool_calls: list[ToolCallInfo]
```

### repository.py

```python
from models import TaskCreate, TaskUpdate

VALID_STATUSES = {"pending", "in_progress", "completed"}

class TaskRepository:
    def __init__(self):
        self.tasks: list[dict] = []
        self.counter = 0

    def create(self, task: TaskCreate) -> dict:
        self.counter += 1
        new_task = {
            "id": self.counter,
            "title": task.title,
            "description": task.description,
            "status": "pending"
        }
        self.tasks.append(new_task)
        return new_task

    def get_all(self, status: str | None = None) -> list[dict]:
        if status:
            return [t for t in self.tasks if t["status"] == status]
        return self.tasks

    def get_by_id(self, task_id: int) -> dict | None:
        for task in self.tasks:
            if task["id"] == task_id:
                return task
        return None

    def update(self, task_id: int, update: TaskUpdate) -> dict | None:
        task = self.get_by_id(task_id)
        if not task:
            return None
        task["title"] = update.title
        if update.description is not None:
            task["description"] = update.description
        if update.status is not None:
            task["status"] = update.status
        return task

    def delete(self, task_id: int) -> bool:
        task = self.get_by_id(task_id)
        if task:
            self.tasks.remove(task)
            return True
        return False

# Singleton
task_repo = TaskRepository()

def get_task_repo() -> TaskRepository:
    return task_repo
```

### agents/scheduler.py

```python
from agents import Agent, function_tool

@function_tool
def set_deadline(task_id: int, deadline: str) -> dict:
    """Set a deadline for a task (format: YYYY-MM-DD)."""
    return {"task_id": task_id, "deadline": deadline, "status": "deadline_set"}

@function_tool
def create_reminder(task_id: int, remind_at: str, message: str = "") -> dict:
    """Create a reminder for a task."""
    return {"task_id": task_id, "remind_at": remind_at, "message": message}

@function_tool
def suggest_time_blocks(task_id: int, hours: int = 2) -> dict:
    """Suggest focused work time blocks for a task."""
    return {
        "task_id": task_id,
        "blocks": [
            {"day": "Monday", "time": "09:00-11:00"},
            {"day": "Wednesday", "time": "14:00-16:00"}
        ]
    }

scheduler_agent = Agent(
    name="scheduler",
    instructions="""You are a scheduling specialist for task management.

Expertise: Deadlines, reminders, time blocking, scheduling strategy.

When helping:
1. Consider task complexity for realistic deadlines
2. Suggest buffer time for unexpected issues
3. Use tools to take action, don't just advise""",
    tools=[set_deadline, create_reminder, suggest_time_blocks],
    model="gpt-4o-mini"
)
```

### agents/collaboration.py

```python
from agents import Agent, function_tool

@function_tool
def assign_to_user(task_id: int, email: str, note: str = "") -> dict:
    """Assign a task to a team member."""
    return {"task_id": task_id, "assigned_to": email, "note": note}

@function_tool
def share_task(task_id: int, emails: list[str], permission: str = "view") -> dict:
    """Share a task with team members."""
    return {"task_id": task_id, "shared_with": emails, "permission": permission}

@function_tool
def create_meeting(task_id: int, attendees: list[str], duration: int = 30) -> dict:
    """Schedule a meeting about a task."""
    return {"task_id": task_id, "attendees": attendees, "duration": duration}

collaboration_agent = Agent(
    name="collaboration",
    instructions="""You are a collaboration specialist for task management.

Expertise: Delegation, sharing, team coordination, meetings.

When helping:
1. Clarify who should be involved
2. Suggest appropriate permissions
3. Use tools to take action, don't just advise""",
    tools=[assign_to_user, share_task, create_meeting],
    model="gpt-4o-mini"
)
```

### agents/triage.py

```python
from agents import Agent, handoff
from .scheduler import scheduler_agent
from .collaboration import collaboration_agent

triage_agent = Agent(
    name="triage",
    instructions="""You route task questions to the right specialist.

Route to SCHEDULER for:
- Deadlines, due dates, timing
- Reminders, notifications
- Time blocking, scheduling

Route to COLLABORATION for:
- Delegation, assigning work
- Sharing, permissions
- Team meetings

For simple questions, answer directly.
When routing, briefly explain why.""",
    tools=[
        handoff(scheduler_agent),
        handoff(collaboration_agent)
    ],
    model="gpt-4o-mini"
)
```

### main.py

```python
from fastapi import FastAPI, Depends, HTTPException, status
from sse_starlette.sse import EventSourceResponse
import json

from models import TaskCreate, TaskUpdate, TaskResponse, HelpRequest
from repository import TaskRepository, get_task_repo, VALID_STATUSES
from agents import Runner
from agents.triage import triage_agent
from agents.scheduler import scheduler_agent
from agents.collaboration import collaboration_agent

app = FastAPI(
    title="Agent-Powered Task Service",
    description="Multi-agent task management with scheduling and collaboration",
    version="1.0.0"
)

# --- CRUD Endpoints ---

@app.post("/tasks", response_model=TaskResponse, status_code=status.HTTP_201_CREATED)
def create_task(task: TaskCreate, repo: TaskRepository = Depends(get_task_repo)):
    if not task.title.strip():
        raise HTTPException(status_code=400, detail="Title cannot be empty")
    return repo.create(task)

@app.get("/tasks", response_model=list[TaskResponse])
def list_tasks(
    status_filter: str | None = None,
    repo: TaskRepository = Depends(get_task_repo)
):
    if status_filter and status_filter not in VALID_STATUSES:
        raise HTTPException(status_code=400, detail=f"Invalid status: {status_filter}")
    return repo.get_all(status=status_filter)

@app.get("/tasks/{task_id}", response_model=TaskResponse)
def get_task(task_id: int, repo: TaskRepository = Depends(get_task_repo)):
    task = repo.get_by_id(task_id)
    if not task:
        raise HTTPException(status_code=404, detail=f"Task {task_id} not found")
    return task

@app.put("/tasks/{task_id}", response_model=TaskResponse)
def update_task(
    task_id: int,
    update: TaskUpdate,
    repo: TaskRepository = Depends(get_task_repo)
):
    if not update.title.strip():
        raise HTTPException(status_code=400, detail="Title cannot be empty")
    if update.status and update.status not in VALID_STATUSES:
        raise HTTPException(status_code=400, detail=f"Invalid status: {update.status}")
    task = repo.update(task_id, update)
    if not task:
        raise HTTPException(status_code=404, detail=f"Task {task_id} not found")
    return task

@app.delete("/tasks/{task_id}")
def delete_task(task_id: int, repo: TaskRepository = Depends(get_task_repo)):
    if not repo.delete(task_id):
        raise HTTPException(status_code=404, detail=f"Task {task_id} not found")
    return {"message": "Task deleted", "id": task_id}

# --- Agent Endpoints ---

def build_task_context(task: dict) -> str:
    return f"""Task Context:
- ID: {task['id']}
- Title: {task['title']}
- Description: {task.get('description') or 'No description'}
- Status: {task['status']}"""

async def run_agent_streaming(agent, task: dict, question: str):
    """Generator for streaming agent responses."""
    context = build_task_context(task)

    yield {"event": "start", "data": json.dumps({
        "task_id": task["id"],
        "agent": agent.name
    })}

    runner = Runner()
    current_agent = agent.name

    async for event in runner.stream(
        agent,
        messages=[{"role": "user", "content": f"{context}\n\nQuestion: {question}"}]
    ):
        if event.type == "agent_start":
            current_agent = event.agent_name
            yield {"event": "handoff", "data": json.dumps({"to": current_agent})}
        elif event.type == "text_delta":
            yield {"event": "token", "data": event.delta}
        elif event.type == "tool_call_start":
            yield {"event": "tool_call", "data": json.dumps({
                "agent": current_agent,
                "tool": event.tool_name,
                "args": event.arguments
            })}
        elif event.type == "tool_call_result":
            yield {"event": "tool_result", "data": json.dumps(event.result)}

    yield {"event": "complete", "data": json.dumps({"final_agent": current_agent})}

@app.post("/tasks/{task_id}/help")
async def triage_help(
    task_id: int,
    request: HelpRequest,
    repo: TaskRepository = Depends(get_task_repo)
):
    """Get help with a task. Automatically routes to the right specialist."""
    task = repo.get_by_id(task_id)
    if not task:
        raise HTTPException(status_code=404, detail=f"Task {task_id} not found")
    return EventSourceResponse(run_agent_streaming(triage_agent, task, request.question))

@app.post("/tasks/{task_id}/schedule")
async def schedule_help(
    task_id: int,
    request: HelpRequest,
    repo: TaskRepository = Depends(get_task_repo)
):
    """Direct access to scheduling specialist."""
    task = repo.get_by_id(task_id)
    if not task:
        raise HTTPException(status_code=404, detail=f"Task {task_id} not found")
    return EventSourceResponse(run_agent_streaming(scheduler_agent, task, request.question))

@app.post("/tasks/{task_id}/collaborate")
async def collaborate_help(
    task_id: int,
    request: HelpRequest,
    repo: TaskRepository = Depends(get_task_repo)
):
    """Direct access to collaboration specialist."""
    task = repo.get_by_id(task_id)
    if not task:
        raise HTTPException(status_code=404, detail=f"Task {task_id} not found")
    return EventSourceResponse(run_agent_streaming(collaboration_agent, task, request.question))

# --- System Endpoints ---

@app.get("/agents/status")
def get_agent_status():
    """List available agents and their capabilities."""
    return {
        "agents": [
            {"name": "triage", "description": "Routes requests to specialists", "type": "router"},
            {"name": "scheduler", "description": "Deadlines, reminders, time blocking", "type": "specialist"},
            {"name": "collaboration", "description": "Delegation, sharing, meetings", "type": "specialist"}
        ]
    }

@app.get("/")
def health():
    return {
        "service": "Agent-Powered Task Service",
        "version": "1.0.0",
        "status": "healthy"
    }
```

## Testing Checklist

Run your service and verify each feature. Don't mark items complete until you've actually tested them:

### CRUD Operations
- [ ] POST /tasks creates task with 201
- [ ] POST /tasks with empty title returns 400
- [ ] GET /tasks returns all tasks
- [ ] GET /tasks?status_filter=pending filters correctly
- [ ] GET /tasks?status_filter=invalid returns 400
- [ ] GET /tasks/1 returns task
- [ ] GET /tasks/999 returns 404
- [ ] PUT /tasks/1 updates task
- [ ] PUT /tasks/1 with empty title returns 400
- [ ] PUT /tasks/1 with invalid status returns 400
- [ ] DELETE /tasks/1 removes task
- [ ] DELETE /tasks/999 returns 404

### Agent Endpoints
- [ ] POST /tasks/1/help streams responses
- [ ] Scheduling questions route to scheduler (check handoff event)
- [ ] Collaboration questions route to collaboration
- [ ] Tool calls appear in stream
- [ ] POST /tasks/1/schedule goes directly to scheduler (no handoff event)
- [ ] POST /tasks/1/collaborate goes directly to collaboration
- [ ] POST /tasks/999/help returns 404 (before streaming starts)

### System Endpoints
- [ ] GET /agents/status lists all three agents
- [ ] GET / returns health check with version

## Reflection Questions

After completing the capstone, reflect on what you've learned:

1. **What was hardest?** Which lesson's concepts took most work to integrate?

2. **What would you change?** If you were designing this API from scratch, what would you do differently?

3. **What's missing for production?** This is a learning project. What would a real production system need that this doesn't have? (Think: persistence, auth, rate limiting, observability...)

4. **How would you test this?** If you had to write automated tests, where would you start? What would be hardest to test?

## Extend With AI

After completing the base capstone, explore these extensions:

**Add Authentication:**

> "I need to add API key authentication to my endpoints. Show me how to use FastAPI's security dependencies to protect the agent endpoints while leaving CRUD endpoints open."

**Implement Rate Limiting:**

> "My agent endpoints are expensive—they call OpenAI. How do I rate limit them to 10 requests per minute per API key? Show me using slowapi or a custom dependency."

**Add Persistence:**

> "The in-memory repository loses data on restart. How do I swap it for SQLite without changing any endpoint code? Show me the repository pattern in action."

**Add Observability:**

> "I want to log every agent request with timing, routing path, and tool calls. Show me how to add structured logging with request correlation IDs."

These are real production concerns. Tackling them with AI guidance is exactly how you'd work in a professional setting.

---

## What You've Built

You've built a production-style multi-agent API:

- **5 CRUD endpoints** with proper validation and error handling
- **3 agent endpoints** with streaming and routing visibility
- **Multi-agent architecture** with triage and specialists
- **Production patterns** throughout: dependency injection, Pydantic validation, SSE streaming

**The bigger picture**: This is the foundation for real AI-powered applications. The same patterns—CRUD for data, streaming for responses, routing for intelligence—appear in ChatGPT, Claude, and every enterprise AI product.

In Part 7, you'll take this further: containerization with Docker, deployment to Kubernetes, and scaling for production traffic.

---

## Chapter Summary

Chapter 40 taught you to expose AI agents via FastAPI:

| Lesson | Core Skill |
|--------|-----------|
| 1 | FastAPI basics, async endpoints, Swagger UI |
| 2 | Pydantic models for request/response validation |
| 3 | CRUD operations with HTTP methods |
| 4 | Error handling with status codes |
| 5 | Dependency injection for testability |
| 6 | SSE streaming with async generators |
| 7 | Multi-agent routing with handoffs |
| 8 | Integration of all patterns |

You can now build HTTP APIs that wrap AI agents—making them accessible to any client through standard REST endpoints. This is how AI gets into production.
