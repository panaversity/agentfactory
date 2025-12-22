# Chapter 40: FastAPI for Agents — Specification (Revised)

**Version**: 2.0.0
**Status**: Draft
**Created**: 2025-12-22
**Revised**: 2025-12-22
**Author**: Claude Code (sp.orchestrator)

---

## 1. Executive Summary

### 1.1 Purpose
Chapter 40 teaches how to **expose agent patterns as production REST APIs**. Students have built agents with OpenAI SDK (Chapter 34), connected them via MCP (Chapters 37-38), and created reusable skills with code execution (Chapter 39). Now they'll make these agents accessible as HTTP services.

**This is NOT a generic FastAPI tutorial.** Students learn FastAPI fundamentals (Lessons 1-5) as necessary scaffolding, then immediately apply them to expose real agent patterns (Lessons 6-11).

### 1.2 Key Distinction
- **Chapters 34-39**: Build agents locally (CLI, SDK)
- **Chapter 40**: Expose agents as HTTP APIs (REST, streaming)

### 1.3 What Makes This Chapter Valuable
1. **Structured Agents in Endpoints** — Not chat completions, real agents with tools
2. **Multi-Agent Handoffs via API** — Triage → Specialist routing
3. **Skills-Guided Behavior** — SKILL.md files as agent system prompts
4. **Code Execution Integration** — Agents that run code via MCP
5. **Production Patterns** — Streaming, context persistence, error handling

---

## 2. Learning Objectives

### 2.1 Chapter-Level Objectives

| # | Objective | Bloom's Level | Assessment |
|---|-----------|---------------|------------|
| LO1 | Create FastAPI endpoints with Pydantic validation | Apply | Working CRUD API |
| LO2 | Expose OpenAI Agents SDK agents via REST | Apply | Agent endpoint with tools |
| LO3 | Implement multi-agent handoffs in API layer | Apply | Triage → Specialist routing |
| LO4 | Integrate SKILL.md files to guide agent behavior | Apply | Skill-loaded agent endpoint |
| LO5 | Connect code execution MCP servers to endpoints | Apply | Agent executes code via API |
| LO6 | Stream agent responses including tool calls | Apply | SSE stream with tool events |

---

## 3. Chapter Structure (11 Lessons)

### Tier A: FastAPI Fundamentals (Lessons 1-5)
Foundation for exposing agents. Uses Task Management domain.

| # | Title | Duration | Focus |
|---|-------|----------|-------|
| 01 | Hello FastAPI | 45 min | First app, uvicorn, Swagger UI |
| 02 | POST and Pydantic Models | 50 min | Request validation, models |
| 03 | Full CRUD Operations | 55 min | GET, PUT, DELETE patterns |
| 04 | Error Handling | 45 min | HTTPException, status codes |
| 05 | Dependency Injection | 50 min | Depends(), repository pattern |

### Tier B: Agent API Patterns (Lessons 6-10)
Core value: Exposing agent capabilities learned in Chapters 34-39.

| # | Title | Duration | Focus |
|---|-------|----------|-------|
| 06 | Streaming Responses | 45 min | SSE foundation for agent streaming |
| 07 | Agents with Tools in Endpoints | 60 min | OpenAI SDK agents, function tools |
| 08 | Multi-Agent Handoffs | 55 min | Triage → Specialist routing |
| 09 | Skills-Guided Agents | 50 min | SKILL.md as system prompts |
| 10 | Code Execution Agents | 55 min | MCP sandbox integration |

### Tier C: Capstone (Lesson 11)
| # | Title | Duration | Focus |
|---|-------|----------|-------|
| 11 | Capstone: Agent-Powered Task Service | 90 min | Multi-agent system with all patterns |

**Total Duration**: ~10 hours

---

## 4. Critical Lesson Specifications

### 4.1 Lesson 7: Agents with Tools in Endpoints

**Why This Matters**: This is where Chapter 40 diverges from generic FastAPI tutorials. Students learn to expose **structured agents** (not chat completions) via REST.

**Concepts from Chapter 34 to Apply**:
- `Agent` class with `instructions` and `tools`
- Function tool definitions with schemas
- Tool execution lifecycle
- Response streaming from agents

**Key Code Pattern**:
```python
from openai import OpenAI
from agents import Agent, Runner, function_tool
from fastapi import FastAPI, Depends
from sse_starlette.sse import EventSourceResponse

app = FastAPI()
client = OpenAI()

# Define a tool the agent can use
@function_tool
def create_subtask(parent_task_id: int, title: str, description: str) -> dict:
    """Create a subtask under a parent task."""
    # In real app, this writes to database
    return {"id": 101, "parent_id": parent_task_id, "title": title}

@function_tool
def set_reminder(task_id: int, remind_at: str) -> dict:
    """Set a reminder for a task."""
    return {"task_id": task_id, "reminder": remind_at}

# Agent with tools - this is what Chapter 34 taught
task_agent = Agent(
    name="task-assistant",
    instructions="""You help users manage their tasks effectively.
    You can create subtasks to break down complex work.
    You can set reminders for deadlines.
    Be concise and action-oriented.""",
    tools=[create_subtask, set_reminder],
    model="gpt-4o-mini"
)

@app.post("/tasks/{task_id}/agent")
async def agent_assist(task_id: int, question: str):
    """Invoke structured agent with tools for task help."""

    # Get task context
    task = get_task(task_id)

    # Run agent with context
    runner = Runner()
    result = await runner.run(
        task_agent,
        messages=[{
            "role": "user",
            "content": f"Task: {task['title']}\n\nQuestion: {question}"
        }]
    )

    return {
        "response": result.final_output,
        "tool_calls": [
            {"name": tc.name, "args": tc.arguments, "result": tc.result}
            for tc in result.tool_calls
        ]
    }

@app.post("/tasks/{task_id}/agent/stream")
async def stream_agent_assist(task_id: int, question: str):
    """Stream agent response including tool call events."""

    task = get_task(task_id)

    async def generate():
        runner = Runner()
        async for event in runner.stream(
            task_agent,
            messages=[{"role": "user", "content": f"Task: {task['title']}\n\n{question}"}]
        ):
            if event.type == "text_delta":
                yield {"event": "token", "data": event.delta}
            elif event.type == "tool_call":
                yield {"event": "tool_call", "data": json.dumps({
                    "name": event.tool_name,
                    "arguments": event.arguments
                })}
            elif event.type == "tool_result":
                yield {"event": "tool_result", "data": json.dumps(event.result)}

        yield {"event": "done", "data": ""}

    return EventSourceResponse(generate())
```

**What Students Learn**:
1. Agents are NOT just chat completions
2. Tools give agents capabilities (create subtasks, set reminders)
3. API returns tool calls so clients know what happened
4. Streaming includes tool events, not just text

---

### 4.2 Lesson 8: Multi-Agent Handoffs

**Why This Matters**: Enterprise agents use specialist handoffs. Students learn the API pattern for routing between agents.

**Concepts from Chapter 34 to Apply**:
- Triage agent that routes requests
- Specialist agents for specific domains
- Context preservation across handoffs
- Handoff as tool call

**Key Code Pattern**:
```python
from agents import Agent, handoff

# Specialist agents
scheduler_agent = Agent(
    name="scheduler",
    instructions="You help with scheduling, deadlines, and time management.",
    tools=[set_deadline, create_reminder, suggest_time_blocks],
    model="gpt-4o-mini"
)

collaboration_agent = Agent(
    name="collaboration",
    instructions="You help with delegation, team coordination, and sharing.",
    tools=[assign_to_user, share_task, create_meeting],
    model="gpt-4o-mini"
)

# Triage agent with handoffs
triage_agent = Agent(
    name="triage",
    instructions="""You route task questions to the right specialist:
    - Scheduling/deadlines/time → scheduler
    - Delegation/teams/sharing → collaboration
    - General questions → answer directly""",
    tools=[
        handoff(scheduler_agent),
        handoff(collaboration_agent)
    ],
    model="gpt-4o-mini"
)

@app.post("/tasks/{task_id}/help")
async def triage_task_help(task_id: int, question: str):
    """Triage endpoint - routes to specialist agents."""

    task = get_task(task_id)
    runner = Runner()

    result = await runner.run(
        triage_agent,
        messages=[{"role": "user", "content": f"Task: {task['title']}\n\n{question}"}]
    )

    return {
        "response": result.final_output,
        "handled_by": result.agent_name,  # Which agent answered
        "handoff_chain": [a.name for a in result.agents_used],
        "tool_calls": [tc.to_dict() for tc in result.tool_calls]
    }
```

**What Students Learn**:
1. Triage pattern from Chapter 34 in API context
2. Response includes which agent handled the request
3. Handoff chain shows routing path
4. Client can understand agent decisions

---

### 4.3 Lesson 9: Skills-Guided Agents

**Why This Matters**: Chapter 39 taught SKILL.md files. Now students use them as agent system prompts for consistent behavior.

**Concepts from Chapter 39 to Apply**:
- SKILL.md structure (persona, questions, principles)
- Loading skills at runtime
- Skill composition for complex agents

**Key Code Pattern**:
```python
import yaml
from pathlib import Path

def load_skill(skill_name: str) -> str:
    """Load a SKILL.md file as agent instructions."""
    skill_path = Path(f".claude/skills/{skill_name}/SKILL.md")
    content = skill_path.read_text()

    # Parse YAML frontmatter
    if content.startswith("---"):
        parts = content.split("---", 2)
        metadata = yaml.safe_load(parts[1])
        body = parts[2].strip()
        return body
    return content

def compose_skills(*skill_names: str) -> str:
    """Compose multiple skills into one instruction set."""
    instructions = []
    for name in skill_names:
        skill_content = load_skill(name)
        instructions.append(f"## {name.title()} Expertise\n{skill_content}")
    return "\n\n".join(instructions)

# Skill-guided agent
@app.post("/tasks/{task_id}/skilled-assist")
async def skilled_assist(
    task_id: int,
    question: str,
    skills: list[str] = ["task-completion", "productivity"]
):
    """Agent guided by loaded skills for consistent behavior."""

    task = get_task(task_id)

    # Compose skills into instructions
    skill_instructions = compose_skills(*skills)

    # Create agent with skill-based instructions
    skilled_agent = Agent(
        name="skilled-task-assistant",
        instructions=f"""{skill_instructions}

Current Task Context:
- Title: {task['title']}
- Description: {task.get('description', 'None')}
- Status: {task['status']}

Apply your expertise to help with this task.""",
        tools=[create_subtask, set_reminder, mark_complete],
        model="gpt-4o-mini"
    )

    runner = Runner()
    result = await runner.run(
        skilled_agent,
        messages=[{"role": "user", "content": question}]
    )

    return {
        "response": result.final_output,
        "skills_applied": skills,
        "tool_calls": [tc.to_dict() for tc in result.tool_calls]
    }
```

**Example SKILL.md** (`.claude/skills/task-completion/SKILL.md`):
```markdown
---
name: task-completion
description: Expert guidance for completing tasks effectively
version: 1.0.0
---

You are a task completion expert who helps users finish their work efficiently.

## Questions to Consider
- What's the smallest next action?
- What's blocking progress?
- Can this be broken into subtasks?
- Is there a deadline that affects priority?

## Principles
1. Always identify the very next physical action
2. Break complex tasks into 15-30 minute chunks
3. Address blockers before pushing forward
4. Celebrate small completions to maintain momentum
```

**What Students Learn**:
1. Skills from Chapter 39 become agent instructions
2. Skills make behavior consistent across requests
3. Multiple skills can compose for complex agents
4. API response shows which skills were applied

---

### 4.4 Lesson 10: Code Execution Agents

**Why This Matters**: Chapter 39 taught MCP code execution. Now students create agents that write and execute code via API.

**Concepts from Chapter 39 to Apply**:
- MCP code execution servers (E2B, Code Interpreter)
- Write-execute-analyze loop
- Sandbox security boundaries

**Key Code Pattern**:
```python
from mcp import ClientSession
from agents import Agent, function_tool

# Connect to code execution MCP server
code_mcp = ClientSession("e2b-code-execution")

@function_tool
async def execute_python(code: str) -> dict:
    """Execute Python code in a sandboxed environment."""
    result = await code_mcp.call_tool(
        "execute",
        {"language": "python", "code": code}
    )
    return {
        "stdout": result.stdout,
        "stderr": result.stderr,
        "return_value": result.return_value
    }

@function_tool
async def analyze_data(data_description: str) -> dict:
    """Write and execute Python to analyze described data."""
    # Agent will write analysis code using this tool
    pass

# Code execution agent
analysis_agent = Agent(
    name="data-analyst",
    instructions="""You analyze task data by writing and executing Python code.

When asked to analyze something:
1. Write Python code to perform the analysis
2. Execute it using the execute_python tool
3. Interpret the results for the user

You have access to pandas, numpy, and matplotlib.""",
    tools=[execute_python],
    model="gpt-4o-mini"
)

@app.post("/tasks/analyze")
async def analyze_tasks(question: str, repo: TaskRepository = Depends(get_task_repo)):
    """Agent that writes and executes code to analyze tasks."""

    # Get all tasks as data context
    all_tasks = repo.get_all()
    task_data = json.dumps(all_tasks, indent=2)

    runner = Runner()
    result = await runner.run(
        analysis_agent,
        messages=[{
            "role": "user",
            "content": f"""Here is the task data:
```json
{task_data}
```

Question: {question}

Write Python code to analyze this data and answer the question."""
        }]
    )

    return {
        "analysis": result.final_output,
        "code_executed": [
            {"code": tc.arguments.get("code"), "result": tc.result}
            for tc in result.tool_calls
            if tc.name == "execute_python"
        ]
    }
```

**What Students Learn**:
1. MCP code execution from Chapter 39 in API context
2. Agent writes code, executes it, returns results
3. API response includes executed code for transparency
4. Sandbox keeps execution safe

---

### 4.5 Lesson 11: Capstone — Agent-Powered Task Service

**The Full System**:
Students build a complete multi-agent task service that combines ALL patterns:

1. **Triage Agent** — Routes requests to specialists
2. **Scheduler Specialist** — Handles deadlines, reminders (with tools)
3. **Collaboration Specialist** — Handles delegation, sharing (with tools)
4. **Analysis Specialist** — Writes and executes code for insights
5. **Skill Integration** — All agents use SKILL.md for consistent behavior
6. **Streaming** — All responses stream with tool call events

**API Endpoints**:
```
POST /tasks              - Create task
GET  /tasks              - List tasks
GET  /tasks/{id}         - Get task
PUT  /tasks/{id}         - Update task
DELETE /tasks/{id}       - Delete task

POST /tasks/{id}/help              - Triage → Specialist (streaming)
POST /tasks/{id}/schedule          - Direct to scheduler agent
POST /tasks/{id}/collaborate       - Direct to collaboration agent
POST /tasks/analyze                - Analysis agent with code execution

GET  /agents/status                - Which agents are available
GET  /skills                       - List loaded skills
```

---

## 5. Non-Goals

- **No authentication** — Security covered in Chapter 58
- **No databases** — In-memory storage, databases in Chapter 47
- **No Docker** — Containerization in Part 7
- **No testing framework** — TDD covered in Chapter 42

---

## 6. Success Criteria

### Content Success
- [ ] Lessons 1-5 provide FastAPI foundation
- [ ] Lesson 7 exposes structured agents with tools
- [ ] Lesson 8 demonstrates multi-agent handoffs
- [ ] Lesson 9 integrates SKILL.md files
- [ ] Lesson 10 connects code execution MCP
- [ ] Lesson 11 combines all patterns

### Student Outcomes
- [ ] Can create agents with tools in endpoints
- [ ] Can implement triage → specialist routing
- [ ] Can load and compose skills for agents
- [ ] Can connect code execution to agents
- [ ] Can stream agent responses with tool events

---

*Specification v2.0.0 - Revised to include agent patterns from Chapters 34 and 39*
