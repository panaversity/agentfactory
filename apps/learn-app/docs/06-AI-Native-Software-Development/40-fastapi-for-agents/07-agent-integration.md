---
title: "Agent Integration"
sidebar_position: 7
chapter: 40
lesson: 7
duration_minutes: 55

skills:
  - name: "Triage Agent Pattern"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student implements triage agent that routes to specialists"

  - name: "Agent Handoff API Design"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student exposes handoff chain in API response"

  - name: "Specialist Agent Composition"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student creates domain-specific specialist agents"

learning_objectives:
  - objective: "Implement triage agent that routes requests to specialists"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "API correctly routes scheduling vs collaboration requests"

  - objective: "Expose handoff metadata in API response"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Response includes handled_by and handoff_chain fields"

  - objective: "Create specialist agents with domain-specific tools"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Each specialist has appropriate tools for its domain"

cognitive_load:
  new_concepts: 3
  assessment: "handoff() function, triage pattern, response metadata for routing visibility"

differentiation:
  extension_for_advanced: "Add fallback agent for unroutable requests; implement confidence scoring"
  remedial_for_struggling: "Start with two agents before adding triage layer"

generated_by: "content-implementer"
source_spec: "specs/040-chapter-40-fastapi-for-agents/spec.md"
created: "2025-12-22"
---

# Agent Integration

This is the lesson everything has been building toward.

You've built FastAPI CRUD endpoints, handled errors, organized code with dependency injection, and implemented streaming. Now you'll expose AI agents via REST—the moment where all these patterns come together.

Real applications don't use one agent for everything. They use **specialists**: one agent for scheduling, another for collaboration, a third for analysis. This lesson teaches the **triage pattern**: a routing agent that hands off requests to the right specialist. You've seen this pattern in Chapter 34 (OpenAI Agents SDK). Now you'll expose it via FastAPI.

## Why Multiple Agents?

Consider a task management AI. Users ask:
- "When should I finish this task?" → **Scheduling domain**
- "Who should work on this?" → **Collaboration domain**
- "What's blocking progress?" → **Analysis domain**

One agent doing all three would need massive instructions, conflicting tools, and confused context. Instead:

```
User Request
     ↓
[Triage Agent] ← Understands intent, routes appropriately
     ↓
  ┌──┴──┬──────┐
  ↓     ↓      ↓
[Scheduler] [Collab] [Analysis]
```

The client sees one endpoint. The routing happens inside. This is the architecture of production AI systems.

## Creating Specialist Agents

Each specialist has focused instructions and domain-specific tools.

**Scheduler Specialist** — Handles time and deadlines:

```python
from agents import Agent, Runner, function_tool

@function_tool
def set_deadline(task_id: int, deadline: str) -> dict:
    """Set a deadline for a task. Format: YYYY-MM-DD"""
    return {"task_id": task_id, "deadline": deadline, "status": "deadline_set"}

@function_tool
def create_reminder(task_id: int, remind_at: str, message: str = "") -> dict:
    """Create a reminder for a task."""
    return {"task_id": task_id, "remind_at": remind_at, "message": message}

@function_tool
def suggest_time_blocks(task_id: int, duration_hours: int) -> dict:
    """Suggest available time blocks for focused work on a task."""
    return {
        "task_id": task_id,
        "suggested_blocks": [
            {"day": "Monday", "time": "09:00-11:00"},
            {"day": "Wednesday", "time": "14:00-16:00"}
        ]
    }

scheduler_agent = Agent(
    name="scheduler",
    instructions="""You are a scheduling specialist for task management.

Your expertise:
- Setting and adjusting deadlines
- Creating reminders at optimal times
- Suggesting focused work blocks
- Time management strategies

Be specific about dates and times. Consider workload when suggesting schedules.
When asked about deadlines, always set them using the set_deadline tool.""",
    tools=[set_deadline, create_reminder, suggest_time_blocks],
    model="gpt-4o-mini"
)
```

**Collaboration Specialist** — Handles people and teamwork:

```python
@function_tool
def assign_to_user(task_id: int, user_email: str, note: str = "") -> dict:
    """Assign a task to another user."""
    return {"task_id": task_id, "assigned_to": user_email, "note": note}

@function_tool
def share_task(task_id: int, user_emails: list[str], permission: str = "view") -> dict:
    """Share a task with other users. Permission: view or edit."""
    return {"task_id": task_id, "shared_with": user_emails, "permission": permission}

@function_tool
def create_meeting(task_id: int, attendees: list[str], duration_minutes: int = 30) -> dict:
    """Schedule a meeting about a task."""
    return {
        "task_id": task_id,
        "attendees": attendees,
        "duration": duration_minutes,
        "status": "meeting_scheduled"
    }

collaboration_agent = Agent(
    name="collaboration",
    instructions="""You are a collaboration specialist for task management.

Your expertise:
- Delegating tasks to team members
- Sharing work with appropriate permissions
- Scheduling meetings for task discussions
- Team coordination strategies

Always confirm who should be involved before taking action.
When delegating, use assign_to_user with a clear note about expectations.""",
    tools=[assign_to_user, share_task, create_meeting],
    model="gpt-4o-mini"
)
```

**Notice the design principle**: Each specialist has:
- **Focused instructions** — Clear about its domain boundaries
- **Domain-specific tools** — Only what it needs, nothing more
- **Distinct name** — For routing visibility in responses

## The Triage Agent with Handoffs

The triage agent uses `handoff()` to transfer control to specialists:

```python
from agents import Agent, handoff

triage_agent = Agent(
    name="triage",
    instructions="""You are a task management router. Your job is to understand
what the user needs and hand off to the right specialist.

Route to scheduler for:
- Deadlines, due dates, timing
- Reminders and notifications
- Time blocking and scheduling
- "When should I..." questions

Route to collaboration for:
- Delegation, assigning work
- Sharing with team members
- Meeting scheduling
- "Who should..." questions

For simple questions that don't need a specialist, answer directly.
When handing off, briefly explain why you're routing to that specialist.""",
    tools=[
        handoff(scheduler_agent),
        handoff(collaboration_agent)
    ],
    model="gpt-4o-mini"
)
```

**How `handoff()` works**: It wraps a specialist agent as a tool. When triage decides to route, it calls the handoff tool with context, and the specialist takes over completely. The original user message flows through, and the specialist's response becomes the final answer.

## The Handoff Endpoint

Now expose this through FastAPI. This is where everything connects:

```python
from fastapi import FastAPI, Depends, HTTPException, status
from pydantic import BaseModel

from repository import TaskRepository, get_task_repo

app = FastAPI(title="Multi-Agent Task API")

class HelpRequest(BaseModel):
    question: str

class ToolCallInfo(BaseModel):
    name: str
    arguments: dict
    result: dict | None = None

class HandoffResponse(BaseModel):
    task_id: int
    question: str
    response: str
    handled_by: str
    handoff_chain: list[str]
    tool_calls: list[ToolCallInfo]

@app.post("/tasks/{task_id}/help", response_model=HandoffResponse)
async def triage_task_help(
    task_id: int,
    request: HelpRequest,
    repo: TaskRepository = Depends(get_task_repo)
):
    """Get help with a task. Automatically routes to the right specialist."""

    # Get task context (remember: validate BEFORE expensive LLM calls)
    task = repo.get_by_id(task_id)
    if not task:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Task with id {task_id} not found"
        )

    # Build context message — the agent needs to know what task we're discussing
    context = f"""Task Context:
- ID: {task['id']}
- Title: {task['title']}
- Description: {task.get('description') or 'No description'}
- Status: {task['status']}

User Question: {request.question}"""

    # Run the triage agent
    runner = Runner()
    result = await runner.run(
        triage_agent,
        messages=[{"role": "user", "content": context}]
    )

    # Extract handoff information for transparency
    agents_used = [agent.name for agent in result.agents_used]
    final_agent = result.agent_name  # The agent that produced the final response

    return HandoffResponse(
        task_id=task_id,
        question=request.question,
        response=result.final_output,
        handled_by=final_agent,
        handoff_chain=agents_used,
        tool_calls=[
            ToolCallInfo(
                name=tc.name,
                arguments=tc.arguments,
                result=tc.result
            )
            for tc in result.tool_calls
        ]
    )
```

**What's happening here**:

1. **Validate early** — Check task exists before the expensive LLM call
2. **Build rich context** — The agent needs task details to give relevant advice
3. **Expose the routing** — `handoff_chain` shows which agents were involved
4. **Show tool calls** — Transparency into what actions were taken

This is the pattern from every lesson coming together: Pydantic models (Lesson 2), CRUD operations (Lesson 3), error handling (Lesson 4), dependency injection (Lesson 5), and async endpoints (Lesson 1).

## Testing the Handoff Endpoint

Start your server and test routing behavior:

```bash
uvicorn main:app --reload
```

**Scheduling request** (should route to scheduler):

```bash
curl -X POST "http://localhost:8000/tasks/1/help" \
  -H "Content-Type: application/json" \
  -d '{"question": "When should I set the deadline for this?"}'
```

Response shows the routing:

```json
{
  "task_id": 1,
  "question": "When should I set the deadline for this?",
  "response": "Based on the task complexity, I recommend setting the deadline for next Friday. I've set it using the deadline tool.",
  "handled_by": "scheduler",
  "handoff_chain": ["triage", "scheduler"],
  "tool_calls": [
    {
      "name": "set_deadline",
      "arguments": {"task_id": 1, "deadline": "2025-01-03"},
      "result": {"task_id": 1, "deadline": "2025-01-03", "status": "deadline_set"}
    }
  ]
}
```

**Collaboration request** (should route to collaboration):

```bash
curl -X POST "http://localhost:8000/tasks/1/help" \
  -H "Content-Type: application/json" \
  -d '{"question": "Who on my team should handle the frontend part?"}'
```

**Simple question** (triage handles directly):

```bash
curl -X POST "http://localhost:8000/tasks/1/help" \
  -H "Content-Type: application/json" \
  -d '{"question": "What is this task about?"}'
```

The `handoff_chain` field tells the story: `["triage"]` means triage answered directly, `["triage", "scheduler"]` means it routed to the scheduler.

## Streaming Handoffs

For real-time visibility into agent thinking and routing:

```python
from sse_starlette.sse import EventSourceResponse
import json

@app.post("/tasks/{task_id}/help/stream")
async def stream_triage_help(
    task_id: int,
    request: HelpRequest,
    repo: TaskRepository = Depends(get_task_repo)
):
    """Stream help response with routing visibility."""

    task = repo.get_by_id(task_id)
    if not task:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Task with id {task_id} not found"
        )

    context = f"Task: {task['title']}\n\nQuestion: {request.question}"

    async def generate():
        yield {
            "event": "start",
            "data": json.dumps({"task_id": task_id, "routing": "analyzing"})
        }

        runner = Runner()
        current_agent = None

        async for event in runner.stream(
            triage_agent,
            messages=[{"role": "user", "content": context}]
        ):
            # Detect agent switches — this is the handoff happening
            if event.type == "agent_start":
                current_agent = event.agent_name
                yield {
                    "event": "handoff",
                    "data": json.dumps({"to_agent": current_agent})
                }

            elif event.type == "text_delta":
                yield {
                    "event": "token",
                    "data": event.delta
                }

            elif event.type == "tool_call":
                yield {
                    "event": "tool_call",
                    "data": json.dumps({
                        "agent": current_agent,
                        "tool": event.tool_name,
                        "arguments": event.arguments
                    })
                }

            elif event.type == "tool_result":
                yield {
                    "event": "tool_result",
                    "data": json.dumps(event.result)
                }

        yield {
            "event": "complete",
            "data": json.dumps({"final_agent": current_agent})
        }

    return EventSourceResponse(generate())
```

The stream shows routing decisions as they happen:

```
event: start
data: {"task_id": 1, "routing": "analyzing"}

event: handoff
data: {"to_agent": "scheduler"}

event: tool_call
data: {"agent": "scheduler", "tool": "set_deadline", "arguments": {...}}

event: tool_result
data: {"task_id": 1, "deadline": "2025-01-03", "status": "deadline_set"}

event: token
data: I've

event: token
data:  set

event: token
data:  the...

event: complete
data: {"final_agent": "scheduler"}
```

A frontend can use these events to show:
- "Analyzing your request..."
- "Routing to scheduling specialist..."
- "Setting deadline..."
- Response text appearing token by token

## Direct Specialist Endpoints

Sometimes you want to bypass triage and go straight to a specialist:

```python
@app.post("/tasks/{task_id}/schedule")
async def direct_schedule_help(
    task_id: int,
    request: HelpRequest,
    repo: TaskRepository = Depends(get_task_repo)
):
    """Direct access to scheduling specialist."""

    task = repo.get_by_id(task_id)
    if not task:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Task with id {task_id} not found"
        )

    runner = Runner()
    result = await runner.run(
        scheduler_agent,  # Direct to specialist, bypassing triage
        messages=[{
            "role": "user",
            "content": f"Task: {task['title']}\n\nQuestion: {request.question}"
        }]
    )

    return {
        "task_id": task_id,
        "response": result.final_output,
        "handled_by": "scheduler",
        "tool_calls": [tc.to_dict() for tc in result.tool_calls]
    }
```

Now your API has three paths:
- `POST /tasks/{id}/help` — Triage routes automatically
- `POST /tasks/{id}/schedule` — Direct to scheduler
- `POST /tasks/{id}/collaborate` — Direct to collaboration

## Hands-On Exercise

Implement the full multi-agent system:

**Step 1**: Create the specialist agents with their tools

**Step 2**: Create the triage agent with handoffs

**Step 3**: Implement the `/tasks/{id}/help` endpoint

**Step 4**: Test routing with different question types:
- "Set a deadline for next week" → scheduler
- "Share this with the design team" → collaboration
- "What's the status of this task?" → triage (direct answer)

**Step 5**: Verify `handoff_chain` in responses shows correct routing

## Challenge: Add a Third Specialist

**Before looking at any solution**, design a third specialist yourself:

**The Problem**: Add an "analysis" specialist that can:
- Estimate completion time based on task complexity
- Identify potential blockers
- Suggest task breakdown into subtasks

Think about:
- What tools does this specialist need?
- How do you write instructions that clearly define its domain?
- How do you update triage to route analytical questions correctly?

Implement it. Test with questions like "How long will this take?" and "What might block progress?" Then compare with AI:

> "I added an analysis specialist like this: [paste your code]. The triage routes to it for [these patterns]. Is there overlap with my other specialists? How would you clarify the domain boundaries?"

## Common Mistakes

**Mistake 1**: Not passing context through handoffs

```python
# Wrong - specialist doesn't know about the task
result = await runner.run(
    triage_agent,
    messages=[{"role": "user", "content": request.question}]  # No task context!
)

# Correct - include full context
result = await runner.run(
    triage_agent,
    messages=[{"role": "user", "content": f"Task: {task['title']}\n\n{request.question}"}]
)
```

The specialist receives the same context triage did. Without context, it can't give relevant advice.

**Mistake 2**: Overlapping specialist domains

```python
# Wrong - both handle "meetings"
scheduler_agent = Agent(tools=[schedule_meeting, ...])  # Overlap!
collaboration_agent = Agent(tools=[create_meeting, ...])  # Overlap!

# Correct - clear domain boundaries
scheduler_agent = Agent(tools=[set_deadline, create_reminder, ...])  # Time-focused
collaboration_agent = Agent(tools=[assign_task, share_task, create_meeting, ...])  # People-focused
```

Overlapping tools confuse triage and lead to inconsistent routing.

**Mistake 3**: Missing routing visibility

```python
# Wrong - client can't see what happened
return {"response": result.final_output}

# Correct - full transparency
return {
    "response": result.final_output,
    "handled_by": result.agent_name,
    "handoff_chain": [a.name for a in result.agents_used],
    "tool_calls": [...]
}
```

Without visibility, debugging is impossible and users can't understand why they got certain answers.

## Refine Your Understanding

After completing the exercise, work through these scenarios with AI:

**Scenario 1: Handle Ambiguous Requests**

> "What if a user asks 'Set up a meeting with John for next Tuesday about the deadline'? This involves both scheduling (the time) and collaboration (John). How should triage handle requests that span multiple specialists?"

When AI suggests an approach, push back:

> "Your solution uses sequential handoffs. But what if the specialists give conflicting advice? Show me how to handle multi-specialist coordination."

**Scenario 2: Implement Confidence Scoring**

> "Modify the triage agent to include a confidence score with each routing decision. If confidence is below 70%, ask the user for clarification instead of routing."

Review AI's solution. Challenge it:

> "How do you measure confidence? Is it based on instruction pattern matching or something else? What if triage is 65% confident about scheduler and 60% confident about collaboration?"

**Scenario 3: Design for Scale**

> "I have 10 specialist agents now. How do I organize triage instructions so they don't become unmanageable? Show me patterns for scalable multi-agent systems."

This explores production architecture—systems with dozens of specialists need different patterns than the two-specialist demo.

---

## Summary

You've built a multi-agent system exposed via FastAPI:

- **Specialist agents** with domain-specific tools and focused instructions
- **Triage agent** using `handoff()` to route requests intelligently
- **API response** includes `handled_by` and `handoff_chain` for transparency
- **Streaming** shows routing decisions and tool calls in real-time
- **Direct endpoints** bypass triage when you know which specialist to use

**The bigger picture**: This is how production AI systems work. ChatGPT, Claude, and enterprise AI applications all use routing and specialists. You've built the same pattern—accessible via standard REST endpoints that any client can consume.

Next lesson, you'll combine everything into a production-ready capstone project.
