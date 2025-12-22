---
title: "Streaming with SSE"
sidebar_position: 6
chapter: 40
lesson: 6
duration_minutes: 55

skills:
  - name: "Server-Sent Events"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student implements SSE endpoint with EventSourceResponse"

  - name: "Async Generators"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student creates async generator for streaming data"

  - name: "Real-time Data Patterns"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student explains when to use streaming vs polling"

learning_objectives:
  - objective: "Implement streaming responses using Server-Sent Events"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "SSE endpoint sends events that browser receives"

  - objective: "Create async generators for producing stream data"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Generator yields events with proper format"

  - objective: "Test streaming endpoints in browser"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student uses EventSource API to receive events"

cognitive_load:
  new_concepts: 4
  assessment: "SSE format, EventSourceResponse, async generators, browser EventSource"

differentiation:
  extension_for_advanced: "Handle connection errors; implement heartbeat/keepalive"
  remedial_for_struggling: "Focus on single-event stream before adding complexity"

generated_by: "content-implementer"
source_spec: "specs/040-chapter-40-fastapi-for-agents/spec.md"
created: "2025-12-22"
---

# Streaming with SSE

Some operations take time. When an AI agent generates a response, you don't want users staring at a loading spinner for 30 seconds. Streaming sends data as it becomes available—token by token for LLMs, update by update for long-running tasks.

This is the lesson that connects everything to agents. In Lesson 7, you'll stream actual agent responses. Here, you build the foundation with simulated data.

## Why Streaming Changes Everything

**Traditional request-response** (what you've built so far):
1. Client sends request
2. Server processes (30 seconds for agent response)
3. Client waits with no feedback...
4. Server sends complete response

**Streaming**:
1. Client sends request
2. Server starts processing
3. Server sends first token immediately
4. Server sends more tokens as available
5. Client sees response forming in real-time

You've experienced this in ChatGPT—words appearing as the model generates them. That's streaming.

**For agents, streaming means**:
- Users see responses forming, not waiting
- Long tool calls show progress
- Failed operations fail fast, not after timeout
- Better perceived performance (first byte matters)

## How SSE Works (Under the Hood)

Server-Sent Events is a simple protocol. The server sends text in a specific format:

```
event: task_update
data: {"task_id": 1, "status": "in_progress"}

event: task_update
data: {"task_id": 1, "status": "completed"}
```

Each event has:
- `event`: Event type (optional, defaults to "message")
- `data`: The payload (must be a string, usually JSON)
- Blank line: Separates events

**Why SSE over WebSockets?**
- SSE is simpler—just HTTP with a special content type
- Works through proxies and load balancers without configuration
- Browser handles reconnection automatically
- One-directional (server → client) which is exactly what streaming needs

WebSockets are bidirectional, which adds complexity you don't need for agent responses.

## Installing sse-starlette

FastAPI doesn't include SSE by default. Add the package:

```bash
uv add sse-starlette
```

This provides `EventSourceResponse`, which handles SSE formatting automatically.

## Your First Streaming Endpoint

```python
from fastapi import FastAPI
from sse_starlette.sse import EventSourceResponse
import asyncio
import json

app = FastAPI(title="Task API")

async def task_updates_generator():
    """Simulates task status updates over time."""
    for i in range(5):
        yield {
            "event": "task_update",
            "data": json.dumps({
                "task_id": i + 1,
                "status": "processing",
                "progress": (i + 1) * 20
            })
        }
        await asyncio.sleep(1)  # Simulate work

    yield {
        "event": "complete",
        "data": json.dumps({"message": "All tasks processed"})
    }

@app.get("/tasks/stream")
async def stream_task_updates():
    return EventSourceResponse(task_updates_generator())
```

**Breaking this down**:

1. **`async def` with `yield`** creates an **async generator**—a function that produces values over time
2. **Each `yield`** sends one SSE event to the client
3. **`await asyncio.sleep(1)`** simulates work (in Lesson 7, this is where the agent generates tokens)
4. **`EventSourceResponse`** wraps the generator and handles SSE formatting

**The key insight**: `yield` doesn't end the function. It pauses, sends data, then continues. This is fundamentally different from `return`.

## Why Async Generators Matter for Agents

In Lesson 7, you'll stream agent responses like this:

```python
async def agent_response_generator(message: str):
    result = runner.run_streamed(agent, messages=[{"role": "user", "content": message}])
    async for event in result.stream_events():
        if event.type == "raw_response_event":
            yield {
                "event": "token",
                "data": json.dumps({"content": event.data})
            }
```

The pattern is identical:
- Async generator yields data
- `EventSourceResponse` sends it
- Client receives tokens as they generate

Master the pattern here with simulated data. Lesson 7 plugs in the real agent.

## Testing in Browser

Swagger UI doesn't work for SSE—it expects regular responses. Open your browser's console:

```javascript
const source = new EventSource('http://localhost:8000/tasks/stream');

source.onmessage = (event) => {
    console.log('Message:', event.data);
};

source.addEventListener('task_update', (event) => {
    console.log('Task update:', JSON.parse(event.data));
});

source.addEventListener('complete', (event) => {
    console.log('Complete:', JSON.parse(event.data));
    source.close();
});

source.onerror = (error) => {
    console.error('Error:', error);
    source.close();
};
```

You'll see events arriving one second apart.

**Important**: The browser automatically reconnects if the connection drops. That's a feature of `EventSource`. For agent responses, you might want to disable this (handled in the client code).

## Streaming with Context

Let's add streaming that relates to a specific task:

```python
from fastapi import Depends, HTTPException, status
from repository import TaskRepository, get_task_repo

async def task_progress_generator(task_id: int, task_title: str):
    """Streams progress updates for a specific task."""
    steps = [
        "Analyzing task...",
        "Processing requirements...",
        "Generating output...",
        "Validating results...",
        "Finalizing...",
    ]

    for i, step in enumerate(steps, 1):
        yield {
            "event": "progress",
            "data": json.dumps({
                "task_id": task_id,
                "task_title": task_title,
                "step": i,
                "total_steps": len(steps),
                "message": step,
                "percentage": int((i / len(steps)) * 100)
            })
        }
        await asyncio.sleep(0.8)

    yield {
        "event": "complete",
        "data": json.dumps({
            "task_id": task_id,
            "status": "completed"
        })
    }

@app.post("/tasks/{task_id}/execute")
async def execute_task(
    task_id: int,
    repo: TaskRepository = Depends(get_task_repo)
):
    # Verify task exists before streaming
    task = repo.get_by_id(task_id)
    if not task:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Task with id {task_id} not found"
        )

    return EventSourceResponse(
        task_progress_generator(task_id, task["title"])
    )
```

**Notice the pattern**:
1. Validate input BEFORE returning the stream
2. Pass context (task_id, task_title) to the generator
3. Generator doesn't need to access the repository—it just yields data

This matters for agents: you'll validate the conversation exists, then stream the response.

## Error Handling in Streams

What happens when an error occurs mid-stream? The client has already received some data.

```python
async def risky_generator():
    try:
        for i in range(10):
            if i == 5:
                raise ValueError("Something went wrong at step 5!")
            yield {
                "event": "step",
                "data": json.dumps({"step": i})
            }
            await asyncio.sleep(0.5)
    except Exception as e:
        # Send error as an event, don't raise
        yield {
            "event": "error",
            "data": json.dumps({"error": str(e)})
        }
```

**The key insight**: Once streaming starts, you can't change the HTTP status code. It's already been sent as 200. So you send an error EVENT, and the client handles it.

For agents, this means:
- Agent starts generating
- Tool call fails mid-response
- Stream an error event
- Client shows error in the UI

## The Complete Streaming Example

```python
from fastapi import FastAPI, Depends, HTTPException, status
from sse_starlette.sse import EventSourceResponse
import asyncio
import json

from repository import TaskRepository, get_task_repo

app = FastAPI(title="Task API")

# Stream 1: System-wide updates
async def system_updates_generator():
    """Simulates system-wide events."""
    events = [
        ("info", {"message": "System started"}),
        ("task_created", {"task_id": 1}),
        ("task_updated", {"task_id": 1, "status": "in_progress"}),
        ("task_completed", {"task_id": 1}),
        ("info", {"message": "Batch complete"}),
    ]

    for event_type, data in events:
        yield {
            "event": event_type,
            "data": json.dumps(data)
        }
        await asyncio.sleep(1)

@app.get("/stream/system")
async def stream_system_updates():
    return EventSourceResponse(system_updates_generator())

# Stream 2: Task-specific progress
async def task_work_generator(task_id: int, task_title: str):
    """Simulates work on a specific task."""
    steps = [
        "Starting task...",
        "Analyzing requirements...",
        "Processing data...",
        "Generating output...",
        "Finalizing...",
    ]

    for i, step in enumerate(steps, 1):
        yield {
            "event": "step",
            "data": json.dumps({
                "task_id": task_id,
                "task_title": task_title,
                "step": i,
                "message": step,
                "progress": int((i / len(steps)) * 100)
            })
        }
        await asyncio.sleep(0.8)

    yield {
        "event": "done",
        "data": json.dumps({
            "task_id": task_id,
            "message": "Task completed successfully"
        })
    }

@app.post("/tasks/{task_id}/execute")
async def execute_task(
    task_id: int,
    repo: TaskRepository = Depends(get_task_repo)
):
    task = repo.get_by_id(task_id)
    if not task:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Task {task_id} not found"
        )

    return EventSourceResponse(
        task_work_generator(task_id, task["title"])
    )

# Stream 3: Countdown (simple demo)
async def countdown_generator(seconds: int):
    """Simple countdown stream."""
    for i in range(seconds, 0, -1):
        yield {
            "event": "tick",
            "data": json.dumps({"remaining": i})
        }
        await asyncio.sleep(1)

    yield {
        "event": "complete",
        "data": json.dumps({"message": "Countdown finished!"})
    }

@app.get("/stream/countdown/{seconds}")
async def stream_countdown(seconds: int):
    if seconds < 1 or seconds > 60:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Seconds must be between 1 and 60"
        )
    return EventSourceResponse(countdown_generator(seconds))
```

## Hands-On Exercise

Build a streaming endpoint for task processing:

**Step 1**: Add sse-starlette to your project

```bash
uv add sse-starlette
```

**Step 2**: Create a streaming endpoint that validates the task exists first

**Step 3**: Test in browser console

```javascript
const source = new EventSource('http://localhost:8000/tasks/1/execute', {
    method: 'POST'  // Note: EventSource is GET-only by default
});
```

Wait—`EventSource` only supports GET! For POST, you need a different approach:

```javascript
// For POST endpoints, use fetch with streaming
async function streamTask(taskId) {
    const response = await fetch(`http://localhost:8000/tasks/${taskId}/execute`, {
        method: 'POST'
    });

    const reader = response.body.getReader();
    const decoder = new TextDecoder();

    while (true) {
        const {done, value} = await reader.read();
        if (done) break;
        console.log(decoder.decode(value));
    }
}

streamTask(1);
```

**Step 4**: Observe events arriving in real-time

## Challenge: Build a Progress Tracker

**Before looking at any solution**, design a streaming endpoint:

**The Problem**: Build an endpoint that simulates an AI agent "thinking":
- Starts with "Analyzing request..."
- Shows 3-5 intermediate "thoughts"
- Ends with a "conclusion"
- Takes about 5 seconds total

Think about:
- What events do you need? (thinking, thought, conclusion?)
- How do you structure the data for each event?
- How would a frontend render this progressively?

Implement it. Then compare with AI:

> "I built a thinking stream like this: [paste your code]. The frontend will need to render each thought in sequence. Is there a better event structure for progressive rendering?"

## Common Mistakes

**Mistake 1**: Forgetting to import json for data serialization

```python
# Wrong - data must be a string
yield {"data": {"task_id": 1}}

# Correct - serialize to JSON string
yield {"data": json.dumps({"task_id": 1})}
```

SSE data must be a string. If you pass a dict, you'll get errors.

**Mistake 2**: Not closing the connection on the client

```javascript
// Wrong - connection stays open forever
const source = new EventSource('/stream');

// Correct - close when done
source.addEventListener('complete', () => source.close());
```

Open connections consume server resources. Always close when done.

**Mistake 3**: Blocking the event loop

```python
# Wrong - blocks other requests
import time
time.sleep(1)  # This is synchronous!

# Correct - use async sleep
await asyncio.sleep(1)
```

Synchronous `time.sleep()` blocks the entire event loop. Other requests can't be processed. Always use `await asyncio.sleep()`.

**Mistake 4**: Returning instead of yielding

```python
# Wrong - sends nothing
async def generator():
    return {"data": "hello"}  # Not a generator!

# Correct - yield makes it a generator
async def generator():
    yield {"data": "hello"}
```

`return` ends the function. `yield` makes it a generator that produces values.

## Refine Your Understanding

After completing the exercise, work through these scenarios with AI:

**Scenario 1: Understand the Protocol**

> "Explain the SSE protocol format in detail. What are the optional fields besides 'event' and 'data'? How does the 'id' field enable reconnection?"

When AI explains, push back:

> "If my server crashes mid-stream and the client reconnects, how do I resume from where I left off? Show me the pattern for resumable streams."

**Scenario 2: Handle Real-World Issues**

> "What happens if the client disconnects mid-stream? My generator might keep running, wasting resources. How do I detect disconnection and clean up?"

Review AI's solution. Challenge it:

> "Your solution uses try/finally. But what if I have resources like database cursors that need cleanup? Show me the pattern for resource cleanup in async generators."

**Scenario 3: Compare Alternatives**

> "When should I use SSE vs WebSockets vs HTTP/2 server push? My agent needs to stream responses, but also handle user interrupts. Does SSE still work?"

This explores the trade-offs between streaming technologies—important for production agent systems.

---

## Summary

You've learned to implement streaming with SSE:

- **sse-starlette**: Provides `EventSourceResponse`
- **Async generators**: `async def` with `yield` for data production
- **Event format**: `event` and `data` fields
- **Browser testing**: Use `EventSource` API in JavaScript
- **Error handling**: Yield error events, don't raise exceptions mid-stream

**The bigger picture**: Streaming transforms user experience. Instead of waiting 30 seconds for a complete response, users see progress immediately. This is how modern AI interfaces feel responsive even when processing takes time.

Next lesson, you'll integrate an AI agent—sending user questions and streaming the agent's response token by token.
