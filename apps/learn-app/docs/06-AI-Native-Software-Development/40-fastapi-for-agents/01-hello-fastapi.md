---
title: "Hello FastAPI"
sidebar_position: 1
chapter: 40
lesson: 1
duration_minutes: 45

skills:
  - name: "Creating FastAPI Applications"
    proficiency_level: "A2"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student creates working FastAPI app with uvicorn"

  - name: "Path and Query Parameters"
    proficiency_level: "A2"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student implements endpoints with path and query params"

  - name: "Using Swagger UI"
    proficiency_level: "A2"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student tests API endpoints using Swagger UI"

learning_objectives:
  - objective: "Create a FastAPI application and run it with uvicorn"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Application runs and responds at http://localhost:8000"

  - objective: "Implement endpoints with path and query parameters"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Endpoints accept parameters and return correct JSON"

  - objective: "Use Swagger UI to explore and test API endpoints"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student successfully uses 'Try it out' to test endpoints"

cognitive_load:
  new_concepts: 5
  assessment: "FastAPI instance, decorators, path params, query params, Swagger UI - within A2 limit"

differentiation:
  extension_for_advanced: "Add multiple endpoints with different HTTP methods; explore the OpenAPI JSON"
  remedial_for_struggling: "Focus on single GET endpoint before adding parameters"

generated_by: "content-implementer"
source_spec: "specs/040-chapter-40-fastapi-for-agents/spec.md"
created: "2025-12-22"
---

# Hello FastAPI

You've built MCP servers with HTTP transports. You've seen how tools communicate over HTTP. Now you'll learn FastAPI—the Python framework that makes building HTTP APIs straightforward and enjoyable.

By the end of this lesson, you'll have a running API with endpoints you can test in your browser. More importantly, you'll understand *why* FastAPI works the way it does—knowledge you'll need when exposing AI agents as services in Lesson 7.

## Why FastAPI for Agents?

In Chapter 38, you built MCP servers that communicate over HTTP. Those servers handle specific MCP protocol messages. FastAPI takes the same HTTP concepts and makes them general-purpose—you can build any kind of API.

But here's the deeper reason: **agents need HTTP interfaces**.

When you build an agent with the OpenAI Agents SDK (Chapter 34), it runs in your Python process. But how do other systems call it? How does a web app trigger your agent? How do multiple users share the same agent?

The answer is HTTP. FastAPI lets you wrap your agents in REST endpoints so any client—browser, mobile app, another service—can send requests and receive responses. Lesson 7 shows exactly how. This lesson builds the foundation.

FastAPI stands out for three reasons:

**1. Automatic Documentation** — Every endpoint you create appears in an interactive Swagger UI. When you expose agents, clients can explore your API without reading code.

**2. Type-Safe Validation** — FastAPI uses Python type hints to validate incoming data. When an agent endpoint receives malformed JSON, FastAPI rejects it before your code runs.

**3. Async-First** — Built on Starlette, FastAPI handles async/await natively. This matters because `Runner.run()` from the Agents SDK is async. Your endpoints need to be async too.

## Creating Your First Application

Let's build a Task API step by step. This same structure will later wrap your agents.

### Step 1: Create a Project with UV

```bash
uv init task-api
cd task-api
```

This creates a proper Python project with `pyproject.toml` for dependency management.

### Step 2: Create and Activate the Virtual Environment

```bash
uv venv
```

Activate the virtual environment:

::::os-tabs

::macos
```bash
source .venv/bin/activate
```

::linux
```bash
source .venv/bin/activate
```

::windows
```powershell
.venv\Scripts\activate
```

::::

**Note**: With recent Python versions, `uv` commands work without manual activation, but activating ensures your shell recognizes project dependencies.

### Step 3: Add Dependencies

```bash
uv add "fastapi[standard]"
```

**What does `[standard]` include?** The `fastapi[standard]` package bundles essential dependencies:

- **fastapi**: The web framework itself
- **uvicorn**: The ASGI server that runs your app (more on this below)
- **httpx**: An HTTP client useful for testing endpoints

For testing (which we'll cover later), add development dependencies:

```bash
uv add --dev pytest pytest-asyncio
```

This updates your `pyproject.toml`:

```toml
[project]
name = "task-api"
version = "0.1.0"
requires-python = ">=3.13"
dependencies = [
    "fastapi[standard]>=0.115.0"
]

[dependency-groups]
dev = [
    "pytest>=8.3.0",
    "pytest-asyncio>=0.24.0",
]
```

### What is Uvicorn and Why Do We Need It?

FastAPI doesn't run by itself—it needs a **server** to handle incoming HTTP requests.

**Uvicorn** is an ASGI (Asynchronous Server Gateway Interface) server. Think of it as the bridge:

```
Browser Request → Uvicorn → FastAPI → Your Code → FastAPI → Uvicorn → Browser Response
```

- **Uvicorn** handles the networking: listening on ports, accepting connections, parsing HTTP
- **FastAPI** handles the application logic: routing, validation, your code

You never interact with uvicorn directly in code—it just runs your FastAPI app. But understanding this separation matters: in production, you might swap uvicorn for another ASGI server (like hypercorn) without changing your FastAPI code.

### Step 4: Create Your First Endpoint

Create `main.py`:

```python
from fastapi import FastAPI

app = FastAPI(
    title="Task API",
    description="A simple task management API"
)

@app.get("/")
def read_root():
    return {"message": "Task API is running"}
```

That's a complete FastAPI application. Let's break it down:

- `FastAPI()` creates the application instance. The `title` and `description` appear in the auto-generated documentation.
- `@app.get("/")` is a **decorator** that tells FastAPI "when someone makes a GET request to `/`, call this function."
- The function returns a dictionary, which FastAPI automatically converts to JSON.

### Step 5: Run Your Application

You have two options for running the server in development:

**Option A: FastAPI CLI (Recommended for development)**

```bash
fastapi dev main.py
```

This command (introduced in FastAPI 0.100.0+) runs your app in development mode with automatic reloading. It's the simplest approach.

**Option B: Uvicorn directly**

```bash
uv run uvicorn main:app --reload
```

- `main:app` means "the `app` object in `main.py`"
- `--reload` restarts the server when you change code

**When to use which?**
- Use `fastapi dev` for quick development—it handles defaults for you
- Use `uvicorn` directly when you need control over host/port:
  ```bash
  uv run uvicorn main:app --host 0.0.0.0 --port 8000 --reload
  ```

**Troubleshooting**: If `fastapi dev` fails, ensure FastAPI isn't installed globally (conflicting with your virtual environment). The explicit `uv run uvicorn` command always works.

Open http://localhost:8000 in your browser. You'll see:

```json
{"message": "Task API is running"}
```

Your API is live.

## What Just Happened?

You typed ~10 lines of Python. The FastAPI + Uvicorn combination gave you:

- **A running web server** — Uvicorn listens on port 8000, handles connections, and routes requests to FastAPI
- **JSON serialization** — Your Python dict → JSON automatically
- **Interactive documentation** — Swagger UI at /docs
- **Request validation** — Try /tasks/abc later → automatic 422 error
- **OpenAPI spec generation** — View at /openapi.json
- **Auto-reload** — Change code, server restarts automatically

You didn't write networking code, serialization, or documentation. This is why FastAPI is called "batteries included."

This matters for agents: when you expose agent capabilities (Lesson 7), you'll get all this infrastructure automatically. Clients will see your agent's endpoints in Swagger UI, send validated requests, and receive JSON responses—without you writing serialization code.

## The Swagger UI Playground

Open http://localhost:8000/docs

You'll see an interactive documentation page. Every endpoint you create appears here automatically. Click on `GET /` to expand it, then click "Try it out" and "Execute."

This is your API playground. No need to use curl or write test scripts—Swagger UI lets you test everything visually.

There's also:
- http://localhost:8000/redoc — Alternative documentation style
- http://localhost:8000/openapi.json — Raw OpenAPI specification (what tools like Swagger consume)

## Path Parameters

Real APIs need dynamic routes. Let's add an endpoint that accepts a task ID:

```python
@app.get("/tasks/{task_id}")
def read_task(task_id: int):
    return {"task_id": task_id, "title": f"Task {task_id}"}
```

The `{task_id}` in the path becomes a **path parameter**. FastAPI:
1. Extracts the value from the URL
2. Converts it to the type you specified (`int`)
3. Passes it to your function

Try it:
- http://localhost:8000/tasks/1 → `{"task_id": 1, "title": "Task 1"}`
- http://localhost:8000/tasks/42 → `{"task_id": 42, "title": "Task 42"}`
- http://localhost:8000/tasks/abc → 422 error (not a valid integer)

That last one shows automatic validation. FastAPI rejects invalid data *before your code runs*. This is crucial—when agents receive bad input, they fail gracefully instead of crashing.

Check Swagger UI—the new endpoint appears with documentation showing the parameter type.

## Query Parameters

Sometimes you want optional parameters. Query parameters appear after `?` in the URL:

```python
@app.get("/tasks/{task_id}")
def read_task(task_id: int, include_details: bool = False):
    task = {"task_id": task_id, "title": f"Task {task_id}"}
    if include_details:
        task["details"] = "This task has additional details"
    return task
```

Now you can:
- http://localhost:8000/tasks/1 → Basic task info
- http://localhost:8000/tasks/1?include_details=true → Task with details

**The rule**: Parameters with default values are optional. Parameters without defaults are required:

```python
@app.get("/search")
def search_tasks(query: str, limit: int = 10):
    return {"query": query, "limit": limit}
```

- http://localhost:8000/search → 422 error (query is required)
- http://localhost:8000/search?query=urgent → Works, limit defaults to 10
- http://localhost:8000/search?query=urgent&limit=5 → Both parameters provided

## Sync vs Async: When Does It Matter?

You might notice we used `def read_root()` not `async def`. FastAPI supports both. Here's when each matters:

**Use `def` (synchronous)** when:
- Just returning data from memory (like our examples)
- Calling synchronous libraries

**Use `async def` (asynchronous)** when:
- Calling external APIs (like LLM providers)
- Database queries with async drivers
- Any I/O that might take time

For now, `def` works because we're returning dictionaries from memory. In Lesson 7, when you call `Runner.run()` to execute agents, you'll need `async def` because agent execution is asynchronous.

```python
# Lesson 7 preview - you'll write this later
@app.post("/chat")
async def chat(message: str):
    result = await runner.run(agent, messages=[...])  # async call
    return {"response": result.final_output}
```

Don't worry about async yet—just know it exists and why it matters.

## Hands-On Exercise

Build a Task API with these endpoints:

1. `GET /` — Returns a welcome message with version
2. `GET /tasks/{task_id}` — Returns a task with the given ID
3. `GET /tasks/{task_id}?details=true` — Returns extra information when details is true

Create the complete `main.py`:

```python
from fastapi import FastAPI

app = FastAPI(title="Task API")

@app.get("/")
def read_root():
    return {"message": "Task API is running", "version": "1.0.0"}

@app.get("/tasks/{task_id}")
def read_task(task_id: int, details: bool = False):
    task = {
        "id": task_id,
        "title": f"Task {task_id}",
        "status": "pending"
    }
    if details:
        task["description"] = "This is a detailed description"
        task["created_at"] = "2025-01-01T00:00:00Z"
    return task
```

Run it:

```bash
fastapi dev main.py
```

Then test in Swagger UI:
1. Open http://localhost:8000/docs
2. Try each endpoint using "Try it out"
3. Toggle the `details` parameter and observe the response change
4. Try an invalid task_id (like "abc") and observe the 422 error

## Challenge: Design Your Own Endpoint

Now apply what you've learned. **Before looking at any solution**, design an endpoint yourself:

**The Problem**: You need an endpoint that filters tasks by status. Users should be able to request only "pending" tasks, only "completed" tasks, or all tasks.

Think about:
- Should status be a path parameter or query parameter?
- What happens if no status is provided?
- What should the URL look like?

Try implementing it. Then compare your design with AI:

> "I designed a task filtering endpoint like this: [paste your code]. I chose [path/query] parameter because [your reasoning]. What would you suggest differently?"

Notice: You're not asking AI to write code for you. You're asking it to *review* your design decision. This is how engineers actually use AI—as a sounding board for ideas, not a code generator.

## Common Mistakes

**Mistake 1**: Forgetting to return a value

```python
# Wrong - returns None, client gets empty response
@app.get("/")
def read_root():
    message = "Hello"
    # Forgot to return!

# Correct - explicit return
@app.get("/")
def read_root():
    return {"message": "Hello"}
```

**Mistake 2**: Expecting FastAPI to catch all type errors

```python
# FastAPI validates that task_id is an integer
@app.get("/tasks/{task_id}")
def read_task(task_id: int):
    return {"task_id": task_id}

# But it doesn't validate business logic
# task_id = -1 is a valid int, even if it makes no sense
# You'll handle this in Lesson 4 (Error Handling)
```

**Mistake 3**: Confusing path and query parameters

```python
# Path parameter - identifies a specific resource
@app.get("/tasks/{task_id}")  # /tasks/123

# Query parameter - filters or modifies the request
@app.get("/tasks")
def list_tasks(status: str | None = None):  # /tasks?status=pending
```

**When to use which?**
- Path: "Give me task 123" → `/tasks/123`
- Query: "Give me tasks filtered by pending status" → `/tasks?status=pending`

## Refine Your Understanding

After completing the exercise, work through these scenarios with AI:

**Scenario 1: Debug a Real Problem**

> "My FastAPI endpoint returns `null` instead of my data. Here's my code: [paste code]. I expected it to return a task object. What's wrong?"

When AI identifies the issue, don't just accept the fix. Ask:

> "Why does Python behave that way? I want to understand the root cause, not just fix the symptom."

**Scenario 2: Evaluate a Design Trade-off**

> "I'm deciding between `/tasks/{status}` and `/tasks?status={status}` for filtering. My endpoint needs to support filtering by status AND by priority. Which design scales better?"

AI will explain. Then push back:

> "But REST conventions say path parameters should identify resources. Is filtering really 'identification'? Show me how real APIs handle this."

**Scenario 3: Extend to Your Domain**

> "I'm building an API for [your domain - recipes, books, whatever]. Design a resource endpoint with one path parameter and two optional query parameters. Explain your choices."

Review AI's design. Find something you'd do differently:

> "Your design uses [X], but I'd prefer [Y] because [your reasoning]. Which approach is more maintainable long-term?"

This is collaborative design. You're not asking "write code for me"—you're having an engineering discussion.

---

## Summary

You've created your first FastAPI application:

- **Project setup**: `uv init` + `uv add "fastapi[standard]"` for proper dependency management
- **Uvicorn**: The ASGI server that runs your FastAPI app (included in `fastapi[standard]`)
- **FastAPI instance**: `app = FastAPI(title="...")` creates your app with auto-documentation
- **Route decorators**: `@app.get("/path")` connects URLs to functions
- **Path parameters**: `{task_id}` in the path, type-validated automatically
- **Query parameters**: Optional parameters with defaults
- **Running**: `fastapi dev main.py` or `uv run uvicorn main:app --reload`
- **Swagger UI**: Interactive documentation at `/docs`

**The bigger picture**: You're building the HTTP layer that will eventually expose AI agents. Every endpoint pattern you learn here transfers directly to agent integration. Uvicorn handles the networking; FastAPI handles your logic.

Next lesson, you'll add POST endpoints with Pydantic models—the same validation layer that ensures agents receive well-formed requests.
