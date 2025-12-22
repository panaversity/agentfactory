---
title: "POST and Pydantic Models"
sidebar_position: 2
chapter: 40
lesson: 2
duration_minutes: 50

skills:
  - name: "Pydantic Model Definition"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student creates Pydantic models with proper field types"

  - name: "Request Body Handling"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student implements POST endpoints that parse request bodies"

  - name: "Response Model Serialization"
    proficiency_level: "A2"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student uses response_model for output validation"

learning_objectives:
  - objective: "Define Pydantic models for request and response validation"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student creates TaskCreate and TaskResponse models"

  - objective: "Implement POST endpoints that create resources"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "POST /tasks creates task and returns it with 201 status"

  - objective: "Handle validation errors from Pydantic"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Student explains 422 error response structure"

cognitive_load:
  new_concepts: 5
  assessment: "Pydantic BaseModel, Optional fields, POST endpoints, request bodies, response_model"

differentiation:
  extension_for_advanced: "Add field validators and custom error messages"
  remedial_for_struggling: "Start with single-field models before adding complexity"

generated_by: "content-implementer"
source_spec: "specs/040-chapter-40-fastapi-for-agents/spec.md"
created: "2025-12-22"
---

# POST and Pydantic Models

GET endpoints retrieve data. POST endpoints create data. To create a task, you need to send data in the request body. FastAPI uses Pydantic models to define what that data should look like and validate it automatically.

This matters for agents: when clients send requests to your agent endpoints (Lesson 7), Pydantic ensures the input is valid *before* your agent sees it. Bad data gets rejected at the door, not halfway through an expensive LLM call.

## Why Pydantic Matters for Agents

In Chapter 37, you built MCP servers that validate tool parameters. Pydantic does the same thing for HTTP APIs. When an agent endpoint receives JSON, Pydantic:

1. Parses the raw JSON bytes
2. Validates data types match your model
3. Checks required fields are present
4. Rejects invalid data with helpful error messages

This validation layer is critical when agents compose tools. One agent's output becomes another's input. Type safety at every boundary prevents cascading failures.

```python
from pydantic import BaseModel

class TaskCreate(BaseModel):
    title: str
    description: str | None = None
```

This model says:
- `title` is required and must be a string
- `description` is optional (can be `None`) and defaults to `None`

## How Pydantic Validates (Under the Hood)

When you write `title: str`, Pydantic:

1. **Checks existence** — Is there a "title" key in the JSON? Missing → `Field required` error
2. **Checks type** — Is the value a string? Wrong type → `string_type` error
3. **Attempts coercion** — `"123"` (string) passes. `123` (int) gets coerced to `"123"`
4. **Passes validated data** — Your function receives a guaranteed string

This is why `task.title` in your function is GUARANTEED to be a string. No defensive `if isinstance(title, str)` checks needed.

But what if you need custom validation? Title must be 3-100 characters:

```python
from pydantic import BaseModel, Field

class TaskCreate(BaseModel):
    title: str = Field(min_length=3, max_length=100)
    description: str | None = None
```

Now Pydantic enforces length constraints automatically. You'll explore more validation in the exercises.

## Defining Task Models

For our Task API, we need two models:

1. **TaskCreate** — What the client sends when creating a task
2. **TaskResponse** — What the API returns

```python
from pydantic import BaseModel

class TaskCreate(BaseModel):
    title: str
    description: str | None = None

class TaskResponse(BaseModel):
    id: int
    title: str
    description: str | None
    status: str
```

**Why two models?** The client shouldn't provide `id` or `status`—those are set by the server. Separating models keeps responsibilities clear:

- Client says: "Create a task with this title"
- Server says: "Here's your task with ID 1, status pending"

This separation matters more as your API grows. You might have `TaskCreate`, `TaskUpdate`, `TaskResponse`, `TaskSummary`—each exposing exactly what that operation needs.

## Creating a POST Endpoint

Add these to your `main.py`:

```python
from fastapi import FastAPI
from pydantic import BaseModel

app = FastAPI(title="Task API")

# Pydantic models
class TaskCreate(BaseModel):
    title: str
    description: str | None = None

class TaskResponse(BaseModel):
    id: int
    title: str
    description: str | None
    status: str

# In-memory storage
tasks: list[dict] = []

@app.post("/tasks", response_model=TaskResponse, status_code=201)
def create_task(task: TaskCreate):
    new_task = {
        "id": len(tasks) + 1,
        "title": task.title,
        "description": task.description,
        "status": "pending"
    }
    tasks.append(new_task)
    return new_task
```

Let's break down the key elements:

- `@app.post("/tasks")` — This endpoint handles POST requests
- `task: TaskCreate` — FastAPI parses the request body as a `TaskCreate` model
- `response_model=TaskResponse` — FastAPI validates the response matches this model
- `status_code=201` — Return 201 Created instead of default 200

## Testing in Swagger UI

Open http://localhost:8000/docs and find the POST endpoint.

1. Click "Try it out"
2. In the request body, enter:
   ```json
   {
     "title": "Learn FastAPI",
     "description": "Complete the tutorial"
   }
   ```
3. Click "Execute"

You'll see a 201 response with the created task:

```json
{
  "id": 1,
  "title": "Learn FastAPI",
  "description": "Complete the tutorial",
  "status": "pending"
}
```

## Validation Errors: What Students Find Confusing

This is where many students get stuck. Let's work through it carefully.

**Try posting with missing title:**

```json
{
  "description": "Missing title"
}
```

FastAPI returns a 422 Unprocessable Entity:

```json
{
  "detail": [
    {
      "type": "missing",
      "loc": ["body", "title"],
      "msg": "Field required",
      "input": {"description": "Missing title"}
    }
  ]
}
```

**Reading this error:**
- `type: "missing"` — What kind of validation failure
- `loc: ["body", "title"]` — Where the error is: in the body, at field "title"
- `msg: "Field required"` — Human-readable explanation
- `input` — What you actually sent

**Why 422 and not 400?**

This confuses people. Here's the distinction:

- **422 Unprocessable Entity** — The JSON is valid, but data doesn't match the schema. Pydantic catches these.
- **400 Bad Request** — Business logic validation failed (e.g., "title can't be empty whitespace"). You handle these in your code.

FastAPI automatically returns 422 for schema violations. You'll add 400 errors in Lesson 4.

**Try posting with wrong type:**

```json
{
  "title": 123
}
```

Response:

```json
{
  "detail": [
    {
      "type": "string_type",
      "loc": ["body", "title"],
      "msg": "Input should be a valid string",
      "input": 123
    }
  ]
}
```

Pydantic caught that `title` should be a string, not a number.

## Response Model Filtering

The `response_model` parameter does more than validation—it filters the output. If your internal data has extra fields, only the model's fields are returned.

```python
@app.post("/tasks", response_model=TaskResponse)
def create_task(task: TaskCreate):
    new_task = {
        "id": len(tasks) + 1,
        "title": task.title,
        "description": task.description,
        "status": "pending",
        "internal_flag": True,  # Won't appear in response
        "debug_info": "extra data"  # Neither will this
    }
    tasks.append(new_task)
    return new_task
```

Only `id`, `title`, `description`, and `status` appear in the response because those are the fields in `TaskResponse`. This is a security feature—you won't accidentally leak internal data.

## In-Memory Storage: A Reality Check

We're using a simple list to store tasks:

```python
tasks: list[dict] = []
```

This works for learning but has real limitations:

- **Resets when you restart** — All tasks disappear
- **No persistence** — Nothing saved to disk
- **No concurrency safety** — Two simultaneous requests could corrupt data
- **Single process only** — Multiple workers don't share the list

These aren't problems for learning. They're problems you'll solve with databases in Chapter 47. For now, understand the CRUD pattern—the storage mechanism is secondary.

## Hands-On Exercise

Build the complete task creation flow:

```python
from fastapi import FastAPI
from pydantic import BaseModel

app = FastAPI(title="Task API")

class TaskCreate(BaseModel):
    title: str
    description: str | None = None

class TaskResponse(BaseModel):
    id: int
    title: str
    description: str | None
    status: str

tasks: list[dict] = []

@app.get("/")
def read_root():
    return {"message": "Task API", "task_count": len(tasks)}

@app.post("/tasks", response_model=TaskResponse, status_code=201)
def create_task(task: TaskCreate):
    new_task = {
        "id": len(tasks) + 1,
        "title": task.title,
        "description": task.description,
        "status": "pending"
    }
    tasks.append(new_task)
    return new_task

@app.get("/tasks")
def list_tasks():
    return tasks
```

Test this workflow:
1. POST a task with title "First task"
2. POST another task with title and description
3. GET /tasks to see both tasks
4. GET / to see the task count
5. Try posting without a title and observe the 422 error

## Challenge: Design a Model with Constraints

**Before looking at any solution**, design a model yourself:

**The Problem**: You need a `TaskCreate` model where:
- `title` is required, 3-100 characters
- `description` is optional, max 500 characters
- `priority` is optional, must be "low", "medium", or "high", defaults to "medium"

Think about:
- How do you enforce character limits?
- How do you restrict to specific values?
- What should the error message say if someone sends "urgent" as priority?

Implement it. Then test with intentionally invalid data. Then compare with AI:

> "I designed a TaskCreate model with these constraints: [paste your code]. I used [approach] for the priority field. Does Pydantic have a better pattern for enum-like fields?"

## Common Mistakes

**Mistake 1**: Using one model for everything

```python
# Wrong - client shouldn't provide id and status
class Task(BaseModel):
    id: int
    title: str
    status: str

@app.post("/tasks")
def create_task(task: Task):  # Client must provide id?
    ...
```

Create separate models for input (TaskCreate) and output (TaskResponse).

**Mistake 2**: Forgetting `response_model`

```python
# Without response_model, you might leak internal data
@app.post("/tasks")
def create_task(task: TaskCreate):
    new_task = {..., "password_hash": "secret123"}  # Oops, exposed!
    return new_task
```

Always use `response_model` to control what's returned.

**Mistake 3**: Optional field without default

```python
# Wrong - this makes description required
description: str | None  # No default!

# Correct - union type with default None
description: str | None = None
```

The `= None` is crucial. Without it, the field is required (just nullable).

## Refine Your Understanding

After completing the exercise, work through these scenarios with AI:

**Scenario 1: Understand the Validation Pipeline**

> "Trace what happens when I POST this JSON to /tasks: `{'title': 123, 'extra_field': 'ignored'}`. Show me each step from raw request to my function parameter."

When AI explains, test your understanding:

> "So if I wanted to REJECT extra fields instead of ignoring them, how would I configure that in Pydantic?"

**Scenario 2: Design a Complex Model**

> "I need a model for creating a Meeting with: title (required), attendees (list of emails), duration_minutes (must be 15, 30, 60, or 90), is_recurring (boolean, defaults to false). Design it."

Review AI's design. Find something to improve:

> "Your attendees field doesn't validate email format. What's the best way to add email validation—regex, Pydantic's EmailStr, or custom validator?"

**Scenario 3: Evaluate Trade-offs**

> "Should I use Python Enum or Literal for the priority field that only allows 'low', 'medium', 'high'? What are the trade-offs of each approach?"

AI will explain both. Push back:

> "You said Enum is more explicit, but my API consumers are JavaScript clients. Which approach produces cleaner OpenAPI documentation?"

This is engineering judgment—you're learning to think through trade-offs, not just accept first answers.

---

## Summary

You've learned to create resources with POST endpoints:

- **Pydantic models**: Define data structure with `BaseModel`
- **How validation works**: Existence check → type check → coercion → pass to function
- **Request bodies**: `task: TaskCreate` parses JSON automatically
- **Validation errors**: 422 with structured error details
- **Response models**: Control output with `response_model`
- **Status codes**: Return 201 for resource creation

**The bigger picture**: Pydantic is the validation layer between the outside world and your code. When agents receive requests, Pydantic ensures the data is valid before expensive LLM calls happen.

Next lesson, you'll implement the full CRUD operations—reading, updating, and deleting tasks.
