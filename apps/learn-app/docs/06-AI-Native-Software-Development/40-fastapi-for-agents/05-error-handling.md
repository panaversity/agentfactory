---
title: "Error Handling"
sidebar_position: 5
chapter: 40
lesson: 5
duration_minutes: 45

skills:
  - name: "HTTP Status Codes"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student explains 2xx, 4xx, 5xx status code categories"

  - name: "HTTPException Usage"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student raises appropriate exceptions for error cases"

  - name: "Error Response Design"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student creates helpful error messages"

learning_objectives:
  - objective: "Use appropriate HTTP status codes for different scenarios"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student correctly uses 200, 201, 400, 404, 422"

  - objective: "Raise HTTPException with meaningful error details"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Error responses include helpful detail messages"

  - objective: "Distinguish between validation errors and business errors"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Student explains when 400 vs 422 is appropriate"

cognitive_load:
  new_concepts: 4
  assessment: "Status code categories, HTTPException, status module, error messages"

differentiation:
  extension_for_advanced: "Implement custom exception handlers and error logging"
  remedial_for_struggling: "Focus on 200, 404, and 422 before other codes"

generated_by: "content-implementer"
source_spec: "specs/040-chapter-40-fastapi-for-agents/spec.md"
created: "2025-12-22"
---

# Error Handling

When things go wrong, your API needs to communicate clearly. A missing task should return 404, not crash the server. Invalid input should return 422, not accept garbage. Good error handling makes APIs predictable—and predictability matters enormously for agents.

## Why Error Handling Matters for Agents

When humans use an API, they read error messages and adjust. When agents call your API, they need to programmatically decide what to do. Clear, consistent errors enable agents to:

- **Retry on transient failures** (5xx errors)
- **Report bad input to users** (4xx errors with helpful messages)
- **Handle missing resources gracefully** (404 → create new one? skip?)
- **Never retry on business rule violations** (400 → input fundamentally wrong)

An agent that can't distinguish "try again later" from "your request is wrong" will either waste resources retrying or fail silently on fixable problems.

## HTTP Status Codes: The Communication Layer

HTTP status codes are a shared language between server and client:

| Range | Category | Meaning | Agent Should |
|-------|----------|---------|--------------|
| 2xx | Success | Request worked | Proceed normally |
| 4xx | Client Error | Client sent something wrong | Fix request, don't retry |
| 5xx | Server Error | Server failed internally | Retry with backoff |

**Common codes you'll use**:

| Code | Name | When to Use |
|------|------|-------------|
| 200 | OK | Request succeeded (default) |
| 201 | Created | Resource created successfully |
| 204 | No Content | Success, nothing to return |
| 400 | Bad Request | Client sent invalid data (business rules) |
| 404 | Not Found | Resource doesn't exist |
| 422 | Unprocessable Entity | Validation failed (Pydantic) |
| 500 | Internal Server Error | Something broke on the server |

**The agent perspective**: A well-designed agent inspects the status code FIRST, then reads the body. This is more reliable than parsing error messages:

```python
# Agent-side code (not your server, but how agents consume your API)
response = await client.get("/tasks/999")
if response.status_code == 404:
    # Resource doesn't exist - create it or skip
    ...
elif response.status_code >= 500:
    # Server problem - retry with exponential backoff
    ...
```

## The HTTPException Class

FastAPI provides `HTTPException` for returning error responses:

```python
from fastapi import HTTPException

@app.get("/tasks/{task_id}")
def get_task(task_id: int):
    task = find_task(task_id)
    if not task:
        raise HTTPException(
            status_code=404,
            detail="Task not found"
        )
    return task
```

**What happens when you raise?**
1. FastAPI stops executing your function
2. Returns the specified status code
3. Sends the detail as JSON

Output:
```
HTTP/1.1 404 Not Found
content-type: application/json

{
  "detail": "Task not found"
}
```

**Why `raise`, not `return`?** Exceptions bubble up through your code. If you have helper functions, they can raise HTTPException directly without needing to propagate error codes back up the call chain.

## Using the status Module

Magic numbers like `404` work, but are harder to read. FastAPI provides named constants:

```python
from fastapi import HTTPException, status

@app.get("/tasks/{task_id}")
def get_task(task_id: int):
    task = find_task(task_id)
    if not task:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Task with id {task_id} not found"
        )
    return task
```

Now the code is self-documenting. Common constants:

```python
status.HTTP_200_OK
status.HTTP_201_CREATED
status.HTTP_204_NO_CONTENT
status.HTTP_400_BAD_REQUEST
status.HTTP_404_NOT_FOUND
status.HTTP_422_UNPROCESSABLE_ENTITY
status.HTTP_500_INTERNAL_SERVER_ERROR
```

**A subtlety**: Python's autocomplete works with `status.HTTP_...`, making it easy to discover available codes. With magic numbers, you'd need to look them up.

## Setting Success Status Codes

Override the default 200 for specific endpoints:

```python
# Return 201 for resource creation
@app.post("/tasks", status_code=status.HTTP_201_CREATED)
def create_task(task: TaskCreate):
    # ...
    return new_task

# Return 204 for deletion (no body)
@app.delete("/tasks/{task_id}", status_code=status.HTTP_204_NO_CONTENT)
def delete_task(task_id: int):
    # ... delete logic
    return None  # No response body with 204
```

**Why 201 for create?** It signals "resource was created" vs "here's a resource that existed." Agents can distinguish between idempotent retrieval and actual creation.

**Why 204 for delete?** The resource is gone—there's nothing meaningful to return. Some APIs return 200 with confirmation; 204 is more semantically correct.

## 400 vs 422: The Distinction That Confuses Everyone

This trips up almost every developer. Let's be precise:

**422 Unprocessable Entity** — Pydantic validation failed. The JSON is valid, but the data doesn't match your schema.

```python
# Pydantic returns 422 automatically when:
# - Required field missing
# - Wrong data type
# - Field constraint violated

class TaskCreate(BaseModel):
    title: str  # If missing, 422

# POST with {"description": "no title"} → 422
```

**400 Bad Request** — Business logic validation failed. The data is valid according to the schema, but it breaks your rules.

```python
@app.post("/tasks")
def create_task(task: TaskCreate):
    # Business rule: title can't be empty whitespace
    if not task.title.strip():
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Title cannot be empty or whitespace"
        )
    # ...
```

**The way to think about it**:
- 422: "Your JSON doesn't match my schema" (Pydantic catches this)
- 400: "Your data passed schema validation but violates business rules" (you catch this)

**For agents**: Both mean "don't retry with the same input." But 422 suggests a type/format problem, while 400 suggests a logical problem. An agent might use this distinction to give users more specific guidance.

## Complete Error Handling Example

```python
from fastapi import FastAPI, HTTPException, status
from pydantic import BaseModel

app = FastAPI(title="Task API")

class TaskCreate(BaseModel):
    title: str
    description: str | None = None

class TaskUpdate(BaseModel):
    title: str
    description: str | None = None
    status: str | None = None

tasks: list[dict] = []
task_counter = 0

VALID_STATUSES = {"pending", "in_progress", "completed"}

def find_task(task_id: int) -> dict | None:
    """Helper to find a task by ID."""
    for task in tasks:
        if task["id"] == task_id:
            return task
    return None

@app.post("/tasks", status_code=status.HTTP_201_CREATED)
def create_task(task: TaskCreate):
    global task_counter

    # Business validation
    if not task.title.strip():
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Title cannot be empty or whitespace"
        )

    task_counter += 1
    new_task = {
        "id": task_counter,
        "title": task.title.strip(),
        "description": task.description,
        "status": "pending"
    }
    tasks.append(new_task)
    return new_task

@app.get("/tasks/{task_id}")
def get_task(task_id: int):
    task = find_task(task_id)
    if not task:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Task with id {task_id} not found"
        )
    return task

@app.put("/tasks/{task_id}")
def update_task(task_id: int, task_update: TaskUpdate):
    task = find_task(task_id)
    if not task:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Task with id {task_id} not found"
        )

    # Validate title
    if not task_update.title.strip():
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Title cannot be empty or whitespace"
        )

    # Validate status
    if task_update.status and task_update.status not in VALID_STATUSES:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Invalid status. Must be one of: {', '.join(VALID_STATUSES)}"
        )

    task["title"] = task_update.title.strip()
    if task_update.description is not None:
        task["description"] = task_update.description
    if task_update.status:
        task["status"] = task_update.status

    return task

@app.delete("/tasks/{task_id}")
def delete_task(task_id: int):
    task = find_task(task_id)
    if not task:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Task with id {task_id} not found"
        )

    tasks.remove(task)
    return {"message": "Task deleted", "id": task_id}
```

## Error Message Design: Helping Agents Help Users

Error messages aren't just for debugging—agents will parse them to inform users. Design them carefully:

**Be specific**:
```python
# Vague - agent can't help user
detail="Error"

# Specific - agent knows what to tell user
detail=f"Task with id {task_id} not found"
```

**Include context**:
```python
# Missing context - what status IS valid?
detail="Invalid status"

# With context - agent can suggest valid options
detail=f"Invalid status '{task_update.status}'. Must be one of: pending, in_progress, completed"
```

**Don't expose internals**:
```python
# Exposes implementation - security risk, unhelpful
detail=f"KeyError: 'tasks' at line 47"

# User-friendly - agent can relay appropriately
detail="An internal error occurred. Please try again."
```

**Consider structured errors for agents**:
```python
# Simple string (works)
detail="Task not found"

# Structured (better for agents)
detail={
    "code": "TASK_NOT_FOUND",
    "message": "Task with id 999 not found",
    "task_id": 999
}
```

The structured format gives agents machine-readable codes while preserving human-readable messages.

## Hands-On Exercise

Test each error scenario in Swagger UI:

**1. Test 404 Not Found**
```bash
GET /tasks/999
# Expected: 404 with "Task with id 999 not found"
```

**2. Test 422 Validation Error**
```bash
POST /tasks
{"description": "Missing title"}
# Expected: 422 with "Field required" for title
```

**3. Test 400 Business Error**
```bash
POST /tasks
{"title": "   "}
# Expected: 400 with "Title cannot be empty or whitespace"
```

**4. Test Invalid Status**
```bash
# First create a task
POST /tasks
{"title": "Test task"}

# Then try invalid status
PUT /tasks/1
{"title": "Test", "status": "invalid"}
# Expected: 400 with "Invalid status. Must be one of..."
```

**5. Test 201 Created**
```bash
POST /tasks
{"title": "Valid task"}
# Expected: 201 status (check response headers)
```

## Challenge: Design a Complete Error Response Format

**Before looking at any solution**, design your own error format:

**The Problem**: You want error responses that include:
- A machine-readable error code (like `TASK_NOT_FOUND`)
- A human-readable message
- Relevant context (task ID, valid options, etc.)
- Consistent structure across all errors

Think about:
- How do you make HTTPException return structured data?
- How do you ensure ALL your endpoints use this format?
- What error codes do you need for a task API?

Implement it for 404 and 400 errors. Then compare with AI:

> "I designed a structured error format like this: [paste your code]. I'm using [approach] to ensure consistency. How would you handle cases where Pydantic returns 422 errors—can I customize those to match my format?"

## Common Mistakes

**Mistake 1**: Forgetting to raise the exception

```python
# Wrong - creates exception but doesn't raise it
@app.get("/tasks/{task_id}")
def get_task(task_id: int):
    if not find_task(task_id):
        HTTPException(status_code=404, detail="Not found")  # Does nothing!
    return task

# Correct - raise the exception
@app.get("/tasks/{task_id}")
def get_task(task_id: int):
    if not find_task(task_id):
        raise HTTPException(status_code=404, detail="Not found")
    return task
```

This is a subtle bug—your code runs without errors but returns wrong data.

**Mistake 2**: Using 200 for errors

```python
# Wrong - 200 for missing resource
@app.get("/tasks/{task_id}")
def get_task(task_id: int):
    task = find_task(task_id)
    if not task:
        return {"error": "Not found"}  # Still 200!

# Correct - 404 for missing
raise HTTPException(status_code=404, detail="Not found")
```

Agents check status codes first. A 200 with an error in the body is confusing and breaks retry logic.

**Mistake 3**: Mixing exception types

```python
# Wrong - raises Python exception, becomes 500
@app.get("/tasks/{task_id}")
def get_task(task_id: int):
    task = find_task(task_id)
    if not task:
        raise ValueError("Not found")  # 500 Internal Server Error

# Correct - use HTTPException for HTTP errors
raise HTTPException(status_code=404, detail="Not found")
```

Python exceptions that escape your function become 500 errors. Users see "Internal Server Error," which is unhelpful and suggests your server is broken (even though the logic is correct).

## Try With AI

Now that you understand error handling, explore advanced patterns for building agent-friendly APIs.

**Prompt 1: Design Error Hierarchies**

```text
I want custom exception classes for my Task API:
- TaskNotFoundError
- InvalidStatusError
- DuplicateTaskError

Show me how to:
1. Create these exception classes
2. Register exception handlers that convert them to proper HTTP responses
3. Handle unknown exceptions gracefully so my API never exposes stack traces

What's the pattern for ensuring I don't forget to register a new exception type?
```

**What you're learning:** This prompt teaches you to design exception hierarchies. You'll discover that a base class like `TaskAPIError` with common fields lets you handle all custom exceptions with one handler, preventing the "forgot to register" problem. This pattern scales as your API grows.

**Prompt 2: Add Structured Logging**

```text
I want to log all 4xx and 5xx errors with request context:
- Request path and method
- Error details and status code
- A correlation ID that links related logs

Show me how to add this to FastAPI middleware. Also, how do I include additional context from inside endpoint functions—like which specific task_id caused the error?
```

**What you're learning:** This prompt develops your observability skills. You'll learn about middleware for cross-cutting concerns, context variables for request-scoped data, and correlation IDs for tracing requests across logs. Essential for debugging agent systems in production.

**Prompt 3: Design Agent-Friendly Errors**

```text
When an agent calls my API, I want error responses that help it recover automatically:

{
  "error_code": "TASK_NOT_FOUND",
  "message": "Task 42 not found",
  "retry_after": null,
  "suggestions": ["create the task first", "check the task ID"]
}

Design this error format and show me how to implement it for 404, 400, and 429 (rate limit) errors. How can an agent use the retry_after field?
```

**What you're learning:** This prompt teaches you to design for machine consumers. Agents can programmatically check `error_code` for decision logic, use `retry_after` for backoff, and potentially surface `suggestions` to users. This structured approach makes your API a better citizen in agent workflows.
