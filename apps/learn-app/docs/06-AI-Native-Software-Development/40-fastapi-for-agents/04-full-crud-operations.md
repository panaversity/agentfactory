---
title: "Full CRUD Operations"
sidebar_position: 4
chapter: 40
lesson: 4
duration_minutes: 55

skills:
  - name: "HTTP Method Semantics"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student explains when to use GET, POST, PUT, DELETE"

  - name: "CRUD Implementation"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student implements all four CRUD operations"

  - name: "Resource Lookup Patterns"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student finds resources by ID and handles not-found cases"

learning_objectives:
  - objective: "Implement GET endpoints for listing and retrieving resources"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "GET /tasks returns all tasks; GET /tasks/{id} returns one"

  - objective: "Implement PUT endpoint for updating resources"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "PUT /tasks/{id} updates and returns the task"

  - objective: "Implement DELETE endpoint for removing resources"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "DELETE /tasks/{id} removes task and returns confirmation"

cognitive_load:
  new_concepts: 4
  assessment: "GET list, GET single, PUT, DELETE - building on POST from previous lesson"

differentiation:
  extension_for_advanced: "Add PATCH for partial updates; implement bulk operations"
  remedial_for_struggling: "Focus on GET operations before moving to PUT and DELETE"

generated_by: "content-implementer"
source_spec: "specs/040-chapter-40-fastapi-for-agents/spec.md"
created: "2025-12-22"
---

# Full CRUD Operations

You've created tasks with POST. Now you complete the picture with Read, Update, and Delete. Together—Create, Read, Update, Delete—these form CRUD, the foundation of data-driven APIs.

This matters for agents: every agent that manages state needs CRUD. When you build agent endpoints (Lesson 7), agents will create conversation sessions, read past messages, update conversation metadata, and delete expired sessions. The patterns you learn here transfer directly.

## Why CRUD Is the Foundation

Consider what agents actually do:

- **Memory agents** create memories, read relevant ones, update importance scores, delete stale entries
- **Task agents** create tasks, read pending work, update status, delete completed items
- **Session agents** create conversations, read context, update metadata, delete expired sessions

Every one of these is CRUD. Master these four operations, and you can build the data layer for any agent.

## HTTP Methods and CRUD

Each CRUD operation maps to an HTTP method:

| Operation | HTTP Method | Endpoint Example | Description |
|-----------|-------------|------------------|-------------|
| Create | POST | POST /tasks | Create a new task |
| Read (all) | GET | GET /tasks | List all tasks |
| Read (one) | GET | GET /tasks/1 | Get task with ID 1 |
| Update | PUT | PUT /tasks/1 | Update task with ID 1 |
| Delete | DELETE | DELETE /tasks/1 | Delete task with ID 1 |

**Why these specific mappings?** HTTP methods have semantics:

- **GET** is *safe*—it doesn't change server state. Browsers can cache GET responses.
- **POST** creates new resources. Not safe, not idempotent.
- **PUT** replaces a resource. Idempotent—calling it twice has the same effect as once.
- **DELETE** removes a resource. Also idempotent.

These semantics matter for agents. If an agent's HTTP call fails partway through, idempotent operations (PUT, DELETE) can be safely retried. Non-idempotent operations (POST) require more careful handling.

## List All Tasks

The simplest read operation—return everything:

```python
@app.get("/tasks")
def list_tasks():
    return tasks
```

But agents rarely want *everything*. Add filtering:

```python
@app.get("/tasks")
def list_tasks(status: str | None = None):
    if status:
        return [t for t in tasks if t["status"] == status]
    return tasks
```

Now clients can:
- `GET /tasks` — All tasks
- `GET /tasks?status=pending` — Only pending tasks
- `GET /tasks?status=completed` — Only completed tasks

For agents, this filtering is essential. A task agent doesn't want to process completed items—it filters for pending work.

## Get Single Task

Retrieve one task by ID:

```python
from fastapi import HTTPException

@app.get("/tasks/{task_id}")
def get_task(task_id: int):
    for task in tasks:
        if task["id"] == task_id:
            return task
    raise HTTPException(status_code=404, detail="Task not found")
```

**What happens when the task doesn't exist?** We raise `HTTPException` with status 404. This is the standard HTTP response for "resource not found."

Test it:
- `GET /tasks/1` → Returns task 1 (if it exists)
- `GET /tasks/999` → Returns 404 error

## Update Task: The Subtle Complexity

This is where students often get confused. Let's work through it carefully.

**PUT replaces the entire resource.** The client sends the complete new version:

```python
class TaskUpdate(BaseModel):
    title: str
    description: str | None = None
    status: str | None = None

@app.put("/tasks/{task_id}")
def update_task(task_id: int, task_update: TaskUpdate):
    for task in tasks:
        if task["id"] == task_id:
            task["title"] = task_update.title
            if task_update.description is not None:
                task["description"] = task_update.description
            if task_update.status is not None:
                task["status"] = task_update.status
            return task
    raise HTTPException(status_code=404, detail="Task not found")
```

**Why a new model `TaskUpdate`?** It allows updating `status`, which `TaskCreate` doesn't include. The client sends:

```json
{
  "title": "Updated title",
  "description": "New description",
  "status": "completed"
}
```

**The confusing part**: Why check `if task_update.description is not None`?

Without this check, sending `{"title": "New"}` would set `description` to `None`, erasing the existing description. We only update fields the client explicitly provides.

**But wait—isn't that PATCH behavior?** Yes, this is actually partial update logic. True PUT would require ALL fields. In practice, most APIs use PUT for what should be PATCH because PATCH has browser support issues historically. You'll see both patterns.

## Delete Task

Remove a task from our list:

```python
@app.delete("/tasks/{task_id}")
def delete_task(task_id: int):
    for i, task in enumerate(tasks):
        if task["id"] == task_id:
            tasks.pop(i)
            return {"message": "Task deleted", "id": task_id}
    raise HTTPException(status_code=404, detail="Task not found")
```

Some APIs return 204 No Content for deletes. We're returning a confirmation message, which helps with debugging.

## The In-Memory Mutation Problem

Look at our update code:

```python
task["title"] = task_update.title
```

We're mutating a dictionary inside a list. This works, but has problems:

**Problem 1: No atomic updates.** If two requests try to update the same task simultaneously, they might interleave:
```
Request A: reads task with status="pending"
Request B: reads task with status="pending"
Request A: sets status="in_progress"
Request B: sets status="completed" (expected: in_progress -> completed, actual: pending -> completed)
```

**Problem 2: No rollback.** If we update title, then description fails validation, the title change persists.

**Problem 3: The delete index bug.** Our delete uses `tasks.pop(i)`. If two deletes run simultaneously on IDs 1 and 3, the indices shift and we might delete the wrong item.

These are real problems that databases solve. For learning CRUD, they don't matter. But when you build production agents, you'll use a database with proper transaction support. We cover this in Chapter 47.

## Complete Implementation

Here's the full `main.py` with all CRUD operations:

```python
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

app = FastAPI(title="Task API")

# Models
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

# Storage
tasks: list[dict] = []
task_counter = 0

# CREATE
@app.post("/tasks", response_model=TaskResponse, status_code=201)
def create_task(task: TaskCreate):
    global task_counter
    task_counter += 1
    new_task = {
        "id": task_counter,
        "title": task.title,
        "description": task.description,
        "status": "pending"
    }
    tasks.append(new_task)
    return new_task

# READ (all)
@app.get("/tasks")
def list_tasks(status: str | None = None):
    if status:
        return [t for t in tasks if t["status"] == status]
    return tasks

# READ (one)
@app.get("/tasks/{task_id}", response_model=TaskResponse)
def get_task(task_id: int):
    for task in tasks:
        if task["id"] == task_id:
            return task
    raise HTTPException(status_code=404, detail="Task not found")

# UPDATE
@app.put("/tasks/{task_id}", response_model=TaskResponse)
def update_task(task_id: int, task_update: TaskUpdate):
    for task in tasks:
        if task["id"] == task_id:
            task["title"] = task_update.title
            if task_update.description is not None:
                task["description"] = task_update.description
            if task_update.status is not None:
                task["status"] = task_update.status
            return task
    raise HTTPException(status_code=404, detail="Task not found")

# DELETE
@app.delete("/tasks/{task_id}")
def delete_task(task_id: int):
    for i, task in enumerate(tasks):
        if task["id"] == task_id:
            tasks.pop(i)
            return {"message": "Task deleted", "id": task_id}
    raise HTTPException(status_code=404, detail="Task not found")
```

## Hands-On Exercise

Test the complete CRUD cycle in Swagger UI:

**Step 1: Create Tasks**
```bash
POST /tasks
{"title": "Learn CRUD", "description": "Complete this lesson"}

POST /tasks
{"title": "Build API", "description": "Create a complete REST API"}

POST /tasks
{"title": "Test API", "description": "Verify all endpoints work"}
```

**Step 2: List and Filter**
```bash
GET /tasks           # See all 3 tasks
GET /tasks?status=pending  # All should be pending
```

**Step 3: Read Single**
```bash
GET /tasks/1         # First task
GET /tasks/999       # Should return 404
```

**Step 4: Update**
```bash
PUT /tasks/1
{"title": "Learn CRUD", "status": "completed"}
```

**Step 5: Verify Update**
```bash
GET /tasks/1         # Status should be "completed"
GET /tasks?status=completed  # Should show task 1
GET /tasks?status=pending    # Should show tasks 2 and 3
```

**Step 6: Delete**
```bash
DELETE /tasks/3      # Delete task 3
GET /tasks           # Should only show tasks 1 and 2
```

## Challenge: Design a Status Workflow

**Before looking at any solution**, design validation rules yourself:

**The Problem**: Tasks should follow a workflow:
- New tasks start as `pending`
- `pending` → `in_progress` → `completed` (normal flow)
- Can go back from `in_progress` → `pending` (blocked)
- Cannot go from `completed` → anything (final state)
- Cannot skip states (`pending` → `completed` directly)

Think about:
- Where do you validate this? In the Pydantic model or the endpoint?
- What error message helps the user understand what went wrong?
- How do you store valid transitions in a maintainable way?

Implement it. Test with invalid transitions. Then compare with AI:

> "I implemented task status workflow validation like this: [paste your code]. I put the logic in [model/endpoint] because [your reasoning]. Would a state machine pattern be cleaner?"

## Common Mistakes

**Mistake 1**: Not returning the updated resource

```python
# Wrong - returns nothing useful
@app.put("/tasks/{task_id}")
def update_task(task_id: int, task_update: TaskUpdate):
    for task in tasks:
        if task["id"] == task_id:
            task["title"] = task_update.title
            return {"message": "Updated"}  # Client doesn't know the new state

# Correct - return the updated resource
@app.put("/tasks/{task_id}")
def update_task(task_id: int, task_update: TaskUpdate):
    for task in tasks:
        if task["id"] == task_id:
            task["title"] = task_update.title
            return task  # Client sees the result
```

**Why return the updated resource?** The client might have stale data. Returning the current state avoids a follow-up GET.

**Mistake 2**: Using wrong HTTP method

```python
# Wrong - POST shouldn't be used for updates
@app.post("/tasks/{task_id}/update")

# Correct - PUT for updates
@app.put("/tasks/{task_id}")

# Wrong - GET with side effects
@app.get("/tasks/{task_id}/delete")

# Correct - DELETE for deletion
@app.delete("/tasks/{task_id}")
```

**Why this matters**: HTTP semantics have meaning. Browsers might prefetch GET requests. Proxies might cache them. If your GET deletes data, you'll have mysterious data loss.

**Mistake 3**: Not handling not-found cases

```python
# Wrong - returns None, causes errors
@app.get("/tasks/{task_id}")
def get_task(task_id: int):
    for task in tasks:
        if task["id"] == task_id:
            return task
    # Falls through, returns None

# Correct - explicit 404
@app.get("/tasks/{task_id}")
def get_task(task_id: int):
    for task in tasks:
        if task["id"] == task_id:
            return task
    raise HTTPException(status_code=404, detail="Task not found")
```

## Refine Your Understanding

After completing the exercise, work through these scenarios with AI:

**Scenario 1: Understand PUT vs PATCH**

> "Explain the difference between PUT and PATCH for updates. My current PUT implementation acts like PATCH—it only updates fields that are provided. Is this wrong? Show me what strict PUT semantics would look like."

When AI explains, push back:

> "You said strict PUT requires all fields. But then a client that only wants to change status has to resend the entire object. What if they have stale data for other fields? Isn't PATCH safer?"

**Scenario 2: Optimize Lookup Performance**

> "My current implementation loops through the list to find tasks. What's the time complexity? Show me how to use a dictionary for O(1) lookups. What trade-offs does this introduce?"

Review AI's suggestion. Challenge it:

> "With a dictionary, I lose ordering. If I need to list tasks in creation order, how do I maintain that while getting O(1) lookups?"

**Scenario 3: Design for Agent Use**

> "I'm building an endpoint for an agent to claim tasks—mark them as 'in_progress' and assign to a worker ID. Two agents might try to claim the same task. Show me how to handle this race condition."

This explores concurrent access—a real production problem you'll face with agents.

---

## Summary

You've implemented complete CRUD operations:

- **Create**: `POST /tasks` with request body
- **Read all**: `GET /tasks` with optional filtering
- **Read one**: `GET /tasks/{id}` with 404 for missing
- **Update**: `PUT /tasks/{id}` returns updated resource
- **Delete**: `DELETE /tasks/{id}` removes resource

**The bigger picture**: CRUD is the data layer for any agent. Memory agents, task agents, session agents—all need these operations. The in-memory implementation has concurrency issues that databases solve, but the patterns transfer directly.

Next lesson, you'll learn proper error handling—returning the right status codes and creating helpful error messages.
