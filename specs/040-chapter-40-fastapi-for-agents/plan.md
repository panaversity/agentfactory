# Chapter 40: FastAPI for Agents — Implementation Plan

**Version**: 1.0.0
**Status**: Approved
**Created**: 2025-12-22
**Spec Reference**: specs/040-chapter-40-fastapi-for-agents/spec.md

---

## 1. Implementation Overview

### 1.1 Approach
**Manual First, Agent Last**: Each lesson builds FastAPI skills incrementally. Students master HTTP fundamentals, Pydantic validation, and dependency injection BEFORE touching agent integration.

### 1.2 Teaching Progression
| Phase | Lessons | Layer | Focus |
|-------|---------|-------|-------|
| Foundation | 1-4 | L1 (Manual) | Core FastAPI patterns |
| Intermediate | 5-6 | L1→L2 | Advanced patterns with AI collaboration |
| Integration | 7 | L2 | Agent wrapping |
| Capstone | 8 | L4 | Spec-driven synthesis |

### 1.3 Single Domain
All lessons use **Task Management** (todo list API). This provides:
- Consistent context across lessons
- Natural complexity progression
- Familiar domain for students
- Perfect fit for agent integration

---

## 2. Lesson Plans

### 2.1 Lesson 1: Hello FastAPI (45 min)

**File**: `01-hello-fastapi.md`

**Learning Objectives**:
- Create a FastAPI application
- Define GET endpoints with path/query parameters
- Run with uvicorn and explore Swagger UI

**Structure**:
1. **Introduction** (5 min)
   - What is FastAPI and why it matters
   - Connection to MCP HTTP transport (Chapter 38)

2. **Core Concept: The Application** (10 min)
   - Creating FastAPI instance
   - Title and description for docs
   - Running with uvicorn

3. **Your First Endpoint** (10 min)
   - `@app.get("/")` decorator
   - Return dictionaries → JSON
   - Test in browser

4. **Path Parameters** (10 min)
   - `@app.get("/tasks/{task_id}")`
   - Type hints for automatic validation
   - 422 error for wrong types

5. **Query Parameters** (5 min)
   - Optional query params
   - Default values
   - Multiple params

6. **Hands-On: Swagger UI Exploration** (5 min)
   - Navigate to `/docs`
   - "Try it out" button
   - See auto-generated OpenAPI spec

**Code Milestones**:
```python
# Milestone 1: Basic app
app = FastAPI(title="Task API")

# Milestone 2: Root endpoint
@app.get("/")
def read_root():
    return {"message": "Task API is running"}

# Milestone 3: Path parameter
@app.get("/tasks/{task_id}")
def read_task(task_id: int):
    return {"task_id": task_id}

# Milestone 4: Query parameter
@app.get("/tasks/{task_id}")
def read_task(task_id: int, details: bool = False):
    return {"task_id": task_id, "details": details}
```

---

### 2.2 Lesson 2: POST and Pydantic Models (50 min)

**File**: `02-post-and-pydantic-models.md`

**Learning Objectives**:
- Define Pydantic models for data validation
- Create POST endpoints with request bodies
- Use response_model for serialization

**Structure**:
1. **Why Pydantic?** (5 min)
   - Type safety for APIs
   - Automatic validation
   - JSON serialization

2. **Defining Models** (15 min)
   - BaseModel inheritance
   - Field types (str, int, Optional)
   - Default values
   - TaskCreate and TaskResponse models

3. **POST Endpoints** (15 min)
   - `@app.post("/tasks")`
   - Request body parsing
   - In-memory storage (list)
   - Return created task

4. **Response Models** (10 min)
   - `response_model=TaskResponse`
   - Field filtering
   - Status code override

5. **Hands-On: Create Tasks** (5 min)
   - POST via Swagger UI
   - Observe validation errors
   - Check 422 for invalid data

**Code Milestones**:
```python
# Milestone 1: TaskCreate model
class TaskCreate(BaseModel):
    title: str
    description: Optional[str] = None

# Milestone 2: TaskResponse model
class TaskResponse(BaseModel):
    id: int
    title: str
    description: Optional[str]
    status: str

# Milestone 3: In-memory storage
tasks: list[dict] = []

# Milestone 4: POST endpoint
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

---

### 2.3 Lesson 3: Full CRUD Operations (55 min)

**File**: `03-full-crud-operations.md`

**Learning Objectives**:
- Implement GET (list), GET (single), PUT, DELETE
- Understand HTTP method semantics
- Handle missing resources

**Structure**:
1. **CRUD Overview** (5 min)
   - Create = POST
   - Read = GET
   - Update = PUT
   - Delete = DELETE

2. **List Tasks (GET all)** (10 min)
   - Return all tasks
   - Optional status filter

3. **Get Single Task** (10 min)
   - Path parameter for ID
   - Find in list
   - Handle not found (intro to HTTPException)

4. **Update Task (PUT)** (15 min)
   - Path parameter + request body
   - Find and modify
   - Return updated task

5. **Delete Task** (10 min)
   - Path parameter
   - Remove from list
   - Return confirmation

6. **Hands-On: Complete CRUD** (5 min)
   - Create → List → Get → Update → Delete flow
   - Verify data persistence

**Code Milestones**:
```python
# Milestone 1: List all
@app.get("/tasks")
def list_tasks(status: Optional[str] = None):
    if status:
        return [t for t in tasks if t["status"] == status]
    return tasks

# Milestone 2: Get by ID
@app.get("/tasks/{task_id}")
def get_task(task_id: int):
    for task in tasks:
        if task["id"] == task_id:
            return task
    raise HTTPException(status_code=404, detail="Task not found")

# Milestone 3: Update
@app.put("/tasks/{task_id}")
def update_task(task_id: int, task_update: TaskCreate):
    for task in tasks:
        if task["id"] == task_id:
            task["title"] = task_update.title
            task["description"] = task_update.description
            return task
    raise HTTPException(status_code=404, detail="Task not found")

# Milestone 4: Delete
@app.delete("/tasks/{task_id}")
def delete_task(task_id: int):
    for i, task in enumerate(tasks):
        if task["id"] == task_id:
            tasks.pop(i)
            return {"message": "Task deleted"}
    raise HTTPException(status_code=404, detail="Task not found")
```

---

### 2.4 Lesson 4: Error Handling (45 min)

**File**: `04-error-handling.md`

**Learning Objectives**:
- Use HTTPException for error responses
- Apply appropriate HTTP status codes
- Handle validation errors gracefully

**Structure**:
1. **HTTP Status Codes** (10 min)
   - 2xx Success (200, 201, 204)
   - 4xx Client Errors (400, 404, 422)
   - 5xx Server Errors (500)

2. **HTTPException** (15 min)
   - Importing from fastapi
   - Status code + detail
   - Using status constants

3. **Validation Errors** (10 min)
   - Pydantic automatic 422
   - Custom validation in endpoint
   - 400 Bad Request vs 422

4. **Hands-On: Error Cases** (10 min)
   - Test 404 for missing task
   - Test 422 for invalid data
   - Test 400 for business rule violation

**Code Milestones**:
```python
from fastapi import HTTPException, status

# Milestone 1: 404 Not Found
@app.get("/tasks/{task_id}")
def get_task(task_id: int):
    task = find_task(task_id)
    if not task:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Task with id {task_id} not found"
        )
    return task

# Milestone 2: 201 Created
@app.post("/tasks", status_code=status.HTTP_201_CREATED)
def create_task(task: TaskCreate):
    # ...

# Milestone 3: 400 Bad Request (business rule)
@app.post("/tasks")
def create_task(task: TaskCreate):
    if not task.title.strip():
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Title cannot be empty"
        )
    # ...
```

---

### 2.5 Lesson 5: Dependency Injection (50 min)

**File**: `05-dependency-injection.md`

**Learning Objectives**:
- Use Depends() for shared resources
- Implement repository pattern
- Understand dependency lifecycle

**Structure**:
1. **Why Dependency Injection?** (10 min)
   - Problem: global state is messy
   - Solution: inject dependencies explicitly
   - Benefits: testability, flexibility

2. **The Depends() Function** (15 min)
   - Creating dependency functions
   - Type hints for IDE support
   - Automatic resolution

3. **Repository Pattern** (15 min)
   - TaskRepository class
   - CRUD methods
   - Singleton instance

4. **Hands-On: Refactor to DI** (10 min)
   - Extract storage to repository
   - Inject into endpoints
   - Verify functionality unchanged

**Layer Transition (L1→L2)**:
- Introduce AI collaboration for repository design
- "Ask AI: How would you organize this repository class?"
- Compare AI suggestion with manual implementation

**Code Milestones**:
```python
# Milestone 1: Repository class
class TaskRepository:
    def __init__(self):
        self.tasks: list[dict] = []
        self.counter = 0

    def create(self, task: TaskCreate) -> dict:
        self.counter += 1
        new_task = {"id": self.counter, **task.model_dump(), "status": "pending"}
        self.tasks.append(new_task)
        return new_task

    def get_all(self) -> list[dict]:
        return self.tasks

    def get_by_id(self, task_id: int) -> Optional[dict]:
        return next((t for t in self.tasks if t["id"] == task_id), None)

# Milestone 2: Dependency function
task_repo = TaskRepository()

def get_task_repo() -> TaskRepository:
    return task_repo

# Milestone 3: Inject in endpoint
@app.get("/tasks")
def list_tasks(repo: TaskRepository = Depends(get_task_repo)):
    return repo.get_all()
```

---

### 2.6 Lesson 6: Streaming with SSE (55 min)

**File**: `06-streaming-with-sse.md`

**Learning Objectives**:
- Create streaming responses with SSE
- Use async generators for data production
- Handle client connections

**Structure**:
1. **Why Streaming?** (10 min)
   - Long-running operations
   - Real-time updates
   - LLM token streaming

2. **Server-Sent Events Basics** (15 min)
   - EventSourceResponse from sse-starlette
   - Event format (event, data, id)
   - Browser EventSource API

3. **Async Generators** (15 min)
   - `async def` + `yield`
   - Simulating task updates
   - Error handling in streams

4. **Hands-On: Task Updates Stream** (15 min)
   - Create update generator
   - Connect from browser
   - Observe real-time events

**Layer 2 Integration**:
- AI collaboration for stream design
- "Ask AI: What edge cases should I handle in SSE?"

**Code Milestones**:
```python
from sse_starlette.sse import EventSourceResponse
import asyncio
import json

# Milestone 1: Basic async generator
async def task_updates_generator():
    for i in range(5):
        yield {
            "event": "task_update",
            "data": json.dumps({"task_id": i + 1, "status": "updated"})
        }
        await asyncio.sleep(1)

# Milestone 2: Streaming endpoint
@app.get("/tasks/stream")
async def stream_task_updates():
    return EventSourceResponse(task_updates_generator())

# Milestone 3: With task context
async def task_progress_generator(task_id: int):
    for progress in range(0, 101, 20):
        yield {
            "event": "progress",
            "data": json.dumps({"task_id": task_id, "progress": progress})
        }
        await asyncio.sleep(0.5)
```

---

### 2.7 Lesson 7: Agent Integration (60 min)

**File**: `07-agent-integration.md`

**Learning Objectives**:
- Wrap AI agent in FastAPI endpoint
- Stream agent responses via SSE
- Handle agent context and errors

**Structure**:
1. **Agent as Service** (10 min)
   - Why wrap agents in APIs?
   - Agent dependency injection
   - Context management

2. **Basic Agent Endpoint** (15 min)
   - OpenAI client setup
   - Task context in system prompt
   - Non-streaming response

3. **Streaming Agent Response** (20 min)
   - Stream=True for OpenAI
   - SSE wrapper for chunks
   - Error handling in stream

4. **Hands-On: Task AI Assistant** (15 min)
   - Implement `/tasks/{id}/assist`
   - Test with Swagger UI
   - Observe streaming in browser

**Layer 2 Approach**:
- Full AI collaboration for prompt design
- "Ask AI: How should I structure the system prompt for task context?"
- Iterate on prompt with AI feedback

**Code Milestones**:
```python
from openai import OpenAI

client = OpenAI()

# Milestone 1: Non-streaming agent endpoint
@app.post("/tasks/{task_id}/assist")
async def assist_with_task(task_id: int, question: str, repo: TaskRepository = Depends(get_task_repo)):
    task = repo.get_by_id(task_id)
    if not task:
        raise HTTPException(status_code=404, detail="Task not found")

    response = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {"role": "system", "content": f"You are helping with task: {task['title']}. Description: {task.get('description', 'None')}"},
            {"role": "user", "content": question}
        ]
    )
    return {"answer": response.choices[0].message.content}

# Milestone 2: Streaming agent endpoint
@app.post("/tasks/{task_id}/assist/stream")
async def stream_assist(task_id: int, question: str, repo: TaskRepository = Depends(get_task_repo)):
    task = repo.get_by_id(task_id)
    if not task:
        raise HTTPException(status_code=404, detail="Task not found")

    async def generate():
        stream = client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": f"Helping with: {task['title']}"},
                {"role": "user", "content": question}
            ],
            stream=True
        )
        for chunk in stream:
            if chunk.choices[0].delta.content:
                yield {"data": chunk.choices[0].delta.content}

    return EventSourceResponse(generate())
```

---

### 2.8 Lesson 8: Capstone — Complete Task API (90 min)

**File**: `08-capstone-complete-task-api.md`

**Learning Objectives**:
- Synthesize all chapter concepts
- Build from specification
- Document API completely

**Structure**:
1. **Specification Review** (10 min)
   - API requirements
   - Endpoint list
   - Feature checklist

2. **Implementation Phase** (50 min)
   - Set up project structure
   - Implement all endpoints
   - Add streaming + agent

3. **Documentation Phase** (15 min)
   - OpenAPI descriptions
   - Example values
   - Response schemas

4. **Testing Phase** (15 min)
   - Full CRUD workflow
   - Error cases
   - Streaming verification

**Specification Given to Students**:
```yaml
project: Task Management API with AI Assistant

endpoints:
  - POST /tasks
      description: Create a new task
      request_body: TaskCreate
      response: TaskResponse (201)

  - GET /tasks
      description: List all tasks
      query_params: status (optional)
      response: List[TaskResponse]

  - GET /tasks/{task_id}
      description: Get single task
      response: TaskResponse (200) or 404

  - PUT /tasks/{task_id}
      description: Update task
      request_body: TaskUpdate
      response: TaskResponse (200) or 404

  - DELETE /tasks/{task_id}
      description: Delete task
      response: 204 or 404

  - POST /tasks/{task_id}/assist
      description: AI help for task
      request_body: question (str)
      response: AssistResponse

  - GET /tasks/stream
      description: Real-time task updates
      response: SSE stream

requirements:
  - Dependency injection for TaskRepository
  - Proper error handling (404, 400, 422)
  - OpenAPI documentation complete
  - Streaming works in browser
  - AI responds with task context
```

---

## 3. File Deliverables

### 3.1 Lesson Files
| File | Lesson | Status |
|------|--------|--------|
| `01-hello-fastapi.md` | Hello FastAPI | Pending |
| `02-post-and-pydantic-models.md` | POST + Pydantic | Pending |
| `03-full-crud-operations.md` | Full CRUD | Pending |
| `04-error-handling.md` | Error Handling | Pending |
| `05-dependency-injection.md` | Dependency Injection | Pending |
| `06-streaming-with-sse.md` | Streaming | Pending |
| `07-agent-integration.md` | Agent Integration | Pending |
| `08-capstone-complete-task-api.md` | Capstone | Pending |

### 3.2 Support Files
| File | Purpose |
|------|---------|
| `README.md` | Chapter overview (exists) |
| `_category_.json` | Docusaurus sidebar config |
| Quiz file (optional) | `09_chapter_40_quiz.md` |

---

## 4. Quality Gates

### 4.1 Per-Lesson Validation
- [ ] YAML frontmatter complete
- [ ] Learning objectives measurable
- [ ] Code examples run correctly
- [ ] Hands-on exercises clear
- [ ] AI collaboration invisible (no framework labels)
- [ ] Cognitive load within B1 limits (7-10 concepts)

### 4.2 Chapter-Level Validation
- [ ] Domain consistent (Task Management)
- [ ] Manual-first progression (L1→L2→L4)
- [ ] Agent integration only in L7-8
- [ ] All code tested and working
- [ ] OpenAPI documentation complete

---

## 5. Implementation Order

1. Create `_category_.json` for Docusaurus
2. Implement Lesson 1 (foundation for all others)
3. Implement Lessons 2-4 (core CRUD)
4. Implement Lessons 5-6 (advanced patterns)
5. Implement Lesson 7 (agent integration)
6. Implement Lesson 8 (capstone synthesis)
7. Validate all lessons
8. Create quiz (optional)

---

*Plan generated by sp.orchestrator following SDD-RI workflow*
*Spec reference: specs/040-chapter-40-fastapi-for-agents/spec.md*
