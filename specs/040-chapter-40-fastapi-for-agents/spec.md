# Chapter 40: FastAPI for Agents — Specification

**Version**: 3.0.0
**Status**: Draft
**Created**: 2025-12-22
**Revised**: 2025-12-27
**Author**: Claude Code (sp.orchestrator)
**Related Issues**: #541, #542, #543, #544, #545, #546

---

## 1. Executive Summary

### 1.1 Purpose

Chapter 40 teaches how to **expose agent patterns as production REST APIs**. Students have built agents with OpenAI SDK (Chapter 34), connected them via MCP (Chapters 37-38), and created reusable skills with code execution (Chapter 39). Now they'll make these agents accessible as HTTP services.

**This chapter is the bare base for all aspects and learning pathways.** Students who complete this chapter can:
- Move directly to Part 7 (Cloud Native) for deployment
- Apply these patterns to any agent SDK
- Build production-ready API services

### 1.2 Key Insight

> **APIs are just functions. Functions become tools. Agents use tools.**

The CRUD APIs students build in Lessons 1-9 become the tools that agents call in Lessons 12-13.

### 1.3 What Makes This Chapter Valuable

1. **Testing First (L1 Manual)** — Students write tests by hand before AI generates them
2. **Production Essentials** — Real database (Neon), real auth (JWT), real security (Argon2)
3. **One Concept Per Lesson** — JWT separate from password hashing, env vars separate from SQLModel
4. **Simple SQLModel Setup** — Basics of relations and postgres with no complexity
5. **APIs → Agent Tools** — Same endpoints become callable by agents

---

## Assumed Knowledge

### What students know BEFORE this chapter

- **Python fundamentals** (Part 5): async/await, type hints, Pydantic models, decorators
- **OpenAI Agents SDK** (Chapter 34): Agent, Runner, function_tool, handoffs
- **MCP patterns** (Chapters 37-38): HTTP/SSE familiar, tool schemas
- **Skills structure** (Chapter 39): SKILL.md format, loading skills

### What this chapter must explain from scratch

- FastAPI application structure (app, routes, uvicorn)
- HTTP methods and REST conventions (GET, POST, PUT, DELETE)
- pytest and TestClient for API testing
- Environment variables and pydantic-settings
- SQLModel and PostgreSQL basics (via Neon)
- JWT authentication flow
- Password hashing with Argon2
- Rate limiting patterns
- FastAPI's Depends() for dependency injection
- Server-Sent Events (SSE) for streaming

---

## 2. Learning Objectives

### 2.1 Chapter-Level Objectives

| #   | Objective                                          | Bloom's Level | Assessment                    |
| --- | -------------------------------------------------- | ------------- | ----------------------------- |
| LO1 | Create FastAPI applications with Pydantic models   | Apply         | Working CRUD API              |
| LO2 | Write pytest tests for API endpoints               | Apply         | Test suite with assertions    |
| LO3 | Configure applications with environment variables  | Apply         | Settings class pattern        |
| LO4 | Connect to PostgreSQL via SQLModel and Neon        | Apply         | Persistent task storage       |
| LO5 | Implement JWT authentication with protected routes | Apply         | Token-based auth flow         |
| LO6 | Hash passwords securely with Argon2                | Apply         | User signup/login             |
| LO7 | Apply rate limiting to protect endpoints           | Apply         | Rate-limited login            |
| LO8 | Use FastAPI's dependency injection pattern         | Apply         | Repository injection          |
| LO9 | Stream responses using Server-Sent Events          | Apply         | SSE endpoint                  |
| LO10| Expose agents as REST endpoints with tools         | Apply         | Agent-powered API             |

---

## 3. Chapter Structure (13 Lessons)

### Tier A: FastAPI Fundamentals with Testing (Lessons 1-5)

Foundation for API development. Uses in-memory Task storage. **Pytest introduced early (L1 Manual First).**

| #  | Title                    | Duration | Focus                                      | Layer |
|----|--------------------------|----------|--------------------------------------------|-------|
| 01 | Hello FastAPI            | 45 min   | First app, uvicorn, Swagger UI             | -     |
| 02 | Pytest Fundamentals      | 45 min   | TestClient, assertions, red-green cycle    | L1    |
| 03 | POST and Pydantic Models | 50 min   | Request validation, models, manual tests   | -     |
| 04 | Full CRUD Operations     | 55 min   | GET, PUT, DELETE patterns, manual tests    | -     |
| 05 | Error Handling           | 45 min   | HTTPException, status codes, error tests   | -     |

**Pedagogical Note (Lesson 2)**: Students write tests **by hand** first. This builds mental models of assertions, fixtures, and test structure. Later lessons (12-13) show AI generating tests as L2 collaboration.

### Tier B: Production Essentials (Lessons 6-9)

Real database, real auth, real security. **One concept per lesson.**

| #  | Title                               | Duration | Focus                                      | Depends On |
|----|-------------------------------------|----------|--------------------------------------------|------------|
| 06 | Environment Variables               | 40 min   | python-dotenv, pydantic-settings, secrets  | L05        |
| 07 | SQLModel + Neon Setup               | 55 min   | Real database, ORM basics, migrations      | L06        |
| 08 | JWT Authentication                  | 50 min   | Tokens, verification, protected routes     | L06        |
| 09 | Password Hashing + Rate Limiting    | 50 min   | Argon2, signup, in-memory rate limiter     | L08        |

**One Concept Per Lesson Principle**:
- Lesson 6: Environment variables (no database yet)
- Lesson 7: Database setup (uses env vars from L06)
- Lesson 8: JWT tokens only (temporary plain-text password)
- Lesson 9: Password hashing + rate limiting (fixes L08's insecurity)

### Tier C: Dependency Injection & Streaming (Lessons 10-11)

Architectural patterns for production code.

| #  | Title                  | Duration | Focus                                      | Layer |
|----|------------------------|----------|--------------------------------------------|-------|
| 10 | Dependency Injection   | 50 min   | Depends(), repository pattern, testability | L2    |
| 11 | Streaming with SSE     | 45 min   | Server-Sent Events, async generators       | -     |

### Tier D: Agent Integration & Capstone (Lessons 12-13)

**The key insight**: APIs become tools.

| #  | Title                                 | Duration | Focus                                      | Layer |
|----|---------------------------------------|----------|--------------------------------------------|-------|
| 12 | Agent Integration                     | 50 min   | APIs → Tools → Streaming agent             | L2    |
| 13 | Capstone: TaskManager Agent Service   | 90 min   | Multi-agent with all patterns              | L4    |

**Total Duration**: ~11 hours

---

## 4. Critical Lesson Specifications

### 4.1 Lesson 2: Pytest Fundamentals (L1 Manual First)

**Why Manual First**: Students understand WHAT tests do before AI generates them. Builds mental model of assertions, fixtures, test structure.

**Layer**: L1 (Manual) — Students write tests by hand, no AI assistance.

**Scope**:
```python
# test_main.py
from fastapi.testclient import TestClient
from main import app

client = TestClient(app)

def test_read_root():
    response = client.get("/")
    assert response.status_code == 200
    assert response.json() == {"message": "Hello World"}

def test_read_root_has_message_key():
    response = client.get("/")
    assert "message" in response.json()
```

**What to Cover**:
1. Why test? (Confidence, refactoring, documentation)
2. pytest basics: test functions, assertions
3. TestClient for FastAPI
4. Running tests: `pytest`, `pytest -v`, `pytest test_main.py::test_name`
5. Test the Hello FastAPI endpoint from Lesson 1
6. Red-Green cycle (write failing test → make it pass)

**What NOT to Cover** (save for later):
- Fixtures (Lesson 3+)
- Mocking (when we have dependencies)
- Async tests (when we use async endpoints)
- AI-generated tests (Lessons 12-13 as L2)

**Dependencies**: `pytest httpx`

---

### 4.2 Lesson 6: Environment Variables

**Why Separate Lesson**: Students need to understand configuration BEFORE:
- Connecting to external databases (Neon)
- Configuring API keys
- Managing secrets safely

**Scope**:
```python
from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    app_name: str = "Task API"
    debug: bool = False
    database_url: str  # Required - will error if missing
    secret_key: str    # For JWT in later lesson

    class Config:
        env_file = ".env"

settings = Settings()
```

**What to Cover**:
1. Why environment variables (secrets, environments, 12-factor apps)
2. Using `python-dotenv` for local development
3. Creating `.env` file with sample values
4. Using `pydantic-settings` for typed configuration
5. Accessing config in FastAPI (Settings class pattern)
6. `.env.example` for team collaboration
7. **Never commit secrets** (`.gitignore` `.env`)

**Dependencies**: `python-dotenv pydantic-settings`

---

### 4.3 Lesson 7: SQLModel + Neon Setup (Fast Track)

**Why Simple**: Get basics of relations and postgres in simple way with no complexity.

**Scope**:
```python
from sqlmodel import SQLModel, Field, create_engine, Session

class Task(SQLModel, table=True):
    id: int | None = Field(default=None, primary_key=True)
    title: str
    description: str | None = None
    completed: bool = False

# Create engine from environment variable
engine = create_engine(settings.database_url)

# Create tables
SQLModel.metadata.create_all(engine)
```

**What to Cover**:
1. Install SQLModel + asyncpg + psycopg2
2. Set up Neon account (free tier) - https://console.neon.tech/
3. Configure connection string via environment variable (NEON_DATABASE_URL)
4. Define Task SQLModel model
5. Convert existing CRUD endpoints to use SQLModel
6. Run migrations with `SQLModel.metadata.create_all()`

**What NOT to Cover** (save for later):
- Repository pattern abstraction (Lesson 10: DI)
- Session dependency injection (Lesson 10: DI)
- Complex migration tools (Alembic)
- Advanced relations (one lesson = one concept)

**Dependencies**: `sqlmodel asyncpg psycopg2-binary`

**Future Integration Note**: Neon MCP server could enable Claude as Digital FTE to manage databases directly. See https://neon.com/guides/neon-mcp-server

---

### 4.4 Lesson 8: JWT Authentication (Tokens Only)

**Why Tokens Only**: Separate concept from password hashing. One lesson = one concept.

**Scope**:
```python
import jwt
from datetime import datetime, timedelta
from fastapi.security import OAuth2PasswordBearer

SECRET_KEY = settings.secret_key
ALGORITHM = "HS256"

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")

def create_access_token(data: dict, expires_delta: timedelta = timedelta(minutes=30)):
    to_encode = data.copy()
    expire = datetime.utcnow() + expires_delta
    to_encode.update({"exp": expire})
    return jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)

async def get_current_user(token: str = Depends(oauth2_scheme)):
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        username = payload.get("sub")
        if username is None:
            raise HTTPException(status_code=401)
    except jwt.InvalidTokenError:
        raise HTTPException(status_code=401)
    return username

@app.get("/tasks")
async def get_my_tasks(current_user: str = Depends(get_current_user)):
    # Only authenticated users reach here
    return await get_tasks_for_user(current_user)
```

**Temporary Login** (fixed in Lesson 9):
```python
# Temporary: plain text check (Lesson 9 adds proper hashing)
@app.post("/token")
async def login(form_data: OAuth2PasswordRequestForm = Depends()):
    user = get_user(form_data.username)
    if not user or form_data.password != "temp_password":  # INSECURE - fixed next lesson
        raise HTTPException(status_code=401)

    token = create_access_token(data={"sub": user.username})
    return {"access_token": token, "token_type": "bearer"}
```

**What to Cover**:
1. Why JWT for APIs (stateless, token structure)
2. Token creation with expiration
3. Token verification dependency
4. OAuth2PasswordBearer pattern
5. Protected endpoints with Depends(get_current_user)
6. Temporary plain-text password (explicitly noted as insecure)

**Dependencies**: `pyjwt`

---

### 4.5 Lesson 9: Password Hashing + Rate Limiting

**Why Combined**: Both are security hardening of the auth system from Lesson 8.

**Password Hashing** (using pwdlib/Argon2, NOT bcrypt):
```python
from pwdlib import PasswordHash

password_hash = PasswordHash.recommended()

def hash_password(password: str) -> str:
    return password_hash.hash(password)

def verify_password(plain: str, hashed: str) -> bool:
    return password_hash.verify(plain, hashed)

@app.post("/users/signup", status_code=201)
async def signup(username: str, password: str, session: SessionDep):
    hashed = hash_password(password)
    user = User(username=username, hashed_password=hashed)
    session.add(user)
    await session.commit()
    return {"username": user.username}
```

**In-Memory Rate Limiting**:
```python
from collections import defaultdict
from time import time

request_counts: dict[str, list[float]] = defaultdict(list)
RATE_LIMIT = 5  # requests
RATE_WINDOW = 60  # seconds

def check_rate_limit(client_ip: str) -> bool:
    now = time()
    request_counts[client_ip] = [
        t for t in request_counts[client_ip]
        if now - t < RATE_WINDOW
    ]

    if len(request_counts[client_ip]) >= RATE_LIMIT:
        return False

    request_counts[client_ip].append(now)
    return True

async def rate_limit(request: Request):
    client_ip = request.client.host
    if not check_rate_limit(client_ip):
        raise HTTPException(status_code=429, detail="Too many requests")

@app.post("/token", dependencies=[Depends(rate_limit)])
async def login(...):
    # Now with proper password verification
    if not user or not verify_password(form_data.password, user.hashed_password):
        raise HTTPException(status_code=401)
    ...
```

**Why In-Memory (Not Redis)?**
| Approach   | Pros            | Cons                      | Use When         |
|------------|-----------------|---------------------------|------------------|
| In-memory  | Simple, no deps | Resets on restart, single | Learning, dev    |
| Redis      | Persistent      | Extra service             | Production       |

**Dependencies**: `"pwdlib[argon2]"`

---

### 4.6 Lesson 12: Agent Integration (Simplified)

**Core Insight**: APIs → Tools → Streaming

**Simplified Scope** (per Issue #545):
```python
from agents import function_tool

# These endpoints we built...
@app.post("/tasks")
async def create_task(task: TaskCreate): ...

@app.get("/tasks/{task_id}")
async def get_task(task_id: int): ...

# Functions become tools
@function_tool
def create_task_tool(title: str, description: str) -> dict:
    """Create a new task for the user."""
    response = client.post("/tasks", json={...})
    return response.json()

@function_tool
def get_task_tool(task_id: int) -> dict:
    """Get a task by ID."""
    response = client.get(f"/tasks/{task_id}")
    return response.json()

# Agent streams via SSE
@app.get("/agent/chat")
async def agent_chat(message: str):
    async def generate():
        async for event in Runner.run_streamed(agent, message):
            yield f"data: {event.model_dump_json()}\n\n"

    return StreamingResponse(generate(), media_type="text/event-stream")
```

**What to Keep**:
- Single agent with 2-3 tools (CRUD operations)
- SSE streaming endpoint
- Basic tool call → response flow

**What to Remove** (moved to Capstone):
- Complex handoff patterns
- Multiple specialist agents
- Elaborate orchestration

**Learning Outcome**:
> "The REST APIs I built are just functions. Functions become tools. An agent uses tools and streams results."

---

### 4.7 Lesson 13: Capstone — Agent-Powered Task Service

**Full System** combining ALL patterns:

1. **Triage Agent** — Routes requests to specialists
2. **Scheduler Specialist** — Handles deadlines, reminders (with tools)
3. **Collaboration Specialist** — Handles delegation, sharing (with tools)
4. **Skill Integration** — All agents use SKILL.md for consistent behavior
5. **Streaming** — All responses stream with tool call events
6. **Full Auth** — JWT + hashed passwords + rate limiting
7. **Real Database** — Neon PostgreSQL via SQLModel

**API Endpoints**:
```
# CRUD (Lessons 1-9)
POST /tasks              - Create task
GET  /tasks              - List tasks
GET  /tasks/{id}         - Get task
PUT  /tasks/{id}         - Update task
DELETE /tasks/{id}       - Delete task

# Auth (Lessons 8-9)
POST /users/signup       - Create account
POST /token              - Get JWT token

# Agent (Lessons 12-13)
POST /tasks/{id}/help    - Triage → Specialist (streaming)
POST /tasks/{id}/schedule - Direct to scheduler agent
GET  /agents/status      - Which agents are available
```

---

## 5. Dependencies (Full Chapter)

```bash
# Core FastAPI
fastapi uvicorn

# Testing (L1 Manual)
pytest httpx

# Configuration
python-dotenv pydantic-settings

# Database
sqlmodel asyncpg psycopg2-binary

# Auth
pyjwt "pwdlib[argon2]"

# Streaming
sse-starlette

# Agent (for Lessons 12-13)
openai-agents
```

---

## 6. Non-Goals (Clarified)

| Topic              | Chapter Coverage                          |
|--------------------|-------------------------------------------|
| Authentication     | **Covered here** (JWT, Argon2, rate limit)|
| Databases          | **Covered here** (SQLModel + Neon basics) |
| Testing            | **Covered here** (pytest L1 Manual)       |
| Docker             | Part 7 (Chapter 50)                       |
| Complex migrations | Chapter 47 (Relational DBs deep dive)     |
| OAuth2/SSO         | Chapter 88 (BetterAuth)                   |

---

## 7. Success Criteria

### Content Success

- [ ] Lesson 2 introduces pytest as L1 Manual (students write tests by hand)
- [ ] Lessons 3-5 include manual test examples for each CRUD operation
- [ ] Lesson 6 creates Settings class pattern used by all later lessons
- [ ] Lesson 7 connects to real Neon PostgreSQL, replaces in-memory storage
- [ ] Lesson 8 implements JWT without password hashing (separated concern)
- [ ] Lesson 9 adds Argon2 hashing and in-memory rate limiting
- [ ] Lesson 10 refactors with Depends() and repository pattern
- [ ] Lesson 12 demonstrates APIs → Tools → Streaming (simplified)
- [ ] Lesson 13 combines all patterns in multi-agent capstone

### Student Outcomes

- [ ] Can write pytest tests for FastAPI endpoints (L1 Manual)
- [ ] Can configure apps with pydantic-settings
- [ ] Can connect to Neon PostgreSQL via SQLModel
- [ ] Can implement JWT authentication with protected routes
- [ ] Can hash passwords with Argon2 and apply rate limiting
- [ ] Can use FastAPI's Depends() for clean architecture
- [ ] Can stream responses via SSE
- [ ] Can expose agents as REST endpoints with tools

### Thesis Alignment

- [ ] Chapter serves as "bare base for all aspects and learning pathways"
- [ ] Students who complete this can move directly to Part 7
- [ ] APIs built here become agent tools (key insight demonstrated)
- [ ] Pattern is applicable to any agent SDK

---

## 8. File Renumbering Required

### Current Files (8 lessons)

```
01-hello-fastapi.md
02-post-and-pydantic-models.md
03-full-crud-operations.md
04-error-handling.md
05-dependency-injection.md
06-streaming-with-sse.md
07-agent-integration.md
08-capstone-agent-powered-task-service.md
```

### Target Files (13 lessons)

```
01-hello-fastapi.md                      (existing)
02-pytest-fundamentals.md                (NEW - Issue #544)
03-post-and-pydantic-models.md           (renumbered from 02)
04-full-crud-operations.md               (renumbered from 03)
05-error-handling.md                     (renumbered from 04)
06-environment-variables.md              (NEW - Issue #542)
07-sqlmodel-neon-setup.md                (NEW - Issue #541)
08-jwt-authentication.md                 (NEW - Issue #543)
09-password-hashing-rate-limiting.md     (NEW - Issue #546)
10-dependency-injection.md               (renumbered from 05)
11-streaming-with-sse.md                 (renumbered from 06)
12-agent-integration.md                  (simplified per Issue #545)
13-capstone-agent-powered-task-service.md (renumbered from 08)
```

---

## 9. Issue Traceability

| Issue | Title                                      | Lesson | Status  |
|-------|-------------------------------------------|--------|---------|
| #544  | Add Pytest Fundamentals (L1 Manual First) | 02     | Pending |
| #542  | Add Environment Variables lesson          | 06     | Pending |
| #541  | Add Fast Track SQLModel Setup             | 07     | Pending |
| #543  | Add JWT Authentication (tokens only)      | 08     | Pending |
| #546  | Add Password Hashing + Rate Limiting      | 09     | Pending |
| #545  | Simplify Agent Integration                | 12     | Pending |

---

*Specification v3.0.0 - Revised per Issues #541-#546. Added 5 new lessons, established one-concept-per-lesson principle, introduced pytest early as L1 Manual First.*
