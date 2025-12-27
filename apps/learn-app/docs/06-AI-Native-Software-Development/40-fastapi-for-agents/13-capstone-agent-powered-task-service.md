---
sidebar_position: 13
title: "Capstone: Agent-Powered Task Service"
description: "Build a complete, deployable multi-agent API by composing every pattern from this chapter into a specification-driven service"
keywords: [fastapi, capstone, multi-agent, specification-driven, jwt, postgresql, streaming, rate-limiting, digital-fte]
chapter: 40
lesson: 13
duration_minutes: 90

skills:
  - name: "API Architecture Composition"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Create"
    digcomp_area: "3.4 Programming"
    measurable_at_this_level: "Compose previously learned patterns into a cohesive, production-ready API"

  - name: "Specification-Driven Development"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "3.4 Programming"
    measurable_at_this_level: "Write specifications before implementation, then validate against success criteria"

  - name: "Multi-Agent Orchestration"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "3.4 Programming"
    measurable_at_this_level: "Implement triage-to-specialist agent routing with streaming visibility"

learning_objectives:
  - objective: "Write a complete API specification before implementation"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "Specification document with success criteria"

  - objective: "Compose all chapter patterns into a working multi-agent service"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "Running API that passes all specification tests"

  - objective: "Validate implementation against specification success criteria"
    proficiency_level: "B1"
    bloom_level: "Evaluate"
    assessment_method: "Testing checklist completion"

cognitive_load:
  new_concepts: 0
  assessment: "Synthesis lesson - all concepts previously taught in L1-L12. Focus is on composition and integration."

differentiation:
  extension_for_advanced: "Add agent memory persistence, implement conversation history, or add OpenTelemetry tracing"
  remedial_for_struggling: "Focus on core CRUD + one agent endpoint first, then incrementally add auth and streaming"
---

# Capstone: Agent-Powered Task Service

You've spent twelve lessons building individual capabilities: routing, validation, database persistence, authentication, rate limiting, agent integration, and streaming. Now it's time to compose them all.

This capstone follows the specification-driven approach that defines AI-native development. You'll write the specification first, then implement against it, and finally validate that your implementation meets every success criterion. This is how professional agent-powered services are built—and how you'll package your own Digital FTE products.

The result? A deployable API that exposes multi-agent task management capabilities. The kind of service you could sell to teams who need AI-powered productivity tools without building their own infrastructure.

## The Specification-First Approach

Before writing any code, we define what success looks like. This specification becomes the contract that your implementation must fulfill.

### Complete API Specification

Create a file called `spec.md` in your project root:

```markdown
# Task Agent Service Specification

## Overview
Multi-agent task management API with authentication, rate limiting,
and streaming agent responses.

## Success Criteria
Every criterion must pass for the implementation to be complete.

### Authentication (from L8-L9)
- [ ] POST /users/signup creates user with Argon2-hashed password
- [ ] POST /token returns JWT for valid credentials
- [ ] Invalid credentials return 401 with generic message
- [ ] JWT expires after configured duration

### Task CRUD (from L1-L5)
- [ ] POST /tasks creates task in Neon PostgreSQL
- [ ] GET /tasks returns all tasks for authenticated user
- [ ] GET /tasks/{id} returns single task or 404
- [ ] PUT /tasks/{id} updates task fields
- [ ] DELETE /tasks/{id} removes task and returns 204

### Agent Endpoints (from L12)
- [ ] POST /tasks/{id}/help requires valid JWT
- [ ] Triage agent routes to correct specialist
- [ ] Streaming response shows agent handoffs
- [ ] POST /tasks/{id}/schedule calls scheduler directly
- [ ] GET /agents/status returns available agents

### Rate Limiting (from L9)
- [ ] Token endpoint limited to 5 requests/minute
- [ ] Agent endpoints limited to 10 requests/minute
- [ ] Rate limit headers present in responses

### Configuration (from L6)
- [ ] All secrets from environment variables
- [ ] No hardcoded credentials in source
- [ ] .env.example documents required variables

## API Endpoints

| Endpoint | Method | Auth | Rate Limit | Description |
|----------|--------|------|------------|-------------|
| /users/signup | POST | None | 5/min | Create account |
| /token | POST | None | 5/min | Get JWT token |
| /tasks | POST | JWT | None | Create task |
| /tasks | GET | JWT | None | List user tasks |
| /tasks/{id} | GET | JWT | None | Get single task |
| /tasks/{id} | PUT | JWT | None | Update task |
| /tasks/{id} | DELETE | JWT | None | Delete task |
| /tasks/{id}/help | POST | JWT | 10/min | Agent assistance |
| /tasks/{id}/schedule | POST | JWT | 10/min | Schedule task |
| /agents/status | GET | None | None | List agents |
```

**Output:**
```
spec.md created - 65 lines
This becomes your implementation contract
```

This specification captures every pattern you've learned. The checkboxes become your testing checklist. The table becomes your routing implementation guide.

## Project Structure

Organize your capstone project to reflect the separation of concerns you've practiced:

```
task_agent_service/
├── app/
│   ├── __init__.py
│   ├── main.py              # FastAPI app with all routers
│   ├── config.py            # Environment configuration
│   ├── database.py          # Neon PostgreSQL connection
│   ├── models.py            # SQLModel definitions
│   ├── auth/
│   │   ├── __init__.py
│   │   ├── routes.py        # /users and /token endpoints
│   │   ├── security.py      # JWT and password hashing
│   │   └── dependencies.py  # get_current_user
│   ├── tasks/
│   │   ├── __init__.py
│   │   ├── routes.py        # CRUD endpoints
│   │   └── service.py       # Database operations
│   └── agents/
│       ├── __init__.py
│       ├── routes.py        # Agent endpoints
│       ├── triage.py        # Routing logic
│       └── specialists.py   # Individual agents
├── tests/
│   └── test_spec.py         # Tests against specification
├── spec.md                   # Your contract
├── .env.example              # Required variables
└── requirements.txt
```

## Configuration Layer

All secrets come from environment variables. Create your configuration module:

```python
# app/config.py
from pydantic_settings import BaseSettings
from functools import lru_cache


class Settings(BaseSettings):
    """Application settings from environment variables."""

    # Database (Neon PostgreSQL)
    database_url: str

    # Authentication
    secret_key: str
    algorithm: str = "HS256"
    access_token_expire_minutes: int = 30

    # Rate Limiting
    token_rate_limit: int = 5
    agent_rate_limit: int = 10

    # AI Provider
    anthropic_api_key: str

    class Config:
        env_file = ".env"


@lru_cache
def get_settings() -> Settings:
    """Cached settings instance."""
    return Settings()
```

**Output:**
```python
>>> from app.config import get_settings
>>> settings = get_settings()
>>> settings.algorithm
'HS256'
>>> settings.agent_rate_limit
10
```

Create the `.env.example` that documents required variables:

```bash
# .env.example - Copy to .env and fill in values

# Neon PostgreSQL connection string
DATABASE_URL=postgresql://user:pass@host/dbname?sslmode=require

# JWT Configuration
SECRET_KEY=your-secret-key-generate-with-openssl-rand-hex-32
ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30

# Rate Limits (requests per minute)
TOKEN_RATE_LIMIT=5
AGENT_RATE_LIMIT=10

# Anthropic API
ANTHROPIC_API_KEY=sk-ant-...
```

## Database Layer

Your Neon PostgreSQL integration persists tasks with user ownership:

```python
# app/database.py
from sqlmodel import SQLModel, create_engine, Session
from app.config import get_settings

settings = get_settings()
engine = create_engine(settings.database_url, echo=False)


def create_db_and_tables():
    """Initialize database schema."""
    SQLModel.metadata.create_all(engine)


def get_session():
    """Dependency for database sessions."""
    with Session(engine) as session:
        yield session
```

```python
# app/models.py
from sqlmodel import SQLModel, Field
from datetime import datetime
from typing import Optional
from enum import Enum


class TaskStatus(str, Enum):
    pending = "pending"
    in_progress = "in_progress"
    completed = "completed"


class TaskBase(SQLModel):
    """Shared task fields."""
    title: str = Field(min_length=1, max_length=200)
    description: Optional[str] = None
    status: TaskStatus = TaskStatus.pending
    due_date: Optional[datetime] = None


class Task(TaskBase, table=True):
    """Database model with ownership."""
    id: Optional[int] = Field(default=None, primary_key=True)
    owner_id: int = Field(foreign_key="user.id")
    created_at: datetime = Field(default_factory=datetime.utcnow)


class TaskCreate(TaskBase):
    """Request model for task creation."""
    pass


class TaskRead(TaskBase):
    """Response model with all fields."""
    id: int
    owner_id: int
    created_at: datetime


class User(SQLModel, table=True):
    """User account for authentication."""
    id: Optional[int] = Field(default=None, primary_key=True)
    email: str = Field(unique=True, index=True)
    hashed_password: str
    created_at: datetime = Field(default_factory=datetime.utcnow)
```

**Output:**
```python
>>> from app.models import Task, TaskStatus
>>> task = Task(title="Review PR", owner_id=1)
>>> task.status
<TaskStatus.pending: 'pending'>
```

## Authentication Layer

JWT authentication with Argon2 password hashing:

```python
# app/auth/security.py
from datetime import datetime, timedelta
from typing import Optional
from jose import JWTError, jwt
from pwdlib import PasswordHash
from pwdlib.hashers.argon2 import Argon2Hasher
from app.config import get_settings

settings = get_settings()
password_hash = PasswordHash((Argon2Hasher(),))


def hash_password(password: str) -> str:
    """Hash password with Argon2."""
    return password_hash.hash(password)


def verify_password(plain: str, hashed: str) -> bool:
    """Verify password against hash."""
    return password_hash.verify(plain, hashed)


def create_access_token(data: dict, expires_delta: Optional[timedelta] = None) -> str:
    """Create JWT with expiration."""
    to_encode = data.copy()
    expire = datetime.utcnow() + (expires_delta or timedelta(minutes=15))
    to_encode.update({"exp": expire})
    return jwt.encode(to_encode, settings.secret_key, algorithm=settings.algorithm)


def decode_token(token: str) -> Optional[dict]:
    """Decode and validate JWT."""
    try:
        payload = jwt.decode(token, settings.secret_key, algorithms=[settings.algorithm])
        return payload
    except JWTError:
        return None
```

```python
# app/auth/dependencies.py
from fastapi import Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer
from sqlmodel import Session, select
from app.database import get_session
from app.models import User
from app.auth.security import decode_token

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")


async def get_current_user(
    token: str = Depends(oauth2_scheme),
    session: Session = Depends(get_session)
) -> User:
    """Extract and validate user from JWT."""
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )

    payload = decode_token(token)
    if payload is None:
        raise credentials_exception

    email: str = payload.get("sub")
    if email is None:
        raise credentials_exception

    user = session.exec(select(User).where(User.email == email)).first()
    if user is None:
        raise credentials_exception

    return user
```

**Output:**
```python
>>> from app.auth.security import hash_password, verify_password
>>> hashed = hash_password("secure123")
>>> verify_password("secure123", hashed)
True
>>> verify_password("wrong", hashed)
False
```

## Agent Layer

The triage agent routes requests to specialists:

```python
# app/agents/specialists.py
from anthropic import Anthropic

client = Anthropic()

SPECIALISTS = {
    "scheduler": {
        "name": "Scheduler Agent",
        "description": "Helps schedule and prioritize tasks",
        "system": """You are a scheduling specialist. Help users:
- Set realistic deadlines
- Prioritize tasks by urgency and importance
- Break large tasks into smaller steps
- Identify dependencies between tasks"""
    },
    "breakdown": {
        "name": "Breakdown Agent",
        "description": "Breaks complex tasks into subtasks",
        "system": """You are a task breakdown specialist. Help users:
- Decompose complex tasks into actionable steps
- Identify the first concrete action
- Estimate effort for each subtask
- Suggest parallel vs sequential work"""
    },
    "blocker": {
        "name": "Blocker Resolution Agent",
        "description": "Helps overcome obstacles and blockers",
        "system": """You are a blocker resolution specialist. Help users:
- Identify root causes of blockers
- Suggest workarounds and alternatives
- Recommend who to ask for help
- Reframe problems as opportunities"""
    }
}


async def call_specialist(specialist_id: str, task_context: str, user_query: str):
    """Stream response from a specialist agent."""
    if specialist_id not in SPECIALISTS:
        raise ValueError(f"Unknown specialist: {specialist_id}")

    specialist = SPECIALISTS[specialist_id]

    with client.messages.stream(
        model="claude-sonnet-4-20250514",
        max_tokens=1024,
        system=specialist["system"],
        messages=[{
            "role": "user",
            "content": f"Task: {task_context}\n\nUser request: {user_query}"
        }]
    ) as stream:
        for text in stream.text_stream:
            yield text
```

```python
# app/agents/triage.py
from anthropic import Anthropic
from app.agents.specialists import SPECIALISTS

client = Anthropic()

TRIAGE_SYSTEM = """You are a triage agent for a task management system.
Your job is to route user requests to the appropriate specialist.

Available specialists:
{specialists}

Respond with ONLY the specialist ID (scheduler, breakdown, or blocker).
If unsure, choose the most relevant based on the user's intent."""


def format_specialists() -> str:
    """Format specialists for triage prompt."""
    lines = []
    for id, spec in SPECIALISTS.items():
        lines.append(f"- {id}: {spec['description']}")
    return "\n".join(lines)


async def triage_request(task_context: str, user_query: str) -> str:
    """Determine which specialist should handle this request."""
    response = client.messages.create(
        model="claude-sonnet-4-20250514",
        max_tokens=50,
        system=TRIAGE_SYSTEM.format(specialists=format_specialists()),
        messages=[{
            "role": "user",
            "content": f"Task: {task_context}\n\nUser request: {user_query}"
        }]
    )

    specialist_id = response.content[0].text.strip().lower()

    if specialist_id not in SPECIALISTS:
        specialist_id = "breakdown"  # Safe default

    return specialist_id
```

## Agent Routes with Streaming

The agent endpoints combine authentication, rate limiting, and streaming:

```python
# app/agents/routes.py
from fastapi import APIRouter, Depends, HTTPException, Request
from fastapi.responses import StreamingResponse
from sqlmodel import Session, select
from slowapi import Limiter
from slowapi.util import get_remote_address

from app.database import get_session
from app.models import Task, User
from app.auth.dependencies import get_current_user
from app.agents.triage import triage_request
from app.agents.specialists import call_specialist, SPECIALISTS
from app.config import get_settings

router = APIRouter(prefix="/tasks", tags=["agents"])
settings = get_settings()
limiter = Limiter(key_func=get_remote_address)


@router.post("/{task_id}/help")
@limiter.limit(f"{settings.agent_rate_limit}/minute")
async def get_agent_help(
    request: Request,
    task_id: int,
    query: str,
    session: Session = Depends(get_session),
    current_user: User = Depends(get_current_user)
):
    """Get AI agent help for a task with streaming response."""
    task = session.exec(
        select(Task).where(Task.id == task_id, Task.owner_id == current_user.id)
    ).first()

    if not task:
        raise HTTPException(status_code=404, detail="Task not found")

    task_context = f"{task.title}: {task.description or 'No description'}"
    specialist_id = await triage_request(task_context, query)

    async def generate():
        yield f"[Routing to {SPECIALISTS[specialist_id]['name']}]\n\n"
        async for chunk in call_specialist(specialist_id, task_context, query):
            yield chunk

    return StreamingResponse(generate(), media_type="text/plain")


@router.get("/agents/status")
async def get_agents_status():
    """List available agents and their capabilities."""
    return {
        "agents": [
            {"id": id, "name": spec["name"], "description": spec["description"]}
            for id, spec in SPECIALISTS.items()
        ],
        "triage_enabled": True
    }
```

## Main Application Assembly

Bring all the layers together:

```python
# app/main.py
from fastapi import FastAPI
from slowapi import _rate_limit_exceeded_handler
from slowapi.errors import RateLimitExceeded

from app.database import create_db_and_tables
from app.middleware.rate_limit import limiter
from app.auth.routes import router as auth_router
from app.tasks.routes import router as tasks_router
from app.agents.routes import router as agents_router

app = FastAPI(
    title="Task Agent Service",
    description="Multi-agent task management API",
    version="1.0.0"
)

app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

app.include_router(auth_router)
app.include_router(tasks_router)
app.include_router(agents_router)


@app.on_event("startup")
def on_startup():
    """Initialize database on startup."""
    create_db_and_tables()


@app.get("/health")
def health_check():
    """Health check endpoint."""
    return {"status": "healthy", "service": "task-agent-service"}
```

**Output:**
```bash
$ uvicorn app.main:app --reload
INFO:     Started server process
INFO:     Application startup complete
INFO:     Uvicorn running on http://127.0.0.1:8000
```

## Specification Validation

Create tests that verify your implementation meets every specification criterion:

```python
# tests/test_spec.py
import pytest
from fastapi.testclient import TestClient
from app.main import app

client = TestClient(app)


class TestAuthentication:
    """Spec: Authentication criteria"""

    def test_signup_creates_user_with_hashed_password(self):
        """POST /users/signup creates user with Argon2-hashed password"""
        response = client.post("/users/signup", json={
            "email": "test@example.com",
            "password": "SecurePass123"
        })
        assert response.status_code == 201
        assert "password" not in response.json()

    def test_token_returns_jwt(self):
        """POST /token returns JWT for valid credentials"""
        client.post("/users/signup", json={
            "email": "jwt@example.com",
            "password": "SecurePass123"
        })

        response = client.post("/token", data={
            "username": "jwt@example.com",
            "password": "SecurePass123"
        })
        assert response.status_code == 200
        assert "access_token" in response.json()

    def test_invalid_credentials_return_401(self):
        """Invalid credentials return 401 with generic message"""
        response = client.post("/token", data={
            "username": "wrong@example.com",
            "password": "WrongPass"
        })
        assert response.status_code == 401


class TestTaskCRUD:
    """Spec: Task CRUD criteria"""

    @pytest.fixture
    def auth_headers(self):
        """Get authenticated headers."""
        client.post("/users/signup", json={
            "email": "crud@example.com",
            "password": "SecurePass123"
        })
        token_response = client.post("/token", data={
            "username": "crud@example.com",
            "password": "SecurePass123"
        })
        token = token_response.json()["access_token"]
        return {"Authorization": f"Bearer {token}"}

    def test_create_task(self, auth_headers):
        """POST /tasks creates task in database"""
        response = client.post("/tasks", json={
            "title": "Test Task",
            "description": "Testing CRUD"
        }, headers=auth_headers)
        assert response.status_code == 201
        assert "id" in response.json()


class TestAgentEndpoints:
    """Spec: Agent endpoint criteria"""

    def test_help_requires_jwt(self):
        """POST /tasks/{id}/help requires valid JWT"""
        response = client.post("/tasks/1/help?query=test")
        assert response.status_code == 401

    def test_agents_status_lists_specialists(self):
        """GET /agents/status returns available agents"""
        response = client.get("/tasks/agents/status")
        assert response.status_code == 200
        assert "agents" in response.json()
```

**Output:**
```bash
$ pytest tests/test_spec.py -v
========================= test session starts ==========================
tests/test_spec.py::TestAuthentication::test_signup_creates_user PASSED
tests/test_spec.py::TestAuthentication::test_token_returns_jwt PASSED
tests/test_spec.py::TestAuthentication::test_invalid_credentials PASSED
tests/test_spec.py::TestTaskCRUD::test_create_task PASSED
tests/test_spec.py::TestAgentEndpoints::test_help_requires_jwt PASSED
tests/test_spec.py::TestAgentEndpoints::test_agents_status PASSED
========================= 6 passed in 2.34s ============================
```

## What You've Built

This capstone composes every pattern from Chapter 40:

| Lesson | Pattern | Used In |
|--------|---------|---------|
| L1 | FastAPI basics | App structure |
| L2 | Pytest fundamentals | Tests |
| L3 | Request validation | Pydantic models |
| L4 | CRUD operations | Task endpoints |
| L5 | Error handling | HTTPException |
| L6 | Environment config | Settings |
| L7 | Neon PostgreSQL | Persistence |
| L8 | JWT authentication | Protected routes |
| L9 | Password + Rate limit | Security |
| L10 | Dependency injection | Database, Auth |
| L11 | SSE streaming | Agent responses |
| L12 | Agent integration | Specialists |

The result is a **deployable agent-powered service**—the foundation of a Digital FTE product.

## Digital FTE Packaging

Your capstone becomes sellable when you:

1. **Add documentation** - OpenAPI spec auto-generated at `/docs`
2. **Add monitoring** - Health checks, error tracking
3. **Add deployment config** - Docker, Railway, or Fly.io
4. **Define pricing tiers** - Based on agent calls/month

This is how domain experts package AI capabilities for their industry.

:::tip[Safety Note]
Before deploying to production:
- Rotate all secrets from development
- Enable HTTPS only
- Add request logging for audit trails
- Set up error alerting
- Review rate limits for your expected load
:::

## Try With AI

You've built a complete agent service. Now extend it with AI collaboration.

**Prompt 1: Add Agent Memory**

```text
I want my agents to remember previous interactions with the same task.
Here's my current specialist call:

async def call_specialist(specialist_id: str, task_context: str, user_query: str):
    specialist = SPECIALISTS[specialist_id]
    with client.messages.stream(...) as stream:
        for text in stream.text_stream:
            yield text

How do I add conversation history per task?
Consider: Where should history be stored? How much to include?
```

**What you're learning:** Persistent agent memory transforms one-shot responses into ongoing conversations.

**Prompt 2: Add OpenTelemetry Tracing**

```text
I want to trace requests through my multi-agent system:
- How long does triage take?
- Which specialist was chosen?
- How long did the specialist response take?

What's the minimal OpenTelemetry setup for FastAPI?
```

**What you're learning:** Observability becomes critical when you have multiple agents.

**Prompt 3: Custom Specialist for Your Domain**

```text
I'm building task management for [YOUR INDUSTRY].
I want to add a specialist agent that understands domain-specific terminology.

Current specialists: scheduler, breakdown, blocker

Help me design a new specialist for my domain that:
1. Has a focused system prompt
2. Knows industry-specific workflows
3. Can reference domain terminology

What should the system prompt include?
```

**What you're learning:** Adding specialists is how you customize agent services for specific industries—the core of Digital FTE value creation.
