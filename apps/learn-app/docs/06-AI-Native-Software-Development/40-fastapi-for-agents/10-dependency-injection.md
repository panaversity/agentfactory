---
sidebar_position: 10
title: "Dependency Injection"
description: "Organize your code with FastAPI's Depends() system—connecting database, settings, and authentication"
keywords: [dependency-injection, depends, fastapi, inversion-of-control, clean-code]
chapter: 40
lesson: 10
duration_minutes: 45

skills:
  - name: "Depends() Pattern"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "3.4 Programming"
    measurable_at_this_level: "Student creates and uses dependency functions"

  - name: "Dependency Chaining"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "3.4 Programming"
    measurable_at_this_level: "Student builds dependencies that use other dependencies"

  - name: "Testing with Dependencies"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "3.4 Programming"
    measurable_at_this_level: "Student explains how to override dependencies in tests"

learning_objectives:
  - objective: "Create custom dependency functions"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Dependency provides resource to endpoint"

  - objective: "Chain dependencies together"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "get_current_user depends on get_session"

  - objective: "Override dependencies for testing"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Tests use mock database"

cognitive_load:
  new_concepts: 4
  assessment: "Depends(), yield dependencies, dependency graph, override"

differentiation:
  extension_for_advanced: "Implement scoped dependencies and custom parameter classes"
  remedial_for_struggling: "Focus on single dependency before chaining"
---

# Dependency Injection

You've been using `Depends()` throughout this chapter without deep explanation. Now let's understand it properly. Dependency injection is how you organize code that needs shared resources: database connections, settings, authentication, services.

The pattern is simple: functions that provide something other functions need. FastAPI calls them automatically and handles cleanup.

## What Is Dependency Injection?

Without DI, you'd do this:

```python
@app.get("/tasks")
def list_tasks():
    # Create database connection in every function
    settings = Settings()
    engine = create_engine(settings.database_url)
    session = Session(engine)
    try:
        tasks = session.exec(select(Task)).all()
        return tasks
    finally:
        session.close()
```

Problems:
- Connection created for every request
- Cleanup code everywhere
- Hard to test (can't inject mock database)
- Repeated boilerplate

With DI:

```python
@app.get("/tasks")
def list_tasks(session: Session = Depends(get_session)):
    return session.exec(select(Task)).all()
```

The `get_session` dependency handles creation and cleanup. Your endpoint focuses on business logic.

## Creating Dependencies

A dependency is just a function (or callable):

```python
def get_settings() -> Settings:
    """Provide application settings."""
    return Settings()


@app.get("/config")
def show_config(settings: Settings = Depends(get_settings)):
    return {"debug": settings.debug}
```

FastAPI:
1. Sees `Depends(get_settings)`
2. Calls `get_settings()`
3. Passes the result to your function
4. All automatically

## Yield Dependencies for Cleanup

Database connections need cleanup. Use `yield` instead of `return`:

```python
def get_session():
    """Provide database session with automatic cleanup."""
    session = Session(engine)
    try:
        yield session  # Provide to endpoint
    finally:
        session.close()  # Cleanup after endpoint completes
```

The code after `yield` runs after your endpoint finishes—even if it raises an exception.

## The Dependencies You've Built

Looking back at this chapter, here's what you've created:

```python
# Settings dependency (L6)
@lru_cache
def get_settings() -> Settings:
    return Settings()


# Database session dependency (L7)
def get_session():
    with Session(engine) as session:
        yield session


# Current user dependency (L8)
async def get_current_user(
    token: str = Depends(oauth2_scheme),
    session: Session = Depends(get_session)
) -> User:
    # Decode token, find user
    ...
```

Notice how `get_current_user` depends on BOTH `oauth2_scheme` AND `get_session`. This is dependency chaining.

## Dependency Chaining

Dependencies can depend on other dependencies:

```python
def get_session():
    with Session(engine) as session:
        yield session


def get_current_user(
    token: str = Depends(oauth2_scheme),
    session: Session = Depends(get_session)
) -> User:
    """Get authenticated user. Depends on session."""
    payload = decode_token(token)
    if not payload:
        raise HTTPException(status_code=401)

    user = session.exec(
        select(User).where(User.email == payload["sub"])
    ).first()
    if not user:
        raise HTTPException(status_code=401)

    return user


def get_admin_user(
    current_user: User = Depends(get_current_user)
) -> User:
    """Get user who is also an admin. Depends on current_user."""
    if not current_user.is_admin:
        raise HTTPException(status_code=403, detail="Admin required")
    return current_user


@app.get("/admin/stats")
def admin_stats(admin: User = Depends(get_admin_user)):
    """Only accessible by admins."""
    return {"users": 100, "tasks": 500}
```

The dependency graph:

```
admin_stats
    └── get_admin_user
            └── get_current_user
                    ├── oauth2_scheme
                    └── get_session
```

FastAPI resolves this automatically. Each dependency runs once per request, even if used multiple times.

## Organizing Dependencies

As your app grows, organize dependencies by concern:

```
app/
├── dependencies/
│   ├── __init__.py
│   ├── database.py    # get_session
│   ├── auth.py        # get_current_user, get_admin_user
│   └── settings.py    # get_settings
├── main.py
└── ...
```

```python
# dependencies/database.py
from sqlmodel import Session
from database import engine


def get_session():
    with Session(engine) as session:
        yield session
```

```python
# dependencies/auth.py
from fastapi import Depends, HTTPException
from fastapi.security import OAuth2PasswordBearer
from sqlmodel import Session, select
from models import User
from auth import decode_token
from .database import get_session

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")


async def get_current_user(
    token: str = Depends(oauth2_scheme),
    session: Session = Depends(get_session)
) -> User:
    credentials_exception = HTTPException(
        status_code=401,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )

    payload = decode_token(token)
    if not payload:
        raise credentials_exception

    user = session.exec(
        select(User).where(User.email == payload.get("sub"))
    ).first()
    if not user:
        raise credentials_exception

    return user
```

```python
# main.py
from fastapi import FastAPI, Depends
from dependencies.database import get_session
from dependencies.auth import get_current_user

app = FastAPI()


@app.get("/tasks")
def list_tasks(
    session: Session = Depends(get_session),
    current_user: User = Depends(get_current_user)
):
    return session.exec(
        select(Task).where(Task.owner_id == current_user.id)
    ).all()
```

## Testing with Dependency Overrides

The real power of DI is testability. Override any dependency:

```python
# test_main.py
from fastapi.testclient import TestClient
from sqlmodel import Session, create_engine, SQLModel
from main import app
from dependencies.database import get_session

# Test database
TEST_DATABASE_URL = "sqlite:///./test.db"
test_engine = create_engine(TEST_DATABASE_URL)


def get_test_session():
    """Override to use test database."""
    with Session(test_engine) as session:
        yield session


# Override the dependency
app.dependency_overrides[get_session] = get_test_session

client = TestClient(app)


def setup_module():
    """Create test tables."""
    SQLModel.metadata.create_all(test_engine)


def test_create_task():
    response = client.post("/tasks", json={"title": "Test"})
    assert response.status_code == 201
```

Your production code uses Neon PostgreSQL. Tests use SQLite. No code changes needed—just dependency override.

## Class-Based Dependencies

For complex dependencies, use classes:

```python
class TaskService:
    """Service for task operations."""

    def __init__(self, session: Session = Depends(get_session)):
        self.session = session

    def list_for_user(self, user_id: int) -> list[Task]:
        return self.session.exec(
            select(Task).where(Task.owner_id == user_id)
        ).all()

    def create(self, task: TaskCreate, owner_id: int) -> Task:
        db_task = Task(**task.dict(), owner_id=owner_id)
        self.session.add(db_task)
        self.session.commit()
        self.session.refresh(db_task)
        return db_task


@app.get("/tasks")
def list_tasks(
    current_user: User = Depends(get_current_user),
    service: TaskService = Depends()  # Uses TaskService.__init__
):
    return service.list_for_user(current_user.id)
```

When you use `Depends()` without arguments on a class, FastAPI:
1. Looks at `__init__` parameters
2. Resolves any `Depends()` in those parameters
3. Creates an instance

## Hands-On Exercise

**Step 1:** Create a dependencies directory structure:

```
app/
├── dependencies/
│   ├── __init__.py
│   ├── database.py
│   └── auth.py
```

**Step 2:** Move get_session to dependencies/database.py

**Step 3:** Move get_current_user to dependencies/auth.py

**Step 4:** Update imports in main.py

**Step 5:** Create a test that overrides get_session:

```python
# test_tasks.py
from fastapi.testclient import TestClient
from sqlmodel import Session, create_engine, SQLModel
from main import app
from dependencies.database import get_session

test_engine = create_engine("sqlite:///./test.db")


def get_test_session():
    with Session(test_engine) as session:
        yield session


app.dependency_overrides[get_session] = get_test_session

client = TestClient(app)


def test_list_tasks_empty():
    SQLModel.metadata.create_all(test_engine)
    # Need to also override auth for this to work
    response = client.get("/tasks")
    assert response.status_code in [200, 401]  # 401 if auth required
```

## Common Mistakes

**Mistake 1:** Creating resources without cleanup

```python
# Wrong - session never closed
def get_session():
    return Session(engine)

# Correct - yield with cleanup
def get_session():
    with Session(engine) as session:
        yield session
```

**Mistake 2:** Forgetting Depends() wrapper

```python
# Wrong - function called at import time!
@app.get("/tasks")
def list_tasks(session: Session = get_session()):
    ...

# Correct - called at request time
@app.get("/tasks")
def list_tasks(session: Session = Depends(get_session)):
    ...
```

**Mistake 3:** Not clearing overrides between tests

```python
# After tests, clear overrides
def teardown_module():
    app.dependency_overrides.clear()
```

## The Dependency Injection Mindset

Think of your app as a graph of dependencies:

```
Endpoint
    └── Business Logic
            ├── Database Session
            │       └── Engine (Settings)
            └── Current User
                    ├── Token (OAuth2)
                    └── Database Session
```

Each node can be:
- Tested in isolation
- Swapped for a mock
- Reused across endpoints

This is what makes FastAPI applications maintainable at scale.

## Try With AI

After completing the exercise, explore these scenarios.

**Prompt 1: Request-Scoped Dependencies**

```text
I have a dependency that creates an expensive resource.
I want to create it once per request and reuse it across
multiple endpoints in that request. How does FastAPI
handle dependency caching within a request?
```

**What you're learning:** FastAPI caches dependency results per request by default. Understanding this prevents unnecessary resource creation.

**Prompt 2: Background Task Dependencies**

```text
I have a background task that needs database access.
But the request's session is closed when the background
task runs. How do I provide dependencies to background tasks?
```

**What you're learning:** Background tasks have different lifecycles than requests. They need their own dependency resolution.

**Prompt 3: Conditional Dependencies**

```text
I want an endpoint that works for both authenticated
and unauthenticated users, but shows different data.
How do I make authentication optional while still
using dependency injection?
```

**What you're learning:** Optional dependencies and conditional logic based on authentication state. Common pattern for APIs with public/private data.
