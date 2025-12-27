---
sidebar_position: 7
title: "SQLModel + Neon Setup"
description: "Connect your FastAPI app to Neon PostgreSQL using SQLModel—the fast track to persistent data"
keywords: [sqlmodel, neon, postgresql, database, orm, fastapi]
chapter: 40
lesson: 7
duration_minutes: 45

skills:
  - name: "SQLModel Table Definition"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "3.4 Programming"
    measurable_at_this_level: "Student creates SQLModel classes with table=True"

  - name: "Neon Database Connection"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "3.4 Programming"
    measurable_at_this_level: "Student connects to Neon using connection string"

  - name: "Session Management"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "3.4 Programming"
    measurable_at_this_level: "Student uses Session for database operations"

learning_objectives:
  - objective: "Create SQLModel table definitions"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Model class with fields maps to database table"

  - objective: "Connect to Neon PostgreSQL"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Connection string from settings, engine created"

  - objective: "Perform CRUD with Session"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Add, query, update, delete work correctly"

cognitive_load:
  new_concepts: 5
  assessment: "SQLModel, table=True, create_engine, Session, Neon setup"

differentiation:
  extension_for_advanced: "Add relationships and foreign keys"
  remedial_for_struggling: "Start with single table before relationships"
---

# SQLModel + Neon Setup

Your Task API currently stores data in a Python list. Restart the server—everything's gone. Real APIs need persistent storage. This lesson connects your FastAPI app to Neon PostgreSQL using SQLModel.

Why this combination?

- **SQLModel** = Pydantic + SQLAlchemy in one. Your models work for both validation AND database.
- **Neon** = PostgreSQL as a service. No installation, free tier, instant setup.

This is the "fast track" approach. No migrations, no complex setup. Get persistent data working, then add complexity later if needed.

## Setting Up Neon

**Step 1:** Go to [neon.tech](https://neon.tech) and create a free account

**Step 2:** Create a new project (default settings are fine)

**Step 3:** Copy your connection string:

```
postgresql://user:password@ep-xxx.region.aws.neon.tech/neondb?sslmode=require
```

**Step 4:** Add to your `.env`:

```bash
DATABASE_URL=postgresql://user:password@ep-xxx.region.aws.neon.tech/neondb?sslmode=require
```

That's it. You have a PostgreSQL database.

## Installing SQLModel

```bash
uv add sqlmodel psycopg2-binary
```

- `sqlmodel` - The ORM that combines Pydantic and SQLAlchemy
- `psycopg2-binary` - PostgreSQL driver

## Defining Your Model

SQLModel classes with `table=True` become database tables:

```python
# models.py
from sqlmodel import SQLModel, Field
from typing import Optional
from datetime import datetime


class Task(SQLModel, table=True):
    """Task stored in database."""
    id: Optional[int] = Field(default=None, primary_key=True)
    title: str = Field(min_length=1, max_length=200)
    description: Optional[str] = None
    status: str = Field(default="pending")
    created_at: datetime = Field(default_factory=datetime.utcnow)
```

**What each part means:**

- `table=True` - This model maps to a database table
- `primary_key=True` - Auto-incrementing ID
- `Field(default=...)` - Column defaults
- `Optional[int] = None` - ID is None until database assigns it

## Connecting to the Database

```python
# database.py
from sqlmodel import SQLModel, create_engine, Session
from config import get_settings

settings = get_settings()

engine = create_engine(settings.database_url, echo=True)


def create_db_and_tables():
    """Create all tables in the database."""
    SQLModel.metadata.create_all(engine)


def get_session():
    """Dependency that provides database sessions."""
    with Session(engine) as session:
        yield session
```

**Key concepts:**

- `create_engine()` - Establishes connection to Neon
- `echo=True` - Logs SQL queries (helpful for debugging)
- `Session` - Context for database operations
- `yield session` - Provides session to endpoint, closes when done

## Creating Tables on Startup

Call `create_db_and_tables()` when the app starts:

```python
# main.py
from fastapi import FastAPI
from database import create_db_and_tables

app = FastAPI()


@app.on_event("startup")
def on_startup():
    create_db_and_tables()
```

First time you run the app, the table is created in Neon.

## CRUD Operations with Session

### Create

```python
from fastapi import Depends
from sqlmodel import Session
from models import Task
from database import get_session


@app.post("/tasks", status_code=201)
def create_task(
    task: Task,
    session: Session = Depends(get_session)
):
    session.add(task)
    session.commit()
    session.refresh(task)  # Get the assigned ID
    return task
```

**Output:**
```json
{
  "id": 1,
  "title": "Learn SQLModel",
  "description": null,
  "status": "pending",
  "created_at": "2024-01-15T10:30:00"
}
```

### Read (List)

```python
from sqlmodel import select


@app.get("/tasks")
def list_tasks(session: Session = Depends(get_session)):
    tasks = session.exec(select(Task)).all()
    return tasks
```

### Read (Single)

```python
from fastapi import HTTPException


@app.get("/tasks/{task_id}")
def get_task(task_id: int, session: Session = Depends(get_session)):
    task = session.get(Task, task_id)
    if not task:
        raise HTTPException(status_code=404, detail="Task not found")
    return task
```

### Update

```python
@app.put("/tasks/{task_id}")
def update_task(
    task_id: int,
    task_update: Task,
    session: Session = Depends(get_session)
):
    task = session.get(Task, task_id)
    if not task:
        raise HTTPException(status_code=404, detail="Task not found")

    task.title = task_update.title
    task.description = task_update.description
    task.status = task_update.status

    session.add(task)
    session.commit()
    session.refresh(task)
    return task
```

### Delete

```python
@app.delete("/tasks/{task_id}", status_code=204)
def delete_task(task_id: int, session: Session = Depends(get_session)):
    task = session.get(Task, task_id)
    if not task:
        raise HTTPException(status_code=404, detail="Task not found")

    session.delete(task)
    session.commit()
    return None
```

## Complete Example

Here's everything together:

```python
# models.py
from sqlmodel import SQLModel, Field
from typing import Optional
from datetime import datetime


class Task(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    title: str = Field(min_length=1, max_length=200)
    description: Optional[str] = None
    status: str = Field(default="pending")
    created_at: datetime = Field(default_factory=datetime.utcnow)
```

```python
# database.py
from sqlmodel import SQLModel, create_engine, Session
from config import get_settings

settings = get_settings()
engine = create_engine(settings.database_url, echo=False)


def create_db_and_tables():
    SQLModel.metadata.create_all(engine)


def get_session():
    with Session(engine) as session:
        yield session
```

```python
# main.py
from fastapi import FastAPI, Depends, HTTPException
from sqlmodel import Session, select
from models import Task
from database import create_db_and_tables, get_session

app = FastAPI(title="Task API")


@app.on_event("startup")
def on_startup():
    create_db_and_tables()


@app.post("/tasks", status_code=201)
def create_task(task: Task, session: Session = Depends(get_session)):
    session.add(task)
    session.commit()
    session.refresh(task)
    return task


@app.get("/tasks")
def list_tasks(session: Session = Depends(get_session)):
    return session.exec(select(Task)).all()


@app.get("/tasks/{task_id}")
def get_task(task_id: int, session: Session = Depends(get_session)):
    task = session.get(Task, task_id)
    if not task:
        raise HTTPException(status_code=404, detail="Task not found")
    return task


@app.put("/tasks/{task_id}")
def update_task(
    task_id: int,
    task_update: Task,
    session: Session = Depends(get_session)
):
    task = session.get(Task, task_id)
    if not task:
        raise HTTPException(status_code=404, detail="Task not found")

    task.title = task_update.title
    task.description = task_update.description
    task.status = task_update.status

    session.add(task)
    session.commit()
    session.refresh(task)
    return task


@app.delete("/tasks/{task_id}", status_code=204)
def delete_task(task_id: int, session: Session = Depends(get_session)):
    task = session.get(Task, task_id)
    if not task:
        raise HTTPException(status_code=404, detail="Task not found")
    session.delete(task)
    session.commit()
```

**Output:**
```bash
$ curl -X POST http://localhost:8000/tasks \
  -H "Content-Type: application/json" \
  -d '{"title": "Learn SQLModel"}'

{"id":1,"title":"Learn SQLModel","description":null,"status":"pending","created_at":"2024-01-15T10:30:00"}

$ curl http://localhost:8000/tasks
[{"id":1,"title":"Learn SQLModel","description":null,"status":"pending","created_at":"2024-01-15T10:30:00"}]
```

Restart the server. The data persists. That's the difference a database makes.

## Hands-On Exercise

**Step 1:** Create a Neon account and get your connection string

**Step 2:** Add DATABASE_URL to your .env

**Step 3:** Create models.py and database.py as shown above

**Step 4:** Update main.py to use the database

**Step 5:** Test CRUD operations through Swagger UI

**Step 6:** Restart the server and verify data persists

## Common Mistakes

**Mistake 1:** Forgetting `table=True`

```python
# Wrong - just a Pydantic model, no database table
class Task(SQLModel):
    title: str

# Correct - creates database table
class Task(SQLModel, table=True):
    title: str
```

**Mistake 2:** Missing primary key

```python
# Wrong - no primary key defined
class Task(SQLModel, table=True):
    title: str

# Correct - has primary key
class Task(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    title: str
```

**Mistake 3:** Not calling session.commit()

```python
# Wrong - changes not saved
session.add(task)
return task  # ID is still None!

# Correct - commit saves to database
session.add(task)
session.commit()
session.refresh(task)  # Now has ID
return task
```

**Mistake 4:** Using wrong connection string

```python
# Wrong - missing sslmode for Neon
DATABASE_URL=postgresql://user:pass@host/db

# Correct - SSL required for Neon
DATABASE_URL=postgresql://user:pass@host/db?sslmode=require
```

## What About Migrations?

This "fast track" approach uses `create_all()` which:
- Creates tables that don't exist
- Does NOT modify existing tables

If you change your model (add a column), you need to either:
1. Drop and recreate the table (loses data)
2. Use Alembic migrations (covered in Chapter 47)

For learning, the fast track is fine. For production, you'll add migrations.

## Try With AI

After completing the exercise, explore these scenarios.

**Prompt 1: Filtering Queries**

```text
I have tasks in my database. How do I filter them?
For example, get only tasks with status="pending"
or tasks created in the last 7 days.

Show me SQLModel select() with where() clauses.
```

**What you're learning:** Select queries can filter, sort, and limit results. This is essential for real-world APIs.

**Prompt 2: Separate Request/Response Models**

```text
My Task model has table=True for the database, but I want
different fields for API requests and responses.
For example, I don't want clients to set created_at.

How do I separate database models from API models in SQLModel?
```

**What you're learning:** Real APIs often have TaskCreate, TaskRead, and Task (database) as separate models. SQLModel supports this pattern.

**Prompt 3: Connection Pooling**

```text
My API might get many concurrent requests. Each request
creates a new database connection with get_session().
Is this efficient? Should I use connection pooling?
```

**What you're learning:** Production databases use connection pools. SQLAlchemy (which SQLModel uses) has pooling built in—understanding when it matters is important.
