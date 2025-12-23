---
sidebar_position: 4
title: "Testing and Quality Gates"
description: "Implement test stages that block deployment on failure"
keywords: [testing, pytest, coverage, quality gates, ci, github actions, linting]
---

# Testing and Quality Gates

You've built images in CI and pushed them to registries. But before deploying an image to production, someone needs to verify it works. In Lesson 1, you learned that the **Test stage** is the quality gate—the checkpoint that prevents broken code from reaching users.

This lesson teaches you how to implement that test stage in GitHub Actions. You'll write tests with pytest, measure code coverage, enforce coverage thresholds, lint your code, and configure your workflow to **fail if any test fails**. No exceptions. No warnings that get ignored. A single failing test stops the entire pipeline.

By the end of this lesson, you'll understand how automated tests become a safety net that developers trust, and how quality gates make deployments safer.

## Why Tests in CI Matter (Beyond "Best Practice")

Without tests in CI, here's what happens:

1. Developer pushes code
2. Image builds successfully
3. Image is deployed to production
4. A subtle bug appears in production (could have been caught by tests)
5. Users notice and report the bug
6. Rollback happens, timeline is disrupted

With tests in CI:

1. Developer pushes code
2. Tests run automatically
3. A test catches the bug
4. The pipeline fails
5. Developer fixes the bug before deployment
6. Tests pass, deployment proceeds

**The test stage is your defense against shipping broken code.** In a team setting, tests are the only thing preventing one person's mistake from taking down everyone's work.

## Unit Testing with pytest

Python projects use pytest for unit testing. Let's understand what a test looks like for your FastAPI agent.

### A Minimal FastAPI Test

Here's your FastAPI agent with a simple endpoint:

```python
# app/main.py
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

app = FastAPI()

class Task(BaseModel):
    id: int
    title: str
    completed: bool = False

@app.post("/tasks")
async def create_task(task: Task):
    if not task.title:
        raise HTTPException(status_code=400, detail="Title is required")
    return {"id": task.id, "title": task.title, "completed": task.completed}

@app.get("/tasks/{task_id}")
async def get_task(task_id: int):
    return {"id": task_id, "title": "Sample Task", "completed": False}
```

**Output:**
```
App defined with POST /tasks and GET /tasks/{id} endpoints
```

Now here's a test for this endpoint:

```python
# tests/test_main.py
import pytest
from fastapi.testclient import TestClient
from app.main import app

client = TestClient(app)

def test_create_task_success():
    """Test creating a task with valid data"""
    response = client.post("/tasks", json={
        "id": 1,
        "title": "Deploy FastAPI agent",
        "completed": False
    })
    assert response.status_code == 201
    assert response.json()["title"] == "Deploy FastAPI agent"

def test_create_task_missing_title():
    """Test that missing title returns 400"""
    response = client.post("/tasks", json={
        "id": 1,
        "title": "",
        "completed": False
    })
    assert response.status_code == 400
```

**Output:**
```
tests/test_main.py::test_create_task_success PASSED
tests/test_main.py::test_create_task_missing_title PASSED
============ 2 passed in 0.25s ============
```

The test creates a client, calls your endpoint, and asserts the response status code and body. If any assertion fails, the test fails.

### Running Tests Locally

Before CI, you run tests on your machine to develop:

```bash
pytest tests/
```

**Output:**
```
tests/test_main.py::test_create_task_success PASSED
tests/test_main.py::test_create_task_missing_title PASSED
tests/test_main.py::test_get_task PASSED
============ 3 passed in 0.34s ============
```

If a test fails:

```bash
pytest tests/test_main.py::test_create_task_success
```

**Output:**
```
tests/test_main.py::test_create_task_success FAILED

AssertionError: assert 400 == 201
```

pytest shows exactly which assertion failed and why. This feedback loop—write code, run tests, see failures, fix code—is how developers build confidence.

## Code Coverage: Measuring Test Quality

A passing test is good, but does it actually test the important parts of your code? **Code coverage** measures what percentage of your code is executed during tests.

### Running Coverage Reports

Use pytest-cov to measure coverage:

```bash
pip install pytest-cov
pytest --cov=app --cov-report=html tests/
```

**Output:**
```
tests/test_main.py::test_create_task_success PASSED
tests/test_main.py::test_create_task_missing_title PASSED
============ 2 passed in 0.25s ============

---------- coverage: platform linux -- python 3.11.0-final-0.7.0 ----------
Name                      Stmts   Miss  Cover
---------------------------------------------
app/main.py                  15      2    87%
app/config.py                8      0   100%
app/models.py               12      3    75%
---------------------------------------------
TOTAL                       35      5    86%
```

Coverage shows:
- **Stmts**: Total lines of code
- **Miss**: Lines not executed by tests
- **Cover**: Percentage executed

If you have 86% coverage, 14% of your code isn't tested. That could be edge cases or error handling that only triggers in production.

### Coverage Thresholds: Enforcing Minimum Quality

A quality gate enforces a minimum coverage threshold. If coverage drops below the threshold, the pipeline fails:

```bash
pytest --cov=app --cov-fail-under=80 tests/
```

**Output (if coverage is 86%):**
```
============ 2 passed in 0.25s ============

---------- coverage: platform linux -- python 3.11.0-final-0.7.0 ----------
TOTAL                       35      5    86%
PASSED ✓ (86% >= 80% threshold)
```

**Output (if coverage is 75%):**
```
============ 2 passed in 0.25s ============

---------- coverage: platform linux -- python 3.11.0-final-0.7.0 ----------
TOTAL                       35     10    75%
FAILED ✗ (75% < 80% threshold)
```

The `--cov-fail-under=80` flag makes pytest exit with a failure code if coverage doesn't meet the threshold. In CI, this failure stops the pipeline.

## Linting: Catching Code Style Issues

Beyond functional tests, **linting** checks code style and catches common mistakes. Tools like ruff or flake8 scan your code for issues without running it.

### Running a Linter

```bash
pip install ruff
ruff check app/ tests/
```

**Output (clean code):**
```
All checks passed!
```

**Output (with issues):**
```
app/main.py:5:1: E302 expected 2 blank lines, found 1
app/main.py:12:80: E501 line too long (85 > 79 characters)
app/main.py:28:15: F841 local variable 'temp' is assigned but never used
```

Linting catches:
- Unused imports
- Undefined variables
- Lines too long
- Missing docstrings
- Inconsistent style

In CI, a linting failure stops the pipeline just like a test failure.

## Quality Gates in GitHub Actions

Now let's put this together in a GitHub Actions workflow. Your CI pipeline needs:

1. Install dependencies
2. Run tests with coverage
3. Run linter
4. Fail the job if any check fails

### Complete Test and Quality Gate Workflow

Here's a workflow that runs tests and enforces quality gates:

```yaml
# .github/workflows/ci.yml
name: CI - Build, Test, and Push

on:
  push:
    branches:
      - main
  pull_request:
    branches:
      - main

jobs:
  test:
    runs-on: ubuntu-latest

    steps:
      - name: Checkout code
        uses: actions/checkout@v4

      - name: Set up Python
        uses: actions/setup-python@v4
        with:
          python-version: '3.11'

      - name: Install dependencies
        run: |
          python -m pip install --upgrade pip
          pip install pytest pytest-cov ruff
          pip install -r requirements.txt

      - name: Run linter
        run: ruff check app/ tests/

      - name: Run tests with coverage
        run: |
          pytest \
            --cov=app \
            --cov-fail-under=80 \
            --cov-report=term-missing \
            --cov-report=html \
            tests/

      - name: Upload coverage report
        uses: actions/upload-artifact@v3
        if: always()
        with:
          name: coverage-report
          path: htmlcov/

  build:
    needs: test
    runs-on: ubuntu-latest

    steps:
      - name: Checkout code
        uses: actions/checkout@v4

      - name: Build Docker image
        run: |
          docker build -t agent-task-service:${{ github.sha }} .
```

**Explanation**: This workflow defines two jobs (test and build) where build only runs if test succeeds (via `needs: test`).

**Output (when tests pass):**
```
test job:
  Checkout code ... DONE
  Set up Python ... DONE
  Install dependencies ... DONE
  Run linter ... PASSED ✓
  Run tests with coverage ...
    tests/test_main.py::test_create_task_success PASSED
    tests/test_main.py::test_create_task_missing_title PASSED
    ============ 2 passed in 0.25s ============
    TOTAL coverage 86% (>= 80% threshold) ✓
  Upload coverage report ... DONE

build job:
  Starts (because test job passed)
  Build Docker image ... DONE
```

**Output (when tests fail):**
```
test job:
  Checkout code ... DONE
  Set up Python ... DONE
  Install dependencies ... DONE
  Run linter ... PASSED ✓
  Run tests with coverage ...
    tests/test_main.py::test_create_task_success FAILED
    AssertionError: assert 400 == 201
  PIPELINE STOPPED ✗

build job:
  SKIPPED (because test job failed)
```

Notice the `needs: test` line in the build job. This creates a dependency: build only starts if test passes. If test fails, build never runs.

## Integration Tests with Service Containers

Unit tests verify individual functions. Integration tests verify components work together—especially when external services are involved. For a FastAPI agent that uses PostgreSQL, integration tests need a real database.

GitHub Actions supports **service containers**—temporary databases or services that spin up for your tests, then tear down.

### Integration Test with PostgreSQL

Here's an integration test that reads from a database:

```python
# tests/test_integration_db.py
import os
import pytest
import asyncpg
from app.main import app

@pytest.fixture
async def db_connection():
    """Connect to test PostgreSQL database"""
    # Environment variable set by GitHub Actions service container
    dsn = os.getenv("DATABASE_URL")
    conn = await asyncpg.connect(dsn)
    yield conn
    await conn.close()

async def test_task_persists_to_db(db_connection):
    """Test that task is saved to database"""
    # Insert task via app endpoint
    client = TestClient(app)
    response = client.post("/tasks", json={
        "id": 1,
        "title": "Test task",
        "completed": False
    })

    # Verify it's in the database
    result = await db_connection.fetchrow(
        "SELECT * FROM tasks WHERE id = $1", 1
    )
    assert result["title"] == "Test task"
```

**Output:**
```
tests/test_integration_db.py::test_task_persists_to_db PASSED
```

This test needs a real PostgreSQL running. GitHub Actions can provide it:

### Workflow with Service Container

```yaml
# .github/workflows/ci.yml (updated)
jobs:
  test:
    runs-on: ubuntu-latest

    services:
      postgres:
        image: postgres:15-alpine
        env:
          POSTGRES_PASSWORD: password
          POSTGRES_DB: test_db
        options: >-
          --health-cmd pg_isready
          --health-interval 10s
          --health-timeout 5s
          --health-retries 5
        ports:
          - 5432:5432

    steps:
      - name: Checkout code
        uses: actions/checkout@v4

      - name: Set up Python
        uses: actions/setup-python@v4
        with:
          python-version: '3.11'

      - name: Install dependencies
        run: |
          pip install pytest pytest-cov asyncpg
          pip install -r requirements.txt

      - name: Create database schema
        run: |
          psql -h localhost -U postgres -d test_db -c "
            CREATE TABLE tasks (
              id SERIAL PRIMARY KEY,
              title VARCHAR NOT NULL,
              completed BOOLEAN DEFAULT FALSE
            );"
        env:
          PGPASSWORD: password

      - name: Run tests (unit + integration)
        run: |
          pytest \
            --cov=app \
            --cov-fail-under=80 \
            tests/
        env:
          DATABASE_URL: postgresql://postgres:password@localhost:5432/test_db
```

**Explanation**: This workflow adds a `services` section with PostgreSQL, waits for health checks, creates the database schema, then runs tests with database connectivity.

**Output:**
```
services:
  postgres: Started on localhost:5432
  Health check passed ✓

test job:
  Create database schema ... DONE
  Run tests (unit + integration) ...
    tests/test_main.py::test_create_task_success PASSED
    tests/test_integration_db.py::test_task_persists_to_db PASSED
    ============ 2 passed in 0.35s ============
  Coverage 86% ✓
```

The service container automatically:
- Starts before tests run
- Provides a PostgreSQL database
- Stops after tests complete
- Tears down completely (no side effects)

## Fail-Fast: Stop on First Failure

Your pipeline should stop immediately when something fails. Don't continue building images and pushing to registries if tests fail.

### Explicit Job Dependencies

The workflow above uses `needs: test` to enforce dependencies:

```yaml
jobs:
  test:
    # Tests run first
    ...

  build:
    needs: test  # Build only runs if test succeeds
    ...

  push:
    needs: build  # Push only runs if build succeeds
    ...
```

**Output (execution order)**:
```
Workflow starts
  → test job begins
    → all tests run
    → if PASS: build job begins
    → if FAIL: build and push are SKIPPED
  → build job (only if test passed)
  → push job (only if build passed)
```

If any job fails, all dependent jobs are skipped. This is fail-fast behavior.

### Step Failure Behavior

By default, if a step fails, the job stops:

```yaml
steps:
  - name: Run linter
    run: ruff check app/  # If this fails...

  - name: Run tests
    run: pytest tests/    # This step never runs
```

**Output (when linter fails)**:
```
Run linter
  ruff check app/
  app/main.py:5:1: E302 expected 2 blank lines
  FAILED ✗

Run tests
  SKIPPED (because linter failed)

Job status: FAILED
```

You can override this with `continue-on-error: true`, but you shouldn't for quality gates. Failures should block the pipeline.

## Uploading Test Artifacts

GitHub Actions can upload test reports and coverage reports as artifacts. These are stored and accessible through the GitHub UI.

```yaml
- name: Upload coverage report
  uses: actions/upload-artifact@v3
  if: always()  # Upload even if tests fail
  with:
    name: coverage-report
    path: htmlcov/

- name: Upload test results
  uses: actions/upload-artifact@v3
  if: always()
  with:
    name: pytest-results
    path: test-results.xml
```

**Output (GitHub UI)**:
```
Artifacts
  ├── coverage-report/
  │   ├── index.html
  │   ├── app_main_py.html
  │   └── status.json
  └── pytest-results/
      └── test-results.xml

[Download] coverage-report (2.4 MB)
[Download] pytest-results (45 KB)
```

After the workflow runs, GitHub provides a download link for these artifacts. Developers can download the HTML coverage report and view what code wasn't tested.

## Complete Multi-Job CI Workflow

Here's a complete workflow combining everything:

```yaml
# .github/workflows/ci.yml
name: CI - Build, Test, and Push

on:
  push:
    branches:
      - main
  pull_request:
    branches:
      - main

env:
  REGISTRY: ghcr.io
  IMAGE_NAME: ${{ github.repository }}/agent-task-service

jobs:
  test:
    name: Test Suite
    runs-on: ubuntu-latest

    services:
      postgres:
        image: postgres:15-alpine
        env:
          POSTGRES_PASSWORD: testpass
          POSTGRES_DB: test_db
        options: >-
          --health-cmd pg_isready
          --health-interval 10s
          --health-timeout 5s
          --health-retries 5
        ports:
          - 5432:5432

    steps:
      - name: Checkout code
        uses: actions/checkout@v4

      - name: Set up Python
        uses: actions/setup-python@v4
        with:
          python-version: '3.11'

      - name: Install dependencies
        run: |
          python -m pip install --upgrade pip
          pip install pytest pytest-cov ruff
          pip install -r requirements.txt

      - name: Lint with ruff
        run: ruff check app/ tests/

      - name: Create test database
        run: |
          psql -h localhost -U postgres -d test_db -c "
            CREATE TABLE tasks (
              id SERIAL PRIMARY KEY,
              title VARCHAR NOT NULL,
              completed BOOLEAN DEFAULT FALSE
            );"
        env:
          PGPASSWORD: testpass

      - name: Run pytest with coverage
        run: |
          pytest \
            --cov=app \
            --cov-fail-under=80 \
            --cov-report=term-missing \
            --cov-report=xml \
            tests/
        env:
          DATABASE_URL: postgresql://postgres:testpass@localhost:5432/test_db

      - name: Upload coverage to artifacts
        uses: actions/upload-artifact@v3
        if: always()
        with:
          name: coverage-report
          path: htmlcov/

  build:
    name: Build Docker Image
    needs: test
    runs-on: ubuntu-latest

    permissions:
      contents: read
      packages: write

    steps:
      - name: Checkout code
        uses: actions/checkout@v4

      - name: Set up Docker Buildx
        uses: docker/setup-buildx-action@v2

      - name: Log in to Container Registry
        uses: docker/login-action@v2
        with:
          registry: ${{ env.REGISTRY }}
          username: ${{ github.actor }}
          password: ${{ secrets.GITHUB_TOKEN }}

      - name: Build and push Docker image
        uses: docker/build-push-action@v4
        with:
          context: .
          push: true
          tags: |
            ${{ env.REGISTRY }}/${{ env.IMAGE_NAME }}:${{ github.sha }}
            ${{ env.REGISTRY }}/${{ env.IMAGE_NAME }}:latest
          cache-from: type=gha
          cache-to: type=gha,mode=max
```

**Output (complete success):**
```
Workflow: CI - Build, Test, and Push

test job:
  Lint with ruff ........................... PASSED ✓
  Run pytest with coverage ................ PASSED ✓
    8 passed, coverage 85% >= 80% threshold
  Upload coverage to artifacts ............. DONE

build job:
  Starts (because test passed)
  Build and push Docker image ............. DONE ✓
    Pushed: ghcr.io/panaversity/agent-task-service:abc123def456
    Pushed: ghcr.io/panaversity/agent-task-service:latest

All jobs passed ✓
```

**Output (test failure):**
```
Workflow: CI - Build, Test, and Push

test job:
  Lint with ruff ........................... PASSED ✓
  Run pytest with coverage ................ FAILED ✗
    tests/test_main.py::test_create_task FAILED
    assert 400 == 201

build job:
  SKIPPED (because test failed)

Workflow failed. Build not triggered.
```

## Key Concepts

**Quality Gate**: An automated checkpoint that must pass before the pipeline continues. If any test fails, coverage drops, or linter finds issues, the pipeline stops.

**Test Coverage**: The percentage of code executed by your tests. Higher coverage (80%+) reduces the risk of uncaught bugs reaching production.

**Fail-Fast**: Stop immediately when a quality gate fails. Don't waste resources building and pushing images if tests will reject the code.

**Service Containers**: Temporary databases or services that spin up for tests and tear down automatically, ensuring tests are isolated and repeatable.

**Artifacts**: Files (like coverage reports) uploaded to GitHub for review. Developers can download and inspect what tests covered.

## Try With AI

Ask Claude: "I have a FastAPI application with 80% test coverage. Add quality gates to my GitHub Actions workflow that fail if coverage drops below 80% or if any linting errors are found."

Before accepting the output:
- Does it use `pytest --cov-fail-under=80`?
- Does it include a separate linting step with ruff?
- Does it fail the job (not just warn) when thresholds aren't met?

After Claude provides the workflow, ask: "Now add integration tests that require a PostgreSQL database using GitHub Actions service containers. The tests should verify that tasks are persisted to the database."

Verify the response includes:
- A `services` section in the job with PostgreSQL configuration
- Health checks to wait for the database to be ready
- A step to create the database schema before tests run
- Environment variables passed to pytest for database connection
- Tests that actually interact with the database (not mocked)

Finally, ask: "Ensure the workflow has three jobs—test, build, and push—where build only runs if test passes, and push only runs if build succeeds. Show me the complete workflow with all three jobs."

Check that:
- Each job has a clear `needs:` dependency
- Test job includes linting, pytest, and coverage checks
- Build job uses `docker/build-push-action`
- Push job (if included) pushes to a registry
- No job runs if its dependency fails
