---
sidebar_position: 2
title: "Pytest Fundamentals"
description: "Write your first API tests by hand before adding automation—understanding the test-first mindset"
keywords: [pytest, testing, testclient, fastapi, tdd, red-green]
chapter: 40
lesson: 2
duration_minutes: 45

skills:
  - name: "Test Function Structure"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "3.4 Programming"
    measurable_at_this_level: "Student writes test functions with assert statements"

  - name: "TestClient Usage"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "3.4 Programming"
    measurable_at_this_level: "Student uses TestClient to make HTTP requests in tests"

  - name: "Red-Green Testing Cycle"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "3.4 Programming"
    measurable_at_this_level: "Student explains write-fail-pass cycle"

learning_objectives:
  - objective: "Write pytest test functions with assertions"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student creates test_*.py files that pytest discovers"

  - objective: "Use TestClient to test FastAPI endpoints"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Tests make GET/POST requests and check responses"

  - objective: "Run tests and interpret results"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student runs pytest and fixes failing tests"

cognitive_load:
  new_concepts: 5
  assessment: "Test functions, assert statements, TestClient, running pytest, interpreting failures"

differentiation:
  extension_for_advanced: "Add parametrized tests and fixtures for shared setup"
  remedial_for_struggling: "Focus on single test function before adding complexity"
---

# Pytest Fundamentals

Before you build APIs, you need to know if they work. Testing isn't something you add later—it's how you verify your code does what you think it does. In this lesson, you'll write tests by hand. No AI assistance. No shortcuts. You need to feel the red-green cycle in your bones.

Why manual first? Because when you ask AI to help with tests later, you need to recognize good tests from bad ones. You need to know what a failing test tells you. You can't evaluate AI suggestions if you've never written tests yourself.

## Why Testing Matters for Agent APIs

When you build APIs that agents call, testing becomes even more critical:

- **Agents can't guess** - They call exactly what your API exposes
- **Errors cascade** - A broken endpoint breaks every agent that uses it
- **Debugging is hard** - Agent failures often trace back to API changes
- **Confidence enables iteration** - Tests let you refactor without fear

By the end of this chapter, every endpoint you build will have tests. This lesson teaches you how.

## Your First Test

Create a file called `test_main.py` in your project:

```python
# test_main.py
from fastapi.testclient import TestClient
from main import app

client = TestClient(app)


def test_read_root():
    response = client.get("/")
    assert response.status_code == 200
    assert response.json() == {"message": "Hello, World!"}
```

**Breaking this down:**

1. **TestClient** wraps your FastAPI app for testing
2. **client.get("/")** makes a GET request to the root endpoint
3. **assert** statements verify the response

If any assert fails, the test fails. That's the entire mechanism.

## Running Tests

Install pytest if you haven't:

```bash
uv add --dev pytest
```

Run your tests:

```bash
pytest test_main.py -v
```

**Output:**
```
========================= test session starts ==========================
collected 1 item

test_main.py::test_read_root PASSED                               [100%]

========================= 1 passed in 0.15s ============================
```

The `-v` flag shows verbose output—which tests ran, which passed.

## The Red-Green Cycle

This is the fundamental rhythm of test-driven development:

1. **Write a failing test (RED)** - Test something that doesn't exist yet
2. **Make it pass (GREEN)** - Write the minimum code to pass
3. **Refactor** - Clean up while tests stay green

Let's practice. Write a test for an endpoint that doesn't exist:

```python
def test_health_check():
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json()["status"] == "healthy"
```

Run it:

```bash
pytest test_main.py::test_health_check -v
```

**Output:**
```
test_main.py::test_health_check FAILED                            [100%]

========================= FAILURES =========================
_________________ test_health_check _________________

    def test_health_check():
        response = client.get("/health")
>       assert response.status_code == 200
E       assert 404 == 200

========================= 1 failed in 0.12s ============================
```

**RED.** The test fails because `/health` doesn't exist. Now make it pass.

Add to `main.py`:

```python
@app.get("/health")
def health_check():
    return {"status": "healthy"}
```

Run the test again:

```bash
pytest test_main.py::test_health_check -v
```

**Output:**
```
test_main.py::test_health_check PASSED                            [100%]

========================= 1 passed in 0.14s ============================
```

**GREEN.** You've completed one red-green cycle.

## Testing POST Requests

POST requests send data. Here's how to test them:

```python
def test_create_item():
    response = client.post(
        "/items",
        json={"name": "Widget", "price": 9.99}
    )
    assert response.status_code == 201
    assert response.json()["name"] == "Widget"
    assert "id" in response.json()
```

The `json=` parameter sends JSON data in the request body. FastAPI's TestClient handles serialization.

## Checking Response Details

Tests can verify any part of the response:

```python
def test_response_structure():
    response = client.get("/items/1")

    # Status code
    assert response.status_code == 200

    # Response body
    data = response.json()
    assert "id" in data
    assert "name" in data
    assert isinstance(data["price"], float)

    # Headers
    assert response.headers["content-type"] == "application/json"
```

## Testing Error Responses

Good tests verify error cases too:

```python
def test_item_not_found():
    response = client.get("/items/99999")
    assert response.status_code == 404
    assert "not found" in response.json()["detail"].lower()


def test_invalid_input():
    response = client.post(
        "/items",
        json={"name": ""}  # Empty name should fail
    )
    assert response.status_code == 422  # Validation error
```

Testing the unhappy path is just as important as testing success.

## Complete Test Example

Here's a test file for the Task API you'll build:

```python
# test_tasks.py
from fastapi.testclient import TestClient
from main import app

client = TestClient(app)


class TestTaskAPI:
    """Tests for task endpoints."""

    def test_create_task(self):
        """POST /tasks creates a new task."""
        response = client.post(
            "/tasks",
            json={"title": "Learn testing", "description": "Write tests first"}
        )
        assert response.status_code == 201
        assert response.json()["title"] == "Learn testing"
        assert response.json()["status"] == "pending"

    def test_list_tasks(self):
        """GET /tasks returns all tasks."""
        response = client.get("/tasks")
        assert response.status_code == 200
        assert isinstance(response.json(), list)

    def test_get_task(self):
        """GET /tasks/{id} returns single task."""
        # First create a task
        create_response = client.post(
            "/tasks",
            json={"title": "Fetch me"}
        )
        task_id = create_response.json()["id"]

        # Then fetch it
        response = client.get(f"/tasks/{task_id}")
        assert response.status_code == 200
        assert response.json()["title"] == "Fetch me"

    def test_task_not_found(self):
        """GET /tasks/{id} returns 404 for missing task."""
        response = client.get("/tasks/99999")
        assert response.status_code == 404

    def test_create_task_without_title(self):
        """POST /tasks without title returns 422."""
        response = client.post(
            "/tasks",
            json={"description": "Missing title"}
        )
        assert response.status_code == 422
```

**Output:**
```bash
$ pytest test_tasks.py -v
========================= test session starts ==========================
test_tasks.py::TestTaskAPI::test_create_task PASSED
test_tasks.py::TestTaskAPI::test_list_tasks PASSED
test_tasks.py::TestTaskAPI::test_get_task PASSED
test_tasks.py::TestTaskAPI::test_task_not_found PASSED
test_tasks.py::TestTaskAPI::test_create_task_without_title PASSED
========================= 5 passed in 0.23s ============================
```

## Test Organization Tips

**Name tests clearly:**
```python
# Good - describes what's being tested
def test_create_task_with_description():

# Bad - vague
def test_task():
```

**One assertion per concept:**
```python
# Good - focused
def test_create_returns_201():
    response = client.post("/tasks", json={"title": "Test"})
    assert response.status_code == 201

def test_create_returns_task_with_id():
    response = client.post("/tasks", json={"title": "Test"})
    assert "id" in response.json()

# Acceptable - related assertions
def test_create_task():
    response = client.post("/tasks", json={"title": "Test"})
    assert response.status_code == 201
    assert "id" in response.json()
```

**Use classes to group related tests:**
```python
class TestTaskCreation:
    def test_with_title_only(self): ...
    def test_with_description(self): ...
    def test_without_title_fails(self): ...

class TestTaskRetrieval:
    def test_get_existing(self): ...
    def test_get_missing(self): ...
```

## Hands-On Exercise

Write tests for the `/` endpoint from Lesson 1:

**Step 1:** Create `test_main.py`:

```python
from fastapi.testclient import TestClient
from main import app

client = TestClient(app)


def test_root_returns_200():
    """GET / returns 200 status."""
    response = client.get("/")
    assert response.status_code == 200


def test_root_returns_message():
    """GET / returns greeting message."""
    response = client.get("/")
    assert "message" in response.json()
```

**Step 2:** Run the tests:

```bash
pytest test_main.py -v
```

**Step 3:** Add a failing test (RED):

```python
def test_greeting_with_name():
    """GET /greet/{name} returns personalized greeting."""
    response = client.get("/greet/Alice")
    assert response.status_code == 200
    assert response.json()["message"] == "Hello, Alice!"
```

**Step 4:** Make it pass (GREEN) by adding the endpoint to `main.py`

**Step 5:** Run all tests to confirm nothing broke

## Common Mistakes

**Mistake 1:** Forgetting to import the app

```python
# Wrong - app not imported
def test_something():
    response = client.get("/")  # client is undefined

# Correct
from main import app
client = TestClient(app)
```

**Mistake 2:** Testing response.json() on non-JSON responses

```python
# Wrong - 204 has no body
def test_delete():
    response = client.delete("/items/1")
    assert response.json()["deleted"] == True  # Fails!

# Correct
def test_delete():
    response = client.delete("/items/1")
    assert response.status_code == 204
```

**Mistake 3:** Tests that depend on each other

```python
# Wrong - test_get assumes test_create ran first
def test_create():
    client.post("/items", json={"name": "Widget"})

def test_get():
    response = client.get("/items/1")  # Assumes ID 1 exists

# Correct - each test is self-contained
def test_get():
    # Create first
    create_response = client.post("/items", json={"name": "Widget"})
    item_id = create_response.json()["id"]
    # Then get
    response = client.get(f"/items/{item_id}")
    assert response.status_code == 200
```

## Why Write Tests Manually?

You're building a skill, not just running commands. When you write tests by hand:

- You understand what makes a test useful
- You recognize edge cases to cover
- You can evaluate AI-generated tests critically
- You debug failing tests confidently

In later lessons, you'll use AI to help generate tests. But you'll be the judge of quality, not a passive consumer.

## Try With AI

After completing the manual exercises above, practice evaluating AI assistance.

**Prompt 1: Review Your Tests**

```text
Here are my tests for a Task API:

[paste your test_tasks.py]

What edge cases am I missing? Don't write the tests for me—
just list what scenarios I should consider adding.
```

**What you're learning:** You wrote tests manually. Now use AI to find gaps in your coverage—but you'll write the additional tests yourself.

**Prompt 2: Understand a Failure**

```text
My test is failing with this error:

AssertionError: assert 404 == 200

Here's my test:
def test_get_task():
    response = client.get("/tasks/1")
    assert response.status_code == 200

What's happening and how do I debug it?
```

**What you're learning:** Interpreting test failures is a core skill. AI can explain what the error means, but you need to understand the fix.

**Prompt 3: Refactoring Tests**

```text
I have tests that repeat setup code:

def test_create():
    client.post("/tasks", json={"title": "Test"})
    ...

def test_get():
    client.post("/tasks", json={"title": "Test"})
    ...

How can I use pytest fixtures to reduce duplication?
Explain the concept before showing code.
```

**What you're learning:** Fixtures are a pytest pattern for shared setup. Understanding why they exist helps you use them correctly.
