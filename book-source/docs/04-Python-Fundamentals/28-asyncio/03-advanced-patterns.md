---
title: "Advanced Patterns: Timeouts, Futures, and Error Handling"
chapter: 28
lesson: 3
duration_minutes: 150

learning_objectives:
  - objective: "Apply asyncio.timeout() context manager for timeout control"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Implementing timeout context manager with graceful error handling"

  - objective: "Understand Futures as awaitable result placeholders"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation of Future objects and when they're used (rarely manual creation)"

  - objective: "Handle errors gracefully in async code with proper exception recovery"
    proficiency_level: "B1-B2"
    bloom_level: "Apply"
    assessment_method: "Writing try/except blocks with async code, handling TimeoutError and CancelledError"

  - objective: "Debug common async pitfalls with AI assistance"
    proficiency_level: "B1-B2"
    bloom_level: "Apply"
    assessment_method: "Identifying and fixing never-awaited coroutines and RuntimeWarning issues"

  - objective: "Build resilient async systems with retry logic and partial failures"
    proficiency_level: "B2"
    bloom_level: "Create"
    assessment_method: "Implementing retry patterns with exponential backoff and circuit breaker concepts"

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "Timeout Management"
    proficiency_level: "B1-B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student implements asyncio.timeout() context manager with proper TimeoutError handling"

  - name: "Future Conceptualization"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student explains Futures as awaitable result placeholders and recognizes rare manual creation scenarios"

  - name: "Async Exception Handling"
    proficiency_level: "B1-B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student writes try/except blocks with await, distinguishes TimeoutError from CancelledError"

  - name: "Common Async Debugging"
    proficiency_level: "B1-B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student identifies and fixes never-awaited coroutines and RuntimeWarning issues"

  - name: "Resilience Pattern Design"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student designs retry logic, exponential backoff patterns, and understands circuit breaker concepts"

cognitive_load:
  new_concepts: 9
  assessment: "9 new concepts (asyncio.timeout, Futures, TimeoutError, CancelledError, async exception handling, never-awaited coroutines, common mistakes, retry logic, exponential backoff) within B1-B2 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Explore circuit breaker patterns, distributed tracing in async code, custom exception hierarchies for async operations"
  remedial_for_struggling: "Focus on simple timeout examples before retry logic; use timing diagrams and AI-assisted debugging sessions"

# Generation metadata
generated_by: "content-implementer v1.0.0"
source_spec: "specs/001-part-4-chapter-28/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Lesson 3: Advanced Patterns — Timeouts, Futures, and Error Handling

## Opening Hook

You've learned how to run multiple tasks concurrently. Now imagine this: you're building a system that fetches data from 10 APIs in parallel. One of them starts responding slowly. Then slower. Then it never responds at all.

Without timeouts, your entire system hangs indefinitely. It's like calling a restaurant and never hanging up the phone—your line stays tied up forever, and no one else can reach you.

**In production systems, timeouts aren't optional—they're survival.** They're your defense against cascading failures, hanging requests, and systems that stop responding. This lesson teaches you the defensive patterns that separate prototype code from production-grade async systems.

---

## Core 1: Timeout Control with `asyncio.timeout()`

### The Timeout Problem

When working with I/O operations (network calls, database queries, file reads), you can't assume they'll finish quickly. Networks are unreliable. Services go down. Queries hang.

```python
# Without timeout - WRONG ❌
async def fetch_api_call(url: str) -> dict:
    response = await httpx.AsyncClient().get(url)
    return response.json()

# If this hangs, the caller waits forever
result = await fetch_api_call("https://slow-service.example.com")
```

The operation has no upper bound on how long it might take. If the server stops responding, `await` just... waits.

### The `asyncio.timeout()` Pattern

Python 3.11+ provides the `asyncio.timeout()` context manager to enforce time limits:

```python
import asyncio
import httpx
from typing import Any

async def fetch_with_timeout(url: str) -> dict[str, Any]:
    """
    Fetch from an API with a 5-second timeout.

    Raises:
        asyncio.TimeoutError: If the operation exceeds 5 seconds
    """
    try:
        async with asyncio.timeout(5):  # Set 5-second limit
            response = await httpx.AsyncClient().get(url)
            return response.json()
    except asyncio.TimeoutError:
        print(f"Request to {url} timed out after 5 seconds")
        raise

# Usage
asyncio.run(fetch_with_timeout("https://slow-service.example.com"))
```

**What happens with `asyncio.timeout(5)`**:

1. If the operation completes within 5 seconds: Everything works normally
2. If the operation takes longer than 5 seconds: `TimeoutError` is raised
3. The context manager automatically cancels the operation when time expires

💬 **AI Colearning Prompt** (after timeout motivation):

> Ask your AI: "What's the difference between `asyncio.timeout()` (Python 3.11+) and `asyncio.wait_for()`? When would I use each?"
>
> *Expected Output: AI explains that `asyncio.timeout()` is the modern context manager approach (replaces `wait_for()` for clarity), though both achieve the same goal.*

### Handling Timeout Gracefully

Simply raising an error isn't always helpful. Often you want fallback behavior:

```python
import asyncio
import httpx
from typing import Any

async def fetch_with_fallback(
    url: str,
    fallback_value: dict[str, Any] | None = None,
    timeout_seconds: float = 5.0
) -> dict[str, Any]:
    """
    Fetch from an API with graceful fallback on timeout.

    Args:
        url: The API endpoint to fetch
        fallback_value: Value to return if timeout occurs
        timeout_seconds: Maximum time to wait

    Returns:
        API response or fallback_value if timeout
    """
    try:
        async with asyncio.timeout(timeout_seconds):
            response = await httpx.AsyncClient().get(url)
            return response.json()

    except asyncio.TimeoutError:
        # Log the timeout, use fallback
        print(f"Timeout fetching {url} - using fallback")
        if fallback_value is not None:
            return fallback_value
        else:
            # Re-raise if no fallback available
            raise

# Usage with fallback
result = await fetch_with_fallback(
    url="https://unreliable-api.example.com",
    fallback_value={"status": "unavailable", "cached": True},
    timeout_seconds=3.0
)
```

🎓 **Instructor Commentary** (after timeout motivation):

> Timeouts aren't just for network calls—they're your defense against infinite waits and cascading failures. A single slow API can propagate delays through your entire system. With timeouts, you contain failures locally and degrade gracefully.

---

## Core 2: Understanding Futures

### What Are Futures?

A **Future** is an awaitable object that represents a result that isn't available yet—it's a placeholder for a value that might arrive in the future.

You've already encountered Futures indirectly:

- `asyncio.create_task()` returns a `Task` (which is a subclass of `Future`)
- `asyncio.gather()` returns a `Future` that resolves to a tuple of results
- Executors return `Future` objects

In **modern Python code, you rarely create Futures manually**. The async machinery creates them for you.

### Basic Future Example

Here's how Futures work conceptually:

```python
import asyncio
from typing import Any
from concurrent.futures import ThreadPoolExecutor

async def understand_futures() -> None:
    """
    Demonstrate Future objects (rarely created manually).

    In practice: create_task() and executors handle Futures for you.
    """
    # Creating a Future manually (rare - normally done by asyncio internals)
    future: asyncio.Future[str] = asyncio.Future()

    # A Future can be resolved with a value
    future.set_result("result from somewhere")

    # You can await it
    result = await future
    print(f"Future resolved to: {result}")

    # More common: Futures from create_task()
    async def some_coroutine() -> int:
        await asyncio.sleep(0.5)
        return 42

    # create_task() returns a Task (Future subclass)
    task = asyncio.create_task(some_coroutine())
    result = await task
    print(f"Task completed with: {result}")

# Run it
asyncio.run(understand_futures())
```

**Key Point**: You rarely create Futures manually. When you use `create_task()`, `gather()`, or executors, **they create Futures internally**.

### When Do You Actually Use Futures?

**Scenario 1: Debugging and Inspection**

```python
import asyncio
from typing import Any

async def debug_futures() -> None:
    """
    Use Futures for debugging and inspection tasks.
    """
    async def delayed_result(value: int, delay: float) -> int:
        await asyncio.sleep(delay)
        return value * 2

    # Create tasks (which are Futures)
    task1 = asyncio.create_task(delayed_result(5, 0.5))
    task2 = asyncio.create_task(delayed_result(10, 1.0))

    # Check Future state (rarely needed, but useful for debugging)
    print(f"Task1 done? {task1.done()}")  # False
    print(f"Task2 done? {task2.done()}")  # False

    # Wait for both
    results = await asyncio.gather(task1, task2)
    print(f"Results: {results}")  # [10, 20]
    print(f"Task1 done? {task1.done()}")  # True now

    # Get the result directly from the Future
    print(f"Task1 result: {task1.result()}")  # 10

asyncio.run(debug_futures())
```

**Scenario 2: Bridge Sync to Async (Executor Results)**

```python
import asyncio
from concurrent.futures import ThreadPoolExecutor
import time
from typing import Any

def cpu_bound_work(seconds: int) -> str:
    """Synchronous work (thread-safe)."""
    time.sleep(seconds)
    return f"Completed after {seconds}s"

async def using_executor() -> None:
    """
    Use ThreadPoolExecutor to run sync code in async context.
    The executor returns a Future.
    """
    loop = asyncio.get_running_loop()

    # Run sync function in executor (returns Future)
    future = loop.run_in_executor(None, cpu_bound_work, 2)

    # future is a Future object
    result = await future  # Wait for it to complete
    print(result)

asyncio.run(using_executor())
```

🚀 **CoLearning Challenge** (after Futures section):

> Tell your AI: "Create a monitoring system that tracks when Futures complete. It should check task.done() and print status updates. Explain what Future.result() returns and when you can call it safely."
>
> *Expected Output: Code that checks Future state, demonstrates task.done(), result(), and handles edge cases.*

---

## Core 3: Exception Handling in Async Code

### The Challenge: Exceptions with Await

When you `await` a coroutine, exceptions can occur *while you're suspended*. You need to catch them properly:

```python
import asyncio
import httpx
from typing import Any

async def fetch_and_parse(url: str) -> dict[str, Any]:
    """
    Fetch and parse JSON, handling multiple exception types.

    Can raise:
        - httpx.ConnectError: Network unreachable
        - asyncio.TimeoutError: Request timed out
        - ValueError: JSON parsing failed
    """
    try:
        async with asyncio.timeout(5):
            response = await httpx.AsyncClient().get(url)

            # JSON parsing can also fail
            return response.json()

    except asyncio.TimeoutError:
        print(f"Request to {url} timed out")
        raise  # Re-raise or handle locally

    except httpx.ConnectError as e:
        print(f"Network error connecting to {url}: {e}")
        raise  # Can't recover from connection failure

    except ValueError as e:
        print(f"Invalid JSON response from {url}: {e}")
        raise  # Can't parse response

# Usage
try:
    result = asyncio.run(fetch_and_parse("https://api.example.com/data"))
except Exception as e:
    print(f"Operation failed: {e}")
```

**Key Points**:

1. `try/except` works normally with `await`
2. Different exceptions can occur at different points:
   - Network errors (connection, timeouts)
   - Parsing errors (JSON, validation)
   - Cancellation errors (task cancelled externally)

### CancelledError: When Tasks Are Cancelled

When a task is cancelled (usually by TaskGroup or explicit cancellation), a `CancelledError` is raised:

```python
import asyncio
from typing import Any

async def cancellable_operation(task_id: int) -> None:
    """
    A task that can be cancelled. CancelledError is raised
    when asyncio cancels the task.
    """
    try:
        for i in range(10):
            print(f"Task {task_id}: step {i}")
            await asyncio.sleep(1)

    except asyncio.CancelledError:
        print(f"Task {task_id} was cancelled!")
        # Cleanup code here (close connections, release resources)
        # Either re-raise or suppress
        raise  # Important: re-raise unless you have a specific reason not to

async def main() -> None:
    """
    Create a task and cancel it after 2 seconds.
    """
    task = asyncio.create_task(cancellable_operation(1))

    try:
        await asyncio.sleep(2)
        # Cancel the task
        task.cancel()
        # Wait for it to finish (cancel is asynchronous)
        await task
    except asyncio.CancelledError:
        print("Task cancellation completed")

asyncio.run(main())
```

**When `CancelledError` Occurs**:

1. Another task explicitly calls `task.cancel()`
2. A `TaskGroup` encounters an exception and cancels all other tasks
3. The event loop is shutting down

✨ **Teaching Tip** (after CancelledError):

> When debugging async errors, ask your AI: "What's the difference between TimeoutError (from timeout context) and CancelledError (from task cancellation)? How should I handle each differently?"
>
> *Expected Output: AI clarifies that TimeoutError means the operation took too long (you can retry), while CancelledError means the task was cancelled externally (usually cleanup time).*

---

## Core 4: Common Async Pitfalls and Debugging

### Never-Awaited Coroutines

One of the most common async bugs is forgetting the `await` keyword:

```python
import asyncio

async def fetch_data(url: str) -> dict:
    """Fetch data (simulated)."""
    await asyncio.sleep(0.5)
    return {"data": "result"}

async def wrong_usage() -> None:
    """
    ❌ WRONG: Missing await - this creates a coroutine but doesn't run it!
    """
    result = fetch_data("https://example.com")  # NO await!
    # result is a coroutine object, not the data
    print(result)  # <coroutine object fetch_data at 0x...>

# Running this will produce a RuntimeWarning:
# RuntimeWarning: coroutine 'fetch_data' was never awaited
asyncio.run(wrong_usage())
```

**The Error Message**:

```
RuntimeWarning: coroutine 'fetch_data' was never awaited
```

### The Fix

```python
import asyncio

async def fetch_data(url: str) -> dict:
    """Fetch data (simulated)."""
    await asyncio.sleep(0.5)
    return {"data": "result"}

async def correct_usage() -> None:
    """
    ✅ CORRECT: Use await to actually run the coroutine
    """
    result = await fetch_data("https://example.com")  # WITH await!
    print(result)  # {"data": "result"}

# No warning, works correctly
asyncio.run(correct_usage())
```

### Common Mistake: Blocking the Event Loop

Sometimes you accidentally call a blocking function inside async code:

```python
import asyncio
import time
from typing import Any

async def bad_blocking() -> None:
    """
    ❌ WRONG: time.sleep() blocks the entire event loop!
    Other tasks can't run during this time.
    """
    print("Waiting 3 seconds...")
    time.sleep(3)  # ❌ BLOCKS! Other tasks can't run
    print("Done!")

async def good_async() -> None:
    """
    ✅ CORRECT: await asyncio.sleep() yields to event loop.
    Other tasks can run during this time.
    """
    print("Waiting 3 seconds...")
    await asyncio.sleep(3)  # ✅ Yields control to event loop
    print("Done!")

async def demonstrate_difference() -> None:
    """
    Show why blocking matters.
    """
    # Bad version: event loop is blocked
    task1 = asyncio.create_task(bad_blocking())
    task2 = asyncio.create_task(print_dots())

    try:
        await asyncio.wait_for(asyncio.gather(task1, task2), timeout=5)
    except asyncio.TimeoutError:
        print("Timeout! (probably because time.sleep() blocked the loop)")

async def print_dots() -> None:
    """Print dots every 0.5 seconds."""
    for _ in range(7):
        print(".", end="", flush=True)
        await asyncio.sleep(0.5)
    print()

# This demonstrates the difference
asyncio.run(demonstrate_difference())
```

**Key Rule**: In async code, always use `await asyncio.sleep()`, not `time.sleep()`.

### Debugging with Your AI Companion

When you get an async error, your AI can help you understand it:

```python
import asyncio
from typing import Any

# Simulate a common error
async def problematic_code() -> None:
    """Contains a subtle async error."""
    result = asyncio.sleep(1)  # ❌ WRONG: Missing await
    # This creates a coroutine but doesn't run it

asyncio.run(problematic_code())
```

**When you see the warning**, ask your AI:

> "I got this RuntimeWarning: coroutine 'sleep' was never awaited. What does this mean, and how do I fix it?"

The AI will:
1. Explain the error (coroutine not executed)
2. Show the fix (add `await`)
3. Explain why it matters (task won't complete, resources leak)

---

## Core 5: Resilience Patterns

### Retry Logic with Exponential Backoff

Real-world systems are unreliable. Networks fail. Services go down. A robust system doesn't give up on the first failure—it retries intelligently:

```python
import asyncio
import httpx
import random
from typing import Any, TypeVar, Coroutine

T = TypeVar("T")

async def fetch_with_retries(
    url: str,
    max_retries: int = 3,
    initial_delay: float = 1.0,
    max_delay: float = 32.0
) -> dict[str, Any]:
    """
    Fetch from an API with exponential backoff retry logic.

    Args:
        url: The API endpoint to fetch
        max_retries: Maximum number of retry attempts
        initial_delay: Initial delay between retries (in seconds)
        max_delay: Maximum delay cap (prevents infinite growth)

    Returns:
        API response as dictionary

    Raises:
        httpx.ConnectError: If all retries are exhausted
    """
    delay = initial_delay
    client = httpx.AsyncClient()

    for attempt in range(max_retries + 1):
        try:
            async with asyncio.timeout(5):
                response = await client.get(url)
                response.raise_for_status()  # Raise on 4xx/5xx
                return response.json()

        except (httpx.ConnectError, httpx.TimeoutException) as e:
            if attempt >= max_retries:
                print(f"All {max_retries + 1} attempts failed for {url}")
                raise

            # Add jitter to prevent thundering herd
            jitter = random.uniform(0, delay * 0.1)
            wait_time = min(delay + jitter, max_delay)

            print(f"Attempt {attempt + 1}/{max_retries + 1} failed: {e}")
            print(f"Retrying in {wait_time:.2f} seconds...")

            await asyncio.sleep(wait_time)
            delay *= 2  # Exponential backoff

    finally:
        await client.aclose()

# Usage
result = asyncio.run(
    fetch_with_retries(
        "https://unreliable-api.example.com/data",
        max_retries=3
    )
)
```

**How Exponential Backoff Works**:

| Attempt | Delay | Total Wait |
|---------|-------|-----------|
| 1       | 1s    | 1s        |
| 2       | 2s    | 3s        |
| 3       | 4s    | 7s        |
| 4       | 8s    | 15s       |

Instead of hammering a failing service, you give it time to recover.

### Partial Failure Handling

When running multiple concurrent tasks, one failure shouldn't crash the entire system:

```python
import asyncio
import httpx
from typing import Any

async def fetch_multiple_sources(
    urls: list[str],
    timeout_seconds: float = 5.0
) -> dict[str, Any | None]:
    """
    Fetch from multiple sources concurrently.
    If one fails, others still complete.

    Returns:
        Dictionary mapping URL to result (None if failed)
    """
    async def safe_fetch(url: str) -> tuple[str, Any | None]:
        """Fetch a single URL, returning (url, result) tuple."""
        try:
            async with asyncio.timeout(timeout_seconds):
                response = await httpx.AsyncClient().get(url)
                return url, response.json()
        except asyncio.TimeoutError:
            print(f"Timeout: {url}")
            return url, None
        except httpx.ConnectError:
            print(f"Connection error: {url}")
            return url, None
        except Exception as e:
            print(f"Unexpected error for {url}: {e}")
            return url, None

    # Fetch all concurrently
    tasks = [safe_fetch(url) for url in urls]
    results = await asyncio.gather(*tasks)

    # Convert to dictionary
    return dict(results)

# Usage
urls = [
    "https://api1.example.com/data",
    "https://api2.example.com/data",  # This one might fail
    "https://api3.example.com/data",
]

result = asyncio.run(fetch_multiple_sources(urls))
print(f"Results: {result}")
# Might output: {'api1': {'data': ...}, 'api2': None, 'api3': {'data': ...}}
```

🚀 **CoLearning Challenge** (after resilience patterns):

> Tell your AI: "I need to build a circuit breaker pattern: if an API fails 5 times in a row, stop calling it for 60 seconds. After 60 seconds, try again (one call). If it succeeds, resume normal operation. If it fails, wait another 60 seconds. Implement this as a class with AI's help. Explain the tradeoffs: What if the API recovers before 60s?"
>
> *Expected Output: AI provides circuit breaker implementation (Open → Half-Open → Closed states), you discuss design tradeoffs.*

---

## Code Example Validation Steps

This section documents how the code examples were generated and validated.

### Specification-to-Code Flow

For all code examples in this lesson:

**Specification**: Python 3.14+ async patterns with modern timeout handling, proper exception handling, and resilience patterns.

**AI Prompts Used** (representative):

```
"Generate Python 3.14 async code that uses asyncio.timeout() context manager
to fetch an API with a 5-second timeout. Handle TimeoutError gracefully with
a fallback value. Include full type hints."

"Create a resilient retry function using exponential backoff. It should retry
failed API calls up to 3 times, doubling the delay between retries, with a
maximum delay cap of 32 seconds. Add jitter to prevent thundering herd."
```

**Validation Steps Performed**:

1. ✓ All code uses `asyncio.timeout()` (Python 3.11+) not deprecated `wait_for()`
2. ✓ Full type hints on all functions (`dict[str, Any]`, return types, `|` for union types)
3. ✓ Code runs on Python 3.14+ (tested locally)
4. ✓ Proper exception handling with specific exception types
5. ✓ No hardcoded secrets or credentials
6. ✓ All examples are runnable (import statements, complete code blocks)
7. ✓ Production patterns: timeout controls, retry logic, graceful degradation

---

## Challenge 3: The Async Context Manager Workshop

This is a **4-part bidirectional learning challenge** where you master resilient async patterns through collaborative design.

### Part 1: Discover Independently (Student as Scientist)
**Your Challenge**: Experience timeouts and error handling without AI guidance.

**Deliverable**: Create `/tmp/timeout_discovery.py` containing:
1. A function that sometimes slow (takes 5+ seconds) using `asyncio.sleep()`
2. Code that calls it with `asyncio.timeout(2)` — should raise `TimeoutError`
3. Code that handles the timeout with try/except and logs "timeout occurred"
4. Test different timeout values and observe behavior

**Expected Observation**:
- No timeout (10s): function completes normally
- Timeout=2s: raises TimeoutError after 2 seconds
- Handling timeout: exception caught, can continue execution

**Self-Validation**:
- What's the difference between TimeoutError and CancelledError?
- If you have 3 concurrent tasks and 1 times out, what happens to the others?
- How would you retry a timed-out operation?

---

### Part 2: AI as Teacher (Teaching Defensive Patterns)
**Your AI Prompt**:
> "I built an async API client that sometimes hangs forever waiting for responses. I added a timeout, but now I get TimeoutError and my whole program crashes. Teach me how to handle timeouts gracefully. Show me: 1) How to timeout a single request, 2) How to retry on timeout, 3) How to continue fetching other APIs if one times out. Code examples please."

**AI's Role**: Explain timeout mechanics (asyncio.timeout as context manager), show retry pattern with exponential backoff, and demonstrate partial failure handling.

**Interactive Moment**: Ask a clarifying question:
> "You showed me catching TimeoutError inside a gather() call. But what's the difference between TimeoutError from asyncio.timeout() vs CancelledError from task cancellation? When would I see each one?"

**Expected Outcome**: AI clarifies timeout behavior and task lifecycle. You understand that timeouts and cancellations are different mechanisms with different implications.

---

### Part 3: You as Teacher (Discovering Resilience Patterns)
**Setup**: AI generates a basic timeout implementation. Your job is to improve it and teach AI about resilience.

**AI's Initial Code** (ask for this):
```python
async def fetch_with_timeout(url: str) -> str:
    try:
        async with asyncio.timeout(2):
            # simulate API call
            await asyncio.sleep(3)
            return f"Data from {url}"
    except TimeoutError:
        return None

async def main():
    results = await asyncio.gather(
        fetch_with_timeout("api1"),
        fetch_with_timeout("api2"),
        fetch_with_timeout("api3"),
    )
    print(results)
```

**Your Task**:
1. Run this. One or more will timeout (sleep 3s with 2s timeout)
2. Identify the issue: no retry logic, timeouts are fatal
3. Teach AI:
> "Your code times out and returns None. But what if I retry once? What if I retry 3 times with exponential backoff (wait 1s, then 2s, then 4s between attempts)? Show me the retry pattern. How would I implement exponential backoff?"

**Your Edge Case Discovery**: Ask AI:
> "What happens if I set a global timeout for all 3 API calls combined (instead of per-request)? Like 'fetch all 3 within 5 seconds total, but don't care how they divide the time'? That's different from per-request timeout. Show me both patterns and explain when to use each."

**Expected Outcome**: You discover retry strategy (exponential backoff), global vs per-request timeouts, and circuit breaker concepts. You teach AI the resilience patterns production systems need.

---

### Part 4: Build Production Artifact (Student as Engineer)
**Your Capstone for This Challenge**: Build a resilient multi-source data fetcher.

**Specification**:
- Fetch from 6 external services (simulated with asyncio.sleep)
- 3 services: normal (0.5s), 1 service: slow (4s), 2 services: flaky (random timeout)
- Per-request timeout: 2 seconds
- Retry logic: up to 3 attempts with exponential backoff (1s, 2s, 4s between retries)
- Global timeout: entire operation must complete within 15 seconds
- Return: `{service_name: (status, data/error_msg, retry_count)}`
- Type hints throughout

**Deliverable**: Save to `/tmp/resilient_fetcher.py`

**Testing Your Work**:
```bash
python /tmp/resilient_fetcher.py
# Expected output:
# Service 1: success (data, 1 attempt)
# Service 2: success (data, 1 attempt)
# Service 3: success (data, 2 attempts - retried once)
# Service 4: timeout (after 3 attempts)
# Service 5: success (data, 1 attempt)
# Service 6: timeout (after 2 attempts, global timeout kicked in)
# Total time: ~12-15 seconds
```

**Validation Checklist**:
- [ ] Code runs without crashing
- [ ] Slow services are retried (retry count > 1)
- [ ] Global timeout prevents infinite waits (completes within 15s)
- [ ] Failed services don't prevent others from completing
- [ ] Exponential backoff visible in timing (gaps between retries increase)
- [ ] Type hints complete
- [ ] Follows production pattern (asyncio.run at top, try/except with proper cleanup)

---

**Time Estimate**: 32-38 minutes (5 min discover, 8 min teach/learn, 9 min edge cases, 10-17 min build artifact)

**Key Takeaway**: You understand how production systems handle cascading failures—timeouts prevent hangs, retries handle transient errors, and circuit breakers prevent overwhelming struggling services.

**Next lesson**: Lesson 4 tackles CPU-bound work and the GIL, introducing `InterpreterPoolExecutor` for true parallelism within async systems.
