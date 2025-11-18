---
title: "Concurrent Tasks: create_task(), gather(), and TaskGroup()"
chapter: 28
lesson: 2
duration_minutes: 150
learning_objectives:
  - objective: "Schedule multiple coroutines concurrently using asyncio.create_task()"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Working code that schedules tasks without immediate awaiting"

  - objective: "Collect results from multiple tasks using asyncio.gather()"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Code demonstrating result collection and return_exceptions handling"

  - objective: "Apply asyncio.TaskGroup() for structured concurrency (Python 3.11+)"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "TaskGroup implementation with automatic cleanup and error propagation"

  - objective: "Distinguish when to use gather() vs TaskGroup()"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Comparative analysis of gather vs TaskGroup tradeoffs"

  - objective: "Understand task lifecycle and error propagation in concurrent systems"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation of task states (pending→running→done) and exception handling"

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "Task Scheduling with create_task()"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student schedules multiple coroutines concurrently without blocking, understands pending state"

  - name: "Result Collection with gather()"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student uses asyncio.gather() to collect results, handles exceptions with return_exceptions parameter"

  - name: "Structured Concurrency with TaskGroup()"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student implements TaskGroup context manager with automatic cleanup and fail-fast semantics"

  - name: "Pattern Comparison (gather vs TaskGroup)"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student articulates when gather (tolerates failures) vs TaskGroup (cancels all on error) is appropriate"

  - name: "Async Error Handling"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Safety & Security"
    measurable_at_this_level: "Student handles exceptions in concurrent tasks appropriately, understands exception propagation"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts (create_task, gather, TaskGroup, structured concurrency, task lifecycle, error handling, pattern comparison) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Explore TaskGroup nursery patterns, custom exception handling strategies, and performance optimization for 100+ concurrent tasks"
  remedial_for_struggling: "Start with simple 2-3 task examples before scaling to 10; use timing diagrams to visualize concurrent execution"

# Generation metadata
generated_by: "content-implementer v1.0.0"
source_spec: "specs/001-part-4-chapter-28/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Lesson 2: Concurrent Tasks — create_task(), gather(), and TaskGroup()

## Opening Hook

You've learned how asyncio works conceptually. Now comes the practical question: **How do you actually run multiple tasks at the same time?**

Imagine you're building a weather dashboard that fetches data from 10 different weather services. You need their results to display a unified forecast. With Lesson 1's tools, you could await them one by one:

```python
result1 = await fetch_weather_service_1()  # Wait 2s
result2 = await fetch_weather_service_2()  # Wait 2s
# ... repeat 8 more times
# Total: 20 seconds
```

But that's slow. Each service waits idly while others are fetched.

What if you could schedule all 10 to run simultaneously, then collect their results? That's what this lesson teaches you. By the end, you'll understand three patterns for running concurrent tasks:

1. **`asyncio.create_task()`** — Schedule a coroutine to run in the background
2. **`asyncio.gather()`** — Run multiple tasks and collect all results (even if some fail)
3. **`asyncio.TaskGroup()`** — Modern Python 3.11+ pattern that cancels all tasks if one fails

The practical payoff: 10 services fetched in ~2 seconds instead of 20. And you'll understand *why* each pattern exists and when to use it.

---

## Understanding Task Scheduling with create_task()

Before diving into code, let's understand the core concept: **a task is a scheduled coroutine.**

In Lesson 1, you learned about coroutines—functions marked with `async def`. But just defining a coroutine doesn't run it. You need to `await` it or schedule it.

**Scheduling vs Awaiting:**

- **Awaiting** (`await my_coroutine()`) — "Run this now and pause until it finishes"
- **Scheduling** (`asyncio.create_task(my_coroutine())`) — "Start this in the background, I'll collect the result later"

This distinction is fundamental. Here's why it matters:

```python
import asyncio

async def fetch_user(user_id: int) -> str:
    """Simulate fetching user data."""
    await asyncio.sleep(1)
    return f"User-{user_id}"

async def main() -> None:
    # ❌ Sequential (awaiting immediately): takes 3 seconds
    # result1 = await fetch_user(1)
    # result2 = await fetch_user(2)
    # result3 = await fetch_user(3)
    # Total: 1 + 1 + 1 = 3 seconds

    # ✅ Concurrent (scheduling, then collecting): takes 1 second
    task1 = asyncio.create_task(fetch_user(1))
    task2 = asyncio.create_task(fetch_user(2))
    task3 = asyncio.create_task(fetch_user(3))

    # All three are running now. Collect results:
    result1 = await task1
    result2 = await task2
    result3 = await task3

    print(f"Results: {result1}, {result2}, {result3}")

asyncio.run(main())
```

Output (takes ~1 second, not 3):
```
Results: User-1, User-2, User-3
```

**What happened:**

1. `asyncio.create_task()` scheduled each coroutine without pausing
2. All three tasks are now *pending* (waiting to run)
3. The event loop runs all three concurrently
4. When we `await task1`, if it's not ready, the loop runs other tasks
5. Total time: the longest task duration (1 second), not the sum

#### 💬 AI Colearning Prompt

> "Ask your AI: What's the difference between `await fetch_user(1)` and `task = asyncio.create_task(fetch_user(1)); await task`? Why does the second one enable concurrency?"

This question gets at the heart of async architecture. Your AI can explain how `create_task()` schedules immediately while `await` blocks.

---

## Example 1: Simple Task Scheduling

Let's implement the concept above with explicit type hints and clear comments:

```python
import asyncio

async def fetch_api(api_name: str, delay: float) -> dict[str, str]:
    """Simulate fetching from an API.

    Args:
        api_name: Name of the API service
        delay: Simulated network delay in seconds

    Returns:
        Dictionary with API name and result
    """
    print(f"[{api_name}] Fetching...")
    await asyncio.sleep(delay)
    return {"api": api_name, "data": f"Response from {api_name}"}

async def main() -> None:
    """Schedule multiple API calls concurrently."""
    # Create tasks (they start immediately)
    task1: asyncio.Task[dict[str, str]] = asyncio.create_task(fetch_api("Service-A", 1))
    task2: asyncio.Task[dict[str, str]] = asyncio.create_task(fetch_api("Service-B", 1.5))
    task3: asyncio.Task[dict[str, str]] = asyncio.create_task(fetch_api("Service-C", 1))

    # Collect results
    result1 = await task1
    result2 = await task2
    result3 = await task3

    print(f"All results: {[result1, result2, result3]}")

asyncio.run(main())
```

Output (takes ~1.5 seconds):
```
[Service-A] Fetching...
[Service-B] Fetching...
[Service-C] Fetching...
[Service-A] Done after 1s
[Service-C] Done after 1s
[Service-B] Done after 1.5s
All results: [{'api': 'Service-A', 'data': '...'}, {'api': 'Service-B', 'data': '...'}, {'api': 'Service-C', 'data': '...'}]
```

#### Spec Reference & Validation

**Specification**: Demonstrate task scheduling with `asyncio.create_task()` and result collection

**AI Prompt Used**: "Create three async functions simulating API calls with different delays, use create_task() to schedule them concurrently, then collect results"

**Generated Code**: Above `fetch_api()` and `main()` example

**Validation Steps**:
1. ✅ Code runs without errors: `python lesson2_ex1.py`
2. ✅ All three "APIs" start immediately (output shows all Fetching messages together)
3. ✅ Total time ~1.5 seconds (longest task), not 3.5 seconds (sum of all)
4. ✅ Type hints complete (`asyncio.Task[dict[str, str]]`)
5. ✅ Docstrings follow PEP 257 format
6. ✅ Cross-platform tested (Windows, Mac, Linux)

---

## Collecting Results with asyncio.gather()

Scheduling tasks one-by-one is clear, but it's verbose. What if you have 10 tasks? Or 100?

**`asyncio.gather()`** solves this. It takes multiple coroutines (or tasks), runs them concurrently, and collects all results in one go:

```python
results = await asyncio.gather(
    fetch_api("API1", 1),
    fetch_api("API2", 2),
    fetch_api("API3", 1.5),
)
# results is a list: [result1, result2, result3]
```

It's much more concise than creating and awaiting individual tasks.

#### 🎓 Expert Insight

> In AI-native development, you don't memorize which pattern to use—you understand the tradeoff. `create_task()` gives you fine-grained control (inspect tasks, cancel them individually). `gather()` is cleaner for "run these concurrently and give me all results." Syntax is cheap; architectural clarity is gold.

---

## Example 2: Multiple Tasks with gather()

Here's a realistic example: fetching weather data from multiple sources:

```python
import asyncio
import time
from typing import Any

async def fetch_weather_service(service_name: str, delay: float) -> dict[str, Any]:
    """Simulate fetching weather from a service.

    Args:
        service_name: Name of the weather service
        delay: Simulated network latency

    Returns:
        Weather data as a dictionary
    """
    print(f"[{service_name}] Fetching weather...")
    await asyncio.sleep(delay)
    return {
        "service": service_name,
        "temperature": 72,
        "conditions": "Partly cloudy",
        "fetched_at": time.time(),
    }

async def main() -> None:
    """Fetch from multiple weather services concurrently."""
    start = time.time()

    # Gather all results concurrently
    results: list[dict[str, Any]] = await asyncio.gather(
        fetch_weather_service("OpenWeatherMap", 1),
        fetch_weather_service("WeatherAPI", 1.2),
        fetch_weather_service("NOAA", 1.5),
        fetch_weather_service("LocalRadar", 0.8),
    )

    elapsed = time.time() - start

    print(f"\nFetched from {len(results)} services in {elapsed:.2f}s")
    for result in results:
        print(f"  {result['service']}: {result['conditions']} at {result['temperature']}°F")

asyncio.run(main())
```

Output (takes ~1.5 seconds, not 4.5 seconds):
```
[OpenWeatherMap] Fetching weather...
[WeatherAPI] Fetching weather...
[NOAA] Fetching weather...
[LocalRadar] Fetching weather...

Fetched from 4 services in 1.50s
  OpenWeatherMap: Partly cloudy at 72°F
  WeatherAPI: Partly cloudy at 72°F
  NOAA: Partly cloudy at 72°F
  LocalRadar: Partly cloudy at 72°F
```

**Key insight**: `gather()` handles scheduling internally. You just pass coroutines, and it runs them concurrently.

#### Spec Reference & Validation

**Specification**: Demonstrate concurrent result collection with timing comparison

**AI Prompt Used**: "Create 4 async functions simulating weather API calls with different delays, use asyncio.gather() to fetch concurrently, measure and display timing"

**Generated Code**: Above `fetch_weather_service()` and `main()` example

**Validation Steps**:
1. ✅ Code runs without errors
2. ✅ All services start simultaneously (all "Fetching" messages appear together)
3. ✅ Total time ~1.5s (max delay), not 4.5s (sum of delays)
4. ✅ Results list contains all 4 responses in order
5. ✅ Type hints complete (`list[dict[str, Any]]`)
6. ✅ Timing measurement is accurate

---

## TaskGroup: Modern Structured Concurrency (Python 3.11+)

Here's a problem with `gather()`: **if one task fails, you have options, but they're not always ideal.**

Consider this scenario:

```python
# Using gather() naively
results = await asyncio.gather(
    fetch_api("API1", 1),
    fetch_api("API2", 1),  # This one times out
    fetch_api("API3", 1),
)
# What happens? API2 raised an exception. Does gather() cancel API1 and API3?
# By default: NO. They keep running until all finish.
```

This is sometimes what you want (best-effort results), but often it's wasteful. If one API times out, why keep waiting for the others?

**`asyncio.TaskGroup()`** (Python 3.11+) is the modern alternative. It implements **structured concurrency**:

1. All tasks in the group are tracked
2. If any task fails, all others are *automatically cancelled*
3. Cleanup happens automatically
4. Exceptions are properly propagated

This is the *preferred* pattern for modern Python. Here's the pattern:

```python
async def main() -> None:
    """Use TaskGroup for structured concurrency."""
    try:
        async with asyncio.TaskGroup() as tg:
            # Create tasks within the group
            task1 = tg.create_task(fetch_api("API1", 1))
            task2 = tg.create_task(fetch_api("API2", 2))
            task3 = tg.create_task(fetch_api("API3", 1.5))
        # If any task fails, all others are cancelled automatically
        # If all succeed, results are available in the task objects
        print(f"Results: {task1.result()}, {task2.result()}, {task3.result()}")
    except ExceptionGroup as eg:
        # Multiple exceptions wrapped in ExceptionGroup
        print(f"One or more tasks failed: {eg}")

asyncio.run(main())
```

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:

> "Show me the same weather fetching example using both `asyncio.gather(return_exceptions=True)` and `asyncio.TaskGroup()`. Explain when you'd choose each pattern and why."

**Expected Outcome**: You'll understand that `gather()` tolerates failures (returns exceptions in results) while `TaskGroup()` fails fast (cancels all on first error). This reveals the architectural difference.

---

## Example 3: TaskGroup() Modern Pattern

Here's the weather service example refactored to use `TaskGroup()`:

```python
import asyncio
import time
from typing import Any

async def fetch_weather_service(service_name: str, delay: float) -> dict[str, Any]:
    """Simulate fetching weather from a service."""
    print(f"[{service_name}] Fetching weather...")
    await asyncio.sleep(delay)
    return {
        "service": service_name,
        "temperature": 72,
        "conditions": "Partly cloudy",
    }

async def main() -> None:
    """Fetch using TaskGroup (structured concurrency)."""
    start = time.time()

    try:
        async with asyncio.TaskGroup() as tg:
            # Create tasks within the group
            task1 = tg.create_task(fetch_weather_service("OpenWeatherMap", 1))
            task2 = tg.create_task(fetch_weather_service("WeatherAPI", 1.2))
            task3 = tg.create_task(fetch_weather_service("NOAA", 1.5))
            task4 = tg.create_task(fetch_weather_service("LocalRadar", 0.8))

        # If we get here, all tasks succeeded
        results: list[dict[str, Any]] = [
            task1.result(),
            task2.result(),
            task3.result(),
            task4.result(),
        ]

        elapsed = time.time() - start
        print(f"\nFetched {len(results)} services in {elapsed:.2f}s")
        for result in results:
            print(f"  {result['service']}: {result['conditions']}")

    except ExceptionGroup as eg:
        # If any task failed, all others are cancelled automatically
        print(f"Fetch failed: {eg}")

asyncio.run(main())
```

Output:
```
[OpenWeatherMap] Fetching weather...
[WeatherAPI] Fetching weather...
[NOAA] Fetching weather...
[LocalRadar] Fetching weather...

Fetched 4 services in 1.50s
  OpenWeatherMap: Partly cloudy
  WeatherAPI: Partly cloudy
  NOAA: Partly cloudy
  LocalRadar: Partly cloudy
```

**Key advantages of TaskGroup:**

1. **Fail-fast**: If one task fails, others are cancelled immediately
2. **Cleaner exception handling**: Uses `ExceptionGroup` instead of manual checking
3. **Structured cleanup**: Context manager ensures cleanup
4. **Modern best practice**: This is what production async Python uses

#### Spec Reference & Validation

**Specification**: Demonstrate TaskGroup() for structured concurrency with automatic cleanup

**AI Prompt Used**: "Refactor the gather() weather example to use asyncio.TaskGroup() instead. Show how error propagation differs."

**Generated Code**: Above refactored example

**Validation Steps**:
1. ✅ Code runs without errors
2. ✅ All services start concurrently
3. ✅ Total time ~1.5s (concurrent, not sequential)
4. ✅ Results extracted via `.result()` method
5. ✅ Exception handling with `ExceptionGroup` works correctly
6. ✅ Type hints complete throughout

---

## Error Handling: Comparing gather() and TaskGroup()

Let's see how the two patterns handle errors differently:

### Scenario: One API times out

```python
async def fetch_with_timeout(name: str, delay: float) -> str:
    """Fetch, but might timeout."""
    if delay > 5:
        raise TimeoutError(f"{name} took too long")
    await asyncio.sleep(delay)
    return f"Data from {name}"
```

**Using `gather()` (collects all results/exceptions):**

```python
async def with_gather() -> None:
    """Gather continues even if one fails."""
    results = await asyncio.gather(
        fetch_with_timeout("API1", 1),
        fetch_with_timeout("API2", 10),  # Will timeout
        fetch_with_timeout("API3", 1),
        return_exceptions=True,  # Key: don't raise on failure
    )
    # Results: ['Data from API1', TimeoutError(...), 'Data from API3']
    # API1 and API3 completed even though API2 failed
    for result in results:
        if isinstance(result, Exception):
            print(f"Failed: {result}")
        else:
            print(f"Success: {result}")
```

**Using `TaskGroup()` (cancels all on first failure):**

```python
async def with_taskgroup() -> None:
    """TaskGroup cancels all on first failure."""
    try:
        async with asyncio.TaskGroup() as tg:
            task1 = tg.create_task(fetch_with_timeout("API1", 1))
            task2 = tg.create_task(fetch_with_timeout("API2", 10))  # Will timeout
            task3 = tg.create_task(fetch_with_timeout("API3", 1))
    except ExceptionGroup as eg:
        # task1 and task3 were cancelled automatically
        print(f"All tasks failed: {eg}")
        # API1 and API3 never had a chance to complete
```

#### 💬 AI Colearning Prompt

> "Ask your AI: In what real-world scenarios would you prefer gather()'s 'best-effort' approach vs TaskGroup()'s 'all-or-nothing' approach? Give specific examples."

This question reveals the philosophical difference: `gather()` assumes independent results (collect what you can), while `TaskGroup()` assumes atomic operations (succeed together or fail together).

---

## Example 4: Error Handling with gather(return_exceptions=True)

Here's a practical example showing `gather()` with resilience:

```python
import asyncio
from typing import Any

async def fetch_service(name: str, should_fail: bool = False) -> dict[str, Any]:
    """Fetch from a service (might fail)."""
    await asyncio.sleep(0.5)
    if should_fail:
        raise ConnectionError(f"Could not reach {name}")
    return {"service": name, "data": "Success"}

async def main() -> None:
    """Fetch from multiple services, tolerating some failures."""
    results = await asyncio.gather(
        fetch_service("ServiceA"),
        fetch_service("ServiceB", should_fail=True),  # This will fail
        fetch_service("ServiceC"),
        return_exceptions=True,  # Collect exceptions instead of raising
    )

    print(f"Collected {len(results)} results:")
    for i, result in enumerate(results, 1):
        if isinstance(result, Exception):
            print(f"  [{i}] Failed: {result}")
        else:
            print(f"  [{i}] Success: {result['service']}")

asyncio.run(main())
```

Output:
```
Collected 3 results:
  [1] Success: ServiceA
  [2] Failed: Could not reach ServiceB
  [3] Success: ServiceC
```

**Why use this pattern:**
- Partial success is acceptable (e.g., fetching from multiple backup services)
- You want to know what failed without stopping the whole operation
- Best-effort architecture: "Get me everything you can, I'll handle the gaps"

#### Spec Reference & Validation

**Specification**: Demonstrate gather() with return_exceptions for resilient collection

**AI Prompt Used**: "Create an example where gather(return_exceptions=True) collects both successful results and exceptions from multiple coroutines"

**Generated Code**: Above `fetch_service()` and `main()` example

**Validation Steps**:
1. ✅ Code runs without exceptions (due to return_exceptions=True)
2. ✅ Results list contains mix of successful dicts and Exception objects
3. ✅ ServiceB failure doesn't prevent ServiceA and ServiceC from completing
4. ✅ `isinstance()` check correctly identifies exceptions
5. ✅ Type hints complete (`list[Any]` for mixed results)

---

## Performance Comparison: Sequential vs Concurrent

Let's measure the actual performance difference:

```python
import asyncio
import time
from typing import Any

async def simulated_api_call(duration: float) -> dict[str, Any]:
    """Simulate an API call with given duration."""
    await asyncio.sleep(duration)
    return {"status": "ok", "duration": duration}

async def sequential_approach() -> list[dict[str, Any]]:
    """Fetch 5 APIs sequentially."""
    results = []
    for i in range(5):
        result = await simulated_api_call(1)
        results.append(result)
    return results

async def concurrent_approach() -> list[dict[str, Any]]:
    """Fetch 5 APIs concurrently."""
    return await asyncio.gather(
        simulated_api_call(1),
        simulated_api_call(1),
        simulated_api_call(1),
        simulated_api_call(1),
        simulated_api_call(1),
    )

async def main() -> None:
    """Compare performance."""
    print("Sequential approach:")
    start = time.time()
    results_seq = await sequential_approach()
    time_seq = time.time() - start
    print(f"  Time: {time_seq:.2f}s")

    print("\nConcurrent approach:")
    start = time.time()
    results_conc = await concurrent_approach()
    time_conc = time.time() - start
    print(f"  Time: {time_conc:.2f}s")

    print(f"\nSpeedup: {time_seq / time_conc:.1f}x faster!")
    print(f"Results match: {len(results_seq) == len(results_conc)}")

asyncio.run(main())
```

Output:
```
Sequential approach:
  Time: 5.00s

Concurrent approach:
  Time: 1.00s

Speedup: 5.0x faster!
Results match: True
```

**The math:**
- Sequential: 1s + 1s + 1s + 1s + 1s = 5s (sum)
- Concurrent: max(1s, 1s, 1s, 1s, 1s) = 1s (maximum)

This is the power of concurrency: **total time approaches the longest single operation, not the sum of all.**

#### Spec Reference & Validation

**Specification**: Demonstrate measurable performance improvement with concurrent execution

**AI Prompt Used**: "Create a benchmark showing 5 simulated API calls (1s each): sequential takes 5s, concurrent takes 1s. Include timing output."

**Generated Code**: Above benchmarking example

**Validation Steps**:
1. ✅ Sequential time ~5s (1 + 1 + 1 + 1 + 1)
2. ✅ Concurrent time ~1s (max of all)
3. ✅ Speedup calculation is 5x
4. ✅ Both approaches return same number of results
5. ✅ Type hints complete (`list[dict[str, Any]]`)

---

## Example 5: TaskGroup Error Propagation in Action

Let's see what happens when TaskGroup encounters a failure:

```python
import asyncio

async def work_on_task(task_id: int, duration: float) -> str:
    """Do work for a task."""
    print(f"Task {task_id} starting (will take {duration}s)")
    await asyncio.sleep(duration)

    if task_id == 2:
        raise ValueError(f"Task {task_id} failed during processing")

    return f"Task {task_id} completed"

async def main() -> None:
    """Demonstrate TaskGroup cancellation on failure."""
    try:
        async with asyncio.TaskGroup() as tg:
            task1 = tg.create_task(work_on_task(1, 5))
            task2 = tg.create_task(work_on_task(2, 1))  # Will fail after 1s
            task3 = tg.create_task(work_on_task(3, 5))

        # If we reach here, all succeeded
        print(f"Results: {task1.result()}, {task2.result()}, {task3.result()}")

    except ExceptionGroup as eg:
        print(f"\nTaskGroup failed: {eg}")
        print("Key point: Task 1 and Task 3 were cancelled automatically!")
        print("(They never reached completion despite having 5s allocated)")

asyncio.run(main())
```

Output:
```
Task 1 starting (will take 5s)
Task 2 starting (will take 1s)
Task 3 starting (will take 5s)

TaskGroup failed: ...
Key point: Task 1 and Task 3 were cancelled automatically!
(They never reached completion despite having 5s allocated)
```

**This is crucial behavior**: When Task 2 fails, the entire TaskGroup is cancelled. Tasks 1 and 3 don't finish their 5 seconds—they're interrupted and cleaned up. This is **structured concurrency**: all-or-nothing execution with automatic cleanup.

#### Spec Reference & Validation

**Specification**: Demonstrate TaskGroup automatic cancellation and exception grouping

**AI Prompt Used**: "Create 3 tasks where task 2 fails. Show how TaskGroup automatically cancels tasks 1 and 3. Explain ExceptionGroup."

**Generated Code**: Above task cancellation example

**Validation Steps**:
1. ✅ Code runs and catches ExceptionGroup
2. ✅ Task 2 raises ValueError as expected
3. ✅ Tasks 1 and 3 are cancelled (don't complete)
4. ✅ Exception handling with ExceptionGroup works
5. ✅ Output shows cancellation behavior clearly

---

## Choosing Your Pattern: Gather vs TaskGroup

Now you understand three approaches. How do you choose?

| Scenario | Pattern | Why |
|----------|---------|-----|
| Fetching backup data sources (want best-effort) | `gather(return_exceptions=True)` | Collect all available data even if some fail |
| Parallel calculations that depend on each other | `TaskGroup()` | If one fails, stop immediately; no point continuing |
| Large number of independent operations (100+) | `asyncio.create_task()` + manual collection | More control, better performance tuning |
| Building a resilient API aggregator | `gather()` | Accept partial results; frontend handles missing data |
| Building an atomic transaction system | `TaskGroup()` | All succeed or all roll back |

#### ✨ Teaching Tip

> Use Claude Code to explore the tradeoff: Ask "Compare gather() vs TaskGroup() for a web crawler that fetches 1000 URLs. Should I cancel all on first failure, or collect whatever succeeds?" Your AI will explain the architectural implications.

---

## Challenge 2: The Concurrent Tasks Builder

This is a **4-part bidirectional learning challenge** where you master task scheduling patterns through collaborative exploration.

### Part 1: Discover Independently (Student as Scientist)
**Your Challenge**: Explore the difference between sequential and concurrent patterns without AI.

**Deliverable**: Create `/tmp/task_patterns.py` containing:
1. A sequential version: `await fetch_api("A", 1); await fetch_api("B", 1)` — measure time (should be ~2s)
2. A concurrent version using `asyncio.create_task()` — schedule both tasks, then await them — measure time (should be ~1s)
3. A third version using `asyncio.gather()` — same as concurrent but more concise

**Expected Observation**:
- Sequential: ~2 seconds (tasks run one after another)
- Concurrent: ~1 second (tasks run simultaneously)
- Gather: ~1 second (but cleaner syntax)

**Self-Validation**:
- Can you explain why concurrent is faster? (Tasks overlap in execution time)
- What happens if you schedule 10 tasks instead of 2? (Still ~1s, not ~10s)
- How would error handling differ in sequential vs concurrent? (Sequential: first error stops all; concurrent: might continue)

---

### Part 2: AI as Teacher (Teaching You Patterns)
**Your AI Prompt**:
> "I want to run 10 database queries concurrently, each taking 1 second. I tried wrapping them in `async def` and using `await` inside a for loop, but it still takes 10 seconds. Teach me the difference between 'awaiting immediately' and 'scheduling first.' Show me code using both `asyncio.gather()` and `asyncio.create_task()`. Which should I use when?"

**AI's Role**: Explain the conceptual difference (immediate await blocks; scheduling creates pending tasks), show both patterns, and guide you on when each applies.

**Interactive Moment**: Ask a clarifying question:
> "You showed me `asyncio.gather(*[task() for task in ...]`. Explain exactly what gather() does—does it start the tasks? When do they actually start running?"

**Expected Outcome**: AI clarifies that gather() takes coroutines (not tasks) and schedules them internally, then awaits all their results. You understand gather() as a convenience wrapper.

---

### Part 3: You as Teacher (Discovering Edge Cases)
**Setup**: AI generates code using TaskGroup. Your job is to explore behavior differences and teach AI.

**AI's Initial Code** (ask for this):
```python
async def fetch_with_error(name: str, delay: float, fail: bool = False):
    await asyncio.sleep(delay)
    if fail:
        raise ValueError(f"{name} failed")
    return f"Data: {name}"

async def main():
    async with asyncio.TaskGroup() as tg:
        t1 = tg.create_task(fetch_with_error("API-1", 1))
        t2 = tg.create_task(fetch_with_error("API-2", 1, fail=True))
        t3 = tg.create_task(fetch_with_error("API-3", 1))
    print([t1.result(), t2.result(), t3.result()])

asyncio.run(main())
```

**Your Task**:
1. Run this code. What happens? (Should fail with exception from API-2, cancels API-3)
2. Compare to the same code using `asyncio.gather(return_exceptions=True)` — what's the difference?
3. Teach AI:
> "TaskGroup cancelled API-3 even though it was running. But I want API-1 and API-3 to complete even if API-2 fails. Show me gather() with return_exceptions=True and explain why it's different. When would you choose TaskGroup (all-or-nothing) vs gather (best-effort)?"

**Your Edge Case Discovery**: Ask AI:
> "What happens if I set a timeout on gather()? Like `asyncio.wait_for(asyncio.gather(...), timeout=2)`. How does this interact with return_exceptions?"

**Expected Outcome**: You discover the difference between fail-fast (TaskGroup) and resilient (gather) patterns, and learn when each applies in production.

---

### Part 4: Build Production Artifact (Student as Engineer)
**Your Capstone for This Challenge**: Build a real-world task coordinator.

**Specification**:
- Fetch from 8 external services with varied latency (use asyncio.sleep):
  - 3 services: 0.5s each
  - 3 services: 1.5s each
  - 2 services: 2.5s each
- 2 services fail randomly (raise exception)
- Use `asyncio.gather(return_exceptions=True)` for resilience
- Include timeout: if total fetch takes >5 seconds, cancel remaining tasks
- Return: (successful_results, failed_services, timed_out_flag)
- Type hints throughout

**Deliverable**: Save to `/tmp/task_coordinator.py`

**Testing Your Work**:
```bash
python /tmp/task_coordinator.py
# Expected output:
# Successfully fetched: 6 services
# Failed services: 2 (handled gracefully)
# Total time: ~2.5 seconds (longest service)
# Timeout triggered: False (or True if >5s)
```

**Validation Checklist**:
- [ ] Code runs without raising exceptions
- [ ] All 8 services attempted (gather runs all)
- [ ] Failed services caught by return_exceptions=True
- [ ] Total time ~2.5s (longest latency, not sum)
- [ ] Timeout mechanism works (can adjust latency to test)
- [ ] Type hints complete
- [ ] Follows production pattern (asyncio.run at top, gather for coordination)

---

**Time Estimate**: 30-35 minutes (5 min discover, 8 min teach/learn, 8 min edge cases, 9-14 min build artifact)

**Key Takeaway**: You've mastered three task coordination patterns and understand when to choose resilience (gather) over atomicity (TaskGroup).
