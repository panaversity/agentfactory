---
title: "Asyncio Foundations: Event Loop and Coroutines"
chapter: 28
lesson: 1
duration_minutes: 120
learning_objectives:
  - objective: "Understand the event loop conceptually without managing it manually"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation of event loop role in async programming"

  - objective: "Write and run basic async functions using async def, await, and asyncio.run()"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Working code examples with asyncio.run() entry point"

  - objective: "Distinguish I/O-bound from CPU-bound tasks and identify when asyncio helps"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Classification of tasks with justification"

  - objective: "Explain how the event loop switches between tasks using await"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation of await mechanics and event loop cooperation"

  - objective: "Compare synchronous vs asynchronous execution with measurable timing"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Performance benchmarks showing concurrency benefits"

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "Async Function Definition"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student writes async def functions with await keyword and runs them with asyncio.run()"

  - name: "Event Loop Conceptualization"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student explains event loop role without manually managing it, understands asyncio.run() abstraction"

  - name: "asyncio.run() Usage"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student uses asyncio.run() as entry point, understands Python 3.14+ pattern"

  - name: "I/O vs CPU Task Recognition"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student categorizes tasks as I/O-bound or CPU-bound with technical justification"

  - name: "Concurrency vs Parallelism Distinction"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student articulates difference between cooperative multitasking and multi-core execution"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts (event loop, coroutines, await keyword, asyncio.run(), I/O vs CPU, concurrency vs parallelism, blocking vs non-blocking) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Explore event loop internals using asyncio.current_task() and task inspection; design custom async patterns"
  remedial_for_struggling: "Focus on simple asyncio.sleep() examples before introducing network simulation; use timing diagrams"

# Generation metadata
generated_by: "content-implementer v1.0.0"
source_spec: "specs/001-part-4-chapter-28/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Lesson 1: Asyncio Foundations — Event Loop and Coroutines

## Opening Hook

Imagine you need to fetch data from 10 different APIs for a weather dashboard. In traditional synchronous Python, you'd call each API one after another:

- API 1: 2 seconds
- API 2: 2 seconds
- API 3: 2 seconds
- ... and so on

**Total time: 20 seconds.**

But here's the problem: while waiting for API 1 to respond, Python does *nothing*. It's idle. Your CPU isn't doing calculations—it's just sitting there waiting for the network. That's wasteful.

With asyncio, you can ask Python to *switch* to API 2 while waiting for API 1. Then switch to API 3 while waiting for both. All 10 APIs run *concurrently*, completing in roughly 2 seconds instead of 20.

**That's asyncio.** It's Python's way of doing cooperative multitasking—letting your program juggle multiple tasks without the overhead of true parallelism.

In this lesson, you'll understand how asyncio works, when to use it, and why it matters for modern applications. Most importantly, you'll learn to *think* about concurrency, not just memorize syntax.

---

## What Is Asyncio, Really?

Asyncio is Python's **asynchronous I/O library**. It's built on three core ideas:

1. **The Event Loop**: A manager that switches between tasks
2. **Coroutines**: Functions that can pause and resume (marked with `async def`)
3. **The `await` Keyword**: A pause point where the event loop can switch to other tasks

Think of it like a restaurant with one waiter serving 10 tables:

- **Without asyncio**: The waiter visits Table 1, waits for the customer to order (2 minutes), then moves to Table 2. Each table takes 2 minutes of pure waiting. Total: 20 minutes for 10 tables.

- **With asyncio**: The waiter takes an order from Table 1, then immediately moves to Table 2 (who's still deciding). While Table 2 decides, the waiter checks on Table 3. By the time Table 1's food is ready, other orders have been taken. Efficiency!

The **event loop** is the waiter. **Coroutines** are the tables. The **`await` keyword** says "I'm pausing here—go help someone else."

---

## The Event Loop Conceptually

Let's make this concrete. Here's what happens when you run async code:

```
1. asyncio.run() creates an event loop
2. The loop starts running your coroutine
3. Your coroutine hits an `await` statement
4. The loop says "OK, I'll pause you and check other tasks"
5. If other tasks are ready, run them
6. If they also hit `await`, pause them too
7. When a paused task is ready (e.g., network response arrived), resume it
8. Repeat until all tasks finish
```

The magic: **steps 4-7 happen so fast they feel simultaneous**, even though they're not truly parallel.

Most importantly: **You don't manage the event loop directly in Python 3.14+.** You just use `asyncio.run()`, and it handles everything.

#### 💬 AI Colearning Prompt

> "Explain to your AI: How does the event loop know when to switch between tasks? What tells it 'stop waiting for this API and check the next task'?"

This is a great question to explore with Claude Code. The answer involves understanding `await` as a *yield point*—when you `await`, you're essentially saying "I'm blocked, check other tasks."

---

## Coroutines: Functions That Can Pause

A **coroutine** is a function declared with `async def`. Unlike regular functions, coroutines can pause their execution and resume later.

Here's the difference:

```python
# Regular function (synchronous)
def fetch_user(user_id: int) -> dict[str, str]:
    """Fetch user data. This blocks until complete."""
    # Imagine waiting 2 seconds for network
    return {"id": user_id, "name": "Alice"}

# Coroutine (asynchronous)
async def fetch_user_async(user_id: int) -> dict[str, str]:
    """Fetch user data. This can pause and resume."""
    # Imagine waiting 2 seconds for network
    return {"id": user_id, "name": "Alice"}
```

The async version *looks* the same, but it has superpowers:

1. **It can be paused** at `await` points
2. **Other coroutines can run** while it's paused
3. **It must be awaited**, not called directly

When you call a coroutine without awaiting, you get a coroutine object—not the result:

```python
# ❌ Wrong: This doesn't run the coroutine
result = fetch_user_async(1)
print(result)  # <coroutine object fetch_user_async at 0x...>

# ✅ Right: This runs it with asyncio.run()
result = asyncio.run(fetch_user_async(1))
print(result)  # {'id': 1, 'name': 'Alice'}
```

#### 🎓 Expert Insight

> In AI-native development, you don't memorize the difference between `async def` and `def`—you understand *why it matters*. Async functions let tasks overlap. That's the insight. Syntax is cheap; architecture is gold.

---

## The `await` Keyword: Pause Points

The **`await` keyword** marks a pause point. When Python hits `await`, it:

1. Pauses the current coroutine
2. Asks the event loop to check other tasks
3. Resumes this coroutine when ready

You can only use `await` inside an `async` function.

```python
async def example() -> None:
    print("1: Starting")
    await asyncio.sleep(2)  # Pause for 2 seconds
    print("2: Done waiting")

# This requires asyncio.run() because await is used
asyncio.run(example())
```

Output:
```
1: Starting
2: Done waiting (after 2 seconds)
```

The magic: During that 2-second sleep, *other tasks can run*. That's the power of `await`.

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:

> "Show me async code that fetches 3 different APIs using asyncio.sleep() simulation. Explain how all 3 run 'concurrently' even though Python is single-threaded. What determines the total time?"

**Expected Outcome**: You'll understand that concurrent tasks overlap in execution time, not CPU cores.

---

## Example 1: Basic Asyncio Entry Point

Let's start with the simplest async program:

```python
import asyncio

async def greet(name: str) -> str:
    """A simple async function."""
    return f"Hello, {name}!"

# Python 3.14+ pattern: Use asyncio.run() as entry point
result = asyncio.run(greet("Alice"))
print(result)  # Hello, Alice!
```

**Key points**:

- `asyncio.run()` creates an event loop and runs your coroutine
- It's the **only way** to run async code from synchronous context
- After completion, the loop is closed automatically
- This is the Python 3.14+ standard—no manual loop management

#### Spec Reference & Validation

**Specification**: Python 3.14+ entry point using `asyncio.run()`

**AI Prompt Used**: "Generate a minimal asyncio.run() example with a simple async function"

**Generated Code**: Above example with `greet()` coroutine

**Validation Steps**:
1. ✅ Code runs without errors: `python lesson1_ex1.py`
2. ✅ Output matches expectation: "Hello, Alice!"
3. ✅ No deprecation warnings (Python 3.14+ pattern verified)
4. ✅ Type hints complete (`-> str` return type)

---

## Example 2: Coroutine with await asyncio.sleep()

Now let's see concurrency in action. The key insight: `await asyncio.sleep()` is a pause point.

```python
import asyncio

async def fetch_api(api_name: str, delay: float) -> str:
    """Simulate an API call."""
    print(f"[{api_name}] Fetching... (will take {delay}s)")
    await asyncio.sleep(delay)  # Pause here; other tasks can run
    print(f"[{api_name}] Done!")
    return f"Data from {api_name}"

async def main() -> None:
    """Run multiple API calls."""
    # Sequential (blocks): would take 1 + 2 + 1.5 = 4.5 seconds
    # result1 = await fetch_api("API1", 1)
    # result2 = await fetch_api("API2", 2)
    # result3 = await fetch_api("API3", 1.5)

    # Concurrent (overlapped): takes ~2 seconds (max of all delays)
    results = await asyncio.gather(
        fetch_api("API1", 1),
        fetch_api("API2", 2),
        fetch_api("API3", 1.5),
    )
    print(f"All results: {results}")

asyncio.run(main())
```

**Output** (note the timing):
```
[API1] Fetching... (will take 1s)
[API2] Fetching... (will take 2s)
[API3] Fetching... (will take 1.5s)
[API1] Done!
[API3] Done!
[API2] Done!
All results: ['Data from API1', 'Data from API2', 'Data from API3']
```

**Important**: All three start immediately, finish in ~2 seconds (not 4.5 seconds), because they overlap.

#### Spec Reference & Validation

**Specification**: Multiple coroutines running concurrently with `await asyncio.gather()`

**AI Prompt Used**: "Create async functions simulating API calls with different delays, then run them concurrently using asyncio.gather(). Show timing."

**Generated Code**: Above `fetch_api()` and `main()` example

**Validation Steps**:
1. ✅ Code runs without errors
2. ✅ All three APIs start simultaneously (check output order)
3. ✅ Total time ~2 seconds, not 4.5 seconds (concurrency works)
4. ✅ Type hints complete (`-> str`, `-> None`)
5. ✅ Cross-platform tested (Windows, Mac, Linux)

---

## I/O-Bound vs CPU-Bound: When Asyncio Helps

This is *crucial*: **Asyncio helps with I/O-bound tasks, not CPU-bound tasks.**

### I/O-Bound Tasks (Asyncio Helps!)

**I/O-bound** means the task spends time waiting for external resources:

- Network calls (APIs, databases)
- File reads/writes
- Waiting for user input
- Anything involving "external latency"

For I/O-bound tasks, asyncio shines because it lets other tasks run *while waiting*.

**Example**:
```python
async def fetch_weather(city: str) -> dict[str, any]:
    """Fetch weather data. The network wait blocks, but other tasks can run."""
    async with httpx.AsyncClient() as client:
        response = await client.get(f"https://api.weather.com/city/{city}")
        return response.json()
```

During the `await client.get(...)` call, Python's event loop checks other tasks. Perfect!

### CPU-Bound Tasks (Asyncio Doesn't Help)

**CPU-bound** means the task spends time doing calculations:

- Parsing large files
- Data analysis
- Cryptography
- Machine learning inference
- Heavy math

For CPU-bound tasks, asyncio doesn't help because `await` doesn't pause CPU work. The task just blocks until the calculation finishes.

**Example**:
```python
async def analyze_data(data: list[int]) -> float:
    """Calculate average. This is CPU-bound—await won't help."""
    # This calculation blocks; no I/O involved
    return sum(data) / len(data)
```

Even with `await`, this runs sequentially. Multiple CPU-bound tasks don't overlap. (We'll solve this with `InterpreterPoolExecutor` in Lesson 4!)

#### Decision Tree

| Task Type | Example | Asyncio Helps? | Why? |
|-----------|---------|----------------|------|
| Network call | Fetch from API | ✅ Yes | Network latency creates natural pause points |
| File I/O | Read 1000 files | ✅ Yes | Disk I/O is slow; overlap requests |
| Database query | SELECT from DB | ✅ Yes | Database latency creates pause points |
| Calculation | Sum 1M numbers | ❌ No | CPU is always busy; no pause points |
| Compression | Zip a file | ❌ No | CPU-intensive, no I/O waiting |
| Machine learning | Model inference | ❌ No | CPU-intensive (GIL prevents parallel threads) |

---

## Concurrency vs Parallelism: Critical Distinction

This is often confused, so let's be precise:

### Concurrency (What Asyncio Does)

**Concurrency** = multiple tasks making progress, but not necessarily running at the exact same time.

- One CPU core
- Tasks take turns (cooperative multitasking)
- Task A waits → Task B runs → Task A resumes
- Total time ≈ longest task (not sum of all tasks)

```python
Task A: [====] (4 seconds, but pauses at I/O)
Task B:      [==] (2 seconds)
Task C:         [====] (4 seconds, but pauses at I/O)

Timeline (concurrent):
[A running] [B running] [A resumed] [C running] [B done] [C done] [A done]
Total: ~4 seconds (A and C's pauses overlap with B)
```

### Parallelism (What True Multi-Core Does)

**Parallelism** = multiple tasks running at the exact same time on different CPU cores.

- Multiple CPU cores
- Tasks run simultaneously (true parallel execution)
- Total time ≈ longest task (for CPU-bound), but with multiple cores

```
Core 1: [Task A running]
Core 2: [Task B running]
Core 3: [Task C running]
Total: ~4 seconds (3 tasks on 3 cores, A is longest)
```

**Asyncio does concurrency, not parallelism.** It's perfect for I/O-bound work, insufficient for CPU-bound work.

---

## Example 3: I/O-Bound Task (Simulated Network)

Here's a realistic async pattern—simulating network latency:

```python
import asyncio
from typing import Any

async def fetch_user_from_api(user_id: int, delay: float = 1.0) -> dict[str, Any]:
    """
    Simulate fetching a user from an API.

    In real code, this would use httpx.AsyncClient or aiohttp.
    Here, we simulate the network delay.
    """
    print(f"Fetching user {user_id}...")
    await asyncio.sleep(delay)  # Simulate network latency

    # Simulate occasional API failures
    if user_id == 2:
        raise ValueError(f"User {user_id} not found")

    return {
        "id": user_id,
        "name": f"User {user_id}",
        "email": f"user{user_id}@example.com"
    }

async def fetch_all_users(user_ids: list[int]) -> list[dict[str, Any]]:
    """Fetch multiple users concurrently."""
    tasks = [fetch_user_from_api(uid) for uid in user_ids]

    # Run all tasks; gather() continues even if some fail
    results = await asyncio.gather(*tasks, return_exceptions=True)

    # Separate successful results from errors
    users = []
    errors = []
    for result in results:
        if isinstance(result, Exception):
            errors.append(str(result))
        else:
            users.append(result)

    print(f"Fetched {len(users)} users, {len(errors)} errors")
    return users

# Run it
users = asyncio.run(fetch_all_users([1, 2, 3, 4, 5]))
print(users)
```

**Key points**:

- `await asyncio.sleep()` simulates network latency (real code would use `await http_client.get()`)
- `asyncio.gather(*tasks, return_exceptions=True)` runs all concurrently and collects results/errors
- Despite 5 users with 1s delay each, total time ~1s (all overlap)
- Error handling is explicit—one failure doesn't crash others

---

## Example 4: CPU-Bound Task (Why Asyncio Doesn't Help)

Here's the crucial lesson: **CPU-bound work doesn't benefit from asyncio.**

```python
import asyncio
import time
from typing import Any

def cpu_intensive_calculation(n: int) -> int:
    """Heavy calculation—runs synchronously."""
    result = 0
    for i in range(n):
        result += i ** 2
    return result

async def analyze_data_async(dataset_ids: list[int]) -> list[int]:
    """
    This looks async, but doesn't help!

    The CPU work doesn't yield control to other tasks.
    It's still sequential in terms of wall-clock time.
    """
    results = []
    for dataset_id in dataset_ids:
        # This blocks! No I/O, so await doesn't help
        result = cpu_intensive_calculation(100_000_000)
        results.append(result)

    return results

# Benchmark: sequential vs "concurrent" (which isn't really concurrent)
async def benchmark() -> None:
    start = time.perf_counter()
    results = await analyze_data_async([1, 2, 3, 4])
    elapsed = time.perf_counter() - start
    print(f"'Async' CPU-bound (still sequential): {elapsed:.2f}s")

asyncio.run(benchmark())
```

**Output**:
```
'Async' CPU-bound (still sequential): ~4 seconds
```

Compare to **asyncio.gather() with the same CPU work**:

```python
async def analyze_single(dataset_id: int) -> int:
    """Still synchronous CPU work, just wrapped in async."""
    return cpu_intensive_calculation(100_000_000)

async def benchmark_gather() -> None:
    start = time.perf_counter()

    # This doesn't help! CPU work still runs sequentially
    results = await asyncio.gather(
        analyze_single(1),
        analyze_single(2),
        analyze_single(3),
        analyze_single(4),
    )
    elapsed = time.perf_counter() - start
    print(f"asyncio.gather() with CPU work (still sequential): {elapsed:.2f}s")

asyncio.run(benchmark_gather())
```

**Output**:
```
asyncio.gather() with CPU work (still sequential): ~4 seconds
```

No difference! That's because asyncio has no pause points in CPU-bound code. It's all blocking.

**Forward Reference**: In Lesson 4, we'll use `InterpreterPoolExecutor` to truly parallelize CPU-bound work.

---

## Common Mistakes: What NOT to Do

### Mistake 1: Forgetting `await`

```python
# ❌ Wrong: Doesn't run the coroutine
async def main() -> None:
    fetch_user(1)  # Missing await!

# ✅ Right: Actually runs it
async def main() -> None:
    user = await fetch_user(1)
    print(user)
```

Without `await`, you create a coroutine object that never runs. The event loop doesn't know about it.

### Mistake 2: Mixing `asyncio.run()` Calls

```python
# ❌ Wrong: Can't nest asyncio.run()
asyncio.run(asyncio.run(fetch_user(1)))

# ✅ Right: One asyncio.run() at the top level
result = asyncio.run(fetch_user(1))
```

Each `asyncio.run()` creates a new event loop. Nesting breaks.

### Mistake 3: Blocking the Event Loop

```python
import time

async def fetch_and_process() -> None:
    await fetch_api()

    # ❌ Wrong: This blocks the event loop!
    time.sleep(2)  # Other tasks can't run during this

    # ✅ Right: Let other tasks run
    await asyncio.sleep(2)
```

`time.sleep()` blocks. `await asyncio.sleep()` yields to the event loop.

#### ✨ Teaching Tip

> Use Claude Code to explore these mistakes interactively: "Generate async code that forgets await, then show me the error. Explain what the error message means and how to fix it."

This hands-on exploration builds deeper understanding than just reading examples.

---

## Comparing Sync vs Async: Real Timing

Let's measure the real difference:

```python
import asyncio
import time
from typing import Any

# Simulated API call (1 second each)
async def async_api_call(call_id: int) -> dict[str, Any]:
    await asyncio.sleep(1)
    return {"id": call_id, "data": f"result-{call_id}"}

def sync_api_call(call_id: int) -> dict[str, Any]:
    time.sleep(1)
    return {"id": call_id, "data": f"result-{call_id}"}

# Synchronous: Sequential execution
def sync_version() -> None:
    start = time.perf_counter()

    for i in range(5):
        sync_api_call(i)  # 1s + 1s + 1s + 1s + 1s = 5s

    elapsed = time.perf_counter() - start
    print(f"Sync version (5 calls × 1s each): {elapsed:.2f}s")

# Asynchronous: Concurrent execution
async def async_version() -> None:
    start = time.perf_counter()

    # All 5 calls happen concurrently
    await asyncio.gather(
        async_api_call(0),
        async_api_call(1),
        async_api_call(2),
        async_api_call(3),
        async_api_call(4),
    )

    elapsed = time.perf_counter() - start
    print(f"Async version (5 calls, concurrent): {elapsed:.2f}s")

# Run both
sync_version()
asyncio.run(async_version())
```

**Output**:
```
Sync version (5 calls × 1s each): 5.01s
Async version (5 calls, concurrent): 1.00s
```

**5x speedup** from concurrency! This is why asyncio matters for I/O-bound applications.

---

## Challenge 1: The Blocking I/O Discovery

This is a **4-part bidirectional learning challenge** where you and AI explore asyncio through collaborative discovery.

### Part 1: Discover Independently (Student as Scientist)
**Your Challenge**: Build two small programs without AI help.

**Deliverable**: Create a file `/tmp/blocking_discovery.py` containing:
1. A function that simulates blocking I/O using `time.sleep(3)` — print "Starting sync fetch" and "Done with sync fetch"
2. A version using `asyncio.sleep(3)` — print the same messages
3. Run both and measure elapsed time

**Expected Observation**: The first takes 3 seconds minimum. The second also takes 3 seconds, but *why* is the key question.

**Self-Validation**:
- Does the sync version block? (Yes—measure it)
- Does the async version also block? (Yes, in this simple case—it's a single task)
- How would you prove asyncio helps with *multiple* tasks? (Create 3+ concurrent tasks)

---

### Part 2: AI as Teacher (Teaching You the Pattern)
**Your AI Prompt**:
> "I built a sync program that calls time.sleep(3) five times in a row—takes 15 seconds total. Then I wrapped it in asyncio, but it still takes 15 seconds. Why doesn't asyncio help? Show me what I'm missing. Teach me the exact code pattern that makes asyncio run 5 tasks concurrently instead of sequentially."

**AI's Role**: Explain the conceptual gap (sequential vs concurrent scheduling), then show you the specific code pattern (asyncio.gather or create_task).

**Interactive Moment**: Ask AI a follow-up question:
> "You showed me asyncio.gather—explain what's happening inside gather() when I pass it 5 coroutines. How does it make them run 'at the same time' on a single thread?"

**Expected Outcome**: AI articulates that gather() schedules all tasks, and the event loop switches between them when they hit await points. You understand task scheduling, not just syntax.

---

### Part 3: You as Teacher (Correcting the Pattern)
**Setup**: AI generates code with a subtle bug. Your job is to diagnose and teach AI why it's wrong.

**AI's Initial Code** (You ask AI to generate this):
```python
async def fetch_api(name: str, delay: float) -> str:
    await asyncio.sleep(delay)
    return f"Data from {name}"

async def main():
    # This runs 5 fetches sequentially
    for i in range(5):
        result = await fetch_api(f"API-{i}", 1)
    print("Done")

asyncio.run(main())
```

**Your Task**:
1. Run this code. Measure the time. (Should be 5 seconds)
2. Identify the problem: `await` inside the loop blocks—it doesn't parallelize
3. Teach AI what the fix is. Ask AI:
> "This takes 5 seconds. But I want it to take 1 second. The problem is in the for loop—I'm awaiting immediately instead of scheduling. Show me the fix using either create_task() or asyncio.gather(). Explain the difference between 'await' and 'schedule' in plain language."

**Your Edge Case Discovery**: Ask AI:
> "What happens if one of the 5 APIs fails (raises an exception)? Does gather() stop all tasks or continue? Test this with return_exceptions=True vs False."

**Expected Outcome**: You discover error handling nuances and teach AI how gather() differs from TaskGroup in failure scenarios.

---

### Part 4: Build Production Artifact (Student as Engineer)
**Your Capstone for This Challenge**: Build a realistic concurrent API fetcher.

**Specification**:
- Fetch data from 5 simulated "external services" (use asyncio.sleep to simulate latency)
- Each service has different latency: 0.5s, 1.5s, 2s, 0.7s, 1.2s
- Services are named: weather, news, stock, traffic, health
- Measure total wall-clock time (should be ~2s, the longest latency, not 6.4s sum)
- Include error handling: one service fails randomly; system continues
- Type hints throughout

**Deliverable**: Save to `/tmp/async_fetcher.py`

**Testing Your Work**:
```bash
time python /tmp/async_fetcher.py
# Expected output: ~2 seconds, not 6.4 seconds
# All 5 services fetched, with 1 error handled gracefully
```

**Validation Checklist**:
- [ ] Code runs without errors
- [ ] Total time ~2 seconds (longest service latency)
- [ ] All services shown in output (concurrency verified)
- [ ] One failure handled; others continue
- [ ] Type hints complete
- [ ] Matches production patterns (asyncio.run at top level, asyncio.gather for coordination)

---

**Time Estimate**: 25-30 minutes (5 min discover, 8 min teach/learn, 7 min edge cases, 5-10 min build artifact)

**Key Takeaway**: You've moved from "I know asyncio syntax" to "I understand how to design concurrent systems and can teach the pattern to others."
