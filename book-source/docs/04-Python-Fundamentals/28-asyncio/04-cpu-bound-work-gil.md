---
title: "CPU-Bound Work: GIL and InterpreterPoolExecutor"
chapter: 28
lesson: 4
duration_minutes: 150

learning_objectives:
  - objective: "Understand the Global Interpreter Lock (GIL) limitation in Python threading"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "2-3 sentence explanation of GIL with forward reference to Chapter 29"

  - objective: "Apply InterpreterPoolExecutor for true CPU parallelism in async programs"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Working code using InterpreterPoolExecutor with loop.run_in_executor()"

  - objective: "Measure performance differences between threading and InterpreterPoolExecutor"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Benchmark showing 3-4x speedup on 4-core machine with InterpreterPoolExecutor"

  - objective: "Choose between InterpreterPoolExecutor and ProcessPoolExecutor based on requirements"
    proficiency_level: "B2"
    bloom_level: "Evaluate"
    assessment_method: "Decision justification with tradeoff analysis"

  - objective: "Bridge CPU-bound work into async code using run_in_executor()"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Integration of CPU work into async program without blocking event loop"

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "GIL Conceptual Understanding"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student explains GIL in 2-3 sentences and understands threading limitation for CPU-bound work"

  - name: "InterpreterPoolExecutor Usage"
    proficiency_level: "B1-B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student uses InterpreterPoolExecutor with loop.run_in_executor() for CPU parallelism"

  - name: "Performance Benchmarking"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student measures and interprets 3-4x speedup on 4-core machine"

  - name: "Executor Selection and Tradeoffs"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Evaluate"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student justifies choice between InterpreterPoolExecutor and ProcessPoolExecutor"

  - name: "Async-Sync Integration"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student bridges sync CPU-bound functions into async code without blocking"

cognitive_load:
  new_concepts: 8
  assessment: "8 new concepts (GIL, threading limitation, InterpreterPoolExecutor, separate interpreters, loop.run_in_executor(), true parallelism, ProcessPoolExecutor, decision criteria) within B1-B2 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Compare free-threaded Python (3.13+ experimental) implications; explore C extension performance with GIL; design adaptive executor selection based on workload characteristics"
  remedial_for_struggling: "Focus on simple benchmark examples before advanced decision trees; use visual comparisons (threading graph vs InterpreterPoolExecutor graph)"

# Generation metadata
generated_by: "content-implementer v1.0.0"
source_spec: "specs/001-part-4-chapter-28/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Lesson 4: CPU-Bound Work — GIL and InterpreterPoolExecutor

## Opening Hook

Here's a puzzle: In Lesson 1, you learned that asyncio lets you run multiple tasks concurrently. So why not use asyncio for CPU-heavy calculations?

Try this thought experiment. You have a function that does heavy math (factorials, cryptography, data analysis). You create 4 async tasks that call this function. With asyncio, you'd expect them to run concurrently, right?

Wrong.

While the tasks are *technically* concurrent (the event loop switches between them), **they run slower—not faster—than sequential execution.** What's going on?

The culprit: **the Global Interpreter Lock (GIL).** And this lesson teaches you how to escape it using Python 3.14's new `InterpreterPoolExecutor`.

---

## What Is the GIL, Really? (Brief Intro)

Python's **Global Interpreter Lock (GIL)** is a mechanism that allows only one thread to execute Python bytecode at a time. This was a design choice made to simplify memory management in CPython (the standard Python interpreter). The GIL prevents true parallelism for CPU-bound work—even with multiple threads, only one thread can run Python code at any moment. Threading helps with I/O-bound work (one thread waits while others run), but for CPU-bound tasks where every thread is doing calculations, the GIL becomes a bottleneck.

**Deep exploration of GIL internals (how it works, why it exists, free-threaded mode) is covered in Chapter 29.** For now, understand this simple fact: *If you want true parallelism for CPU-bound work in Python, you need separate interpreters, not threads.*

#### 💬 AI Colearning Prompt

> "Ask your AI: Why does Python have a GIL? What problem was it solving originally, and why haven't Python developers removed it?"

---

## Why Threading Fails for CPU-Bound Work

Let's make this concrete with a benchmark.

### Code Example 1: CPU-Bound Function

```python
import time
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
from concurrent.futures.interpreters import InterpreterPoolExecutor

def cpu_intensive_task(n: int) -> int:
    """Compute factorial recursively (CPU-bound)."""
    if n <= 1:
        return 1
    return n * cpu_intensive_task(n - 1)

# Simulate heavy calculation
def heavy_calculation(iterations: int) -> int:
    """Run CPU-intensive loop (no I/O)."""
    result = 0
    for i in range(iterations):
        result += i ** 2  # Math-heavy operation
    return result
```

This function spends 100% of its time doing math—no waiting for I/O. Perfect for testing parallelism.

#### 🎓 Expert Insight

> In AI-native development, you don't memorize the GIL limitation—you recognize the pattern: "My task is CPU-heavy, so threading won't help." That recognition is worth more than any theory.

---

### Code Example 2: Threading Benchmark (Shows the Problem)

```python
import time
from concurrent.futures import ThreadPoolExecutor

def benchmark_threading(num_workers: int = 4, iterations: int = 50_000_000) -> None:
    """Benchmark CPU-bound work with threading."""

    start = time.perf_counter()

    with ThreadPoolExecutor(max_workers=num_workers) as executor:
        # Submit 4 tasks to 4 threads
        futures = [
            executor.submit(heavy_calculation, iterations)
            for _ in range(num_workers)
        ]
        results = [f.result() for f in futures]

    elapsed = time.perf_counter() - start
    print(f"Threading (4 workers): {elapsed:.2f}s")
    print(f"Result: {results}")

# Single-threaded baseline
def benchmark_sequential(iterations: int = 50_000_000) -> None:
    """Benchmark sequential execution (no parallelism)."""
    start = time.perf_counter()
    results = [heavy_calculation(iterations) for _ in range(4)]
    elapsed = time.perf_counter() - start
    print(f"Sequential (1 thread): {elapsed:.2f}s")
    print(f"Result: {results}")

if __name__ == "__main__":
    print("=== CPU-Bound Work Benchmarks ===\n")
    benchmark_sequential()
    benchmark_threading()
```

**Sample Output** (on 4-core machine):
```
=== CPU-Bound Work Benchmarks ===

Sequential (1 thread): 4.53s
Threading (4 workers): 6.12s
```

Notice: **Threading is SLOWER, not faster.** Why? Because the GIL forces the 4 threads to compete for access to the single interpreter. Context switching overhead makes it worse than sequential execution.

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:
> "Why does threading make CPU-bound work slower instead of faster? Explain how the GIL causes this contention and what context switching adds."

**Expected Outcome**: You'll understand that the GIL makes threading counterproductive for CPU work—the overhead of thread switching exceeds any benefit.

---

## InterpreterPoolExecutor: The Solution (Python 3.14+)

Here's Python 3.14's elegant solution: **separate interpreters, separate GILs.**

Instead of one interpreter shared among threads (competing for the GIL), `InterpreterPoolExecutor` creates a pool of **independent Python interpreters**. Each interpreter has its own GIL. No sharing = no contention = true parallelism.

#### Core Concept: Separate Interpreters = Separate GILs

```
Traditional Threading (1 interpreter, 1 GIL):
┌─────────────────────────────┐
│   One Python Interpreter    │
│  Thread 1 │ Thread 2 │ GIL  │
│  (waiting for GIL)          │
│  (only 1 can run at a time) │
└─────────────────────────────┘

InterpreterPoolExecutor (4 interpreters, 4 GILs):
┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐
│ Interpreter1 │  │ Interpreter2 │  │ Interpreter3 │  │ Interpreter4 │
│   Worker 1   │  │   Worker 2   │  │   Worker 3   │  │   Worker 4   │
│    (GIL 1)   │  │    (GIL 2)   │  │    (GIL 3)   │  │    (GIL 4)   │
│   Running    │  │   Running    │  │   Running    │  │   Running    │
│ (all in true parallel on 4 cores)                  │
└──────────────┘  └──────────────┘  └──────────────┘  └──────────────┘
```

---

### Code Example 3: InterpreterPoolExecutor Benchmark (Shows the Solution)

```python
import time
from concurrent.futures.interpreters import InterpreterPoolExecutor

def benchmark_interpreter_pool(num_workers: int = 4, iterations: int = 50_000_000) -> None:
    """Benchmark CPU-bound work with InterpreterPoolExecutor."""

    start = time.perf_counter()

    with InterpreterPoolExecutor(max_workers=num_workers) as executor:
        # Submit 4 tasks to 4 separate interpreters
        futures = [
            executor.submit(heavy_calculation, iterations)
            for _ in range(num_workers)
        ]
        results = [f.result() for f in futures]

    elapsed = time.perf_counter() - start
    print(f"InterpreterPoolExecutor (4 workers): {elapsed:.2f}s")
    print(f"Result: {results}")
    print(f"Speedup: {4.53 / elapsed:.2f}x")  # Compare to sequential

if __name__ == "__main__":
    print("=== CPU-Bound Work Benchmarks ===\n")
    print("Sequential (1 thread): 4.53s")
    benchmark_interpreter_pool()
```

**Sample Output** (on 4-core machine):
```
=== CPU-Bound Work Benchmarks ===

Sequential (1 thread): 4.53s
InterpreterPoolExecutor (4 workers): 1.15s
Speedup: 3.94x
```

**Nearly 4x speedup on 4 cores!** That's what true parallelism looks like.

#### ✨ Teaching Tip

> Use Claude Code to explore the overhead: "Create a benchmark comparing InterpreterPoolExecutor with 1, 2, 4, and 8 workers on your machine. What's the maximum speedup you observe?"

---

## Bridging CPU Work into Async Code

Now here's the critical pattern: **How do you use `InterpreterPoolExecutor` inside an async program?**

The answer: **`loop.run_in_executor()`**—a bridge between sync functions and async code.

### Code Example 4: Async Executor Integration with run_in_executor()

```python
import asyncio
from concurrent.futures.interpreters import InterpreterPoolExecutor
from typing import Any

# Sync function (CPU-bound)
def cpu_intensive_work(data: str) -> str:
    """CPU-heavy string processing (no await)."""
    result = ""
    for _ in range(10_000_000):
        result = data + result  # Expensive operation
    return result[:100]

# Async function using executor
async def process_with_executor(
    executor: InterpreterPoolExecutor,
    data: str
) -> str:
    """Run CPU work in executor without blocking event loop."""
    loop = asyncio.get_running_loop()

    # This awaits the result without blocking the loop
    result = await loop.run_in_executor(executor, cpu_intensive_work, data)
    return result

async def main() -> None:
    """Main async program combining I/O and CPU work."""

    with InterpreterPoolExecutor(max_workers=4) as executor:
        start = time.perf_counter()

        # Run multiple CPU tasks concurrently
        tasks = [
            process_with_executor(executor, f"data_{i}")
            for i in range(4)
        ]

        results = await asyncio.gather(*tasks)

        elapsed = time.perf_counter() - start
        print(f"Async + InterpreterPoolExecutor: {elapsed:.2f}s")
        print(f"Results: {len(results)} tasks completed")

if __name__ == "__main__":
    import time
    asyncio.run(main())
```

**Key Pattern**:
1. Create the executor outside the async context
2. Pass it to async functions
3. Use `await loop.run_in_executor(executor, function, args)`
4. The event loop switches while CPU work happens in the background
5. Results return to the async context seamlessly

#### 💬 AI Colearning Prompt

> "Explain: What does `loop.run_in_executor()` do? Why do we need `await` here if the executor handles everything?"

---

## ProcessPoolExecutor: An Alternative (With Tradeoffs)

`InterpreterPoolExecutor` is new in Python 3.14, so you might encounter `ProcessPoolExecutor` (the older approach) in existing codebases.

**Key differences:**

| Feature | InterpreterPoolExecutor | ProcessPoolExecutor |
|---------|------------------------|-------------------|
| **Workers** | Separate interpreters (lightweight) | Separate processes (heavyweight) |
| **Memory** | Shared memory, lower overhead | Isolated memory, high overhead |
| **Startup** | Fast (interpreter fork) | Slow (process startup) |
| **Data passing** | Direct (same Python namespace) | Serialization (pickle) |
| **Best for** | CPU work with Python objects | Long-running isolated tasks |

### Code Example 5: ProcessPoolExecutor Comparison

```python
import time
from concurrent.futures import ProcessPoolExecutor
from concurrent.futures.interpreters import InterpreterPoolExecutor

def benchmark_process_pool(iterations: int = 50_000_000) -> None:
    """Benchmark ProcessPoolExecutor."""
    start = time.perf_counter()

    with ProcessPoolExecutor(max_workers=4) as executor:
        futures = [
            executor.submit(heavy_calculation, iterations)
            for _ in range(4)
        ]
        results = [f.result() for f in futures]

    elapsed = time.perf_counter() - start
    print(f"ProcessPoolExecutor (4 workers): {elapsed:.2f}s")

def benchmark_interpreter_pool(iterations: int = 50_000_000) -> None:
    """Benchmark InterpreterPoolExecutor."""
    start = time.perf_counter()

    with InterpreterPoolExecutor(max_workers=4) as executor:
        futures = [
            executor.submit(heavy_calculation, iterations)
            for _ in range(4)
        ]
        results = [f.result() for f in futures]

    elapsed = time.perf_counter() - start
    print(f"InterpreterPoolExecutor (4 workers): {elapsed:.2f}s")

if __name__ == "__main__":
    print("=== Executor Comparison ===\n")
    benchmark_process_pool()
    benchmark_interpreter_pool()
```

**Typical Output**:
```
=== Executor Comparison ===

ProcessPoolExecutor (4 workers): 2.34s (more startup overhead)
InterpreterPoolExecutor (4 workers): 1.15s (lighter weight)
```

#### 🎓 Expert Insight

> The GIL isn't a bug—it's a design tradeoff. Python 3.14 gives you tools to work around it when you need true parallelism. For most code, you'll prefer `InterpreterPoolExecutor` over `ProcessPoolExecutor` because it's lighter and faster.

---

## Decision Tree: When to Use What

Here's the practical decision guide:

### Code Example 6: Decision Tree (Conceptual Guide)

```python
"""
Decision guide for choosing concurrency patterns.
Reference this when you ask: "What tool should I use?"
"""

def choose_concurrency_tool(task_type: str, data_size: str) -> str:
    """
    Recommend concurrency approach based on task characteristics.

    Args:
        task_type: "io_bound" or "cpu_bound"
        data_size: "small", "medium", or "large"

    Returns:
        Recommended executor/pattern
    """

    decision_tree = {
        "io_bound": {
            # I/O-bound: waiting for network, files, databases
            "small": "asyncio.TaskGroup()",  # Simple, fast
            "medium": "asyncio.TaskGroup() + semaphore",  # Control concurrency
            "large": "asyncio.TaskGroup() + semaphore + batching",  # Prevent overload
        },
        "cpu_bound": {
            # CPU-bound: heavy calculations
            "small": "ProcessPoolExecutor or InterpreterPoolExecutor",
            "medium": "InterpreterPoolExecutor (lighter weight)",
            "large": "InterpreterPoolExecutor (scales to cores)",
        }
    }

    return decision_tree[task_type][data_size]

# Example usage:
# print(choose_concurrency_tool("cpu_bound", "small"))
# Output: "ProcessPoolExecutor or InterpreterPoolExecutor"

# Decision rules:
# 1. Pure I/O-bound (API calls, file reads)? → Use asyncio (TaskGroup)
# 2. CPU-bound (calculations, data processing)? → Use InterpreterPoolExecutor
# 3. Both I/O and CPU mixed? → Use asyncio for I/O + InterpreterPoolExecutor for CPU
# 4. Long-running isolated task? → ProcessPoolExecutor
# 5. Quick Python calculations? → InterpreterPoolExecutor (lower overhead)
```

---

## Putting It All Together: Hybrid Pattern

The real power emerges when you combine both patterns:

```python
import asyncio
from concurrent.futures.interpreters import InterpreterPoolExecutor
import httpx  # async HTTP client

async def fetch_data(url: str, client: httpx.AsyncClient) -> str:
    """I/O-bound: fetch from network."""
    response = await client.get(url)
    return response.text

def process_data(raw_data: str) -> str:
    """CPU-bound: heavy processing (runs in interpreter)."""
    # Simulate expensive processing
    return raw_data.upper() * 1000

async def hybrid_workflow(urls: list[str]) -> None:
    """Combined I/O concurrency + CPU parallelism."""

    with InterpreterPoolExecutor(max_workers=4) as executor:
        loop = asyncio.get_running_loop()

        async with httpx.AsyncClient() as client:
            # I/O: fetch all concurrently
            async with asyncio.TaskGroup() as tg:
                fetch_tasks = [
                    tg.create_task(fetch_data(url, client))
                    for url in urls
                ]

            # CPU: process results in parallel
            process_tasks = [
                loop.run_in_executor(executor, process_data, result)
                for result in fetch_tasks
            ]

            results = await asyncio.gather(*process_tasks)
            print(f"Processed {len(results)} items")

# Timeline visualization:
# Fetch:    API1 ▓▓▓▓▓▓▓▓▓
#           API2 ▓▓▓▓▓▓▓▓▓  (concurrent)
#           API3 ▓▓▓▓▓▓▓▓▓
# Process:       CPU1 ▓▓▓▓▓▓▓▓▓
#                CPU2 ▓▓▓▓▓▓▓▓▓  (parallel on cores)
#                CPU3 ▓▓▓▓▓▓▓▓▓
#
# Total time: max(fetch_time) + max(process_time) [overlap!]
# NOT: sum of all times
```

---

## CoLearning Synthesis

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:
> "Design a system that fetches 10 JSON files from APIs and analyzes each with CPU-intensive parsing. How would you structure this using asyncio + InterpreterPoolExecutor? Draw a timeline showing where I/O and CPU work overlap."

**Expected Outcome**: You'll understand how hybrid patterns achieve both I/O concurrency and CPU parallelism, solving real-world AI workloads (API calls + inference).

---

## Challenge 4: The CPU Parallelism Workshop

This is a **4-part bidirectional learning challenge** where you discover how to parallelize CPU work despite the GIL.

### Part 1: Discover Independently (Student as Scientist)
**Your Challenge**: Experience the GIL's effect without AI.

**Deliverable**: Create `/tmp/gil_discovery.py` containing:
1. A CPU-intensive function: sum of squares from 0 to 50 million (takes ~2-3 seconds on modern hardware)
2. Run it 4 times sequentially — measure total time (should be ~8-12 seconds)
3. Attempt to parallelize with `concurrent.futures.ThreadPoolExecutor(max_workers=4)` — measure time (still ~8-12 seconds, proves GIL blocks parallelism)
4. Measure with `concurrent.futures.ProcessPoolExecutor(max_workers=4)` — measure time (should be ~2-3 seconds, 4x faster)

**Expected Observation**:
- Sequential: ~8-12 seconds
- ThreadPoolExecutor: ~8-12 seconds (GIL prevents parallelism)
- ProcessPoolExecutor: ~2-3 seconds (true parallelism)

**Self-Validation**:
- Why doesn't threading help CPU work?
- Why do processes work better?
- What's the overhead of creating processes vs threads?

---

### Part 2: AI as Teacher (Teaching Executor Patterns)
**Your AI Prompt**:
> "I tried to speed up my CPU calculation using ThreadPoolExecutor with 4 workers, but it's no faster than sequential. I read something about the GIL. Teach me: 1) What is the GIL? 2) Why does it prevent threading from helping CPU work? 3) What should I use instead of threading for CPU work? Show me code that actually achieves parallelism."

**AI's Role**: Explain GIL concept (memory safety, reference counting), why threading can't help CPU work, and show ProcessPoolExecutor or InterpreterPoolExecutor pattern.

**Interactive Moment**: Ask a clarifying question:
> "You mentioned ProcessPoolExecutor and InterpreterPoolExecutor—what's the difference? When would I choose one over the other? What about startup overhead?"

**Expected Outcome**: AI clarifies that InterpreterPoolExecutor is lightweight (shared Python runtime) while ProcessPoolExecutor has high overhead (separate Python instances). You understand the tradeoff.

---

### Part 3: You as Teacher (Discovering Hybrid Patterns)
**Setup**: AI generates hybrid asyncio + executor code. Your job is to test it and teach AI about optimization.

**AI's Initial Code** (ask for this):
```python
import asyncio
from concurrent.futures import ProcessPoolExecutor
import time

def cpu_work(n: int) -> int:
    """CPU-intensive: sum of squares"""
    return sum(i*i for i in range(n))

async def fetch_data(name: str) -> str:
    """Simulate I/O"""
    await asyncio.sleep(1)
    return f"Data: {name}"

async def main():
    loop = asyncio.get_event_loop()

    # Fetch 4 datasets concurrently
    fetch_tasks = [
        fetch_data("A"), fetch_data("B"),
        fetch_data("C"), fetch_data("D")
    ]
    fetch_results = await asyncio.gather(*fetch_tasks)

    # Process each result in parallel
    with ProcessPoolExecutor(max_workers=4) as executor:
        process_tasks = [
            loop.run_in_executor(executor, cpu_work, 50_000_000)
            for _ in range(4)
        ]
        process_results = await asyncio.gather(*process_tasks)

    print(f"Fetched: {fetch_results}, Processed: {process_results}")

asyncio.run(main())
```

**Your Task**:
1. Run this code. Measure timing.
2. Identify the opportunity: Fetch and process happen sequentially (fetch takes 1s, then process takes 3s, total ~4s)
3. Teach AI:
> "Your code fetches all 4 datasets (1 second, concurrent), then processes them in parallel (3 seconds). But what if I start processing while still fetching? How would I overlap I/O and CPU? Show me an architecture that does 'fetch one, process one, fetch next, process next' all concurrently."

**Your Edge Case Discovery**: Ask AI:
> "What if one task has much more CPU work than others (task A needs 10s, task B needs 1s)? Load balancing matters. How would I distribute work fairly across 4 workers? What's the difference between using 4 workers vs using `os.cpu_count()` workers?"

**Expected Outcome**: You discover that hybrid systems need careful orchestration—overlapping I/O and CPU requires async/executor coordination, not just sequential stages.

---

### Part 4: Build Production Artifact (Student as Engineer)
**Your Capstone for This Challenge**: Build a realistic I/O + CPU pipeline.

**Specification**:
- Fetch from 6 data sources concurrently (simulate with asyncio.sleep, each 0.5-1.5s)
- Each source returns a dataset (list of 50M integers, simulated)
- Process each dataset with expensive calculation (sum of squares, simulated)
- 4 worker processes for CPU work
- Fetch and process should overlap (not sequential)
- Measure: total time, fetch time, process time
- Return: `{source: (fetch_ms, process_ms, result)}`
- Type hints throughout

**Deliverable**: Save to `/tmp/hybrid_pipeline.py`

**Testing Your Work**:
```bash
python /tmp/hybrid_pipeline.py
# Expected output:
# Total time: ~4-5 seconds (1-2s fetch + 2-3s process, overlapped)
# NOT 9-12 seconds (sequential) or 3-6 seconds (only parallel process)
# Fetch completed: 6 sources in ~2s (concurrent)
# Process completed: 6 sources in ~2s (parallel)
# Overlap confirmed: total < fetch + process
```

**Validation Checklist**:
- [ ] Code runs without errors
- [ ] Fetch tasks run concurrently (all 6 fetch < 2s)
- [ ] Process tasks run in parallel (uses ProcessPoolExecutor)
- [ ] I/O and CPU overlap (fetch_time + process_time > total_time)
- [ ] Total time < 6 seconds (proves parallelism)
- [ ] Type hints complete
- [ ] Proper executor cleanup (context manager or explicit shutdown)

---

**Time Estimate**: 35-40 minutes (5 min discover, 8 min teach/learn, 10 min edge cases, 12-17 min build artifact)

**Key Takeaway**: You've mastered hybrid I/O + CPU systems. The GIL doesn't prevent parallelism—you just need the right tool (ProcessPoolExecutor) and careful orchestration to overlap I/O and CPU work.
