---
title: "Lesson 5: Choosing Your Concurrency Approach"
chapter: 29
lesson: 5
duration_minutes: 90

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Concurrency Decision Framework"
    proficiency_level: "B1-B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving & Computational Thinking"
    measurable_at_this_level: "Student can classify workloads as CPU vs I/O bound, apply decision framework to choose appropriate approach, and justify selection"

  - name: "Workload Classification (CPU vs I/O)"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving & Computational Thinking"
    measurable_at_this_level: "Student can identify whether a workload is CPU-bound (compute-intensive), I/O-bound (network/file dependent), or mixed"

  - name: "Single-Threaded Optimization"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving & Computational Thinking"
    measurable_at_this_level: "Student understands that no concurrency is often the fastest approach and recognizes when overhead elimination matters"

  - name: "Free-Threaded Python Application"
    proficiency_level: "B1-B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving & Computational Thinking"
    measurable_at_this_level: "Student knows when free-threaded Python is appropriate and can build CPU-bound parallel systems with it"

  - name: "Multiprocessing for Isolation"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving & Computational Thinking"
    measurable_at_this_level: "Student understands process isolation, when it's necessary, and Python 3.14's improvements (forkserver, Process.interrupt)"

  - name: "Asyncio for I/O Concurrency"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving & Computational Thinking"
    measurable_at_this_level: "Student recognizes asyncio's strengths for I/O-bound work and connects Python 3.14's CLI debugging tools"

  - name: "Benchmarking Methodology"
    proficiency_level: "B1-B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving & Computational Thinking"
    measurable_at_this_level: "Student can design and execute benchmarks measuring execution time, CPU usage, and memory; interpret results critically"

  - name: "Hybrid Concurrency Patterns"
    proficiency_level: "B1-B2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving & Computational Thinking"
    measurable_at_this_level: "Student understands combining free-threaded + asyncio for systems with both CPU and I/O components"

  - name: "Python 3.14 Async/Multiprocessing Improvements"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Remember"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student recalls asyncio CLI debugging (ps, pstree), thread-safe event loop, forkserver default, Process.interrupt"

learning_objectives:
  - objective: "Classify workloads as CPU-bound, I/O-bound, or mixed and apply the decision framework to choose appropriate concurrency approach"
    proficiency_level: "B1-B2"
    bloom_level: "Apply"
    assessment_method: "Student correctly chooses approach for 5+ diverse scenarios; justifies reasoning"

  - objective: "Explain when single-threaded execution is fastest and when concurrency overhead is justified"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Student articulates why baseline matters and when overhead elimination is critical"

  - objective: "Design and execute benchmarks comparing multiple concurrency approaches"
    proficiency_level: "B1-B2"
    bloom_level: "Apply"
    assessment_method: "Student implements working benchmark measuring time, CPU, memory; interprets results; explains which approach is optimal"

  - objective: "Apply Python 3.14 improvements (asyncio CLI, multiprocessing enhancements) to debug and optimize concurrent systems"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student uses asyncio ps/pstree to inspect running tasks; leverages forkserver for safer multiprocessing"

  - objective: "Design hybrid concurrency systems combining free-threaded + asyncio for production workloads"
    proficiency_level: "B1-B2"
    bloom_level: "Understand"
    assessment_method: "Student describes architecture for AI system with both CPU reasoning and I/O operations"

cognitive_load:
  new_concepts: 9
  assessment: "9 concepts within B1-B2 limit of 10: (1) workload classification, (2) decision framework, (3) single-threaded baseline, (4) free-threaded suitability, (5) multiprocessing isolation, (6) asyncio I/O patterns, (7) benchmarking methodology, (8) Python 3.14 tools/improvements, (9) hybrid patterns. JUSTIFIED: This is synthesis lesson—applying prior knowledge to make production decisions."

differentiation:
  extension_for_advanced: "For B2 students: Implement complex hybrid systems (free-threaded agents + asyncio API layer + multiprocessing data pipeline); profile memory and CPU in detail; analyze scaling characteristics as core count increases"
  remedial_for_struggling: "For struggling students: Focus on decision framework table (concept 2) as primary anchor; work through 3-4 simple scenarios with explicit framework application; skip hybrid patterns until basic framework is solid"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/part-4-chapter-29/spec.md (Lesson 5)"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Lesson 5: Choosing Your Concurrency Approach

You've now learned about CPython's architecture (Lesson 1), its performance evolution (Lesson 2), the traditional GIL (Lesson 3), and free-threaded Python (Lesson 4). You understand what changed and why it matters. But there's a critical gap in your knowledge: **how do you actually choose which approach to use?**

This lesson teaches the most important skill in concurrent programming: making the right decision. You'll learn a decision framework that accounts for workload type, scale requirements, and isolation needs. You'll discover that no concurrency is often faster than bad concurrency, and you'll build benchmarking skills to validate your choices.

By the end of this lesson, you'll be able to look at a problem and confidently decide: "This needs single-threaded optimization," or "This needs free-threaded Python," or "This needs multiprocessing," and prove it with benchmarks.

---

## Section 1: The Decision Framework (The Heart of This Lesson)

The most important table in concurrent programming is simple: **workload type matters more than any other factor.**

| **Workload Type** | **Nature** | **Best Approach** | **Why** | **Python 3.14 Notes** |
|---|---|---|---|---|
| **CPU-Bound (Sequential)** | Compute-heavy, single task | **Single-threaded** | No concurrency overhead; fastest | Still the baseline—measure it first |
| **CPU-Bound (Parallel, Shared State)** | Compute-heavy, parallel, needs shared memory | **Free-Threaded Python** | True parallelism without process overhead; 5-10% single-threaded cost for 2-4x parallel gain | *New in 3.14* — this is revolutionary |
| **CPU-Bound (Parallel, Isolated)** | Compute-heavy, parallel, processes are independent | **Multiprocessing** | Process isolation when needed; `forkserver` default safer than `fork` | *Improved in 3.14* — safer defaults |
| **I/O-Bound (Low Concurrency)** | Network/file I/O, event-driven, 10-100 concurrent tasks | **Asyncio** | Single-threaded event loop; no context switching overhead | *Improved in 3.14* — CLI debugging (ps, pstree) |
| **Hybrid (CPU + I/O)** | Both CPU reasoning and I/O operations | **Free-Threaded + Asyncio** | Threads for CPU work; asyncio for I/O within/across threads | *New in 3.14* — official support for combination |

**Read this table carefully.** This single framework drives 80% of concurrency decisions. Everything in this lesson builds from it.

#### 💬 AI Colearning Prompt

Let's test your understanding:

> "Walk me through 10 real-world scenarios. For each, classify as CPU-bound (sequential/parallel) or I/O-bound. Then I'll guess which approach you'd choose. Then you ask me to justify my choice. Here are the scenarios: (1) PDF text extraction for 1000 documents, (2) web scraper downloading 100 pages, (3) local file compression with zlib, (4) REST API handling 50 concurrent requests, (5) matrix multiplication for ML training, (6) database query processing 1M rows, (7) AI agent reasoning task, (8) real-time stock price updates from APIs, (9) batch image resizing, (10) multi-agent negotiation. For each, tell me: Is this CPU or I/O? What's the best approach?"

**Expected outcome**: You'll develop intuition for the decision framework. You'll see patterns: anything involving network/file I/O is I/O-bound; anything computing without waiting is CPU-bound.

---

## Section 2: Single-Threaded Execution (The Baseline You Must Measure)

Here's a truth that surprises many developers: **no concurrency is often the fastest option.**

Why? Concurrency adds overhead:
- Creating threads/processes (memory, startup time)
- Synchronization (locks, context switches)
- IPC (inter-process communication, serialization)

For small workloads or sequential operations, this overhead exceeds any benefit.

### When Single-Threaded is Best

1. **Sequential Tasks**: Processing items one after another where order matters
2. **Single Unit of Work**: One large task that can't be parallelized
3. **Small Workloads**: Where total runtime is milliseconds (overhead dominates)
4. **Memory Constrained**: Creating extra processes/threads isn't viable

### The Baseline Rule

**ALWAYS measure single-threaded performance first.** This is your baseline. Any concurrency approach must beat this baseline, accounting for overhead.

```python
import time

def cpu_task(n: int) -> int:
    """CPU-bound task: calculate sum of square roots."""
    total = 0
    for i in range(n):
        total += (i ** 2) ** 0.5
    return total

def benchmark_single_threaded(n: int) -> float:
    """Single-threaded baseline."""
    start = time.perf_counter()
    result = cpu_task(n)
    elapsed = time.perf_counter() - start
    return elapsed

# Measure the baseline
if __name__ == "__main__":
    baseline = benchmark_single_threaded(15**6)
    print(f"Single-threaded baseline: {baseline:.3f}s")
    # Later: any parallel approach must beat this to justify overhead
```

#### 🎓 Expert Insight

In AI-native development, you don't guess at concurrency. You **measure the baseline first**. Syntax and frameworks are cheap—understanding when overhead is justified is gold. Many developers add threading/multiprocessing and wonder why it's slower. The reason: they didn't measure the baseline.

---

## Section 3: Free-Threaded Python (True Parallelism on Shared State)

**When to use**: CPU-bound work that needs parallelism WITH shared state

Free-threaded Python (introduced in Lesson 4) removes the GIL, enabling true parallel execution of Python threads. This is revolutionary because:
- No process isolation overhead
- No IPC/serialization
- Shared memory means data sharing is direct
- 5-10% single-threaded overhead for 2-4x parallel gains on 4 cores

### Real Example: AI Multi-Agent System

Imagine a system where 4 AI agents collaborate on a problem, sharing reasoning state:

```python
import threading
import time
from typing import Any

class Agent:
    """AI agent doing CPU-bound reasoning."""

    def __init__(self, name: str, iterations: int = 15**6):
        self.name = name
        self.iterations = iterations
        self.result = None

    def reason(self) -> None:
        """CPU-intensive reasoning task."""
        total = 0
        for i in range(self.iterations):
            total += (i ** 2) ** 0.5
        self.result = total

def benchmark_free_threaded(num_agents: int = 4) -> float:
    """Benchmark: 4 agents reasoning in parallel."""
    agents = [Agent(f"Agent-{i}") for i in range(num_agents)]
    threads = [threading.Thread(target=agent.reason) for agent in agents]

    start = time.perf_counter()
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    elapsed = time.perf_counter() - start

    return elapsed

# With free-threaded Python 3.14:
# Expected: ~3-4x speedup on 4-core machine (not the 4x you'd get without overhead)
if __name__ == "__main__":
    free_threaded_time = benchmark_free_threaded(4)
    print(f"Free-threaded (4 agents): {free_threaded_time:.3f}s")
```

**The gain**: On a 4-core machine with free-threaded Python, this runs in ~1/3 to 1/4 the time of single-threaded. The 5-10% overhead is more than compensated by true parallelism.

#### 🚀 CoLearning Challenge (25 min mark)

Ask your AI Companion:

> "Compare free-threaded Python to traditional threading with the GIL. Create a benchmark that shows: (1) traditional GIL threading stays around baseline time (no speedup), (2) free-threaded achieves 2-4x speedup. Explain why the GIL prevented the speedup and free-threading enables it."

**Expected outcome**: You'll understand viscerally why free-threading is revolutionary. You'll see that traditional threading doesn't actually parallelize CPU work.

---

## Section 4: Multiprocessing (True Parallelism with Isolation)

**When to use**: CPU-bound work needing true parallelism AND process isolation

Multiprocessing creates separate Python processes, each with its own GIL. This:
- Achieves true parallelism (not blocked by GIL)
- Isolates processes (one crash doesn't kill all)
- Adds overhead (process creation, IPC, serialization)

Python 3.14 improves multiprocessing with:
- **forkserver default**: Safer than `fork` (avoids deadlocks from threading states)
- **Process.interrupt()**: Graceful shutdown of worker processes
- **Context inheritance**: Better control over initialization

### Real Example: Distributed Processing

```python
import multiprocessing
import time

def cpu_task(n: int) -> int:
    """Same CPU-bound task."""
    total = 0
    for i in range(n):
        total += (i ** 2) ** 0.5
    return total

def benchmark_multiprocessing(n: int, num_processes: int = 4) -> float:
    """Benchmark: 4 processes running CPU task in parallel."""
    start = time.perf_counter()

    with multiprocessing.Pool(num_processes) as pool:
        # Each process gets a chunk of work
        tasks = [n for _ in range(num_processes)]
        results = pool.map(cpu_task, tasks)

    elapsed = time.perf_counter() - start
    return elapsed

if __name__ == "__main__":
    mp_time = benchmark_multiprocessing(15**6, 4)
    print(f"Multiprocessing (4 processes): {mp_time:.3f}s")
    # Note: Process creation overhead makes this slower for small workloads
    # But scales well for large, independent CPU tasks
```

#### ✨ Teaching Tip (35 min mark)

Use Claude Code to explore multiprocessing edge cases:

> "Show me what happens if a multiprocessing worker crashes. How does the Pool handle it? What if I want to interrupt all workers gracefully? Show me Python 3.14's Process.interrupt() in action."

This teaches you the exception handling and lifecycle management that matters in production.

---

## Section 5: Asyncio (I/O-Bound Concurrency Without Threads)

**When to use**: I/O-bound work with event-driven architecture (10-1000 concurrent tasks)

Asyncio uses a single thread with an event loop. When one task waits for I/O (network, file), the loop switches to another task. This:
- Handles 100s of concurrent connections
- No thread overhead (single thread)
- Scales to high concurrency
- Limited by CPU (you need threads/processes for CPU work alongside I/O)

This connects directly to Chapter 28 (Asyncio fundamentals). Lesson 5 adds Python 3.14 improvements.

### Python 3.14 Asyncio Improvements: CLI Debugging

Python 3.14 adds production-grade debugging tools for asyncio:

```python
# Save as async_example.py
import asyncio

async def agent_task(name: str, duration: int) -> None:
    """Simulate an AI agent processing."""
    print(f"{name} starting...")
    await asyncio.sleep(duration)
    print(f"{name} finished")

async def main() -> None:
    """Run 3 agents concurrently."""
    tasks = [
        asyncio.create_task(agent_task("Agent-1", 5)),
        asyncio.create_task(agent_task("Agent-2", 3)),
        asyncio.create_task(agent_task("Agent-3", 7))
    ]
    await asyncio.gather(*tasks)

if __name__ == "__main__":
    asyncio.run(main())
```

Run this, then inspect with Python 3.14's asyncio CLI tools:

```bash
# In another terminal, get the process ID
ps aux | grep async_example.py

# Show all running tasks (flat list)
python -m asyncio ps <PID>

# Show task hierarchy (tree structure)
python -m asyncio pstree <PID>
```

These commands show you:
- What tasks are currently running
- How long each has been running
- Stack traces for debugging deadlocks

#### 💬 AI Colearning Prompt

> "I'm debugging an asyncio program that seems stuck. One task finished, but two others never completed. How do I use `python -m asyncio ps` and `pstree` to diagnose what's happening? Show me the commands and explain what the output tells me."

**Expected outcome**: You'll learn to diagnose asyncio deadlocks and hangs—critical production skills.

---

## Section 6: Python 3.14 Asyncio Improvements (Deep Dive)

Python 3.14 makes asyncio production-ready with three major additions:

### 1. Thread-Safe Event Loop

Previously, calling asyncio from threads was error-prone. Python 3.14 guarantees thread-safe event loop access:

```python
import asyncio
import threading

async def async_task() -> str:
    await asyncio.sleep(0.1)
    return "Result from async"

def call_async_from_thread(loop: asyncio.AbstractEventLoop) -> None:
    """Call async code from a regular thread—now thread-safe in 3.14."""
    future = asyncio.run_coroutine_threadsafe(async_task(), loop)
    result = future.result()
    print(f"Async result: {result}")

if __name__ == "__main__":
    loop = asyncio.new_event_loop()
    thread = threading.Thread(target=call_async_from_thread, args=(loop,))
    thread.start()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(async_task())
    thread.join()
```

### 2. Faster Event Loop Performance

Python 3.14 improves event loop speed by ~10-20% through optimizations in task switching. This compounds for systems handling 100s of concurrent connections.

### 3. Better Error Reporting

Error messages now clearly indicate which task failed and why, making debugging significantly faster.

#### 🎓 Expert Insight

Production systems live or die on debuggability. Python 3.14's asyncio CLI tools transform asyncio from "black box" to "transparent." You can now see exactly what's running, what's blocked, and why.

---

## Section 7: Python 3.14 Multiprocessing Improvements

Python 3.14 improves multiprocessing with safer defaults and better control:

### 1. forkserver Default (Safer Initialization)

Multiprocessing on Unix/Linux has two creation methods:
- **fork**: Copies entire parent process (fast but unsafe if parent uses threads)
- **spawn**: Creates fresh child process (slow but safe)
- **forkserver** (NEW DEFAULT): Starts server process, forks from clean state (fast AND safe)

```python
import multiprocessing

# Python 3.14 uses forkserver by default on Unix/Linux
# This is equivalent to explicit configuration in earlier versions:

if __name__ == "__main__":
    # Explicitly set method (useful for clarity)
    ctx = multiprocessing.get_context('forkserver')
    process = ctx.Process(target=some_function)
    process.start()
    process.join()
```

### 2. Process.interrupt() for Graceful Shutdown

Previously, you had to use signals or `terminate()` (forceful). Python 3.14 adds interrupt:

```python
import multiprocessing
import signal

def worker() -> None:
    try:
        for i in range(100):
            print(f"Working... {i}")
            time.sleep(1)
    except KeyboardInterrupt:
        print("Worker interrupted gracefully")

if __name__ == "__main__":
    p = multiprocessing.Process(target=worker)
    p.start()

    time.sleep(5)

    # Graceful interrupt (sends signal, waits for cleanup)
    p.interrupt()  # NEW in Python 3.14
    p.join(timeout=2)

    if p.is_alive():
        p.terminate()  # Only forceful kill if necessary
```

#### 🚀 CoLearning Challenge (65 min mark)

Tell your AI Companion:

> "Implement all 5 benchmarks (single-threaded, free-threaded, multiprocessing, asyncio, and hybrid free-threaded+asyncio) for a CPU task. Compare execution times on your machine. Which is fastest? Why? How does overhead change with workload size?"

**Expected outcome**: You'll build the benchmarking muscles needed to make production decisions. You'll see that overhead patterns vary with workload size.

---

## Section 8: Benchmarking Methodology (Measure Before and After)

You can't choose the right approach without data. Here's the rigorous methodology:

### Step 1: Define Your Task

```python
import time

def benchmark_task(n: int) -> int:
    """CPU-bound task: sum of square roots."""
    total = 0
    for i in range(n):
        total += (i ** 2) ** 0.5
    return total
```

### Step 2: Measure Baseline (Single-Threaded)

```python
def measure_baseline(n: int, runs: int = 3) -> float:
    """Run multiple times; return average."""
    times = []
    for _ in range(runs):
        start = time.perf_counter()
        benchmark_task(n)
        elapsed = time.perf_counter() - start
        times.append(elapsed)

    avg = sum(times) / len(times)
    min_time = min(times)
    max_time = max(times)

    print(f"Baseline: avg={avg:.3f}s, min={min_time:.3f}s, max={max_time:.3f}s")
    return avg
```

### Step 3: Measure Each Approach

```python
def measure_approach(approach_name: str, func, n: int, runs: int = 3) -> tuple[float, float]:
    """Measure any approach and compare to baseline."""
    times = []
    for _ in range(runs):
        start = time.perf_counter()
        func(n)
        elapsed = time.perf_counter() - start
        times.append(elapsed)

    avg = sum(times) / len(times)
    speedup = baseline / avg  # How much faster than baseline?

    print(f"{approach_name}: avg={avg:.3f}s, speedup={speedup:.2f}x")
    return avg, speedup
```

### Step 4: Interpret Results

| **Speedup** | **Interpretation** | **Decision** |
|---|---|---|
| **< 1.0x** | Slower than baseline | Don't use this approach |
| **1.0-1.2x** | Overhead not justified | Stick with simpler approach |
| **1.2-2.0x** | Worth considering; check overhead for your scale | Use if workload is large |
| **> 2.0x** | Clear winner; go with this | Use in production |

#### 💬 AI Colearning Prompt

> "Ask your AI: What factors besides execution time matter when choosing a concurrency approach? Consider: memory usage (how many processes/threads can I create?), complexity (how hard to debug?), latency (is 95th percentile important?), and maintainability (can my team understand this code?)."

**Expected outcome**: You'll see that the decision framework isn't just about speed—it's about holistic trade-offs.

---

## Section 9: Real-World AI Workload Patterns

Let's connect this to AI systems (the book's focus):

### Pattern 1: AI Inference (CPU-Bound, Parallel)
- **Nature**: Running inference on multiple inputs (batch processing)
- **Best Approach**: Free-threaded Python
- **Why**: CPU-intensive, needs parallelism, agents share model state
- **Example**: 4 agents analyzing documents in parallel

### Pattern 2: Web API Backend (I/O-Bound, Event-Driven)
- **Nature**: Handling 50-500 concurrent API requests
- **Best Approach**: Asyncio
- **Why**: Network I/O dominates; event loop handles concurrency elegantly
- **Example**: REST API serving AI inference requests

### Pattern 3: Data Pipeline (CPU + I/O Mixed)
- **Nature**: Fetch data (I/O), process (CPU), store (I/O)
- **Best Approach**: Free-threaded + Asyncio Hybrid
- **Why**: Threads handle CPU; asyncio handles I/O concurrency
- **Example**: Ingest data from APIs, process with agents, save results

### Pattern 4: Distributed AI (Multi-Machine)
- **Nature**: Train/infer across multiple servers
- **Best Approach**: Multiprocessing (local) + Kubernetes (distributed)
- **Why**: Process isolation; distributed orchestration
- **Example**: Part 13 content (Dapr actors)

#### ✨ Teaching Tip (72 min mark)

Ask Claude Code:

> "Design a production AI system: 10 agents (5 do NLP inference, 5 fetch data from APIs). Which concurrency approach for each group? How do they interact? Draw an architecture diagram."

This teaches you to think in systems, not isolated components.

---

## Section 10: Hybrid Approaches (Free-Threaded + Asyncio)

For complex systems with both CPU and I/O work, combine free-threading and asyncio:

```python
import asyncio
import threading
from concurrent.futures import ThreadPoolExecutor

async def io_task(url: str) -> str:
    """Simulate I/O-bound work (network fetch)."""
    # In real code, use aiohttp; here we simulate
    await asyncio.sleep(1)  # Represents network latency
    return f"Data from {url}"

def cpu_task(data: str) -> str:
    """CPU-bound work in thread pool."""
    # Simulate CPU-intensive processing
    result = 0
    for i in range(10**6):
        result += (i ** 2) ** 0.5
    return f"Processed: {data}"

async def hybrid_workflow() -> None:
    """Combine I/O (asyncio) and CPU (threads) work."""
    # Create thread pool for CPU work
    with ThreadPoolExecutor(max_workers=4) as executor:
        # Fetch data concurrently (asyncio)
        urls = ["url1", "url2", "url3", "url4"]
        fetch_tasks = [io_task(url) for url in urls]
        data_results = await asyncio.gather(*fetch_tasks)

        # Process each in thread pool (CPU work)
        loop = asyncio.get_event_loop()
        process_tasks = [
            loop.run_in_executor(executor, cpu_task, data)
            for data in data_results
        ]
        final_results = await asyncio.gather(*process_tasks)

        for result in final_results:
            print(result)

if __name__ == "__main__":
    asyncio.run(hybrid_workflow())
```

This pattern scales to production:
- Asyncio handles 100s of concurrent I/O operations
- Threads handle CPU work without blocking I/O
- Free-threading (Python 3.14) ensures threads actually parallelize

---

## Section 11: Decision Tree Summary

Use this decision tree when facing a new problem:

1. **Is the workload I/O-bound?** (network, file, database)
   - YES → **Asyncio** (unless you need CPU parallelism too)
   - NO → Continue to step 2

2. **Is the workload CPU-bound and needs parallelism?**
   - YES → Continue to step 3
   - NO → **Single-threaded** (no overhead needed)

3. **Do the parallel workers need to share memory/state?**
   - YES → **Free-Threaded Python** (Python 3.14+)
   - NO → Continue to step 4

4. **Do processes need isolation or are they independent?**
   - Isolation needed → **Multiprocessing** (with forkserver)
   - Independent → **Multiprocessing** (any method)

5. **Do you have both CPU and I/O work?**
   - YES → **Free-Threaded + Asyncio Hybrid**
   - NO → You've already chosen in steps 1-4

#### 🎓 Expert Insight

Memorizing this tree won't help you. Understanding the WHY behind each decision makes you production-ready. Every choice reflects trade-offs: overhead vs speedup, isolation vs sharing, simplicity vs performance.

---

## Challenge 5: The Concurrency Strategy Selection Workshop

This is a **4-part bidirectional learning challenge** where you master decision-making using the concurrency selection framework.

### Part 1: Discover Independently (Student as Scientist)
**Your Challenge**: Classify real-world workloads using the decision framework.

**Deliverable**: Create `/tmp/concurrency_classification.py` containing:
1. Classify 8 real AI workloads:
   - API server (web requests)
   - ML model inference (CPU-bound)
   - Data pipeline (fetch + process + store)
   - Chat bot (I/O for API calls, CPU for LLM)
   - Web scraper (many HTTP requests + text parsing)
   - Vector database (I/O-bound queries)
   - Report generator (CPU-heavy calculations)
   - Real-time monitoring (continuous I/O + light processing)
2. For each: decide single-threaded, free-threaded, multiprocessing, asyncio, or hybrid
3. Document reasoning for each

**Expected Observation**:
- I/O-dominant workloads map to asyncio
- CPU-dominant workloads map to free-threaded or multiprocessing
- Hybrid workloads need combined approaches
- Single-threaded is rare but appropriate for sequential-only tasks

**Self-Validation**:
- Can you explain your classification choices?
- Would your choices change with different Python versions?
- Where would you use each approach in a real system?

---

### Part 2: AI as Teacher (Teaching Decision Framework)
**Your AI Prompt**:
> "I classified 8 workloads using the concurrency decision framework. Teach me: 1) Are my classifications correct? 2) For each workload type, what are the performance implications of wrong choices? 3) How does Python 3.14 free-threading change the classification (some that needed multiprocessing before might not now)? 4) When would I deliberately choose a suboptimal approach and why?"

**AI's Role**: Validate classifications, explain consequences, discuss tradeoffs, clarify when "wrong" choice is actually pragmatic.

**Interactive Moment**: Ask a clarifying question:
> "You said 'choice depends on SLA requirements.' Can you show me an example where the same workload would use different concurrency approaches based on latency vs throughput requirements?"

**Expected Outcome**: AI clarifies that decision-making is contextual, based on constraints (latency, memory, availability, team expertise). You learn systems thinking.

---

### Part 3: You as Teacher (Discovering Edge Cases)
**Setup**: AI generates a benchmark comparing all approaches on a hybrid workload. Your job is to verify and teach AI about real-world constraints.

**AI's Initial Code** (ask for this):
> "Create a comprehensive benchmark: web scraper that downloads 100 pages (I/O-bound, 1s each) and extracts text (CPU-bound, 0.5s each). Compare: (a) single-threaded sequential, (b) asyncio for fetch + sync processing, (c) asyncio for fetch + free-threaded processing, (d) asyncio for fetch + ProcessPoolExecutor processing. Show time and resource usage for each."

**Your Task**:
1. Run all approaches. Measure timing and memory usage
2. Identify surprises (does theory match practice?)
3. Teach AI:
> "Theory says asyncio + free-threaded should be fastest, but memory usage is higher. ProcessPoolExecutor is slower but uses 50% less memory. How do I decide in production where memory budget is tight?"

**Your Edge Case Discovery**: Ask AI:
> "What if the hybrid workload is unbalanced: 1000 pages to fetch (5 seconds total) but only 50MB of text to process (1 second total)? Fetch dominates. How does this imbalance affect which approach is best?"

**Expected Outcome**: You discover that benchmarks reveal real tradeoffs. You learn to choose based on actual constraints, not theory alone.

---

### Part 4: Build Production Artifact (Student as Engineer)
**Your Capstone for This Challenge**: Build an interactive concurrency strategy recommendation engine.

**Specification**:
- Accept workload characteristics: `{workload_type, latency_requirement_ms, memory_budget_mb, cpu_cores, has_expensive_startup}`
- Apply decision framework: return recommended approach + reasoning
- Show alternatives with tradeoffs
- Include benchmark template for validation
- Provide migration guide if changing approaches
- Type hints throughout

**Deliverable**: Save to `/tmp/concurrency_selector.py`

**Testing Your Work**:
```bash
python /tmp/concurrency_selector.py
# Interactive prompt:
# Enter workload type: hybrid
# Latency requirement (ms): 100
# Memory budget (mb): 256
# CPU cores: 4
# Has expensive startup (y/n): n
#
# === Recommendation ===
# Approach: Asyncio + Free-Threaded Python 3.14
# Reasoning: Hybrid workload with tight latency needs free-threading for CPU+concurrency overlap
# Expected speedup: 3.2x vs sequential
# Memory overhead: ~80MB per thread
# Migration effort: Medium (need thread-safety review)
#
# === Alternatives ===
# 1. ProcessPoolExecutor: Higher memory, no latency advantage
# 2. Pure asyncio: Can't handle CPU-bound part efficiently
```

**Validation Checklist**:
- [ ] Code runs without errors
- [ ] Accepts realistic workload characteristics
- [ ] Recommends appropriate concurrency approach
- [ ] Reasoning is clear and data-driven
- [ ] Alternatives section present
- [ ] Migration guidance helpful
- [ ] Type hints complete

---

**Time Estimate**: 32-38 minutes (6 min discover, 8 min teach/learn, 8 min edge cases, 10-16 min build artifact)

**Key Takeaway**: Concurrency decisions aren't about "best" approach—they're about "best for your constraints." Master the decision framework and benchmark on your actual workload to make confident, data-driven choices.
