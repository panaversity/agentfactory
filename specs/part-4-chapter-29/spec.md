# Chapter Specification: CPython and GIL

**Feature Branch**: `part-4-chapter-29`
**Created**: 2025-11-09
**Status**: Draft
**Chapter Number**: 29 (Part 4: Python Fundamentals)
**Complexity Tier**: Advanced (B1-B2)

---

## Success Evals *(FIRST - defined BEFORE other sections)*

These criteria define how we measure learning success for Chapter 29. All evals are aligned with business goals: producing developers who can build production AI systems.

### Comprehension Evals (Internal Understanding)

- **EVAL-001**: **CPython Internals Understanding** - 75%+ of students can explain (in writing, 150+ words) how CPython's interpreter executes Python code (bytecode compilation → execution → memory management), distinguishing it from other implementations (PyPy, Jython). *Measured via: Exercise submission requiring explanation without looking at materials*

- **EVAL-002**: **GIL Evolution Comprehension** - 80%+ of students can describe the progression from traditional GIL (pre-3.13) to free-threaded Python (3.14+), including: (a) what the GIL prevented, (b) why it's now optional, (c) performance characteristics (5-10% overhead vs 40%). *Measured via: Quiz with scenario-based questions*

- **EVAL-003**: **Concurrency Decision-Making** - 70%+ of students correctly identify the appropriate concurrency approach (traditional threading, multiprocessing, free-threaded, asyncio) for 5 different real-world scenarios (CPU-bound AI inference, I/O-bound web scraping, mixed workload, etc.). *Measured via: Decision matrix exercise with justification*

### Skill Acquisition Evals (Can DO)

- **EVAL-004**: **Free-Threading Detection** - 85%+ of students can write code (without reference) to: (a) detect if free-threading is available, (b) check if GIL is enabled/disabled at runtime, (c) verify Python build type. *Measured via: Timed coding exercise (5 minutes)*

- **EVAL-005**: **Benchmark Implementation** - 75%+ of students successfully implement a working benchmark comparing threading vs multiprocessing vs free-threaded Python for a CPU-bound task, correctly interpreting results. *Measured via: Capstone project evaluation (code runs, produces valid results, interpretation correct)*

- **EVAL-006**: **Multi-Agent System Building** - 60%+ of students build a functional multi-agent AI system demonstrating true parallelism with free-threaded Python (2+ agents reasoning concurrently on separate cores). *Measured via: Capstone project technical review (agents run in parallel, shared state managed safely, performance gain demonstrated)*

### Engagement & Accessibility Evals

- **EVAL-007**: **Reading Level Compliance** - Content maintains B1-B2 reading level (Flesch-Kincaid Grade 9-11, verified via automated tools). Complex technical terms are explained on first use. *Measured via: Automated readability analysis + spot-check review*

- **EVAL-008**: **Completion Rate** - 70%+ of students who start Chapter 29 complete all 6 lessons (tracked via lesson view analytics). *Measured via: Analytics dashboard*

- **EVAL-009**: **Exercise Submission** - 65%+ of students submit at least 3 out of 4 "Try With AI" exercise responses per lesson (indicating active engagement with AI co-learning pattern). *Measured via: Submission tracking system*

### Professional Application Evals (Production Readiness)

- **EVAL-010**: **AI-Native Context Understanding** - 70%+ of students can articulate (in writing, 100+ words) how free-threaded Python enables better multi-agent AI systems compared to traditional GIL-constrained Python, with specific reference to production deployment scenarios (Ray, Kubernetes). *Measured via: Short essay question in final assessment*

- **EVAL-011**: **Forward-Looking Preparation** - 75%+ of students correctly identify which Parts 10-13 deployment topics will benefit from free-threading knowledge (e.g., Ray actor systems, Dapr microservices, Kubernetes resource utilization). *Measured via: Multiple-choice assessment connecting Ch 29 to future chapters*

---

## Topic Summary

Chapter 29 teaches the CPython implementation and the revolutionary Global Interpreter Lock (GIL) evolution in Python 3.14. For 30 years, the GIL prevented true parallel execution of Python threads. In October 2025, Python 3.14 officially supported free-threaded builds (PEP 703/779), reducing overhead from 40% to 5-10% and enabling TRUE multi-core parallelism—the biggest change in Python's history.

Students learn three dimensions: (1) **CPython internals** (how the interpreter works, bytecode execution, memory management), (2) **Practical concurrency decisions** (when to use threading vs multiprocessing vs free-threaded Python vs asyncio), and (3) **AI-native applications** (building production multi-agent systems that leverage true parallelism). This knowledge is critical for Parts 10-13 where students deploy AI systems with Docker, Kubernetes, Ray, and Dapr—technologies that benefit dramatically from free-threading.

**Why this matters**: Multi-agent AI systems (the book's core focus) can now reason in parallel on separate CPU cores, not just pseudo-concurrently. A 4-agent system on a 4-core machine can achieve 2-4x performance gains with free-threaded Python. Understanding when and how to use this capability is essential for production AI development.

---

## Prerequisites

**Required Prior Knowledge** (Chapters 1-28):

**Foundation** (Chapters 1-11):
- Chapter 1-4: AI-Driven Development principles (AIDD workflow, validation-first)
- Chapter 5-6: Claude Code and Gemini CLI (AI co-learning partners)
- Chapter 9: Markdown (documentation, clarity)

**Python Fundamentals** (Chapters 12-28):
- Chapter 13: Python basics (syntax, REPL, execution)
- Chapter 14: Data types (int, float, str, type hints)
- Chapter 17: **Control Flow and Loops** (for loops, while loops, iteration - CRITICAL for benchmarking)
- Chapter 18: Lists, tuples, dict (data structures for shared state)
- Chapter 20: Functions and modules (code organization)
- Chapter 21: **Exception Handling** (try/except for thread safety - CRITICAL)
- Chapter 28: **Asyncio** (async/await, event loops, I/O concurrency - connects to free-threading)

**Assumed Technical Maturity**:
- Comfortable with command-line interfaces (Ch 7: Bash)
- Understands functions, loops, exception handling
- Can read and interpret Python code examples
- Has completed 27 prior chapters (advanced tier readiness)

---

## Learning Objectives

By the end of Chapter 29, students will be able to:

**LO-1: Understand CPython Architecture** (B1 proficiency)
- Explain what CPython is and how it differs from other Python implementations (PyPy, Jython, IronPython, MicroPython)
- Describe the Python execution pipeline: source code → bytecode → interpreter → execution
- Identify when CPython's design choices (reference counting, GC, C API) matter for performance

**LO-2: Master GIL Evolution and Free-Threading** (B1-B2 proficiency - PRIMARY FOCUS)
- Explain the traditional GIL behavior (why it exists, what it prevents, limitations)
- Describe Python 3.14's free-threaded mode (PEP 703/779): installation, detection, runtime control
- Compare performance characteristics: traditional GIL vs free-threaded (5-10% overhead vs 2-10x multi-threaded gains)
- Understand the three-phase GIL removal roadmap (3.13 experimental → 3.14 supported → 3.15+ default)

**LO-3: Make Informed Concurrency Decisions** (B1-B2 proficiency)
- Distinguish between CPU-bound and I/O-bound workloads (definition + real examples)
- Choose the correct approach for a given scenario:
  - Traditional threading (I/O-bound, shared state needed)
  - Multiprocessing (CPU-bound, isolation required)
  - Free-threaded Python (CPU-bound, shared state, true parallelism)
  - Asyncio (I/O-bound, event-driven, single thread)
- Design benchmarks to validate concurrency approach choices

**LO-4: Build AI-Native Systems with Free-Threading** (B2 proficiency - ADVANCED)
- Implement a multi-agent AI system demonstrating true parallel reasoning (2+ agents on separate cores)
- Build a benchmarking dashboard comparing concurrency approaches for AI workloads
- Connect free-threading knowledge to production deployment (preview of Ray, Kubernetes resource utilization)
- Use Python 3.14 asyncio CLI introspection tools (`python -m asyncio ps`, `pstree`) for debugging

**LO-5: Apply AI-Native Learning Pattern** (Meta-skill - B1-B2)
- Describe intent using type hints and clear code structure (not formal specifications - Part 4 appropriate)
- Explore concepts with AI (Claude Code/Gemini CLI as co-reasoning partners)
- Validate understanding through experiments and tests
- Learn from errors by asking AI diagnostic questions

---

## Content Outline

### Section 1: CPython Architecture & Implementation (Lessons 1-2)

**Lesson 1: What is CPython?** (B1 proficiency)
- CPython as reference implementation (written in C, official version)
- Python execution pipeline: .py → bytecode (.pyc) → CPython interpreter → execution
- Alternative implementations comparison (PyPy JIT, Jython JVM, IronPython .NET, MicroPython embedded - brief overview)
- How to detect CPython: `platform.python_implementation()`
- Memory management: reference counting + garbage collection
- **CoLearning Prompts**: 2-3 throughout lesson
- **Try With AI**: 4 prompts (Recall → Understand → Apply → Analyze)

**Lesson 2: CPython Performance Evolution (Python 3.14)** (B1 proficiency)
- Tail-call interpreter architecture (3-5% faster pyperformance benchmarks)
- Incremental garbage collection (order of magnitude faster pauses)
- Deferred annotation evaluation (PEP 649/749)
- Remote debugging interface (PEP 768) - brief mention
- Impact on AI workloads (faster execution, lower latency)
- **CoLearning Prompts**: 2-3 throughout lesson
- **Try With AI**: 4 prompts (progressive exploration)

### Section 2: The GIL Evolution (Lessons 3-4)

**Lesson 3: The Traditional GIL (Pre-3.13)** (B1-B2 proficiency)
- What the GIL is: mutex protecting CPython interpreter state
- Why it exists: simplifies C API, memory safety, thread safety
- What it prevents: true parallel execution of Python threads (even on multi-core)
- CPU-bound vs I/O-bound distinction (CRITICAL concept):
  - CPU-bound: Computation-heavy (prime calculations, data processing, AI inference)
  - I/O-bound: Waiting for external resources (network, files, database)
  - GIL releases during I/O (why threading works for I/O-bound)
- Traditional workarounds: multiprocessing (separate interpreters), C extensions release GIL
- **CoLearning Prompts**: 3-4 throughout lesson (deep exploration)
- **Try With AI**: 4 prompts (connect to real AI workloads)

**Lesson 4: Free-Threaded Python (3.14+ Production Ready)** (B1-B2 proficiency - PRIMARY LESSON)
- The GIL is now OPTIONAL (biggest Python change in 30 years)
- Three-phase roadmap:
  - Python 3.13: Experimental free-threaded builds (~40% overhead)
  - **Python 3.14: Officially supported** (~5-10% overhead) ← WE ARE HERE
  - Python 3.15+: Likely default (Phase III)
- PEP 703 (free-threading implementation) + PEP 779 (criteria for supported status)
- How free-threading works: per-thread state, lock-free data structures, thread-safe built-ins
- Installing free-threaded Python:
  - macOS/Windows: Official installers with free-threading option
  - Linux: Build with `./configure --disable-gil`
  - Docker: `python:3.14t` image tag (t = free-threaded)
- Detection code:
  ```python
  import sys
  print(f"Free-threading: {sys._is_gil_enabled() == False}")
  print(f"Build supports: {sys._is_gil_enabled() is not None}")
  ```
- Runtime control: `PYTHON_GIL=1` (enable), `PYTHON_GIL=0` (disable if supported)
- Performance characteristics:
  - Single-threaded: 5-10% overhead (down from 40% in 3.13)
  - Multi-threaded: 2-10x gains on CPU-bound tasks (real benchmarks)
- Thread safety: built-in types (dict, list, set) use internal locks (but still use threading.Lock for complex operations)
- **CoLearning Prompts**: 4-5 throughout lesson (most critical lesson)
- **Try With AI**: 4 prompts (hands-on detection, configuration, validation)

### Section 3: Concurrency Patterns & Decision Framework (Lessons 5-6)

**Lesson 5: Choosing Your Concurrency Approach** (B1-B2 proficiency)
- **Decision Framework** (CORE PRACTICAL SKILL):

  | Use Case | Best Approach | Why |
  |----------|--------------|-----|
  | I/O-bound (network, files) | Asyncio or threading | GIL releases during I/O, single-thread efficient |
  | CPU-bound (single task) | Single thread (no concurrency) | No overhead, simplest |
  | CPU-bound (parallel tasks) | **Free-threaded Python** | True parallelism, shared memory, 2-10x gains |
  | CPU-bound (isolation needed) | Multiprocessing | Separate memory spaces, process isolation |
  | Mixed workload | Free-threaded + asyncio | Best of both: parallel CPU + async I/O |

- Benchmarking all four approaches (code examples):
  - Setup: CPU-bound task (prime number calculation)
  - Test 1: Single-threaded baseline
  - Test 2: Traditional threading (GIL-constrained)
  - Test 3: Multiprocessing (separate processes)
  - Test 4: Free-threaded Python (true parallelism)
  - Test 5: Asyncio (for comparison - shows I/O vs CPU difference)
- Interpreting results: execution time, CPU utilization, memory usage
- When to use which: decision tree + real-world examples
- **Asyncio improvements in 3.14** (connects to Ch 28):
  - CLI introspection: `python -m asyncio ps <PID>`, `pstree <PID>`
  - Thread-safe event loop (works with free-threading)
  - 10-20% faster single-threaded performance
- Multiprocessing changes in 3.14:
  - Default `forkserver` on Unix (safer than `fork`)
  - `Process.interrupt()` for graceful shutdown
  - Context inheritance (`thread_inherit_context=True`)
- **CoLearning Prompts**: 3-4 throughout lesson (decision-making practice)
- **Try With AI**: 4 prompts (scenario analysis, approach selection, benchmarking)

**Lesson 6: Capstone - Multi-Agent Concurrency System** (B2 proficiency - INTEGRATION PROJECT)
- **Project**: Hybrid capstone combining multi-agent system + benchmarking dashboard
- **Part A: Multi-Agent AI System** (demonstrates AI-native parallelism):
  - Build 3-4 AI agents performing CPU-bound reasoning tasks (e.g., text analysis, data processing)
  - Implement with free-threaded Python (agents run truly parallel on separate cores)
  - Shared state management (thread-safe data structures, locks where needed)
  - Demonstrate performance gain (compare free-threaded vs traditional threading)
- **Part B: Benchmarking Dashboard**:
  - Tool to compare threading vs multiprocessing vs free-threaded for different workloads
  - Visual output (ASCII charts or simple terminal output)
  - Real-time performance metrics (execution time, CPU%, memory)
  - Practical utility: students can use this for their own projects
- **Integration**: Multi-agent system IS the benchmark (agents are the workload being tested)
- **AI-Native Context**: Connect to production deployment (preview):
  - "In Part 11 (Kubernetes), you'll deploy this multi-agent system as pods, maximizing CPU utilization with free-threading"
  - "In Part 14 (Dapr Actors), you'll scale this pattern to distributed virtual actors with Ray"
- **Type hints**: All code uses Python 3.14+ type hints (modern patterns)
- **Error handling**: Proper exception handling for thread safety (applies Ch 21 knowledge)
- **CoLearning Prompts**: 2-3 throughout lesson (project guidance)
- **Try With AI**: 4 prompts (project scaffolding, debugging, optimization, reflection)

### Common Mistakes (Integrated Throughout Lessons)

**Mistake 1: Assuming GIL Can't Be Removed** (Lesson 3-4)
- ❌ OLD: "The GIL can never be removed from CPython" (pre-3.13 thinking)
- ✅ NEW: "Python 3.14 makes free-threading production-ready; it's now optional" (current reality)

**Mistake 2: Using Threading for CPU-Bound Tasks (Traditional Python)** (Lesson 5)
- ❌ WRONG: `threading.Thread` for prime number calculation (GIL prevents parallelism)
- ✅ RIGHT: Free-threaded Python for CPU-bound parallelism, or multiprocessing for isolation

**Mistake 3: Not Detecting Free-Threading Availability** (Lesson 4)
- ❌ WRONG: Assuming free-threading works without checking build type
- ✅ RIGHT: `sys._is_gil_enabled() == False` to verify before writing parallel code

**Mistake 4: Forgetting Thread Safety in Free-Threaded Mode** (Lesson 6)
- ❌ WRONG: Accessing shared dict without locks in free-threaded Python
- ✅ RIGHT: Use `threading.Lock()` for complex shared state operations (even though built-ins have internal locks)

**Mistake 5: Ignoring Performance Characteristics** (Lesson 5)
- ❌ WRONG: "Free-threading is always faster" (5-10% single-threaded overhead)
- ✅ RIGHT: Benchmark to confirm multi-threaded gain outweighs overhead

### AI Exercise Integration (Throughout Chapter)

**"Try With AI" Pattern** (Every lesson ends with this):
- **Prompt 1** (Recall): "Explain in your own words what [concept] means. Then ask your AI: 'Is my explanation correct? What am I missing?'"
- **Prompt 2** (Understand): "Ask your AI: 'Why did Python designers choose [X] instead of [Y]?' Then explore edge cases."
- **Prompt 3** (Apply): "Tell your AI: 'Help me write code to [practical task]. Explain each step.' Then validate the code."
- **Prompt 4** (Analyze/Synthesize): "Ask your AI: 'How does free-threading in Python 3.14 enable better multi-agent AI systems? Connect this to Ray and Kubernetes (Parts 11-14).'"

**CoLearning Throughout Lessons** (Not just at end):
- **💬 AI Colearning Prompt**: After introducing concept, encourage AI exploration
- **🎓 Instructor Commentary**: "Syntax is cheap — semantics is gold" mantra
- **🚀 CoLearning Challenge**: Practice describing intent, AI generates, student validates
- **✨ Teaching Tip**: Build AI tool literacy (Claude Code/Gemini CLI effective use)

---

## Code Examples (Specifications for 8-10 Examples)

### Example 1: CPython Detection (Lesson 1)
**Purpose**: Verify Python implementation type
**Complexity**: A2 (beginner - simple detection)
**Concepts Used**: `platform` module, function calls, print
**Code**:
```python
import platform

implementation = platform.python_implementation()
print(f"Python implementation: {implementation}")
# Output: "Python implementation: CPython"
```
**AI Prompt**: "Explain what `platform.python_implementation()` does. What would this output on PyPy or Jython?"

### Example 2: Free-Threading Detection (Lesson 4)
**Purpose**: Check if free-threading is available and active
**Complexity**: B1 (intermediate - requires understanding GIL concepts)
**Concepts Used**: `sys` module, conditional logic, boolean operations, type hints
**Code**:
```python
import sys
from typing import Optional

def check_free_threading() -> dict[str, bool | str]:
    """Detect free-threading availability and status."""
    gil_enabled: Optional[bool] = sys._is_gil_enabled()

    return {
        "build_supports_free_threading": gil_enabled is not None,
        "gil_currently_enabled": gil_enabled if gil_enabled is not None else "N/A",
        "free_threading_active": gil_enabled == False
    }

result = check_free_threading()
print(f"Free-threading support: {result}")
```
**AI Prompt**: "Ask your AI: 'Walk me through this code line by line. Why does `gil_enabled is not None` check if the build supports free-threading? What does `sys._is_gil_enabled()` return in different Python builds?'"

### Example 3: CPU-Bound vs I/O-Bound Demonstration (Lesson 3)
**Purpose**: Show the difference between workload types
**Complexity**: B1 (intermediate - loops, functions, timing)
**Concepts Used**: Functions, loops, `time` module, CPU computation, I/O operations
**Code**:
```python
import time
from typing import Callable

def cpu_bound_task(n: int) -> int:
    """CPU-intensive: calculate sum of squares."""
    return sum(i ** 2 for i in range(n))

def io_bound_task(duration: float) -> str:
    """I/O-bound: simulate waiting for network/file."""
    time.sleep(duration)  # Simulates I/O wait
    return f"Waited {duration} seconds"

def measure_time(task: Callable, *args) -> float:
    """Measure execution time of a task."""
    start = time.time()
    task(*args)
    return time.time() - start

# CPU-bound takes actual CPU time
cpu_time = measure_time(cpu_bound_task, 10_000_000)
print(f"CPU-bound time: {cpu_time:.3f}s")

# I/O-bound just waits (GIL releases during sleep)
io_time = measure_time(io_bound_task, 2.0)
print(f"I/O-bound time: {io_time:.3f}s")
```
**AI Prompt**: "Ask your AI: 'Why does `time.sleep()` release the GIL, but the sum calculation doesn't? Explain what's happening in the CPython interpreter for each task.'"

### Example 4: Benchmark - Traditional Threading (Lesson 5)
**Purpose**: Show GIL limitation on CPU-bound tasks
**Complexity**: B1-B2 (threading concepts, benchmarking)
**Concepts Used**: `threading` module, threads, CPU-bound task, timing
**Code**:
```python
import threading
import time

def cpu_task(n: int) -> int:
    """CPU-bound calculation."""
    total = 0
    for i in range(n):
        total += (i ** 2) ** 0.5  # Arbitrary computation
    return total

def benchmark_threading(n: int, num_threads: int) -> float:
    """Benchmark CPU task with traditional threading (GIL-constrained)."""
    threads = []
    start = time.time()

    for _ in range(num_threads):
        thread = threading.Thread(target=cpu_task, args=(n,))
        threads.append(thread)
        thread.start()

    for thread in threads:
        thread.join()

    return time.time() - start

# Test with 4 threads on CPU-bound task
time_taken = benchmark_threading(15**6, 4)
print(f"Traditional threading (4 threads): {time_taken:.2f}s")
# Expected: ~same as single-threaded (GIL prevents parallelism)
```
**AI Prompt**: "Ask your AI: 'Why don't we see a 4x speedup with 4 threads here? Explain what the GIL is doing during this benchmark.'"

### Example 5: Benchmark - Multiprocessing (Lesson 5)
**Purpose**: Show process-based parallelism (no GIL)
**Complexity**: B1-B2 (multiprocessing concepts)
**Concepts Used**: `multiprocessing` module, processes, CPU-bound task
**Code**:
```python
import multiprocessing
import time

def cpu_task(n: int) -> int:
    """Same CPU-bound task as Example 4."""
    total = 0
    for i in range(n):
        total += (i ** 2) ** 0.5
    return total

def benchmark_multiprocessing(n: int, num_processes: int) -> float:
    """Benchmark CPU task with multiprocessing (separate interpreters)."""
    processes = []
    start = time.time()

    for _ in range(num_processes):
        process = multiprocessing.Process(target=cpu_task, args=(n,))
        processes.append(process)
        process.start()

    for process in processes:
        process.join()

    return time.time() - start

if __name__ == "__main__":
    time_taken = benchmark_multiprocessing(15**6, 4)
    print(f"Multiprocessing (4 processes): {time_taken:.2f}s")
    # Expected: ~2-4x faster than single-threaded (true parallelism)
```
**AI Prompt**: "Ask your AI: 'Compare this to the threading example. Why is multiprocessing faster for CPU-bound tasks? What are the tradeoffs (memory, startup time, communication)?'"

### Example 6: Benchmark - Free-Threaded Python (Lesson 5)
**Purpose**: Demonstrate true parallelism with free-threading
**Complexity**: B2 (advanced - requires free-threaded Python build)
**Concepts Used**: Threading + free-threading mode, detection, benchmarking
**Code**:
```python
import threading
import time
import sys

def cpu_task(n: int) -> int:
    """Same CPU-bound task."""
    total = 0
    for i in range(n):
        total += (i ** 2) ** 0.5
    return total

def benchmark_free_threaded(n: int, num_threads: int) -> float | str:
    """Benchmark with free-threaded Python (if available)."""
    # Check if free-threading is available
    if sys._is_gil_enabled() is None:
        return "Free-threading not supported in this build"
    if sys._is_gil_enabled() == True:
        return "GIL is enabled (free-threading disabled)"

    # Free-threading is active
    threads = []
    start = time.time()

    for _ in range(num_threads):
        thread = threading.Thread(target=cpu_task, args=(n,))
        threads.append(thread)
        thread.start()

    for thread in threads:
        thread.join()

    return time.time() - start

# Test
result = benchmark_free_threaded(15**6, 4)
if isinstance(result, str):
    print(result)
else:
    print(f"Free-threaded (4 threads): {result:.2f}s")
    # Expected: ~2-4x faster than traditional threading (true parallelism)
```
**AI Prompt**: "Ask your AI: 'How does this compare to traditional threading and multiprocessing? When would you choose free-threaded Python over multiprocessing?'"

### Example 7: Asyncio CLI Introspection (Lesson 5)
**Purpose**: Use Python 3.14 asyncio debugging tools
**Complexity**: B1-B2 (asyncio concepts from Ch 28)
**Concepts Used**: Asyncio, async/await, task management, CLI tools
**Setup Code** (running async program):
```python
# save as async_example.py
import asyncio

async def agent_task(name: str, duration: int):
    """Simulate an AI agent processing."""
    print(f"{name} starting...")
    await asyncio.sleep(duration)
    print(f"{name} finished")

async def main():
    """Run 3 agents concurrently."""
    tasks = [
        asyncio.create_task(agent_task("Agent 1", 5)),
        asyncio.create_task(agent_task("Agent 2", 3)),
        asyncio.create_task(agent_task("Agent 3", 7))
    ]
    await asyncio.gather(*tasks)

if __name__ == "__main__":
    asyncio.run(main())
```
**Introspection Commands** (in separate terminal while program runs):
```bash
# Get process ID
ps aux | grep async_example.py

# Show all running tasks (flat list)
python -m asyncio ps <PID>

# Show task hierarchy (tree structure)
python -m asyncio pstree <PID>
```
**AI Prompt**: "Ask your AI: 'Explain what these CLI tools show. How do they help debug async programs? When would you use `ps` vs `pstree`?'"

### Example 8: Simple Multi-Agent System (Lesson 6 - Capstone Scaffolding)
**Purpose**: Foundation for capstone project
**Complexity**: B2 (advanced - integrates multiple concepts)
**Concepts Used**: Free-threading, classes, type hints, threading, shared state
**Code** (simplified version - students expand in capstone):
```python
import threading
import sys
import time
from typing import List
from dataclasses import dataclass

@dataclass
class AgentResult:
    """Result from an AI agent's computation."""
    agent_id: int
    result: int
    duration: float

class AIAgent:
    """Simple AI agent performing CPU-bound reasoning."""

    def __init__(self, agent_id: int):
        self.agent_id = agent_id

    def reason(self, data: int) -> AgentResult:
        """Simulate CPU-intensive reasoning."""
        start = time.time()
        result = sum(i ** 2 for i in range(data))
        duration = time.time() - start
        return AgentResult(self.agent_id, result, duration)

def run_multi_agent_system(num_agents: int, data_size: int) -> List[AgentResult]:
    """Run multiple agents in parallel (if free-threading available)."""
    if sys._is_gil_enabled() != False:
        print("⚠️  Free-threading not active. Performance may be limited.")

    agents = [AIAgent(i) for i in range(num_agents)]
    results: List[AgentResult] = []
    threads = []

    def agent_worker(agent: AIAgent, data: int):
        result = agent.reason(data)
        results.append(result)

    start = time.time()

    # Launch agents in parallel
    for agent in agents:
        thread = threading.Thread(target=agent_worker, args=(agent, data_size))
        threads.append(thread)
        thread.start()

    # Wait for completion
    for thread in threads:
        thread.join()

    total_time = time.time() - start
    print(f"\n🤖 Multi-Agent System Completed in {total_time:.2f}s")
    return results

if __name__ == "__main__":
    results = run_multi_agent_system(num_agents=4, data_size=5_000_000)
    for r in results:
        print(f"Agent {r.agent_id}: {r.duration:.2f}s")
```
**AI Prompt**: "Ask your AI: 'How would you expand this into the full capstone project? What benchmarking features should I add? How can I make this production-ready for deployment in Kubernetes (Part 11)?'"

---

## Acceptance Criteria

These criteria validate that the chapter meets all Success Evals and learning objectives. All criteria reference evals explicitly.

### Content Quality Criteria

- **AC-001**: ✅ All 6 lessons follow AI-Native Learning pattern (describe intent → explore → validate → learn from errors) - **References: EVAL-005, EVAL-006**
- **AC-002**: ✅ Every lesson ends with "Try With AI" section ONLY (4 prompts, Bloom's progression) - NO summaries/checklists after - **References: Constitutional Rule**
- **AC-003**: ✅ Reading level is B1-B2 (Flesch-Kincaid Grade 9-11, verified via automated analysis) - **References: EVAL-007**
- **AC-004**: ✅ Complex terms explained on first use (e.g., "bytecode", "reference counting", "mutex") - **References: EVAL-007**
- **AC-005**: ✅ No SDD terminology ("specification-driven development") - uses Part 4 appropriate language ("describe intent") - **References: Forward Reference Rules**
- **AC-006**: ✅ No Chapter 30+ references (SDD taught in Part 5) - **References: Forward Reference Rules**

### Technical Accuracy Criteria

- **AC-007**: ✅ All code examples run on Python 3.14+ (tested on Python 3.14.0) - **References: EVAL-004, EVAL-005, EVAL-006**
- **AC-008**: ✅ All code uses modern type hints (`dict[str, int]`, `list[AgentResult]`, `X | None`) - **References: Python Version Requirements**
- **AC-009**: ✅ Free-threading information is 100% accurate (PEP 703/779, performance characteristics, installation methods) - **References: EVAL-002**
- **AC-010**: ✅ Benchmarks produce valid, reproducible results (threading vs multiprocessing vs free-threaded comparisons) - **References: EVAL-005**
- **AC-011**: ✅ CPython internals explanations are technically correct (bytecode, interpreter, GC, GIL mechanics) - **References: EVAL-001**

### Learning Outcome Criteria (Aligned with Evals)

- **AC-012**: ✅ Students can explain CPython vs other implementations after Lesson 1 - **References: EVAL-001, LO-1**
- **AC-013**: ✅ Students can detect free-threading availability after Lesson 4 - **References: EVAL-004, LO-2**
- **AC-014**: ✅ Students can choose correct concurrency approach for 5 scenarios after Lesson 5 - **References: EVAL-003, LO-3**
- **AC-015**: ✅ Students can build working multi-agent system after Lesson 6 - **References: EVAL-006, LO-4**
- **AC-016**: ✅ Students understand how free-threading enables better AI systems (production context) - **References: EVAL-010, LO-4**

### CoLearning Pedagogy Criteria

- **AC-017**: ✅ Each lesson contains 2-5 💬 AI Colearning Prompts (exploratory, not just end-of-lesson) - **References: Constitutional Rule 9**
- **AC-018**: ✅ Each lesson contains 2-3 🎓 Instructor Commentaries ("syntax cheap, semantics gold" mantra) - **References: Constitutional Rule 9**
- **AC-019**: ✅ Each lesson contains 1-3 🚀 CoLearning Challenges (specification-driven thinking practice) - **References: Constitutional Rule 9**
- **AC-020**: ✅ Each lesson contains 1-2 ✨ Teaching Tips (effective AI tool use) - **References: Constitutional Rule 9**
- **AC-021**: ✅ Tone is conversational (you, your, we), not documentation style - **References: Constitutional Rule 9**

### Prerequisite & Progression Criteria

- **AC-022**: ✅ Prerequisites explicitly listed (Chapters 1-28, especially 17, 21, 28) - **References: Prerequisites Section**
- **AC-023**: ✅ Concepts build progressively (B1 → B1-B2 → B2 across lessons) - **References: Complexity Tier, EVAL-002**
- **AC-024**: ✅ No forward references to concepts taught later in chapter (Lesson N only uses Lessons 1 to N concepts) - **References: Pedagogical Ordering Rules**
- **AC-025**: ✅ Connects to prerequisite chapters explicitly (Ch 17 loops in benchmarks, Ch 21 exceptions in thread safety, Ch 28 asyncio in Lesson 5) - **References: Prerequisites Section**

### Production & Future-Looking Criteria

- **AC-026**: ✅ Capstone project demonstrates production-relevant patterns (multi-agent parallelism, benchmarking) - **References: EVAL-006, LO-4**
- **AC-027**: ✅ Lesson 6 previews Parts 10-13 deployment topics (Ray, Kubernetes, resource utilization) WITHOUT teaching them - **References: EVAL-011, Forward Reference Rules**
- **AC-028**: ✅ All examples follow AIDD validation-first approach (test, validate, verify) - **References: Chapters 1-11 AIDD principles**

---

## Complexity Tier: Advanced (B1-B2)

**CEFR Proficiency Levels**: B1 (Threshold) → B1-B2 (Intermediate-Advanced) → B2 (Vantage)

**Progression Across Chapter**:
- **Lessons 1-2**: B1 (foundational concepts, detection, understanding)
- **Lessons 3-4**: B1-B2 (GIL evolution, free-threading deep dive, installation)
- **Lesson 5**: B1-B2 (decision-making, benchmarking, analysis)
- **Lesson 6**: B2 (capstone integration project, production preview)

**Cognitive Load Management**:
- **Max 10 concepts per lesson** (advanced tier allows more depth than beginner 5 or intermediate 7)
- **Progressive complexity**: Each lesson builds on previous, no sudden jumps
- **Spaced repetition**: Core concepts (GIL, free-threading, benchmarking) revisited across multiple lessons with increasing depth

**Expected Lesson Count**: 6 lessons
- Lessons 1-5: Foundational + practice (4-5 lessons)
- Lesson 6: Capstone integration project (1 lesson)

**Estimated Time**: 6-8 hours total
- Lessons 1-2: 1 hour each (foundational)
- Lessons 3-4: 1.5 hours each (deep technical content)
- Lesson 5: 1.5 hours (decision-making, benchmarking)
- Lesson 6: 2-3 hours (capstone project implementation)

**Prerequisites Enforced**:
- All Chapters 1-28 required
- Special emphasis on Ch 17 (loops for benchmarking), Ch 21 (exceptions for thread safety), Ch 28 (asyncio for comparison)
- Students have completed 27 prior chapters (advanced readiness assumed)

---

## Assumptions & Constraints

### Assumptions

**ASM-001**: Students have Python 3.14+ installed (standard for Part 4 Python chapters)
**ASM-002**: Students can access official Python documentation via MCP tools (context7 integration)
**ASM-003**: Students are comfortable with command-line interfaces (prerequisite Ch 7: Bash)
**ASM-004**: Students have completed all prerequisite chapters (1-28) and possess intermediate Python proficiency
**ASM-005**: Students use Claude Code or Gemini CLI as AI co-learning partners (prerequisite Ch 5-6)
**ASM-006**: Free-threaded Python builds are available for macOS, Windows, Linux (as of Python 3.14 official release)
**ASM-007**: Multi-core CPU is available for benchmarking (most modern machines have 2+ cores)
**ASM-008**: Students can run multiple terminal windows simultaneously (for asyncio CLI introspection in Lesson 5)

### Constraints

**CON-001**: **Python Version** - All content must use Python 3.14+ (free-threading production-ready)
**CON-002**: **No SDD Terminology** - Part 4 appropriate language only ("describe intent" not "write specifications")
**CON-003**: **No Forward References** - NO Chapter 30+ concepts (SDD taught in Part 5)
**CON-004**: **Reading Level** - Maintain B1-B2 (Grade 9-11, verified via Flesch-Kincaid)
**CON-005**: **Lesson Closure** - ALL lessons end with "Try With AI" ONLY (no summaries, checklists after)
**CON-006**: **CoLearning Elements** - Must include 💬🎓🚀✨ throughout lessons (not just at end)
**CON-007**: **Type Hints** - All code examples must use modern Python 3.14 type hints
**CON-008**: **Security** - No `eval()`, `shell=True`, hardcoded secrets in examples
**CON-009**: **Cognitive Load** - Max 10 concepts per lesson (advanced tier limit)
**CON-010**: **AI-Native Framing** - Connect all content to AI/agent development (book's core vision)
**CON-011**: **Production Preview** - Lesson 6 must preview Parts 10-13 WITHOUT teaching deployment (forward-looking context only)

---

## Related Features / Dependencies

### Prerequisite Chapters (MUST complete before Chapter 29)

**Foundation** (Chapters 1-11):
- Chapter 1-4: AIDD principles, methodology, validation-first
- Chapter 5-6: Claude Code, Gemini CLI (AI co-learning tools)
- Chapter 7: Bash essentials (command-line proficiency)
- Chapter 9: Markdown (documentation, clarity)

**Python Core** (Chapters 12-28):
- Chapter 13: Python basics (syntax, execution, REPL)
- Chapter 14: Data types (type hints foundation)
- **Chapter 17: Control Flow and Loops** (CRITICAL - for benchmarking loops)
- Chapter 18: Lists, tuples, dict (data structures for shared state)
- Chapter 20: Functions and modules (code organization)
- **Chapter 21: Exception Handling** (CRITICAL - thread safety, try/except)
- **Chapter 28: Asyncio** (CRITICAL - connects to free-threading + asyncio improvements)

### Future Chapters (Chapter 29 prepares for these)

**Part 5: Spec-Driven Development** (Chapters 30-34):
- Chapter 30: SDD fundamentals (formal specification writing - beyond Part 4 "describe intent")
- Chapter 32: AI orchestration (manager patterns for multi-agent systems)

**Part 6: AI Native Development** (Chapters 35-37):
- Chapter 36: OpenAI Agents SDK (building agentic applications with real AI models)
- Chapter 37: Multi-agent orchestration (advanced patterns building on Ch 29 knowledge)

**Part 11: Docker & Kubernetes** (Chapters 47-49):
- Chapter 48: Kubernetes basics (deploy multi-agent systems as pods)
- Chapter 49: Production Kubernetes (free-threading enables better CPU utilization)

**Part 14: Dapr Actors** (Chapters 55-56):
- Chapter 55: Dapr virtual actors (stateful agents - free-threading enables true parallel actor execution)
- Chapter 56: Durable workflows (long-running agent tasks benefit from parallelism)

### External Dependencies

**Tools Required**:
- Python 3.14+ (with optional free-threaded build for Lessons 4-6)
- Claude Code or Gemini CLI (AI co-learning partners)
- Terminal emulator (for CLI introspection in Lesson 5)
- Text editor/IDE with Python support

**Documentation Sources** (via MCP):
- Python.org official docs (Python 3.14 "What's New", free-threading guide)
- PEP 703 (free-threading implementation)
- PEP 779 (criteria for supported status)
- PEP 659 (specializing adaptive interpreter)

**No External Libraries Required**:
- All examples use Python standard library only
- `threading`, `multiprocessing`, `asyncio`, `time`, `sys`, `platform` (all built-in)

---

## Success Validation Approach

### How We'll Measure Success Evals

**EVAL-001 to EVAL-003** (Comprehension Evals):
- **Method**: Written exercises + quizzes embedded in "Try With AI" sections
- **Data Collection**: Exercise submission tracking + manual review of 20% sample
- **Pass Threshold**: 70-80% of students achieve target proficiency
- **Timeline**: Assessed after each lesson completion (immediate feedback)

**EVAL-004 to EVAL-006** (Skill Acquisition Evals):
- **Method**: Timed coding exercises (EVAL-004), capstone project evaluation (EVAL-005, EVAL-006)
- **Data Collection**: Automated code execution tests + manual technical review
- **Pass Threshold**: 60-85% of students produce working code
- **Timeline**: EVAL-004 after Lesson 4, EVAL-005/006 after Lesson 6 (capstone)

**EVAL-007 to EVAL-009** (Engagement & Accessibility Evals):
- **Method**: Automated readability analysis (EVAL-007), analytics dashboard (EVAL-008, EVAL-009)
- **Data Collection**: Flesch-Kincaid scoring tools, lesson view tracking, submission tracking
- **Pass Threshold**: 65-70% completion/submission rates
- **Timeline**: Continuous monitoring throughout chapter rollout

**EVAL-010 to EVAL-011** (Professional Application Evals):
- **Method**: Short essay questions (100-150 words) + multiple-choice assessments
- **Data Collection**: Written response review (20% sample) + automated quiz scoring
- **Pass Threshold**: 70-75% demonstrate production context understanding
- **Timeline**: Final assessment after Lesson 6 completion

### Iteration Plan

**If Evals Fail** (< target threshold):

1. **Analyze Failure Mode**:
   - Which lessons are students struggling with? (analytics + quiz data)
   - Which concepts are misunderstood? (review written responses, identify patterns)
   - Are "Try With AI" prompts effective? (submission quality analysis)

2. **Revise Content**:
   - Add more CoLearning prompts (💬🎓🚀✨) if engagement low
   - Simplify explanations if reading level too high (adjust for B1 vs B2)
   - Add more code examples if skill acquisition failing
   - Strengthen connections to prerequisites if foundational gaps

3. **Re-Test**:
   - Implement changes in next cohort
   - Compare results (before/after revision)
   - Iterate until evals pass

**Success Indicators** (evals passing):
- Content is production-ready
- Students demonstrate proficiency across all three dimensions (internals, decisions, AI applications)
- Chapter successfully prepares students for Parts 10-13 deployment topics

---

**END OF SPECIFICATION**

---

## Metadata for Planning Phase

**Next Phase**: `/sp.clarify` (if clarifications needed) OR `/sp.plan` (if spec complete)

**Spec Completeness**:
- ✅ Success Evals defined FIRST (11 evals covering all dimensions)
- ✅ Learning objectives measurable and aligned with evals (5 objectives)
- ✅ Content outline detailed (6 lessons, progressive B1 → B1-B2 → B2)
- ✅ Code examples specified (8 examples with purpose, complexity, AI prompts)
- ✅ Acceptance criteria reference evals explicitly (28 criteria)
- ✅ Complexity tier appropriate (B1-B2 advanced)
- ✅ Prerequisites explicit (Chapters 1-28, emphasis on 17, 21, 28)
- ✅ Assumptions and constraints documented

**No [NEEDS CLARIFICATION] markers** - All decisions made based on:
- Chapter title anchor: "CPython and GIL" (from chapter-index.md)
- User decisions: Python 3.14 focus, comprehensive outcomes, hybrid capstone
- Python 3.14 documentation: Free-threading production-ready, performance characteristics
- Book vision: AI-native development, multi-agent systems, production deployment
- Constitution: AI-Native Learning pattern, graduated teaching, CoLearning pedagogy

**Ready for**: Planning phase (`/sp.plan`) to break this spec into lesson-by-lesson structure with CEFR proficiency mapping.
