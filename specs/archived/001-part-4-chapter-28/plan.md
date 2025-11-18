# Chapter 28: Asyncio — Detailed Lesson Plan

**Chapter Type**: Technical (Code-Focused)
**Branch**: `001-part-4-chapter-28`
**Status**: Ready for Development
**Part**: 4 (Python Fundamentals)
**Complexity Tier**: Advanced (B1-B2 proficiency)
**Python Version**: 3.14+ (modern patterns ONLY)
**Estimated Total Duration**: 12-14 hours (6 lessons × 2-2.5h each)

---

## Chapter Overview & Architecture

This chapter teaches Python 3.14+ asyncio from an AI-native development perspective, where students learn to build concurrent systems using modern patterns (`asyncio.run()`, `TaskGroup()`, `InterpreterPoolExecutor`) while leveraging AI as a co-reasoning partner. Students will understand when asyncio helps (I/O-bound), when it doesn't (CPU-bound), and how Python 3.14's `InterpreterPoolExecutor` solves the GIL limitation for hybrid AI workloads.

**Key Learning Progression**:
1. **Lesson 1**: Foundational concepts (event loop, coroutines, `asyncio.run()`)
2. **Lesson 2**: Concurrent task patterns (`create_task()`, `gather()`, `TaskGroup()`)
3. **Lesson 3**: Advanced patterns (timeouts, Futures, error handling)
4. **Lesson 4**: CPU-bound work and GIL (InterpreterPoolExecutor solution)
5. **Lesson 5**: Hybrid workloads (I/O + CPU overlap)
6. **Lesson 6**: Capstone (AI Agent System with all patterns integrated)

**Integration Points**:
- **Prior Chapters**: Assumes completion of Ch 20 (Functions), Ch 21 (Exception Handling), Ch 24-27 (OOP, Pydantic)
- **Prerequisite Knowledge**: Basic Python syntax, type hints, exception handling, OOP
- **Forward Reference**: Ch 29 (CPython and GIL) covers deep GIL internals; this chapter teaches practical workarounds only
- **AI Tool Onboarding**: Part 4 students have access to Claude Code / Gemini CLI (taught in Parts 2-3)

---

## Lesson Architecture (6 Lessons)

### Lesson 1: Asyncio Foundations - Event Loop and Coroutines

**Proficiency Target**: B1 (can apply asyncio patterns to new I/O problems)
**Duration**: 2-2.5 hours
**Estimated Word Count**: 2,500-3,000 words + 5 code examples

**Learning Objectives** (Bloom's progression):
1. Understand the event loop conceptually without managing it manually (Understand — B1)
2. Write and run basic async functions using `async def`, `await`, and `asyncio.run()` (Apply — B1)
3. Distinguish I/O-bound from CPU-bound tasks and identify when asyncio helps (Analyze — B1)
4. Explain how the event loop switches between tasks using `await` (Understand — B1)
5. Compare synchronous vs asynchronous execution with measurable timing (Analyze — B1)

**Skills Taught** (CEFR/Bloom's mapping):
1. **Async Function Definition** — B1 — Technical — Understand
   - Measurable: Student writes `async def` functions with `await` keyword
2. **Event Loop Conceptualization** — B1 — Conceptual — Understand
   - Measurable: Student explains event loop role without managing it directly
3. **asyncio.run() Usage** — B1 — Technical — Apply
   - Measurable: Student runs async functions with `asyncio.run()` entry point
4. **I/O vs CPU Task Recognition** — B1 — Conceptual — Analyze
   - Measurable: Student categorizes given tasks as I/O-bound or CPU-bound with justification
5. **Concurrency vs Parallelism Distinction** — B1 — Conceptual — Understand
   - Measurable: Student articulates difference between concurrent execution and parallel execution

**Key Concepts** (Max 10 for B1 tier):
1. Event loop (what it does, how `asyncio.run()` manages it)
2. Coroutines (`async def` syntax, how they differ from regular functions)
3. `await` keyword (pausing points, cooperative yielding)
4. `asyncio.run()` (Python 3.14+ entry point, automatic loop management)
5. I/O-bound vs CPU-bound (when asyncio helps, when it doesn't)
6. Concurrency vs Parallelism (single-threaded cooperative vs multi-core execution)
7. Blocking vs Non-blocking (what blocks the event loop)

**Content Outline**:
- **Hook** (3-5 min): Why does Python need asyncio? Real-world problem (10 sequential API calls taking 50s vs 5s concurrently)
- **Core 1: Event Loop Basics** (20 min): What is an event loop? How does `asyncio.run()` abstract it? Diagram showing task switching
- **Core 2: Coroutines** (20 min): `async def` vs regular `def`, how `await` works, what happens during `await`
- **Core 3: I/O vs CPU** (15 min): When does asyncio help? Examples of I/O-bound (network) vs CPU-bound (calculation)
- **Core 4: Simple Example** (10 min): Weather API fetch example with timing comparison
- **Core 5: Common Mistakes** (10 min): Forgetting `await`, blocking the loop, trying to use asyncio.run() in Jupyter without nest_asyncio
- **Engagement Elements** (throughout): 💬 Prompts asking students to explain event loop, 🎓 Commentary on why understanding matters, 🚀 Challenges with AI, ✨ Tips

**Code Examples** (5 total):
1. **Sync vs Async Timing Comparison**
   - Purpose: Demonstrate time savings from concurrency
   - Complexity: Simple (2 functions fetching simulated data)
   - Type Hints: Full (async def functions with proper return types)
   - AI Prompt: "Show me async code fetching 3 APIs in parallel, with timing comparison to sequential"
   - Expected Output: Both versions demonstrating ~3x speedup with `asyncio.run()`

2. **Basic asyncio.run() Entry Point**
   - Purpose: Show modern Python 3.14+ pattern
   - Complexity: Trivial (single coroutine)
   - Type Hints: Full
   - Code: Simple async function with `asyncio.run()` wrapper
   - Focus: Demonstrate that students don't manage event loop directly

3. **Coroutine with await asyncio.sleep()**
   - Purpose: Show how `await` works and event loop cooperation
   - Complexity: Simple
   - Type Hints: Full
   - AI Prompt: "Generate a function that simulates an API call using asyncio.sleep(), explain what happens during the sleep"
   - Expected Output: Student understands other tasks run during sleep

4. **I/O-Bound Task (Simulated Network)**
   - Purpose: Realistic async pattern
   - Complexity: Moderate (error handling, timeout simulation)
   - Type Hints: Full with `dict[str, Any]` for API responses
   - Includes: Docstring explaining why this is I/O-bound

5. **CPU-Bound Task with asyncio (Shows Why It Doesn't Help)**
   - Purpose: Demonstrate asyncio doesn't parallelize CPU work (preview for Lesson 4)
   - Complexity: Moderate
   - Type Hints: Full
   - Includes: Comment explaining GIL limitation, preview of `InterpreterPoolExecutor` solution

**CoLearning Elements** (positioned throughout):
- 💬 **Prompt 1** (after "Event Loop Basics"): "Explain to your AI: How does the event loop know when to switch between tasks? What tells it?"
- 🎓 **Commentary 1** (after Concurrency explanation): "You don't memorize asyncio syntax—you understand WHEN to make things concurrent and WHY. That's the professional skill."
- 🚀 **Challenge 1** (after Simple Example): "Ask your AI to generate async code fetching 3 different APIs and explain the execution order"
- ✨ **Tip 1** (after Common Mistakes): "Use your AI to explore: 'What happens if I forget await before asyncio.sleep()?' Then run it and see the warning"

**Practice Approach**:
- Hands-on: Write 3 small async functions (simulate I/O, demonstrate timing, compare sync vs async)
- With AI: Ask Claude Code to generate async versions of given sync functions
- Validation: Measure execution time, verify concurrency actually occurs

**End-of-Lesson: Try With AI** (4 prompts, Bloom's progression):

1. **Understand Level**: "Ask your AI: 'Explain what happens when I await asyncio.sleep(1) in an async function. Why doesn't it block other tasks?'"
   - Expected Output: AI explains that sleep pauses that specific coroutine, allowing event loop to run others

2. **Apply Level**: "Tell your AI: 'Generate async code that fetches 3 URLs in parallel using asyncio.run(). The URLs don't need to be real—simulate with asyncio.sleep().' Then explain the timing."
   - Expected Output: Student explains why all 3 fetch concurrently (not sequentially)

3. **Analyze Level**: "Ask your AI: 'Show me both a sync and async version of fetching 5 APIs. Measure the time difference on my machine. Why is async faster?'"
   - Expected Output: Student measures difference, articulates why (concurrency, not parallelism)

4. **Evaluate Level**: "Given a task (e.g., 'read 100 files'), ask your AI: 'Is this I/O-bound or CPU-bound? Would asyncio help here? Why or why not?' Validate the answer."
   - Expected Output: Student classifies correctly and justifies with reasoning about I/O vs CPU

---

### Lesson 2: Concurrent Tasks - create_task(), gather(), TaskGroup()

**Proficiency Target**: B1 (can write structured concurrent code)
**Duration**: 2-2.5 hours
**Estimated Word Count**: 2,500-3,000 words + 6 code examples

**Learning Objectives** (Bloom's progression):
1. Schedule multiple coroutines concurrently using `asyncio.create_task()` (Apply — B1)
2. Collect results from multiple tasks using `asyncio.gather()` (Apply — B1)
3. Apply `asyncio.TaskGroup()` for structured concurrency (Python 3.11+, best practice) (Apply — B1)
4. Distinguish when to use `gather()` vs `TaskGroup()` (Analyze — B1)
5. Understand task lifecycle and error propagation (Understand — B1)

**Skills Taught** (CEFR/Bloom's mapping):
1. **Task Scheduling with create_task()** — B1 — Technical — Apply
   - Measurable: Student schedules multiple coroutines concurrently without blocking
2. **Result Collection with gather()** — B1 — Technical — Apply
   - Measurable: Student uses `gather()` to collect results, handles exceptions with `return_exceptions`
3. **Structured Concurrency with TaskGroup()** — B1 — Technical — Apply
   - Measurable: Student implements error handling with automatic task cancellation
4. **Gather vs TaskGroup Comparison** — B1 — Conceptual — Analyze
   - Measurable: Student explains tradeoffs (gather tolerates failures, TaskGroup cancels all on first error)
5. **Async Error Handling** — B1 — Technical — Apply
   - Measurable: Student handles exceptions in concurrent tasks appropriately

**Key Concepts** (Max 10 for B1 tier):
1. `asyncio.create_task()` (scheduling coroutines to run concurrently)
2. `asyncio.gather()` (collecting multiple results, `return_exceptions` parameter)
3. `asyncio.TaskGroup()` (structured concurrency context manager)
4. Structured concurrency (automatic cleanup, fail-fast error propagation)
5. Task lifecycle (pending → running → done/cancelled)
6. Error handling in concurrent tasks (how exceptions propagate)
7. Pattern comparison (gather vs TaskGroup tradeoffs)

**Content Outline**:
- **Hook** (3-5 min): Problem: How do you run 10 tasks concurrently and collect their results without nested callbacks?
- **Core 1: create_task() Scheduling** (20 min): Creating tasks, scheduling without awaiting immediately, understanding pending state
- **Core 2: gather() Pattern** (20 min): Collecting results, `return_exceptions` for resilience, comparing to sequential
- **Core 3: TaskGroup() Best Practice** (20 min): Modern Python 3.11+ pattern, automatic error propagation and cancellation, why it's preferred
- **Core 4: Error Handling Strategies** (15 min): Task failures, exception propagation, partial success scenarios
- **Core 5: Performance Comparisons** (10 min): Benchmarking sequential vs concurrent with actual timing
- **Engagement Elements** (throughout): CoLearning prompts about structured concurrency philosophy

**Code Examples** (6 total):
1. **Simple create_task() Scheduling**
   - Purpose: Introduce task creation pattern
   - Complexity: Simple
   - Type Hints: Full with `asyncio.Task[T]` generic
   - Shows: Schedule task without immediate await

2. **Multiple Tasks with gather()**
   - Purpose: Collect results from parallel tasks
   - Complexity: Moderate (10 tasks, timing measurement)
   - Type Hints: Full with `list[dict[str, Any]]`
   - Comparison: Sequential vs concurrent timing

3. **TaskGroup() Modern Pattern**
   - Purpose: Preferred Python 3.11+ approach
   - Complexity: Moderate
   - Type Hints: Full
   - Context manager usage demonstrating automatic cleanup

4. **Error Handling with gather(return_exceptions=True)**
   - Purpose: Resilient collection
   - Complexity: Moderate (some tasks fail)
   - Type Hints: Full with `list[T | Exception]`
   - Shows: Collecting both successes and failures

5. **TaskGroup Error Propagation**
   - Purpose: Demonstrate fail-fast behavior
   - Complexity: Moderate
   - Type Hints: Full
   - Shows: One task failure cancels all others

6. **Benchmarking: Concurrent vs Sequential**
   - Purpose: Measurable performance improvement
   - Complexity: Moderate
   - Type Hints: Full with timing decorators
   - Output: Actual timing comparison (e.g., 10 × 1s API calls: 10s sequential vs 1s concurrent)

**CoLearning Elements**:
- 💬 **Prompt 1** (after TaskGroup explanation): "Why does TaskGroup cancel all tasks when one fails? When is this useful?"
- 🎓 **Commentary 1** (after structured concurrency definition): "Structured concurrency is like RAII for async code—cleanup happens automatically, no manual cancellation needed"
- 🚀 **Challenge 1** (after pattern comparison): "Implement concurrent API fetching with both gather() and TaskGroup(). Explain when you'd choose each."
- ✨ **Tip 1** (after error handling): "Ask your AI: 'Show me a scenario where gather() is better than TaskGroup()' (Hint: when you want best-effort results even if some fail)"

**Practice Approach**:
- Hands-on: Convert sequential API fetching to concurrent with TaskGroup()
- With AI: "Generate code that fetches 10 URLs concurrently and returns results even if 2 fail"
- Validation: Measure execution time, verify all tasks start, handle partial failures

**End-of-Lesson: Try With AI** (4 prompts, Bloom's progression):

1. **Understand Level**: "Ask your AI: 'What's the difference between awaiting a task immediately vs scheduling it with create_task() first?'"
   - Expected Output: AI explains that create_task() schedules immediately, await later waits for result

2. **Apply Level**: "Tell your AI: 'I need to fetch 5 APIs concurrently. Generate code using TaskGroup() that returns results or handles errors gracefully.'"
   - Expected Output: Student verifies all 5 APIs run in parallel, errors don't break others

3. **Analyze Level**: "Compare: Show me fetch code using gather(return_exceptions=True) vs TaskGroup(). When would you use each? Why?"
   - Expected Output: Student articulates gather continues on failure vs TaskGroup cancels all

4. **Synthesize Level**: "Design: You have 100 tasks to run concurrently. Some take 1s, some 10s. Ask your AI to optimize for best performance with TaskGroup(). Explain bottlenecks."
   - Expected Output: Student identifies that slowest task determines total time, discusses strategies

---

### Lesson 3: Advanced Patterns - Timeouts, Futures, Error Handling

**Proficiency Target**: B1-B2 (advanced applied patterns)
**Duration**: 2-2.5 hours
**Estimated Word Count**: 2,500-3,000 words + 6 code examples

**Learning Objectives** (Bloom's progression):
1. Apply timeout controls using `asyncio.timeout()` context manager (Apply — B1-B2)
2. Understand Futures as awaitable result placeholders (Understand — B1)
3. Handle errors gracefully in async code with proper exception recovery (Apply — B1-B2)
4. Debug common async pitfalls with AI assistance (Apply — B1-B2)
5. Build resilient async systems with retry logic and partial failures (Create — B2)

**Skills Taught** (CEFR/Bloom's mapping):
1. **Timeout Management** — B1-B2 — Technical — Apply
   - Measurable: Student implements timeouts with `asyncio.timeout()` context manager
2. **Future Conceptualization** — B1 — Conceptual — Understand
   - Measurable: Student explains Futures as awaitable result placeholders (rarely creates manually)
3. **Async Exception Handling** — B1-B2 — Technical — Apply
   - Measurable: Student writes try/except blocks with async code, handles TimeoutError and CancelledError
4. **Common Async Debugging** — B1-B2 — Technical — Apply
   - Measurable: Student identifies and fixes never-awaited coroutines, RuntimeWarning issues
5. **Resilience Patterns** — B2 — Technical — Create
   - Measurable: Student designs retry logic, exponential backoff, circuit breaker patterns

**Key Concepts** (Max 10 for B1-B2 tier):
1. `asyncio.timeout()` (context manager for timeout control, Python 3.11+)
2. Futures (conceptual understanding as awaitable objects)
3. TimeoutError (handling timeout exceptions)
4. CancelledError (when tasks are cancelled)
5. Exception handling in async (try/except with await)
6. Never-awaited coroutines (RuntimeWarning detection)
7. Common async mistakes (blocking the loop, missing await)
8. Retry logic and exponential backoff
9. Partial failure handling

**Content Outline**:
- **Hook** (3-5 min): Why do timeouts matter? (Services hanging indefinitely, cascading failures)
- **Core 1: Timeout Control** (20 min): `asyncio.timeout()` context manager, TimeoutError handling, practical patterns
- **Core 2: Futures Conceptual** (15 min): What are Futures? When are they used? (Rarely create manually in modern code)
- **Core 3: Exception Handling** (20 min): try/except with async, TimeoutError vs CancelledError, recovery strategies
- **Core 4: Common Pitfalls** (15 min): Never-awaited coroutines, blocking the loop, debugging techniques
- **Core 5: Resilience Patterns** (15 min): Retry logic, exponential backoff, circuit breaker concepts
- **Engagement Elements** (throughout): CoLearning on defensive programming

**Code Examples** (6 total):
1. **Simple asyncio.timeout() Usage**
   - Purpose: Basic timeout pattern
   - Complexity: Simple
   - Type Hints: Full
   - Shows: Context manager with TimeoutError handling

2. **Handling TimeoutError Gracefully**
   - Purpose: Resilient timeout handling
   - Complexity: Moderate (fallback strategies)
   - Type Hints: Full
   - Shows: Retry or fallback instead of failing completely

3. **Future Basic Usage**
   - Purpose: Understand Futures (rarely created manually)
   - Complexity: Simple
   - Type Hints: Full
   - Note: Emphasize that most code uses create_task() instead

4. **Exception Handling in Async Code**
   - Purpose: Proper error handling with await
   - Complexity: Moderate
   - Type Hints: Full
   - Shows: try/except around await, handling multiple exception types

5. **Detecting and Fixing Never-Awaited Coroutines**
   - Purpose: Common debugging scenario
   - Complexity: Simple-Moderate
   - Type Hints: Full
   - Shows: RuntimeWarning example and fix

6. **Retry Logic with Exponential Backoff**
   - Purpose: Production-grade resilience
   - Complexity: Moderate-Advanced
   - Type Hints: Full
   - Shows: Decorator or helper function for retries with exponential backoff

**CoLearning Elements**:
- 💬 **Prompt 1** (after timeout section): "What's the difference between asyncio.timeout() and asyncio.wait_for()?"
- 🎓 **Commentary 1** (after timeout motivation): "Timeouts aren't just for network calls—they're your defense against infinite waits and cascading failures"
- 🚀 **Challenge 1** (after retry logic): "Build a resilient async function that retries on failure with exponential backoff and jitter"
- ✨ **Tip 1** (after common pitfalls): "When you get a RuntimeWarning, ask your AI: 'Why did my coroutine never get awaited?' (Hint: look for missing await)"

**Practice Approach**:
- Hands-on: Implement timeout handling in async functions
- With AI: "Generate code with retry logic and exponential backoff for unreliable API calls"
- Validation: Test timeout scenarios, verify retries work, measure backoff timing

**End-of-Lesson: Try With AI** (4 prompts, Bloom's progression):

1. **Understand Level**: "Ask your AI: 'How does asyncio.timeout() work? What happens when the timeout expires?'"
   - Expected Output: AI explains timeout context manager and TimeoutError raising

2. **Apply Level**: "Tell your AI: 'Generate code that fetches an API with a 5-second timeout. If it times out, retry up to 3 times with exponential backoff.'"
   - Expected Output: Student verifies timeout works, retries trigger, backoff increases

3. **Analyze Level**: "Compare TimeoutError vs CancelledError. Ask your AI: 'When does each occur? How should I handle them differently?'"
   - Expected Output: Student understands TimeoutError from timeout context, CancelledError from task cancellation

4. **Create Level**: "Design a circuit breaker pattern: if API fails 5 times, stop calling it for 60s. Ask your AI for implementation. Explain tradeoffs."
   - Expected Output: Student designs pattern, understands when circuit breaker prevents cascading failures

---

### Lesson 4: CPU-Bound Work - GIL and InterpreterPoolExecutor

**Proficiency Target**: B1-B2 (understanding limitation and solution)
**Duration**: 2-2.5 hours
**Estimated Word Count**: 2,500-3,000 words + 6 code examples

**Learning Objectives** (Bloom's progression):
1. Understand the GIL limitation briefly (2-3 sentences, detailed coverage in Ch 29) (Understand — B1)
2. Apply `InterpreterPoolExecutor` for true parallelism in async programs (Apply — B1-B2)
3. Measure performance difference between threading and `InterpreterPoolExecutor` (Analyze — B1-B2)
4. Recognize when to use `ProcessPoolExecutor` vs `InterpreterPoolExecutor` (Analyze — B1-B2)
5. Architect hybrid I/O+CPU systems (Create — B2)

**Skills Taught** (CEFR/Bloom's mapping):
1. **GIL Conceptual Understanding** — B1 — Conceptual — Understand
   - Measurable: Student explains GIL in 2-3 sentences (brief intro, deep dive in Ch 29)
2. **InterpreterPoolExecutor Usage** — B1-B2 — Technical — Apply
   - Measurable: Student uses `InterpreterPoolExecutor` with `loop.run_in_executor()` for CPU-bound work
3. **Performance Benchmarking** — B1-B2 — Technical — Apply
   - Measurable: Student measures 3-4x speedup on 4-core machine with `InterpreterPoolExecutor` vs threading
4. **Executor Selection** — B1-B2 — Conceptual — Analyze
   - Measurable: Student chooses `InterpreterPoolExecutor` vs `ProcessPoolExecutor` with justification
5. **Hybrid System Architecture** — B2 — Technical — Create
   - Measurable: Student designs systems combining concurrent I/O and parallel CPU work

**Key Concepts** (Max 10 for B1-B2 tier):
1. GIL (Global Interpreter Lock) - brief intro (2-3 sentences, full explanation in Ch 29)
2. Why threading doesn't parallelize CPU-bound Python (GIL contention)
3. `InterpreterPoolExecutor` (Python 3.14 feature, separate interpreters = separate GILs)
4. `loop.run_in_executor()` (bridging sync functions into async code)
5. True parallelism (multiple CPU cores utilized)
6. `ProcessPoolExecutor` (alternative, higher overhead, when to use)
7. Performance measurement (timing CPU-bound work with/without parallelism)
8. GIL workarounds (sub-interpreters, separate processes)

**Content Outline**:
- **Hook** (3-5 min): Why does Python need a GIL? Show threading doesn't parallelize CPU work (benchmark)
- **Core 1: GIL Brief Intro** (10 min): 2-3 sentence explanation, forward reference to Ch 29, why it exists (memory safety)
- **Core 2: InterpreterPoolExecutor Solution** (20 min): Python 3.14 feature, separate interpreters = separate GILs = true parallelism
- **Core 3: Bridging Sync to Async** (15 min): `loop.run_in_executor()` pattern, integrating CPU work into async code
- **Core 4: Performance Measurement** (15 min): Benchmarks showing threading vs InterpreterPoolExecutor (4-core = ~4x speedup)
- **Core 5: ProcessPoolExecutor Alternative** (10 min): Higher overhead, when to use instead of InterpreterPoolExecutor
- **Engagement Elements** (throughout): CoLearning on Python's design tradeoffs

**Code Examples** (6 total):
1. **CPU-Bound Function Example**
   - Purpose: Define heavy calculation
   - Complexity: Simple
   - Type Hints: Full
   - Shows: Function that spends CPU time (no I/O)

2. **Threading Benchmark (No Speedup)**
   - Purpose: Demonstrate GIL limitation
   - Complexity: Moderate
   - Type Hints: Full
   - Output: Threading with 4 threads takes ~same time as single-threaded (proves GIL contention)

3. **InterpreterPoolExecutor Benchmark (4x Speedup)**
   - Purpose: Show InterpreterPoolExecutor solves it
   - Complexity: Moderate
   - Type Hints: Full
   - Output: InterpreterPoolExecutor with 4 workers achieves 4x speedup

4. **Bridging CPU-Bound to Async with run_in_executor()**
   - Purpose: Integrate CPU work into async program
   - Complexity: Moderate
   - Type Hints: Full with `Awaitable[T]` return type
   - Shows: CPU work doesn't block event loop

5. **ProcessPoolExecutor Alternative**
   - Purpose: Compare to InterpreterPoolExecutor
   - Complexity: Moderate
   - Type Hints: Full
   - Shows: Higher overhead from serialization vs InterpreterPoolExecutor (same interpreter semantics)

6. **Decision Tree: I/O vs CPU vs Hybrid**
   - Purpose: When to use each pattern
   - Complexity: Moderate (conceptual)
   - Type Hints: Full with examples
   - Shows: Guide for choosing appropriate concurrency tool

**CoLearning Elements**:
- 💬 **Prompt 1** (after GIL intro): "Why does Python have a GIL? What problem was it solving?"
- 🎓 **Commentary 1** (after GIL brief intro): "The GIL isn't a bug—it's a design tradeoff. Python 3.14 gives you tools to work around it when you need true parallelism"
- 🚀 **Challenge 1** (after benchmarks): "Benchmark a CPU-intensive task with threading, InterpreterPoolExecutor, and ProcessPoolExecutor. Explain the results."
- ✨ **Tip 1** (after executor comparison): "Ask your AI: 'When should I use InterpreterPoolExecutor vs just running a subprocess?'" (Hint: overhead and semantics)

**Practice Approach**:
- Hands-on: Implement CPU-intensive function, measure single-threaded time
- With AI: "Generate code using InterpreterPoolExecutor to parallelize this function on 4 cores"
- Validation: Measure speedup (should be close to 4x on 4-core machine)

**GIL Forward Reference**:
Include explicit callout: "Chapter 29 covers GIL internals deeply—how it evolved, free-threaded Python mode (Python 3.13+), and implications for C extensions. For now, understand that `InterpreterPoolExecutor` gives you a practical workaround."

**End-of-Lesson: Try With AI** (4 prompts, Bloom's progression):

1. **Understand Level**: "Ask your AI: 'What is the Global Interpreter Lock (GIL) in Python? Why do we need it?'"
   - Expected Output: AI explains memory safety reason, mentions Chapter 29 for deep dive

2. **Apply Level**: "Tell your AI: 'I have a CPU-intensive calculation. Generate code using InterpreterPoolExecutor to run it in parallel. Show timing.'"
   - Expected Output: Student verifies 3-4x speedup on 4-core machine

3. **Analyze Level**: "Ask your AI: 'Compare InterpreterPoolExecutor vs ProcessPoolExecutor for this task: [CPU-intensive function]. Which is better? Why?'"
   - Expected Output: Student understands InterpreterPoolExecutor overhead is lower (same interpreter namespace)

4. **Synthesize Level**: "Design: You need to fetch 10 APIs AND process each response with CPU-intensive analysis. How would you architect this? Ask your AI for implementation."
   - Expected Output: Student designs TaskGroup for I/O, InterpreterPoolExecutor for CPU, understands overlap benefits

---

### Lesson 5: Hybrid Workloads - Combining I/O and CPU

**Proficiency Target**: B2 (advanced architectural thinking)
**Duration**: 2-2.5 hours
**Estimated Word Count**: 2,500-3,000 words + 6 code examples

**Learning Objectives** (Bloom's progression):
1. Combine asyncio (I/O concurrency) with `InterpreterPoolExecutor` (CPU parallelism) (Create — B2)
2. Build systems where I/O and CPU work overlap efficiently (Create — B2)
3. Apply hybrid patterns to AI-native applications (API calls + inference) (Apply — B2)
4. Architect for real-world performance optimization (Analyze/Evaluate — B2)
5. Optimize resource management and avoid bottlenecks (Create — B2)

**Skills Taught** (CEFR/Bloom's mapping):
1. **Hybrid Workload Pattern** — B2 — Technical — Create
   - Measurable: Student designs systems combining concurrent I/O and parallel CPU work
2. **I/O + CPU Overlap** — B2 — Conceptual — Apply
   - Measurable: Student explains how fetch and process stages can overlap for efficiency
3. **AI-Native Workload Patterns** — B2 — Technical — Apply
   - Measurable: Student applies pattern to API calls + inference scenarios
4. **Resource Limiting** — B2 — Technical — Apply
   - Measurable: Student manages concurrent I/O and CPU worker pools appropriately
5. **Performance Optimization** — B2 — Technical — Analyze/Create
   - Measurable: Student identifies bottlenecks and optimizes architecture

**Key Concepts** (Max 10 for B1-B2 tier):
1. Hybrid workload pattern (asyncio + InterpreterPoolExecutor together)
2. I/O + CPU overlap (fetch data while processing previous batch)
3. AI-native workload characteristics (API calls + inference)
4. Executor integration (using `loop.run_in_executor()` within async functions)
5. Resource management (limiting concurrent I/O and CPU work)
6. Performance optimization (pipelining, batching)
7. Real-world patterns (data fetching + processing pipelines)
8. Bottleneck analysis (CPU-bound vs I/O-bound stages)

**Content Outline**:
- **Hook** (3-5 min): Problem: Fetch 1000 items and process each. Sequential = slow. All parallel = OOM. What's optimal?
- **Core 1: Hybrid Concept** (15 min): Why combine I/O and CPU? Pipeline pattern where stages overlap
- **Core 2: Simple Hybrid Example** (20 min): Fetch API → Process (CPU) → Return results, all concurrent
- **Core 3: Batch Processing Pattern** (15 min): Fetch N items, process in parallel, optimal batch size
- **Core 4: Pipeline Pattern** (15 min): Fetch → Transform → Store, overlapping stages
- **Core 5: Resource Limiting** (10 min): Max concurrent I/O (avoid overwhelming server), max CPU workers (cores), optimal ratios
- **Core 6: Performance Analysis** (10 min): Measure bottlenecks, identify what's limiting throughput
- **Engagement Elements** (throughout): CoLearning on systems thinking

**Code Examples** (6 total):
1. **Simple Hybrid: Fetch → Process**
   - Purpose: Basic pattern
   - Complexity: Moderate
   - Type Hints: Full
   - Shows: One API call triggers one CPU task

2. **Batch Processing Pattern**
   - Purpose: Fetch N items, process in parallel
   - Complexity: Moderate-Advanced
   - Type Hints: Full with `list[T]` and batch management
   - Shows: Optimal batch size for throughput

3. **Pipeline Pattern (Fetch → Transform → Store)**
   - Purpose: Overlapping stages
   - Complexity: Advanced
   - Type Hints: Full
   - Shows: Each stage independent, data flows through

4. **AI Workload Simulation (API + Inference)**
   - Purpose: Realistic AI pattern
   - Complexity: Moderate
   - Type Hints: Full
   - Shows: Fetch data concurrently, process "inference" in parallel

5. **Resource Limiting (Semaphores)**
   - Purpose: Limit concurrent I/O and CPU work
   - Complexity: Moderate
   - Type Hints: Full
   - Shows: Using `asyncio.Semaphore()` to cap concurrent tasks

6. **Full Hybrid System with Error Handling**
   - Purpose: Production-grade example
   - Complexity: Advanced
   - Type Hints: Full
   - Shows: Error handling, timeouts, resource cleanup, logging

**CoLearning Elements**:
- 💬 **Prompt 1** (after hybrid concept): "How would you optimize a system that fetches 1000 items and processes each? What's the bottleneck?"
- 🎓 **Commentary 1** (after AI-native workload): "AI apps are inherently hybrid—API calls are I/O, inference is CPU. You need both patterns to build efficient systems"
- 🚀 **Challenge 1** (after pipeline pattern): "Design a data processing pipeline: fetch from API → transform (CPU) → store in DB (I/O). Optimize for throughput."
- ✨ **Tip 1** (after resource limiting): "Ask your AI: 'What's the optimal number of workers for my machine?' (Hint: depends on cores and I/O latency)"

**Practice Approach**:
- Hands-on: Implement simple hybrid (fetch + process)
- With AI: "Generate optimized pipeline for 1000 items: fetch, analyze, store"
- Validation: Measure throughput, identify bottlenecks, optimize batch size

**End-of-Lesson: Try With AI** (4 prompts, Bloom's progression):

1. **Understand Level**: "Ask your AI: 'In a system that fetches data and processes it, how can I/O and CPU work overlap efficiently?'"
   - Expected Output: AI explains pipelining—next fetch starts while previous processes

2. **Apply Level**: "Tell your AI: 'Generate code that fetches 100 items from an API and processes each with CPU-intensive analysis concurrently.'"
   - Expected Output: Student verifies all 100 fetch concurrently and process in parallel

3. **Analyze Level**: "Ask your AI: 'What's the bottleneck in my hybrid system—CPU or I/O? How would you identify it? How do I optimize?'"
   - Expected Output: Student uses timing/profiling to identify limiting factor, adjusts worker counts

4. **Design Level**: "Architect: You need to fetch 10,000 items, process each, store results. Optimize for 4-core machine with 10 Gbps network. Ask your AI for design and implementation."
   - Expected Output: Student designs batch size, worker counts, resource management for optimal throughput

---

### Lesson 6: Capstone - AI Agent System (Multi-Service Concurrent Fetching + Parallel Inference)

**Proficiency Target**: B2 (integration and application)
**Duration**: 2.5-3 hours
**Estimated Word Count**: 3,000-3,500 words + capstone project

**Learning Objectives** (Bloom's progression):
1. Integrate all chapter concepts into a production-style AI agent (Create — B2)
2. Build a system fetching from multiple services concurrently (Create — B2)
3. Process responses with simulated LLM inference in parallel (Create — B2)
4. Handle errors, timeouts, and graceful degradation (Create — B2)
5. Explain architecture and performance tradeoffs to AI collaborator (Evaluate — B2)

**Skills Taught** (CEFR/Bloom's mapping):
1. **System Integration** — B2 — Technical — Create
   - Measurable: Student integrates all 5 prior lessons into one cohesive system
2. **Multi-Service Orchestration** — B2 — Technical — Create
   - Measurable: Student implements TaskGroup for 3+ concurrent API sources
3. **Parallel Inference Simulation** — B2 — Technical — Create
   - Measurable: Student processes responses in parallel with InterpreterPoolExecutor
4. **Production Error Handling** — B2 — Technical — Create
   - Measurable: Student implements timeouts, retries, partial success handling
5. **Architecture Explanation** — B2 — Technical — Evaluate
   - Measurable: Student articulates design decisions and tradeoffs to AI

**Key Concepts** (All from prior lessons, now integrated):
1. TaskGroup for concurrent API fetching
2. InterpreterPoolExecutor for parallel inference
3. Timeout management and error handling
4. Graceful degradation (best-effort results)
5. Performance measurement and optimization
6. Real-world architectural patterns

**Capstone Project Specification**:

**System Description**: Build an AI Agent that queries multiple information sources (weather API, news API, knowledge base) concurrently, processes each response with simulated LLM inference (CPU-intensive text analysis), and aggregates results into a unified response—all while handling failures gracefully.

**Requirements**:
1. **Concurrent Fetching**: Use `TaskGroup()` to fetch from 3+ sources in parallel
2. **Parallel Processing**: Use `InterpreterPoolExecutor` to simulate LLM inference (heavy string processing)
3. **Error Handling**: If one source times out, continue with others (partial success)
4. **Timeout Control**: Each API call has 5s timeout, overall system has 15s timeout
5. **Graceful Degradation**: System provides best-effort response even if 1-2 sources fail
6. **Performance**: Demonstrate hybrid execution is 2-3x faster than sequential

**Code Skeleton** (students implement with AI guidance):

```python
# types and configuration
async def fetch_weather(city: str) -> dict[str, Any]: ...
async def fetch_news(query: str) -> list[dict[str, Any]]: ...
async def query_knowledge_base(question: str) -> str: ...

def simulate_llm_inference(text: str) -> str:
    # CPU-intensive processing (runs in InterpreterPoolExecutor)
    ...

async def ai_agent_query(user_question: str) -> dict[str, Any]:
    # Student implements using TaskGroup + InterpreterPoolExecutor
    ...
```

**Acceptance Criteria**:
- [ ] All 3 sources fetched concurrently (measured: ~5s total, not 15s sequential)
- [ ] Processing happens in parallel (CPU usage spikes across multiple cores)
- [ ] System handles partial failures (1 source down → still returns results from others)
- [ ] Total execution time < 15s with timeouts applied
- [ ] Code includes type hints, error handling, and logging
- [ ] Student explains architecture to AI and receives feedback (validates understanding)

**Content Outline**:
- **Hook** (5 min): Show what students will build—real-world AI agent pattern
- **Part 1: Architecture Overview** (15 min): Diagram showing concurrent fetch + parallel inference
- **Part 2: Code Walkthrough** (20 min): Skeleton code explained, what student implements, what AI helps with
- **Part 3: Implementation Guidance** (30 min): Step-by-step implementation with AI partnership pattern
- **Part 4: Testing and Optimization** (15 min): Measure performance, identify bottlenecks, optimize
- **Part 5: Reflection and Extension** (10 min): What you learned, how to extend further
- **Engagement Elements** (throughout): CoLearning on building real systems

**Capstone Code Example** (Complete structure with guidance):
```python
# File: ai_agent.py
import asyncio
from concurrent.futures import Executor
from typing import Any

# Simulated API sources (students implement with AI help)
async def fetch_weather(city: str, *, timeout: float = 5.0) -> dict[str, Any]:
    """Fetch weather data for a city. Simulated with asyncio.sleep()."""
    try:
        async with asyncio.timeout(timeout):
            # Simulate network call
            await asyncio.sleep(2)
            return {"city": city, "temp": 72, "condition": "sunny"}
    except asyncio.TimeoutError:
        return {"error": "Weather API timeout"}

async def fetch_news(query: str, *, timeout: float = 5.0) -> list[dict[str, Any]]:
    """Fetch news articles. Simulated with asyncio.sleep()."""
    try:
        async with asyncio.timeout(timeout):
            await asyncio.sleep(1.5)
            return [
                {"title": f"Article 1 about {query}", "snippet": "..."},
                {"title": f"Article 2 about {query}", "snippet": "..."},
            ]
    except asyncio.TimeoutError:
        return [{"error": "News API timeout"}]

def simulate_llm_inference(text: str) -> str:
    """CPU-intensive inference simulation. Runs in executor."""
    # Simulate heavy processing
    import time
    start = time.time()
    while time.time() - start < 2:  # 2 seconds of CPU work
        _ = sum(i**2 for i in range(100000))
    return f"Analysis of '{text[:50]}...': [Result from inference]"

async def ai_agent_query(
    user_question: str,
    executor: Executor | None = None
) -> dict[str, Any]:
    """
    Main agent function. Students implement this with AI help.

    Pattern:
    1. Use TaskGroup for concurrent fetching
    2. Use executor.run_in_executor() for CPU-bound inference
    3. Handle errors gracefully
    4. Return aggregated results
    """
    # Student implementation here
    pass

# Performance benchmark
async def benchmark_hybrid_system() -> None:
    """Compare sequential vs concurrent+parallel execution."""
    print("Running hybrid AI agent benchmark...")

    from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
    from time import perf_counter

    # Sequential version (baseline)
    # ...

    # Hybrid concurrent+parallel version
    # ...

    # Print timing comparison

if __name__ == "__main__":
    asyncio.run(ai_agent_query("What's the weather and latest news?"))
```

**Practice Approach**:
- Scaffold 1: Implement fetch functions (students write, AI reviews)
- Scaffold 2: Implement concurrent fetching with TaskGroup (guided implementation)
- Scaffold 3: Integrate inference processing with executor (guided implementation)
- Scaffold 4: Add error handling and timeouts (students design, AI generates)
- Scaffold 5: Measure and optimize performance (students measure, AI suggests optimizations)

**CoLearning Elements**:
- 💬 **Prompt 1** (after architecture overview): "Explain how this architecture handles the case where 2/3 sources timeout"
- 🎓 **Commentary 1** (after code walkthrough): "This is what production AI systems look like—resilient, concurrent, and optimized"
- 🚀 **Challenge 1** (after implementation): "Add caching to avoid re-fetching recent queries within 60 seconds"
- ✨ **Tip 1** (after performance measurement): "Ask AI to review your error handling strategy and suggest improvements"

**End-of-Lesson: Try With AI** (4 prompts, Bloom's progression):

1. **Understand Level**: "Ask your AI: 'Walk me through the execution flow of this AI agent. What happens in parallel vs sequentially?'"
   - Expected Output: AI explains concurrent fetching, then parallel inference, then aggregation

2. **Apply Level**: "Tell your AI: 'I need to handle the case where one source is permanently down. Implement fallback logic.'"
   - Expected Output: Student verifies system returns results from working sources, handles missing source gracefully

3. **Analyze Level**: "Ask your AI: 'Where is the bottleneck in this system? CPU or I/O? How do I identify it? How would you optimize?'"
   - Expected Output: Student uses profiling/timing to identify bottleneck, adjusts worker counts or API calls

4. **Evaluate Level**: "Review: Ask your AI to evaluate your implementation against production requirements: fault tolerance, performance, scalability. What would you change for real-world use?"
   - Expected Output: Student discusses rate limiting, circuit breakers, monitoring, real API integration

---

## Lesson Scaffolding Strategy

### Complexity Progression Across Lessons

| Lesson | CEFR Level | Cognitive Load | Bloom's Max | Concepts | Focus |
|--------|------------|----------------|------------|----------|-------|
| 1 | B1 | 7 concepts | Analyze | Foundations + I/O identification | Understanding basics |
| 2 | B1 | 7 concepts | Analyze | Task patterns + error propagation | Concurrent patterns |
| 3 | B1-B2 | 9 concepts | Create | Timeouts + debugging + resilience | Advanced error handling |
| 4 | B1-B2 | 8 concepts | Create | GIL + executors + benchmarking | CPU-bound solutions |
| 5 | B2 | 8 concepts | Create | Hybrid architecture + optimization | System design |
| 6 | B2 | Integration | Evaluate | All concepts integrated | Real-world application |

### Cognitive Load Management

**Per Lesson**:
- **Lesson 1**: Max 7 new concepts (B1 tier) ✓ Validated
- **Lesson 2**: Max 7 new concepts (B1 tier) ✓ Validated
- **Lesson 3**: Max 9 new concepts (B1-B2 tier) ✓ Validated
- **Lesson 4**: Max 8 new concepts (B1-B2 tier) ✓ Validated
- **Lesson 5**: Max 8 new concepts (B2 tier) ✓ Validated
- **Lesson 6**: Integration (no new concepts, application only) ✓ Validated

**CoLearning Elements**: 4 per lesson (💬 🎓 🚀 ✨) positioned throughout, not just at end

**Lesson Closures**: "Try With AI" section ONLY (4 prompts, Bloom's progression) — no separate Key Takeaways, What's Next, or Checklists

### Skills Proficiency Progression (CEFR Validation)

**Lesson 1 → Lesson 6 Progression**:
- Lesson 1: A1-B1 (Remember/Understand to Apply)
- Lesson 2: B1 (Apply, introducing Analyze)
- Lesson 3: B1-B2 (Apply to Create, advanced resilience)
- Lesson 4: B1-B2 (Apply to Create, new tool introduction)
- Lesson 5: B2 (Create, architectural thinking)
- Lesson 6: B2 (Evaluate/Synthesize, integration)

✓ **No regression**: Each lesson builds on prior (no backward steps in proficiency)
✓ **Smooth progression**: No jumps in cognitive complexity
✓ **B1 → B2 transition**: Lessons 3-4 act as bridge (introduce advanced concepts in scaffolded way)

---

## Integration & Cross-Chapter Alignment

### Dependencies (Chapters Required Before This)

- **Chapter 20**: Module and Functions (function composition, imports)
- **Chapter 21**: Exception Handling (try/except, error recovery)
- **Chapter 24-25**: OOP Part I & II (classes for capstone code)
- **Chapter 26-27**: Data Classes and Pydantic (type annotations for data)
- **Chapters 1-11**: AIDD mindset (validation-first, AI as partner)

All dependencies are already ✅ implemented (as of Chapter Index update 2025-11-09).

### Forward References (Chapters Building on This)

- **Chapter 29** (CPython and GIL): Will cover GIL deep-dive (internals, free-threaded mode, C extensions). This chapter provides practical workaround, Ch 29 provides theoretical understanding.
- **Chapter 30+ & Parts 6-13**: Will apply asyncio patterns in agent systems, deployment, and production systems

### Cross-Part Alignment

- **Part 4 Language**: Uses "describe intent to AI", "ask your AI", "validate with AI" — NOT formal SDD terminology (that comes in Part 5)
- **Part 4 Pedagogy**: AI-Native Learning pattern (understand → explore with AI → validate → learn from errors)
- **Part 4 Tools**: Students have Claude Code / Gemini CLI (from Parts 2-3), know how to ask AI for code generation

### Technical Standards (Constitution Compliance)

✓ **Python 3.14+**: All code uses modern patterns (no deprecated APIs like `get_event_loop()`)
✓ **Type Hints**: 100% coverage on all functions (`list[str]`, `dict[str, Any]`, `X | None`)
✓ **Testing**: All code examples testable and working
✓ **Security**: No hardcoded secrets, no unsafe patterns
✓ **Accessibility**: Clear explanations, no gatekeeping language
✓ **Inclusive Examples**: Diverse scenarios, gender-neutral language

---

## Content Elements & Validations

### Code Examples (Total: 29 across 6 lessons)

**Lesson 1**: 5 examples
- Sync vs Async timing
- asyncio.run() entry point
- Basic coroutine with sleep
- I/O-bound task (simulated network)
- CPU-bound task (shows asyncio limitation)

**Lesson 2**: 6 examples
- Simple create_task() scheduling
- Multiple tasks with gather()
- TaskGroup() modern pattern
- Error handling with gather(return_exceptions=True)
- TaskGroup error propagation
- Benchmarking sequential vs concurrent

**Lesson 3**: 6 examples
- Simple asyncio.timeout() usage
- Handling TimeoutError gracefully
- Future basic usage
- Exception handling in async code
- Detecting/fixing never-awaited coroutines
- Retry logic with exponential backoff

**Lesson 4**: 6 examples
- CPU-bound function example
- Threading benchmark (no speedup)
- InterpreterPoolExecutor benchmark (4x speedup)
- Bridging CPU-bound to async
- ProcessPoolExecutor alternative
- Decision tree: I/O vs CPU vs Hybrid

**Lesson 5**: 6 examples
- Simple hybrid: fetch → process
- Batch processing pattern
- Pipeline pattern (fetch → transform → store)
- AI workload simulation (API + inference)
- Resource limiting with Semaphores
- Full hybrid system with error handling

**Lesson 6**: Capstone project (1 complete integrated system)

### "Try With AI" Prompts (Total: 24 across 6 lessons, Bloom's progression)

Each lesson: 4 prompts
- Prompt 1: Understand level
- Prompt 2: Apply level
- Prompt 3: Analyze level
- Prompt 4: Synthesize/Evaluate level

Progression across lessons:
- Lesson 1: Understand → Apply → Analyze → Evaluate
- Lesson 2: Understand → Apply → Analyze → Synthesize
- Lesson 3: Understand → Apply → Analyze → Create
- Lesson 4: Understand → Apply → Analyze → Synthesize
- Lesson 5: Understand → Apply → Analyze → Design
- Lesson 6: Understand → Apply → Analyze → Evaluate

All prompts designed for AI collaboration (Claude Code as co-reasoning partner).

### CoLearning Elements (Total: 24 across 6 lessons)

Per lesson: 4 elements (💬 🎓 🚀 ✨)
- 💬 AI Colearning Prompt (reflective question)
- 🎓 Instructor Commentary (metacognitive insight)
- 🚀 CoLearning Challenge (hands-on extension)
- ✨ Teaching Tip (debugging/exploration guidance)

Positioned throughout each lesson (not just at end).

---

## Validation & Quality Gates

### Pre-Implementation Validation Checklist

- [ ] All 6 lessons defined with complete specifications ✓
- [ ] Learning objectives measurable and Bloom's-aligned ✓
- [ ] Skills mapped to CEFR proficiency levels ✓
- [ ] Cognitive load validated (max 10 concepts/lesson) ✓
- [ ] Code examples specified with AI prompts ✓
- [ ] No deprecated Python APIs (3.14+ only) ✓
- [ ] Type hints on all functions ✓
- [ ] Error handling patterns included ✓
- [ ] Performance benchmarking included ✓
- [ ] Capstone integrates all concepts ✓
- [ ] CoLearning elements (💬🎓🚀✨) positioned ✓
- [ ] "Try With AI" sections (4 prompts each, Bloom's progression) ✓
- [ ] No forward references except brief Ch 29 GIL mention ✓
- [ ] Lesson closures are "Try With AI" ONLY (no separate closings) ✓

### Technical Review Criteria

**After implementation, verify**:

1. **Pedagogical**:
   - [ ] Learning objectives measured by code examples / exercises
   - [ ] Scaffolding graduated (easier → harder)
   - [ ] Cognitive load appropriate for tier

2. **Technical**:
   - [ ] All code runs without errors
   - [ ] Type hints complete (mypy --strict passes)
   - [ ] No deprecated patterns (Python 3.14 validated)
   - [ ] Security practices demonstrated
   - [ ] Cross-platform tested (Windows/Mac/Linux)

3. **Alignment**:
   - [ ] Matches spec exactly (all user stories addressed)
   - [ ] Follows output styles (lesson.md format)
   - [ ] Applies domain skills contextually
   - [ ] Respects complexity tier

4. **Content Quality**:
   - [ ] Clear, engaging writing
   - [ ] Diverse examples (not all same scenario)
   - [ ] Real-world relevance
   - [ ] Accessibility (no gatekeeping language)

---

## Follow-Up & Next Steps

**After lessons are implemented & validated**:

1. **Run technical review** with validation-auditor agent
2. **Gather user feedback** (beta testing if applicable)
3. **Address issues** (critical → major → minor)
4. **Prepare for Part 5**: Ch 29 (CPython and GIL) will continue GIL deep-dive
5. **Integrate into Part 5**: Chapter 30+ will show SDD formal methodology applied to agent systems (building on Ch 28 patterns)

**Risks & Mitigations**:

| Risk | Mitigation |
|------|-----------|
| Students confuse concurrency with parallelism | Explicit distinction in Lesson 1, benchmarks in Lessons 2, 4, 5 showing difference |
| Students continue using deprecated patterns | Prominent Python 3.14+ warnings, deprecation error examples |
| GIL brief intro confuses students | Limited to 2-3 sentences, explicit forward reference to Ch 29 with rationale |
| InterpreterPoolExecutor is new (Python 3.14), fewer online examples | Position as cutting-edge, use AI tools to fill gaps, provide complete working examples |
| Capstone too complex | Scaffold with skeleton code, iterative implementation with AI guidance |
| Students attempt to run async code in Jupyter | Teach `asyncio.run()` pattern which works in modern Jupyter, mention `nest_asyncio` as fallback only |
| Cognitive overload (10 concepts per lesson) | Graduated complexity across lessons, CoLearning elements paced throughout, hands-on validation via AI |

---

## File Organization & References

**Output Files** (to be created during implementation phase):

```
book-source/docs/04-Python-Fundamentals/28-asyncio/
├── readme.md                    # Chapter overview (2-3 paragraphs + learning objectives)
├── 01-asyncio-foundations.md    # Lesson 1
├── 02-concurrent-tasks.md       # Lesson 2
├── 03-advanced-patterns.md      # Lesson 3
├── 04-cpu-bound-work-gil.md     # Lesson 4
├── 05-hybrid-workloads.md       # Lesson 5
└── 06-capstone-ai-agent.md      # Lesson 6
```

**Reference Files**:
- Constitution: `.specify/memory/constitution.md` (v3.0.2)
- Chapter Index: `specs/book/chapter-index.md`
- Output Styles: `.claude/output-styles/lesson.md`
- Skills: `.claude/skills/` (contextually applied)

**Specification File**:
- `specs/001-part-4-chapter-28/spec.md` (source of all requirements above)

---

## Summary for Implementation

This plan translates the approved specification into 6 detailed, interconnected lessons:

1. **Lesson 1** teaches asyncio fundamentals with modern `asyncio.run()` pattern
2. **Lesson 2** covers concurrent task patterns with `TaskGroup()` as modern best practice
3. **Lesson 3** adds advanced patterns (timeouts, error handling, resilience)
4. **Lesson 4** solves CPU-bound limitations with `InterpreterPoolExecutor` (Python 3.14 killer feature)
5. **Lesson 5** combines I/O and CPU for realistic hybrid workloads (essential for AI apps)
6. **Lesson 6** integrates everything into a production AI agent system (capstone)

**Key Pedagogical Features**:
- ✅ CEFR proficiency mapped (B1 → B1-B2 progression)
- ✅ Cognitive load validated (max 10 concepts/lesson)
- ✅ Bloom's taxonomy progression across lessons
- ✅ CoLearning elements (💬🎓🚀✨) throughout
- ✅ "Try With AI" closures only (4 prompts per lesson, Bloom's progression)
- ✅ Python 3.14+ only (zero deprecated APIs)
- ✅ Type hints 100% coverage
- ✅ Real-world patterns (not toy examples)
- ✅ AI as co-reasoning partner (not just coding assistant)

**Ready for**: `/sp.tasks` command to generate actionable task checklist for implementation phase.

---

**Plan Version**: 1.0 | **Created**: 2025-11-09 | **Status**: Ready for Development
