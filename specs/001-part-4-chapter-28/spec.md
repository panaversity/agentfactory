# Feature Specification: Chapter 28 - Asyncio (Python 3.14+)

**Feature Branch**: `001-part-4-chapter-28`
**Created**: 2025-11-09
**Status**: Draft
**Part**: 4 (Python Fundamentals)
**Chapter Number**: 28
**Complexity Tier**: Advanced (B1-B2 proficiency)
**Python Version**: 3.14+ (modern patterns ONLY)

**Input**: Chapter 28: "Asyncio" - Teaching Python 3.14 modern async patterns with AI-Native Learning methodology for advanced learners (Chapters 24-29). Focus on asyncio fundamentals, TaskGroup structured concurrency, InterpreterPoolExecutor for true parallelism, and hybrid I/O+CPU workloads critical for AI-native applications.

---

## Overview

This chapter teaches Python 3.14+ asyncio from an AI-native development perspective, where students learn to build concurrent systems using modern patterns (`asyncio.run()`, `TaskGroup()`, `InterpreterPoolExecutor`) while leveraging AI as a co-reasoning partner. Students will understand when asyncio helps (I/O-bound), when it doesn't (CPU-bound), and how Python 3.14's `InterpreterPoolExecutor` solves the GIL limitation for hybrid AI workloads.

**Target Audience**: Students who have completed Chapters 1-27, ready for advanced Python concepts with real-world AI application patterns.

**Learning Pattern**: AI-Native Learning (describe intent → explore with AI → validate together → learn from errors)

---

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Learn Asyncio Fundamentals (Priority: P1)

As a Python developer, I want to understand what asyncio is and when to use it, so I can write concurrent programs that handle I/O-bound tasks efficiently.

**Why this priority**: Foundational understanding of asyncio architecture is prerequisite for all subsequent async patterns. Without this, students cannot reason about concurrent behavior.

**Independent Test**: Student can explain the event loop concept, write a basic async function with `async def` and `await`, and identify whether a given task is I/O-bound or CPU-bound.

**Acceptance Scenarios**:

1. **Given** a synchronous Python program making 3 API calls sequentially, **When** student converts it to async using `asyncio.run()` and `TaskGroup()`, **Then** total execution time approaches single API call time (concurrent execution achieved)
2. **Given** a CPU-intensive calculation function, **When** student wraps it in asyncio, **Then** student can explain why it doesn't achieve parallelism and describe the GIL limitation
3. **Given** an async function with `await asyncio.sleep(1)`, **When** student asks their AI to explain what happens during the sleep, **Then** student understands how the event loop switches to other tasks

---

### User Story 2 - Master Concurrent Task Patterns (Priority: P1)

As a developer building network-heavy applications, I want to run multiple tasks concurrently using modern Python 3.14 patterns, so I can maximize throughput and minimize wait time.

**Why this priority**: Core async skill - most real-world applications need concurrent I/O (API calls, database queries, file operations). This is the practical application of asyncio.

**Independent Test**: Student can write code using `asyncio.create_task()`, `asyncio.gather()`, and `asyncio.TaskGroup()`, explaining when to use each pattern and demonstrating structured concurrency principles.

**Acceptance Scenarios**:

1. **Given** a list of 10 URLs to fetch, **When** student implements concurrent fetching with `TaskGroup()`, **Then** all URLs are fetched in parallel and errors in one task cancel all others (structured concurrency)
2. **Given** multiple async operations with different completion times, **When** student uses `asyncio.gather(return_exceptions=True)`, **Then** all results (including exceptions) are collected and student explains the tradeoff vs TaskGroup
3. **Given** an async operation that might hang, **When** student applies `asyncio.timeout(5)`, **Then** operation fails gracefully after 5 seconds and student understands timeout management

---

### User Story 3 - Handle CPU-Bound Work in Async Programs (Priority: P2)

As a developer building AI applications, I want to combine I/O-bound and CPU-bound work in the same async program, so I can fetch data concurrently and process it in parallel without blocking.

**Why this priority**: Python 3.14's killer feature for AI workloads. Most AI apps need both concurrent API calls (I/O) and parallel computation (inference, data processing). This is THE advanced pattern.

**Independent Test**: Student can use `InterpreterPoolExecutor` with `loop.run_in_executor()` to achieve true parallelism for CPU-bound tasks within an async program, and can explain the GIL limitation that makes this necessary.

**Acceptance Scenarios**:

1. **Given** a CPU-intensive function (e.g., heavy calculation), **When** student runs it via `InterpreterPoolExecutor` within an async program, **Then** multiple instances run in true parallel (4 cores = 4x speedup demonstrated)
2. **Given** a hybrid workload (fetch 10 APIs + process each response), **When** student implements using `TaskGroup()` for fetching + `InterpreterPoolExecutor` for processing, **Then** I/O and CPU work overlap efficiently (total time < sequential time)
3. **Given** a simple threading example that doesn't parallelize CPU work, **When** student compares it to `InterpreterPoolExecutor`, **Then** student can articulate the GIL problem and why separate interpreters solve it

---

### User Story 4 - Build Production AI Agent System (Capstone) (Priority: P2)

As a developer building AI-native applications, I want to create a system that fetches data from multiple sources concurrently and processes it with simulated LLM inference in parallel, so I can demonstrate real-world async + parallel patterns.

**Why this priority**: Capstone integrates all chapter concepts (asyncio, TaskGroup, InterpreterPoolExecutor, error handling, timeouts) into a realistic AI workload. Demonstrates why this chapter matters for AI-native development.

**Independent Test**: Student implements a multi-service AI agent that: (1) concurrently fetches from 3+ APIs, (2) processes each response with CPU-intensive simulation, (3) handles errors gracefully, (4) completes significantly faster than sequential execution.

**Acceptance Scenarios**:

1. **Given** specifications for an AI agent system, **When** student implements concurrent API fetching with `TaskGroup()`, **Then** all APIs are called in parallel with proper error handling
2. **Given** fetched API data, **When** student processes each with simulated LLM inference using `InterpreterPoolExecutor`, **Then** processing happens in true parallel (multiple CPU cores utilized)
3. **Given** a complete AI agent system, **When** student adds timeout controls and error recovery, **Then** system degrades gracefully when services are slow or unavailable
4. **Given** the complete capstone project, **When** student asks AI to explain the execution flow, **Then** student can articulate how I/O concurrency and CPU parallelism combine for optimal performance

---

### Edge Cases

- What happens when an async function never awaits (becomes blocking)?
- How does the system behave when `TaskGroup` encounters an exception in one task?
- What occurs when `InterpreterPoolExecutor` runs out of worker interpreters?
- How does timeout interact with nested async calls?
- What happens if student tries to `await` a non-awaitable object?
- How does garbage collection affect running tasks?
- What occurs when mixing `asyncio.gather()` with `TaskGroup()` in the same program?

---

## Requirements *(mandatory)*

### Functional Requirements

**Foundational Concepts (Lessons 1-2)**:

- **FR-001**: Chapter MUST teach event loop concept without requiring students to manage it manually (Python 3.14 `asyncio.run()` abstracts this)
- **FR-002**: Chapter MUST demonstrate the difference between synchronous and asynchronous execution using measurable time comparisons
- **FR-003**: Students MUST understand coroutines (`async def`) and the `await` keyword through both explanation and hands-on code
- **FR-004**: Chapter MUST explain when asyncio is appropriate (I/O-bound) vs when it's not (CPU-bound) with concrete examples
- **FR-005**: All code examples MUST use Python 3.14+ patterns (zero deprecated APIs like `get_event_loop()`)

**Concurrent Task Patterns (Lessons 2-3)**:

- **FR-006**: Chapter MUST teach `asyncio.create_task()` for scheduling coroutines
- **FR-007**: Chapter MUST demonstrate `asyncio.gather()` for collecting multiple async results
- **FR-008**: Chapter MUST introduce `asyncio.TaskGroup()` as the modern structured concurrency pattern
- **FR-009**: Students MUST learn the difference between `gather()` (continue on error with `return_exceptions`) and `TaskGroup()` (cancel all on first error)
- **FR-010**: Chapter MUST teach `asyncio.timeout()` context manager for timeout control
- **FR-011**: Chapter MUST explain Futures at a conceptual level (awaitable objects, result placeholders)

**CPU-Bound Work & GIL (Lesson 4)**:

- **FR-012**: Chapter MUST briefly explain the GIL (2-3 sentences: "Python threads share one GIL, preventing true parallelism for CPU-bound code")
- **FR-013**: Chapter MUST defer deep GIL exploration to Chapter 29 (explicit forward reference with rationale)
- **FR-014**: Chapter MUST introduce `InterpreterPoolExecutor` as Python 3.14's solution for true parallelism
- **FR-015**: Students MUST understand that `InterpreterPoolExecutor` creates separate interpreters with separate GILs (no sharing = true parallel)
- **FR-016**: Chapter MUST demonstrate measurable performance difference (4 cores = ~4x speedup) between threading (no speedup) and `InterpreterPoolExecutor` (true parallel)

**Hybrid Workloads (Lesson 5)**:

- **FR-017**: Chapter MUST teach the pattern: `TaskGroup()` for I/O concurrency + `InterpreterPoolExecutor` for CPU parallelism
- **FR-018**: Students MUST implement a hybrid workload demonstrating both I/O and CPU overlap
- **FR-019**: Chapter MUST position this pattern as essential for AI-native applications (API calls + inference)

**Capstone Project (Lesson 6)**:

- **FR-020**: Capstone MUST integrate: concurrent API fetching, CPU-parallel processing, error handling, timeout controls
- **FR-021**: Capstone MUST simulate real AI workload (fetch data from 3+ sources, process with simulated LLM inference)
- **FR-022**: Students MUST demonstrate measurable performance improvement over sequential execution
- **FR-023**: Capstone MUST include error scenarios (API timeout, processing failure) and graceful degradation

**AI-Native Learning Integration**:

- **FR-024**: Every lesson MUST include CoLearning elements (💬 AI Colearning Prompt, 🎓 Instructor Commentary, 🚀 CoLearning Challenge, ✨ Teaching Tip) positioned throughout content (not just at end)
- **FR-025**: Every lesson MUST end with "Try With AI" section ONLY (4 prompts, Bloom's progression: Remember → Understand → Apply → Analyze/Synthesize)
- **FR-026**: Chapter MUST use Part 4 appropriate language ("describe intent", "ask your AI", "validate together") NOT formal SDD terminology ("write specification")
- **FR-027**: All code examples MUST include modern type hints (`list[str]`, `dict[str, Any]`, `X | None`)
- **FR-028**: Chapter MUST teach troubleshooting through AI partnership ("Ask AI: 'Why did this fail?'") not exhaustive error catalogs

### Key Entities *(chapter-specific context)*

This is an educational chapter, not a data-driven feature. Key "entities" are conceptual learning objectives:

- **Event Loop**: The core asyncio abstraction that manages coroutine execution (students understand conceptually, don't manage directly)
- **Coroutine**: An `async def` function that can be paused and resumed (students write these using `async`/`await`)
- **Task**: A scheduled coroutine managed by the event loop (students create with `asyncio.create_task()`)
- **TaskGroup**: A context manager for structured concurrency (Python 3.14+ modern pattern students should prefer)
- **Future**: A placeholder for a result not yet available (students understand conceptually for debugging, rarely create manually)
- **InterpreterPoolExecutor**: A pool of separate Python interpreters for true parallelism (students use with `loop.run_in_executor()`)

---

## Success Criteria *(mandatory)*

### Measurable Outcomes

**Learning Comprehension**:

- **SC-001**: Students can correctly identify whether a given task is I/O-bound or CPU-bound in 90%+ of cases (tested via "Try With AI" exercises)
- **SC-002**: Students can convert a synchronous multi-step program to async and achieve measurable concurrency (total time ≈ longest single operation, not sum of all)
- **SC-003**: Students can explain the event loop concept and `await` behavior when asked by AI in their own words (comprehension, not memorization)

**Code Quality**:

- **SC-004**: 100% of student code examples use Python 3.14+ patterns (zero `get_event_loop()`, zero deprecated APIs)
- **SC-005**: Students correctly apply `TaskGroup()` for structured concurrency (failing fast when appropriate) vs `gather()` (collecting all results/exceptions)
- **SC-006**: All student async code includes proper type hints and handles errors explicitly (no silent failures)

**Performance Understanding**:

- **SC-007**: Students can demonstrate measurable speedup using `InterpreterPoolExecutor` for CPU-bound work (benchmark showing 3-4x improvement on 4-core machine)
- **SC-008**: Students build a hybrid I/O+CPU system that completes faster than sequential execution by 40%+ (measured in capstone)
- **SC-009**: Students can articulate the GIL limitation and why `InterpreterPoolExecutor` solves it (brief explanation, defer details to Ch 29)

**AI Partnership**:

- **SC-010**: Students successfully debug async errors using AI as co-reasoning partner (e.g., "Ask AI: 'Why am I getting RuntimeError: This event loop is already running?'")
- **SC-011**: Students can request AI to generate async code from clear intent descriptions and validate the output (specification-first thinking in practice)
- **SC-012**: Students use AI to explore edge cases and deepen understanding (not just copy code but ask "why" and "what if")

**Capstone Completion**:

- **SC-013**: 85%+ of students successfully complete the AI Agent System capstone demonstrating hybrid I/O+CPU patterns
- **SC-014**: Capstone systems handle at least 3 concurrent API fetches + parallel processing with proper error recovery
- **SC-015**: Students can explain their capstone architecture to AI and receive constructive feedback (architecture review via AI dialogue)

---

## Content Outline *(6 lessons, advanced tier)*

### Lesson 1: Asyncio Foundations - Event Loop and Coroutines

**Learning Objectives**:
- Understand what asyncio is and why it exists (cooperative multitasking for I/O)
- Explain the event loop conceptually without managing it manually
- Write and run basic async functions using `async def`, `await`, and `asyncio.run()`
- Identify I/O-bound vs CPU-bound tasks

**Key Concepts** (Max 10: cognitive load validated):
1. **Event loop** (what it does, how `asyncio.run()` manages it)
2. **Coroutines** (`async def` syntax, how they differ from regular functions)
3. **await keyword** (pausing points, cooperative yielding)
4. **asyncio.run()** (Python 3.14+ entry point, automatic loop management)
5. **I/O-bound vs CPU-bound** (when asyncio helps, when it doesn't)
6. **Concurrency vs Parallelism** (single-threaded cooperative vs multi-core execution)
7. **Blocking vs Non-blocking** (what blocks the event loop)

**Code Examples**:
1. Sync vs Async comparison (coffee/pastry modernized for Python 3.14)
2. Simple `asyncio.run()` entry point
3. Basic coroutine with `await asyncio.sleep()`
4. I/O-bound task (simulated network call)
5. CPU-bound task (showing asyncio doesn't help here - preview for Lesson 4)

**CoLearning Elements** (positioned throughout):
- 💬 Prompt: "Explain how the event loop knows when to switch between tasks"
- 🎓 Commentary: "You don't memorize asyncio syntax—you understand WHEN to make things concurrent and WHY"
- 🚀 Challenge: "Ask AI to generate async code fetching 3 URLs and explain the execution order"
- ✨ Tip: "Use your AI to explore: 'What happens if I forget await?'"

---

### Lesson 2: Concurrent Tasks - create_task(), gather(), TaskGroup()

**Learning Objectives**:
- Schedule multiple coroutines concurrently with `asyncio.create_task()`
- Collect results from multiple tasks using `asyncio.gather()`
- Apply `asyncio.TaskGroup()` for structured concurrency (Python 3.14+ best practice)
- Understand when to use `gather()` vs `TaskGroup()`

**Key Concepts** (Max 10):
1. **asyncio.create_task()** (scheduling coroutines to run concurrently)
2. **asyncio.gather()** (collecting multiple results, `return_exceptions` parameter)
3. **asyncio.TaskGroup()** (structured concurrency context manager)
4. **Structured concurrency** (automatic cleanup, fail-fast error propagation)
5. **Task lifecycle** (pending → running → done/cancelled)
6. **Error handling in concurrent tasks** (how exceptions propagate)
7. **Pattern comparison** (gather vs TaskGroup tradeoffs)

**Code Examples**:
1. Creating multiple tasks with `create_task()`
2. Using `gather()` to run tasks concurrently
3. `TaskGroup()` structured concurrency example
4. Error propagation in `TaskGroup` (cancel all on first error)
5. `gather(return_exceptions=True)` collecting all results/errors
6. Comparing execution time: sequential vs concurrent

**CoLearning Elements**:
- 💬 Prompt: "Why does TaskGroup cancel all tasks when one fails? When is this useful?"
- 🎓 Commentary: "Structured concurrency is like RAII for async code—cleanup happens automatically"
- 🚀 Challenge: "Implement concurrent API fetching with both gather() and TaskGroup(), explain when you'd choose each"
- ✨ Tip: "Ask AI: 'Show me a scenario where gather() is better than TaskGroup()'"

---

### Lesson 3: Advanced Patterns - Timeouts, Futures, Error Handling

**Learning Objectives**:
- Apply timeout controls using `asyncio.timeout()` context manager
- Understand Futures as awaitable result placeholders
- Handle errors gracefully in async code
- Debug common async pitfalls with AI assistance

**Key Concepts** (Max 10):
1. **asyncio.timeout()** (context manager for timeout control, Python 3.11+)
2. **Futures** (conceptual understanding as awaitable objects)
3. **TimeoutError** (handling timeout exceptions)
4. **CancelledError** (when tasks are cancelled)
5. **Exception handling in async** (try/except with await)
6. **Never-awaited coroutines** (RuntimeWarning detection)
7. **Common async mistakes** (blocking the loop, missing await)

**Code Examples**:
1. Using `asyncio.timeout()` context manager
2. Handling TimeoutError gracefully
3. Future basic usage (rarely created manually, understanding for debugging)
4. Exception handling patterns in async code
5. Detecting and fixing never-awaited coroutines
6. Debugging async errors with AI (screenshot of asking AI about RuntimeWarning)

**CoLearning Elements**:
- 💬 Prompt: "What's the difference between asyncio.timeout() and asyncio.wait_for()?"
- 🎓 Commentary: "Timeouts aren't just for network calls—they're your defense against infinite waits"
- 🚀 Challenge: "Build a resilient async function that retries on failure with exponential backoff"
- ✨ Tip: "When you get a RuntimeWarning, ask AI: 'Why did my coroutine never get awaited?'"

---

### Lesson 4: CPU-Bound Work - GIL and InterpreterPoolExecutor

**Learning Objectives**:
- Understand the GIL limitation briefly (defer deep-dive to Ch 29)
- Apply `InterpreterPoolExecutor` for true parallelism in async programs
- Measure performance difference between threading and InterpreterPoolExecutor
- Recognize when to use ProcessPoolExecutor vs InterpreterPoolExecutor

**Key Concepts** (Max 10):
1. **GIL (Global Interpreter Lock)** - brief intro (2-3 sentences, full explanation in Ch 29)
2. **Why threading doesn't parallelize CPU-bound Python** (GIL contention)
3. **InterpreterPoolExecutor** (Python 3.14 feature, separate interpreters = separate GILs)
4. **loop.run_in_executor()** (bridging sync functions into async code)
5. **True parallelism** (multiple CPU cores utilized)
6. **ProcessPoolExecutor** (alternative, higher overhead, when to use)
7. **Performance measurement** (timing CPU-bound work with/without parallelism)

**Code Examples**:
1. CPU-bound function (heavy calculation)
2. Demonstrating threading doesn't parallelize CPU work (benchmark)
3. Using `InterpreterPoolExecutor` for true parallelism (benchmark showing speedup)
4. Bridging CPU-bound work into async with `run_in_executor()`
5. Comparing ProcessPoolExecutor vs InterpreterPoolExecutor overhead
6. Decision tree: I/O-bound → asyncio, CPU-bound → InterpreterPoolExecutor

**CoLearning Elements**:
- 💬 Prompt: "Why does Python have a GIL? What problem was it solving?"
- 🎓 Commentary: "The GIL isn't a bug—it's a design tradeoff. Python 3.14 gives you tools to work around it"
- 🚀 Challenge: "Benchmark a CPU-intensive task with threading, InterpreterPoolExecutor, and ProcessPoolExecutor—explain the results"
- ✨ Tip: "Ask AI: 'When should I use InterpreterPoolExecutor vs just running a subprocess?'"

---

### Lesson 5: Hybrid Workloads - Combining I/O and CPU

**Learning Objectives**:
- Combine asyncio (I/O concurrency) with InterpreterPoolExecutor (CPU parallelism)
- Build systems where I/O and CPU work overlap efficiently
- Apply hybrid patterns to AI-native applications
- Architect for real-world performance optimization

**Key Concepts** (Max 10):
1. **Hybrid workload pattern** (asyncio + InterpreterPoolExecutor together)
2. **I/O + CPU overlap** (fetch data while processing previous batch)
3. **AI-native workload characteristics** (API calls + inference)
4. **Executor integration** (using `loop.run_in_executor()` within async functions)
5. **Resource management** (limiting concurrent I/O and CPU work)
6. **Performance optimization** (pipelining, batching)
7. **Real-world patterns** (data fetching + processing pipelines)

**Code Examples**:
1. Simple hybrid: fetch API → process with InterpreterPoolExecutor
2. Batch processing pattern (fetch N items, process in parallel)
3. Pipeline pattern (fetch → process → store, overlapping stages)
4. AI workload simulation (concurrent API calls + parallel "inference")
5. Resource limiting (max concurrent I/O + max CPU workers)
6. Full hybrid system with error handling and timeouts

**CoLearning Elements**:
- 💬 Prompt: "How would you optimize a system that fetches 1000 items and processes each? What's the bottleneck?"
- 🎓 Commentary: "AI apps are inherently hybrid—API calls are I/O, inference is CPU. You need both patterns"
- 🚀 Challenge: "Design a data processing pipeline: fetch from API → transform (CPU) → store in DB (I/O)"
- ✨ Tip: "Ask AI: 'What's the optimal number of workers for my machine?' (Hint: depends on cores and I/O latency)"

---

### Lesson 6: Capstone - AI Agent System (Multi-Service Concurrent Fetching + Parallel Inference)

**Learning Objectives**:
- Integrate all chapter concepts into a production-style AI agent
- Build a system fetching from multiple services concurrently
- Process responses with simulated LLM inference in parallel
- Handle errors, timeouts, and graceful degradation

**Capstone Specification**:

**System Description**: Build an AI Agent that queries multiple information sources (weather API, news API, knowledge base) concurrently, processes each response with simulated LLM inference (CPU-intensive text analysis), and aggregates results into a unified response—all while handling failures gracefully.

**Requirements**:
1. **Concurrent Fetching**: Use `TaskGroup()` to fetch from 3+ sources in parallel
2. **Parallel Processing**: Use `InterpreterPoolExecutor` to simulate LLM inference (heavy string processing)
3. **Error Handling**: If one source times out, continue with others (partial success)
4. **Timeout Control**: Each API call has 5s timeout, overall system has 15s timeout
5. **Graceful Degradation**: System provides best-effort response even if 1-2 sources fail
6. **Performance**: Demonstrate hybrid execution is 2-3x faster than sequential

**Code Structure**:
```python
# Skeleton provided, students implement details with AI guidance
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
- All 3 sources fetched concurrently (measured: ~5s total, not 15s sequential)
- Processing happens in parallel (CPU usage spikes across multiple cores)
- System handles partial failures (1 source down → still returns results from others)
- Total execution time < 15s with timeouts applied
- Code includes type hints, error handling, and logging

**CoLearning Elements**:
- 💬 Prompt: "Explain how this architecture handles the case where 2/3 sources timeout"
- 🎓 Commentary: "This is what production AI systems look like—resilient, concurrent, and optimized"
- 🚀 Challenge: "Add caching to avoid re-fetching recent queries"
- ✨ Tip: "Ask AI to review your error handling strategy and suggest improvements"

---

## Assumptions

**Python Environment**:
- Students have Python 3.14+ installed (assumed from Chapter 12: UV package manager setup)
- Students understand basic Python syntax, functions, type hints (Chapters 13-27)
- Students have completed Chapters 20 (Functions), 21 (Exception Handling), 24-25 (OOP) for prerequisite concepts

**Prior Knowledge**:
- Students understand synchronous programming flow
- Students are comfortable with type hints (`list[str]`, `dict[str, Any]`)
- Students have used AI tools (Claude Code / Gemini CLI) for previous chapters (Part 1-3 intro)
- Students understand the validation-first mindset from Chapters 1-11 (AIDD principles)

**Technical Defaults**:
- Examples use `httpx` for async HTTP (installed via UV)
- Simulated workloads use `time.sleep()` wrapped in executors (no real AI APIs required for learning)
- Performance benchmarks assume 4-core machine (results scale with cores)
- All code runs in modern Python 3.14+ environments (no backward compatibility needed)

**Pedagogical Approach**:
- CoLearning elements (💬🎓🚀✨) appear throughout lessons (not just at end)
- "Try With AI" is the ONLY lesson closure (4 prompts, Bloom's progression)
- NO summaries, checklists, or "what's next" sections after "Try With AI"
- Part 4 language used ("describe intent", "ask your AI") NOT formal SDD ("write specification")

---

## Out of Scope

**Not Covered in This Chapter**:
- ❌ Deep GIL internals (deferred to Chapter 29: CPython and GIL)
- ❌ Free-threaded Python mode (experimental in 3.13, mention as future direction)
- ❌ Advanced event loop customization (low-level event loop API)
- ❌ Asyncio transports and protocols (low-level networking)
- ❌ Production deployment patterns (Docker, Kubernetes - deferred to Parts 11-13)
- ❌ Real LLM API integration (using openai/anthropic SDKs - covered in Part 6)
- ❌ Asynchronous generators and context managers (advanced Python, not critical for intro)
- ❌ Event loop policy customization (rarely needed, deprecated warnings in 3.14)
- ❌ Subprocesses with asyncio (subprocess management is advanced topic)

**Explicitly NOT Using (Deprecated/Legacy)**:
- ❌ `asyncio.get_event_loop()` (deprecated in 3.14, use `asyncio.run()`)
- ❌ Manual event loop management (`loop.run_until_complete()`, `loop.run_forever()`)
- ❌ Child watcher APIs (removed in 3.14: `MultiLoopChildWatcher`, etc.)
- ❌ `nest_asyncio` (Jupyter-specific workaround, not core asyncio knowledge)

**Forward References** (Taught in Later Chapters):
- Chapter 29 will cover GIL deep-dive (how it works, free-threaded mode, C extensions)
- Chapter 30+ will teach formal Specification-Driven Development (this chapter uses informal AI-Native Learning)
- Part 6 will cover OpenAI Agents SDK and real agentic patterns (this chapter simulates AI workloads)
- Part 10 will cover production realtime systems (WebSockets, streaming, voice)

---

## Dependencies

**Prerequisites** (Required Chapters):
- **Chapter 20**: Module and Functions (understanding function composition, imports)
- **Chapter 21**: Exception Handling (try/except, error recovery patterns)
- **Chapter 24-25**: OOP Part I & II (classes for structuring capstone code)
- **Chapters 1-11**: AIDD mindset (validation-first, AI as co-reasoning partner)

**Python Knowledge** (Assumed from Chapters 13-27):
- Variables, data types, type hints
- Control flow (if/else, loops)
- Functions (defining, calling, parameters, return values)
- Exception handling (try/except/finally)
- Classes and objects (basic OOP)
- Modules and imports

**Technical Dependencies**:
- Python 3.14+ (installed via UV package manager from Chapter 12)
- `httpx` library (async HTTP client, install: `uv add httpx`)
- Standard library: `asyncio`, `time`, `concurrent.futures`

**AI Tools** (From Part 2):
- Claude Code or Gemini CLI installed and configured (Chapters 5-6)
- Students comfortable asking AI for explanations, code generation, debugging help

**No Forward Dependencies**: This chapter does NOT require knowledge from Chapters 29+ (GIL deep-dive deferred, SDD taught later)

---

## Risks & Mitigations

**Risk 1**: Students confuse asyncio concurrency with parallelism
**Mitigation**: Explicitly teach the distinction in Lesson 1, reinforce with performance benchmarks showing asyncio doesn't speed up CPU-bound work

**Risk 2**: Students continue using deprecated patterns (`get_event_loop()`) from online examples
**Mitigation**: Prominently feature "Python 3.14+ Only" warnings, show deprecation errors, explain why old patterns are outdated

**Risk 3**: GIL brief intro confuses students without deep explanation
**Mitigation**: Limit to 2-3 sentences in Lesson 4, explicit forward reference to Chapter 29 with rationale ("We'll explore GIL internals deeply in the next chapter")

**Risk 4**: `InterpreterPoolExecutor` is new (Python 3.14), fewer online examples
**Mitigation**: Position as cutting-edge, use MCP to pull official Python docs, rely on AI tools to fill knowledge gaps

**Risk 5**: Capstone may be too complex (combines many concepts)
**Mitigation**: Provide skeleton code structure, students implement with AI guidance (not from scratch), validate incrementally

**Risk 6**: Students try to run async code in Jupyter notebooks and hit event loop conflicts
**Mitigation**: Chapter teaches `asyncio.run()` pattern which works correctly in modern Jupyter; mention `nest_asyncio` as workaround if needed but DON'T teach it as core pattern

**Risk 7**: Cognitive overload (10 concepts per lesson at B1-B2 proficiency)
**Mitigation**: Graduated complexity across lessons, CoLearning elements paced throughout, hands-on practice with immediate validation via AI

---

## Open Questions

None at this time. All critical decisions resolved in PHASE 0 user answers:
- ✅ Python 3.14+ only (no legacy content)
- ✅ Brief GIL intro with forward reference to Ch 29
- ✅ Capstone: AI Agent System (multi-service fetch + parallel inference)
