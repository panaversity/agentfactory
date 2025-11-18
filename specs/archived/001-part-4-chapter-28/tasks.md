---
description: "Implementation tasks for Chapter 28: Asyncio"
feature: "001-part-4-chapter-28"
created: "2025-11-09"
---

# Tasks: Chapter 28 - Asyncio

**Input**: Design documents from `/specs/001-part-4-chapter-28/`
**Prerequisites**: plan.md (complete), spec.md (complete, 4 user stories)

**Tests**: Not applicable for educational content (students validate learning through exercises)

**Organization**: Tasks organized by user story (lesson) to enable lesson-by-lesson implementation.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different lessons, no dependencies)
- **[Story]**: Which user story/lesson this task belongs to (US1=Lesson 1, US2=Lesson 2, etc.)
- Include exact file paths in descriptions

## Path Conventions

**Chapter content location**: `book-source/docs/04-Part-4-Python-Fundamentals/28-asyncio/`

**Lessons**:
- Lesson 1: `01-asyncio-foundations.md`
- Lesson 2: `02-concurrent-tasks.md`
- Lesson 3: `03-advanced-patterns.md`
- Lesson 4: `04-cpu-bound-work-gil.md`
- Lesson 5: `05-hybrid-workloads.md`
- Lesson 6: `06-capstone-ai-agent.md`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Prepare chapter directory structure and validate prerequisites

- [ ] T001 Create chapter directory: `book-source/docs/04-Part-4-Python-Fundamentals/28-asyncio/`
- [ ] T002 [P] Validate Python 3.14+ installed and accessible via `python3 --version`
- [ ] T003 [P] Install required dependencies: `uv pip install httpx asyncio-types`
- [ ] T004 [P] Validate prerequisite chapters completed (Ch 20, 21, 24-27)
- [ ] T005 Create chapter intro.md with navigation and prerequisites
- [ ] T006 [P] Prepare code examples directory: `book-source/docs/04-Part-4-Python-Fundamentals/28-asyncio/code-examples/`

**Checkpoint**: Chapter structure ready, all prerequisites validated

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Create shared resources needed by all lessons

**⚠️ CRITICAL**: No lesson implementation can begin until this phase is complete

- [ ] T007 Document Python 3.14+ asyncio deprecations list (reference for all lessons): no `get_event_loop()`, no `nest_asyncio`, no child watchers
- [ ] T008 [P] Create shared code validation script: `check_python_314_compliance.py` (scans for deprecated APIs)
- [ ] T009 [P] Create benchmark utility: `benchmark_helpers.py` (timing functions used across lessons)
- [ ] T010 [P] Document AI-Native Learning pedagogy guidelines for chapter (CoLearning elements, Try With AI format)
- [ ] T011 Validate chapter-index.md references Chapter 28 correctly (title: "Asyncio", Part 4, status: planned → in-progress)

**Checkpoint**: Foundation ready - lesson implementation can begin in parallel

---

## Phase 3: User Story 1 - Learn Asyncio Foundations (Priority: P1) 🎯 FIRST LESSON

**Goal**: Students understand event loop, coroutines, `asyncio.run()`, and I/O vs CPU distinction

**Independent Test**: Student can write async code with `asyncio.run()`, explain event loop conceptually, and identify I/O-bound vs CPU-bound tasks

### Implementation for User Story 1 (Lesson 1)

- [ ] T012 [P] [US1] Create lesson file: `book-source/docs/04-Part-4-Python-Fundamentals/28-asyncio/01-asyncio-foundations.md`
- [ ] T013 [US1] Write lesson YAML frontmatter (title, sidebar_position: 1, proficiency: B1, skills metadata)
- [ ] T014 [US1] Write Hook section (3-5 min): Real-world problem - 10 sequential API calls vs concurrent
- [ ] T015 [US1] Write Core 1: Event Loop Basics (20 min) - conceptual explanation, `asyncio.run()` abstraction, task switching diagram
- [ ] T016 [P] [US1] Create Code Example 1: Sync vs Async Timing Comparison (code-examples/lesson1_example1_sync_vs_async.py)
- [ ] T017 [P] [US1] Create Code Example 2: Basic asyncio.run() Entry Point (code-examples/lesson1_example2_basic_entry.py)
- [ ] T018 [US1] Write Core 2: Coroutines (20 min) - `async def` vs `def`, how `await` works, cooperation
- [ ] T019 [P] [US1] Create Code Example 3: Coroutine with asyncio.sleep() (code-examples/lesson1_example3_coroutine_sleep.py)
- [ ] T020 [US1] Write Core 3: I/O vs CPU (15 min) - when asyncio helps, when it doesn't, concrete examples
- [ ] T021 [P] [US1] Create Code Example 4: I/O-Bound Task Simulation (code-examples/lesson1_example4_io_bound.py)
- [ ] T022 [P] [US1] Create Code Example 5: CPU-Bound Task (shows asyncio doesn't help) (code-examples/lesson1_example5_cpu_bound.py)
- [ ] T023 [US1] Add CoLearning elements throughout (💬 Prompt after Event Loop, 🎓 Commentary after Concurrency, 🚀 Challenge after Simple Example, ✨ Tip after Common Mistakes)
- [ ] T024 [US1] Write Common Mistakes section (10 min) - forgetting `await`, blocking the loop
- [ ] T025 [US1] Write "Try With AI" section ONLY (4 prompts: Understand → Apply → Analyze → Evaluate)
- [ ] T026 [US1] Validate all code examples: Python 3.14+ patterns, full type hints, no deprecated APIs
- [ ] T027 [US1] Run compliance check: `python check_python_314_compliance.py 01-asyncio-foundations.md`
- [ ] T028 [US1] Validate cognitive load: Max 7 concepts (event loop, coroutines, await, asyncio.run, I/O vs CPU, concurrency vs parallelism, blocking)

**Checkpoint**: Lesson 1 complete, tested, validated - students can write basic async code

---

## Phase 4: User Story 2 - Master Concurrent Task Patterns (Priority: P1)

**Goal**: Students write structured concurrent code using `create_task()`, `gather()`, and modern `TaskGroup()`

**Independent Test**: Student can schedule multiple tasks, collect results with `gather()` and `TaskGroup()`, and explain tradeoffs

### Implementation for User Story 2 (Lesson 2)

- [ ] T029 [P] [US2] Create lesson file: `book-source/docs/04-Part-4-Python-Fundamentals/28-asyncio/02-concurrent-tasks.md`
- [ ] T030 [US2] Write lesson YAML frontmatter (title, sidebar_position: 2, proficiency: B1, skills metadata)
- [ ] T031 [US2] Write Hook section (3-5 min): Problem - running 10 tasks concurrently without nested callbacks
- [ ] T032 [US2] Write Core 1: create_task() Scheduling (20 min) - creating tasks, pending state, concurrent execution
- [ ] T033 [P] [US2] Create Code Example 1: Simple create_task() Scheduling (code-examples/lesson2_example1_create_task.py)
- [ ] T034 [US2] Write Core 2: gather() Pattern (20 min) - collecting results, `return_exceptions`, resilience
- [ ] T035 [P] [US2] Create Code Example 2: Multiple Tasks with gather() (code-examples/lesson2_example2_gather.py)
- [ ] T036 [US2] Write Core 3: TaskGroup() Best Practice (20 min) - Python 3.11+ pattern, automatic cancellation, why preferred
- [ ] T037 [P] [US2] Create Code Example 3: TaskGroup() Modern Pattern (code-examples/lesson2_example3_taskgroup.py)
- [ ] T038 [P] [US2] Create Code Example 4: gather() with Error Handling (code-examples/lesson2_example4_gather_errors.py)
- [ ] T039 [P] [US2] Create Code Example 5: TaskGroup() Error Propagation (code-examples/lesson2_example5_taskgroup_errors.py)
- [ ] T040 [P] [US2] Create Code Example 6: Performance Benchmark (code-examples/lesson2_example6_benchmark.py)
- [ ] T041 [US2] Write Core 4: Error Handling Strategies (15 min) - task failures, exception propagation
- [ ] T042 [US2] Write Core 5: Performance Comparisons (10 min) - benchmarking sequential vs concurrent
- [ ] T043 [US2] Add CoLearning elements throughout (4 total: 💬🎓🚀✨)
- [ ] T044 [US2] Write "Try With AI" section ONLY (4 prompts: Understand → Apply → Analyze → Evaluate)
- [ ] T045 [US2] Validate all code examples: Python 3.14+ patterns, full type hints
- [ ] T046 [US2] Run compliance check: `python check_python_314_compliance.py 02-concurrent-tasks.md`
- [ ] T047 [US2] Validate cognitive load: Max 7 concepts (create_task, gather, TaskGroup, structured concurrency, task lifecycle, error handling, patterns)

**Checkpoint**: Lesson 2 complete - students write structured concurrent code

---

## Phase 5: User Story 3 - Handle Advanced Patterns (Priority: P2)

**Goal**: Students apply timeouts, understand Futures, handle errors gracefully, build resilience

**Independent Test**: Student implements timeout controls, retry logic, and explains Future concept

### Implementation for User Story 3 (Lesson 3)

- [ ] T048 [P] [US3] Create lesson file: `book-source/docs/04-Part-4-Python-Fundamentals/28-asyncio/03-advanced-patterns.md`
- [ ] T049 [US3] Write lesson YAML frontmatter (title, sidebar_position: 3, proficiency: B1-B2, skills metadata)
- [ ] T050 [US3] Write Hook section (3-5 min): Production async requires timeouts and error recovery
- [ ] T051 [US3] Write Core 1: asyncio.timeout() Pattern (20 min) - context manager, TimeoutError handling
- [ ] T052 [P] [US3] Create Code Example 1: Basic Timeout Pattern (code-examples/lesson3_example1_timeout.py)
- [ ] T053 [P] [US3] Create Code Example 2: Timeout with Error Handling (code-examples/lesson3_example2_timeout_errors.py)
- [ ] T054 [US3] Write Core 2: Futures Concept (15 min) - awaitable result placeholders, when you encounter them
- [ ] T055 [P] [US3] Create Code Example 3: Understanding Futures (code-examples/lesson3_example3_futures.py)
- [ ] T056 [US3] Write Core 3: Exception Handling in Async (20 min) - try/except with await, exception propagation
- [ ] T057 [P] [US3] Create Code Example 4: Async Exception Handling (code-examples/lesson3_example4_exceptions.py)
- [ ] T058 [US3] Write Core 4: Debugging Never-Awaited Coroutines (15 min) - common warnings, AI-assisted debugging
- [ ] T059 [P] [US3] Create Code Example 5: Debugging Never-Awaited (code-examples/lesson3_example5_debugging.py)
- [ ] T060 [US3] Write Core 5: Resilience Patterns (15 min) - retry logic, exponential backoff
- [ ] T061 [P] [US3] Create Code Example 6: Retry with Exponential Backoff (code-examples/lesson3_example6_retry.py)
- [ ] T062 [US3] Add CoLearning elements throughout (4 total: 💬🎓🚀✨)
- [ ] T063 [US3] Write "Try With AI" section ONLY (4 prompts: Understand → Apply → Analyze → Create)
- [ ] T064 [US3] Validate all code examples: Python 3.14+ patterns, full type hints
- [ ] T065 [US3] Run compliance check: `python check_python_314_compliance.py 03-advanced-patterns.md`
- [ ] T066 [US3] Validate cognitive load: Max 9 concepts (timeout, TimeoutError, Futures, exception handling, retry, exponential backoff, resilience, debugging, warning messages)

**Checkpoint**: Lesson 3 complete - students build resilient async code

---

## Phase 6: User Story 4 - Handle CPU-Bound Work (Priority: P2)

**Goal**: Students understand GIL briefly, use `InterpreterPoolExecutor` for true parallelism, bridge to async

**Independent Test**: Student demonstrates 4x speedup with `InterpreterPoolExecutor` on 4-core machine, explains GIL limitation

### Implementation for User Story 4 (Lesson 4)

- [ ] T067 [P] [US4] Create lesson file: `book-source/docs/04-Part-4-Python-Fundamentals/28-asyncio/04-cpu-bound-work-gil.md`
- [ ] T068 [US4] Write lesson YAML frontmatter (title, sidebar_position: 4, proficiency: B1-B2, skills metadata)
- [ ] T069 [US4] Write Hook section (3-5 min): Asyncio doesn't help CPU-bound work - why?
- [ ] T070 [US4] Write Core 1: Brief GIL Explanation (5 min, 2-3 sentences) - Python threads share one GIL, no true CPU parallelism
- [ ] T071 [US4] Add explicit forward reference to Chapter 29 (deep GIL internals, free-threaded mode)
- [ ] T072 [US4] Write Core 2: InterpreterPoolExecutor Solution (20 min) - separate interpreters, separate GILs, Python 3.14+
- [ ] T073 [P] [US4] Create Code Example 1: CPU-Bound Function (code-examples/lesson4_example1_cpu_function.py)
- [ ] T074 [P] [US4] Create Code Example 2: Threading Benchmark (no speedup) (code-examples/lesson4_example2_threading.py)
- [ ] T075 [P] [US4] Create Code Example 3: InterpreterPoolExecutor Benchmark (4x speedup) (code-examples/lesson4_example3_interpreter_pool.py)
- [ ] T076 [US4] Write Core 3: Bridging CPU Work to Async (15 min) - `loop.run_in_executor()` pattern
- [ ] T077 [P] [US4] Create Code Example 4: Async Executor Bridging (code-examples/lesson4_example4_executor_bridge.py)
- [ ] T078 [US4] Write Core 4: ProcessPoolExecutor Alternative (10 min) - when to use processes vs interpreters
- [ ] T079 [P] [US4] Create Code Example 5: ProcessPoolExecutor Comparison (code-examples/lesson4_example5_process_pool.py)
- [ ] T080 [P] [US4] Create Code Example 6: Decision Tree (when to use what) (code-examples/lesson4_example6_decision_tree.py)
- [ ] T081 [US4] Add CoLearning elements throughout (4 total: 💬🎓🚀✨)
- [ ] T082 [US4] Write "Try With AI" section ONLY (4 prompts: Understand → Apply → Analyze → Create)
- [ ] T083 [US4] Validate all code examples: Python 3.14+ patterns, full type hints, performance benchmarks included
- [ ] T084 [US4] Run compliance check: `python check_python_314_compliance.py 04-cpu-bound-work-gil.md`
- [ ] T085 [US4] Validate cognitive load: Max 8 concepts (GIL, threading limitation, InterpreterPoolExecutor, separate interpreters, run_in_executor, ProcessPoolExecutor, decision criteria, benchmarking)

**Checkpoint**: Lesson 4 complete - students solve CPU-bound problems with true parallelism

---

## Phase 7: User Story 5 - Build Hybrid Workloads (Priority: P2)

**Goal**: Students combine I/O concurrency + CPU parallelism for AI-native patterns

**Independent Test**: Student builds system with TaskGroup (I/O) + InterpreterPoolExecutor (CPU) that's 40%+ faster than sequential

### Implementation for User Story 5 (Lesson 5)

- [ ] T086 [P] [US5] Create lesson file: `book-source/docs/04-Part-4-Python-Fundamentals/28-asyncio/05-hybrid-workloads.md`
- [ ] T087 [US5] Write lesson YAML frontmatter (title, sidebar_position: 5, proficiency: B2, skills metadata)
- [ ] T088 [US5] Write Hook section (3-5 min): AI apps need both I/O concurrency and CPU parallelism
- [ ] T089 [US5] Write Core 1: Hybrid Pattern Overview (15 min) - TaskGroup for I/O + InterpreterPoolExecutor for CPU
- [ ] T090 [P] [US5] Create Code Example 1: Simple Hybrid (fetch + process) (code-examples/lesson5_example1_simple_hybrid.py)
- [ ] T091 [US5] Write Core 2: Batch Processing Pattern (20 min) - fetch many, process in parallel
- [ ] T092 [P] [US5] Create Code Example 2: Batch Processing System (code-examples/lesson5_example2_batch_processing.py)
- [ ] T093 [US5] Write Core 3: Pipeline Pattern (20 min) - fetch → transform → store with overlap
- [ ] T094 [P] [US5] Create Code Example 3: Pipeline Architecture (code-examples/lesson5_example3_pipeline.py)
- [ ] T095 [US5] Write Core 4: AI Workload Simulation (15 min) - realistic AI pattern (API + inference)
- [ ] T096 [P] [US5] Create Code Example 4: AI Workload Simulation (code-examples/lesson5_example4_ai_workload.py)
- [ ] T097 [US5] Write Core 5: Resource Limiting (10 min) - Semaphores for controlling concurrency
- [ ] T098 [P] [US5] Create Code Example 5: Resource Limiting with Semaphores (code-examples/lesson5_example5_semaphores.py)
- [ ] T099 [P] [US5] Create Code Example 6: Production Hybrid System (code-examples/lesson5_example6_production.py)
- [ ] T100 [US5] Add CoLearning elements throughout (4 total: 💬🎓🚀✨)
- [ ] T101 [US5] Write "Try With AI" section ONLY (4 prompts: Understand → Apply → Analyze → Create)
- [ ] T102 [US5] Validate all code examples: Python 3.14+ patterns, full type hints, performance benchmarks
- [ ] T103 [US5] Run compliance check: `python check_python_314_compliance.py 05-hybrid-workloads.md`
- [ ] T104 [US5] Validate cognitive load: Max 8 concepts (hybrid pattern, TaskGroup + Executor combination, batch processing, pipeline, AI workload pattern, Semaphores, resource limiting, optimization)

**Checkpoint**: Lesson 5 complete - students build production hybrid systems

---

## Phase 8: User Story 6 - Build AI Agent System Capstone (Priority: P2) 🎯 CAPSTONE

**Goal**: Students integrate all chapter concepts into realistic AI agent (concurrent fetch + parallel processing)

**Independent Test**: Student implements multi-service AI agent with 3+ concurrent API fetches, parallel processing, error handling, timeout controls, 40%+ faster than sequential

### Implementation for User Story 6 (Lesson 6 - Capstone)

- [ ] T105 [P] [US6] Create lesson file: `book-source/docs/04-Part-4-Python-Fundamentals/28-asyncio/06-capstone-ai-agent.md`
- [ ] T106 [US6] Write lesson YAML frontmatter (title, sidebar_position: 6, proficiency: B2, skills metadata)
- [ ] T107 [US6] Write Hook section (5 min): Real AI systems combine all asyncio patterns
- [ ] T108 [US6] Write Project Overview (10 min): Multi-service AI agent architecture (Weather + News + Knowledge Base → concurrent fetch → parallel inference → aggregation)
- [ ] T109 [US6] Write Requirements Section (15 min): Functional requirements, performance requirements, error handling requirements
- [ ] T110 [US6] Write Architecture Section (20 min): System diagram, component breakdown, data flow
- [ ] T111 [P] [US6] Create skeleton code: Capstone starter template (code-examples/lesson6_capstone_starter.py)
- [ ] T112 [US6] Write Implementation Guide Part 1 (20 min): Concurrent API fetching with TaskGroup
- [ ] T113 [US6] Write Implementation Guide Part 2 (20 min): Parallel processing with InterpreterPoolExecutor
- [ ] T114 [US6] Write Implementation Guide Part 3 (15 min): Error handling and timeout controls
- [ ] T115 [US6] Write Implementation Guide Part 4 (15 min): Result aggregation and reporting
- [ ] T116 [P] [US6] Create complete solution: Capstone reference implementation (code-examples/lesson6_capstone_solution.py)
- [ ] T117 [US6] Write Testing Section (15 min): How to validate performance, error scenarios to test
- [ ] T118 [US6] Write Extension Challenges (10 min): Ideas for expanding the capstone (caching, retry logic, real APIs)
- [ ] T119 [US6] Add CoLearning elements throughout (4 total: 💬🎓🚀✨)
- [ ] T120 [US6] Write "Try With AI" section ONLY (4 prompts: Understand → Apply → Analyze → Evaluate)
- [ ] T121 [US6] Validate skeleton and solution code: Python 3.14+ patterns, full type hints, comprehensive error handling
- [ ] T122 [US6] Run compliance check: `python check_python_314_compliance.py 06-capstone-ai-agent.md`
- [ ] T123 [US6] Performance test: Verify capstone achieves 40%+ speedup vs sequential (benchmark both implementations)
- [ ] T124 [US6] Integration validation: All Lessons 1-5 concepts appear in capstone

**Checkpoint**: Capstone complete - students have built realistic AI agent system

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Final quality checks, navigation, and chapter-level polish

- [ ] T125 Update chapter intro.md with lesson summaries and learning path
- [ ] T126 [P] Validate all internal links (lesson cross-references, code example links)
- [ ] T127 [P] Validate all forward references (only Ch 29 GIL reference allowed)
- [ ] T128 [P] Run full chapter compliance audit: All 6 lessons use Python 3.14+ only
- [ ] T129 [P] Validate all "Try With AI" sections follow 4-prompt Bloom's progression
- [ ] T130 [P] Validate all CoLearning elements present (4 per lesson × 6 lessons = 24 total)
- [ ] T131 Create chapter-level README documenting code examples and running them
- [ ] T132 [P] Test all 29 code examples execute successfully with Python 3.14+
- [ ] T133 [P] Validate type checking: Run `mypy --strict` on all code examples
- [ ] T134 Update chapter-index.md: Change Chapter 28 status from "📋 Planned" to "✅ Implemented"
- [ ] T135 Create chapter summary document for constitution alignment validation
- [ ] T136 Generate PHR (Prompt History Record) for entire chapter implementation

**Checkpoint**: Chapter 28 complete, validated, ready for technical review

---

## Dependencies & Parallel Execution

### User Story Completion Order

**Can be parallelized** (after foundational phase):
- US1 (Lesson 1) ⚡ Independent
- US2 (Lesson 2) ⚡ Independent (but better after Lesson 1 for learning progression)
- US3 (Lesson 3) ⚡ Independent (but better after Lessons 1-2)

**Sequential dependencies**:
- US4 (Lesson 4) → Requires US1 (I/O vs CPU understanding)
- US5 (Lesson 5) → Requires US2 (TaskGroup) + US4 (InterpreterPoolExecutor)
- US6 (Lesson 6) → Requires US1-5 (capstone integrates everything)

### Parallel Execution Examples

**Within a single lesson** (example: Lesson 2):
```bash
# Code examples can be created in parallel (different files)
T033, T035, T037, T038, T039, T040 [all marked with [P]]

# Writing sections can proceed in parallel
T032 (Core 1), T034 (Core 2), T036 (Core 3) [with coordination]
```

**Across lessons** (after Foundation complete):
```bash
# These lessons can be written in parallel by different implementers:
Lesson 1 (T012-T028)
Lesson 2 (T029-T047)  
Lesson 3 (T048-T066)

# But Lesson 4-6 have sequential dependencies (see above)
```

---

## Implementation Strategy

### MVP Scope (Minimum Viable Chapter)

**Recommended MVP**: User Story 1 + User Story 2 (Lessons 1-2)

**Rationale**:
- Lesson 1 teaches foundation (event loop, coroutines, asyncio.run())
- Lesson 2 teaches practical patterns (TaskGroup, gather)
- Together they deliver 80% of practical asyncio value
- Students can build real concurrent I/O systems after Lesson 2

**Incremental Delivery**:
1. **Week 1**: Lessons 1-2 (MVP - foundational + concurrent tasks)
2. **Week 2**: Lessons 3-4 (advanced patterns + CPU-bound solutions)
3. **Week 3**: Lessons 5-6 (hybrid workloads + capstone)

### Validation Checklist Per Lesson

Before marking any lesson complete, verify:

- [ ] YAML frontmatter complete (title, sidebar_position, proficiency, skills metadata)
- [ ] All code examples tested with Python 3.14+
- [ ] Type hints: 100% coverage (`mypy --strict` passes)
- [ ] No deprecated APIs (compliance check passes)
- [ ] CoLearning elements: 4 per lesson (💬🎓🚀✨)
- [ ] Lesson closure: "Try With AI" section ONLY (4 prompts, Bloom's progression)
- [ ] Cognitive load validated (max concepts: 7 for B1, 10 for B1-B2, no limit for B2 but keep reasonable)
- [ ] No forward references (except brief Ch 29 GIL mention in Lesson 4)
- [ ] AI-Native Learning language (Part 4: "describe intent", not formal SDD)

---

## Policy Note for Lesson Authors

**Lesson Closure Pattern** (CRITICAL):

Within Chapter 28, each lesson MUST end with a single final section titled **"Try With AI"** containing exactly 4 prompts following Bloom's taxonomy progression:

1. **Understand Level**: "Ask your AI: [question about concepts]"
2. **Apply Level**: "Tell your AI: [request to generate code]"
3. **Analyze Level**: "Ask your AI: [request to compare/explain tradeoffs]"
4. **Create/Evaluate Level**: "Given [scenario], ask your AI: [design/assess solution]"

**DO NOT include**:
- ❌ "Key Takeaways" section
- ❌ "What's Next" section
- ❌ "Summary" section
- ❌ Separate "Exercises" section

**AI Tool Usage**:
- Part 4 students have completed AI tool onboarding (Chapters 5-6: Claude Code, Gemini CLI)
- Instructions should say: "Use your preferred AI companion (Claude Code, Gemini CLI, or ChatGPT)"
- Provide CLI examples where appropriate: `claude "generate async code for X"`

---

## Summary

**Total Tasks**: 136
**User Stories**: 6 (mapped to 6 lessons)
**Parallel Opportunities**: 45 tasks marked [P]
**Code Examples**: 29 across all lessons
**Estimated Timeline**: 3 weeks (MVP in Week 1, complete in Week 3)

**Next Step**: Begin with Phase 1 (Setup) → Phase 2 (Foundation) → Phase 3 (Lesson 1)
