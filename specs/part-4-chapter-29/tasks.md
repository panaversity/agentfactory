# Implementation Tasks: Chapter 29 - CPython and GIL

**Feature**: Chapter 29 - CPython and GIL (Part 4: Python Fundamentals)
**Status**: Ready for Implementation
**Created**: 2025-11-09
**Branch**: `part-4-chapter-29`

---

## Overview

This document provides actionable, dependency-ordered tasks for implementing Chapter 29. Tasks are organized by lesson to enable sequential implementation with validation gates between lessons.

**Chapter Structure**:
- 6 lessons (B1 → B1-B2 → B2 proficiency progression)
- 8 code examples (tested and working)
- 50+ CoLearning prompts integrated throughout
- Capstone project: Multi-Agent Concurrency System + Benchmarking Dashboard

**Implementation Strategy**:
- **Sequential lesson implementation** (Lesson 1 → 6)
- Validation gate after each lesson (reading level, code testing, CoLearning compliance)
- **Parallel opportunities**: Code examples within a lesson can be written concurrently
- **MVP Scope**: Lessons 1-4 (foundational understanding + free-threading)
- **Full Chapter**: All 6 lessons for complete learning experience

**Total Estimated Effort**: 40-50 hours (including validation and iteration)

---

## Task Summary

| Phase | Tasks Count | Estimated Effort | Parallelizable |
|-------|-------------|------------------|----------------|
| Setup | 3 | 2 hours | 0 |
| Lesson 1 | 8 | 6 hours | 3 |
| Lesson 2 | 8 | 6 hours | 3 |
| Lesson 3 | 9 | 7 hours | 4 |
| Lesson 4 | 11 | 9 hours | 5 |
| Lesson 5 | 10 | 8 hours | 5 |
| Lesson 6 | 9 | 8 hours | 3 |
| Validation | 6 | 4 hours | 3 |
| **Total** | **64** | **50 hours** | **26** |

---

## Dependency Graph

```
Setup (Phase 1)
    ↓
Lesson 1 (Phase 2) → Validation 1
    ↓
Lesson 2 (Phase 3) → Validation 2
    ↓
Lesson 3 (Phase 4) → Validation 3
    ↓
Lesson 4 (Phase 5) → Validation 4  [PRIMARY LESSON]
    ↓
Lesson 5 (Phase 6) → Validation 5
    ↓
Lesson 6 (Phase 7) → Validation 6  [CAPSTONE]
    ↓
Final Validation (Phase 8)
```

**Critical Path**: Setup → Lessons 1-6 sequential → Final Validation
**Parallel Opportunities**: Code examples within each lesson (marked with [P])

---

## Phase 1: Setup & Prerequisites

### Objectives
- Create directory structure for Chapter 29
- Set up validation tools (reading level, code testing)
- Prepare CoLearning element templates

### Tasks

- [ ] T001 Create chapter directory structure at `book-source/docs/04-Part-4-Python-Fundamentals/29-cpython-gil/`
- [ ] T002 Create readme.md file with chapter overview, navigation, and 6 lesson links
- [ ] T003 Set up Python 3.14+ code testing environment (verify free-threading build availability for examples)

**Acceptance Criteria**:
- ✅ Directory exists with correct path
- ✅ readme.md includes chapter title, description, lesson list (1-6)
- ✅ Python 3.14.0+ installed and accessible for code testing

---

## Phase 2: Lesson 1 - What is CPython?

**CEFR Proficiency**: B1
**Estimated Time**: 60 minutes (student), 6 hours (implementation)
**Learning Objectives**: LO-1 (CPython architecture understanding)
**Success Evals**: EVAL-001 (75%+ CPython internals comprehension)

### Content Tasks

- [ ] T004 Write Lesson 1 introduction (hook: CPython as Python's "original version", 2-3 paragraphs)
- [ ] T005 Write Section 1: What is CPython? (reference implementation definition, C language, official version)
- [ ] T006 Write Section 2: Python Execution Pipeline (source → bytecode → interpreter → execution)
- [ ] T007 Write Section 3: Memory Management (reference counting + garbage collection)
- [ ] T008 Write Section 4: Alternative Implementations (PyPy, Jython, IronPython, MicroPython - brief comparison)
- [ ] T009 Write Section 5: How to Detect CPython (platform module, practical use)

### Code Example Tasks

- [ ] T010 [P] Implement Example 1: CPython detection code in `book-source/docs/04-Part-4-Python-Fundamentals/29-cpython-gil/01-what-is-cpython.md`
  - Code: `platform.python_implementation()` with type hints
  - Test: Verify returns "CPython" on standard Python
  - AI Prompt: "Explain what platform.python_implementation() does. What would this output on PyPy or Jython?"

### CoLearning Element Tasks

- [ ] T011 Integrate 3 💬 AI Colearning Prompts throughout Lesson 1:
  - Prompt 1: After CPython definition ("Explain what makes CPython the 'reference' implementation")
  - Prompt 2: After execution pipeline ("Walk me through what happens when I type `python script.py`")
  - Prompt 3: After alternatives ("When would you choose PyPy over CPython?")

- [ ] T012 [P] Integrate 2 🎓 Instructor Commentaries:
  - Commentary 1: After code example ("In AI-native development, you don't memorize interpreter internals—you understand WHEN they matter")
  - Commentary 2: After memory management ("Syntax is cheap; understanding garbage collection's impact on performance is gold")

- [ ] T013 [P] Integrate 2 🚀 CoLearning Challenges:
  - Challenge 1: After alternatives ("Generate a comparison table of CPython vs PyPy vs Jython")
  - Challenge 2: Before Try With AI ("Ask AI to explain how bytecode compilation enables Python's portability")

- [ ] T014 [P] Integrate 1 ✨ Teaching Tip:
  - Tip 1: When demonstrating detection code ("Use Claude Code to explore edge cases: 'What does this return on PyPy?'")

### Lesson Closure Task

- [ ] T015 Write "Try With AI" section (4 prompts, Bloom's progression):
  - Prompt 1 (Remember): "In your own words, explain what CPython is. Then ask your AI: 'Is my explanation correct? What am I missing?'"
  - Prompt 2 (Understand): "Ask your AI: 'Why did Python creators write CPython in C instead of Python itself? What are the tradeoffs?'"
  - Prompt 3 (Apply): "Tell your AI: 'Help me write code to detect if I'm running CPython vs another implementation. Explain each step.'"
  - Prompt 4 (Analyze/Synthesize): "Ask your AI: 'How does CPython's design enable the free-threading improvements in Python 3.14? Connect this to multi-agent AI systems (preview of Lesson 4).'"

**Acceptance Criteria (Lesson 1)**:
- ✅ 7 concepts covered (no more than 10 for B1 tier)
- ✅ Reading level: B1-B2 (Flesch-Kincaid Grade 9-11)
- ✅ All CoLearning elements present (💬🎓🚀✨)
- ✅ Lesson ends with "Try With AI" ONLY (no summaries after)
- ✅ Code Example 1 tested and working
- ✅ No forward references to Lesson 2+ concepts

---

## Phase 3: Lesson 2 - CPython Performance Evolution (Python 3.14)

**CEFR Proficiency**: B1
**Estimated Time**: 60 minutes (student), 6 hours (implementation)
**Learning Objectives**: LO-1 (CPython architecture)
**Success Evals**: EVAL-001 (CPython internals), connects to EVAL-002 (Python 3.14 context)

### Content Tasks

- [ ] T016 Write Lesson 2 introduction (hook: Python 3.14 performance improvements, AI workload context)
- [ ] T017 Write Section 1: Tail-Call Interpreter Architecture (3-5% faster pyperformance, how it works)
- [ ] T018 Write Section 2: Incremental Garbage Collection (order of magnitude faster pauses, impact on latency)
- [ ] T019 Write Section 3: Deferred Annotation Evaluation (PEP 649/749, type hints performance)
- [ ] T020 Write Section 4: Remote Debugging Interface (PEP 768, brief mention)
- [ ] T021 Write Section 5: Impact on AI Workloads (faster execution, lower latency, connects to multi-agent systems)

### Code Example Tasks

- [ ] T022 [P] Add performance comparison code snippet (optional - demonstrates pyperformance benchmarks)
  - Show before/after Python 3.14 improvements conceptually
  - No actual benchmark needed (just illustrative)

### CoLearning Element Tasks

- [ ] T023 Integrate 3 💬 AI Colearning Prompts:
  - Prompt 1: After tail-call interpreter ("How does tail-call optimization make Python faster?")
  - Prompt 2: After incremental GC ("Why does incremental GC reduce pause times? What's the tradeoff?")
  - Prompt 3: After AI workload section ("How do these improvements help multi-agent AI systems specifically?")

- [ ] T024 [P] Integrate 2 🎓 Instructor Commentaries:
  - Commentary 1: After performance improvements ("You don't need to understand tail-call mechanics—you need to know WHEN performance matters for your AI workloads")
  - Commentary 2: Before conclusion ("Python 3.14 isn't just faster—it's AI-native ready")

- [ ] T025 [P] Integrate 1 🚀 CoLearning Challenge:
  - Challenge 1: After Python 3.14 improvements ("Ask AI to explain PEP 659 specializing adaptive interpreter and how it works with free-threading")

- [ ] T026 [P] Integrate 1 ✨ Teaching Tip:
  - Tip 1: When discussing performance ("Use your AI to compare Python 3.13 vs 3.14 release notes for performance highlights")

### Lesson Closure Task

- [ ] T027 Write "Try With AI" section (4 prompts):
  - Prompt 1 (Remember): "List the 4 major performance improvements in Python 3.14. Then ask AI to validate your list."
  - Prompt 2 (Understand): "Ask your AI: 'Explain how incremental GC works and why it matters for long-running AI agents.'"
  - Prompt 3 (Apply): "Tell your AI: 'Show me how to check my Python version and confirm I'm using 3.14+. Include code.'"
  - Prompt 4 (Analyze/Synthesize): "Ask your AI: 'Connect tail-call interpreter improvements to the free-threading work in Lesson 4. How do they complement each other?'"

**Acceptance Criteria (Lesson 2)**:
- ✅ 6 concepts covered (within B1 limit)
- ✅ Reading level: B1-B2
- ✅ All CoLearning elements present
- ✅ Lesson ends with "Try With AI" ONLY
- ✅ Builds on Lesson 1 (no repetition, only extension)
- ✅ Forward reference to Lesson 4 is preview-only (not teaching free-threading yet)

---

## Phase 4: Lesson 3 - The Traditional GIL (Pre-3.13)

**CEFR Proficiency**: B1-B2 (complexity increases)
**Estimated Time**: 90 minutes (student), 7 hours (implementation)
**Learning Objectives**: LO-2 (GIL evolution - traditional behavior), LO-3 (CPU vs I/O distinction)
**Success Evals**: EVAL-002 (GIL comprehension), EVAL-003 (concurrency decision-making foundation)

### Content Tasks

- [ ] T028 Write Lesson 3 introduction (hook: GIL as Python's "biggest constraint for 30 years")
- [ ] T029 Write Section 1: What is the GIL? (mutex protecting interpreter state, definition)
- [ ] T030 Write Section 2: Why the GIL Exists (simplifies C API, memory safety, thread safety)
- [ ] T031 Write Section 3: What the GIL Prevents (true parallel execution, even on multi-core)
- [ ] T032 Write Section 4: CPU-Bound vs I/O-Bound Workloads (CRITICAL DISTINCTION):
  - CPU-bound definition + examples (prime calculations, data processing, AI inference)
  - I/O-bound definition + examples (network requests, file reading, database queries)
  - Why GIL releases during I/O (system calls)
  - Why GIL blocks during CPU work (interpreter state protection)
- [ ] T033 Write Section 5: Traditional Workarounds (multiprocessing, C extensions release GIL)

### Code Example Tasks

- [ ] T034 [P] Implement Example 3: CPU-bound vs I/O-bound demonstration in `03-traditional-gil.md`
  - Code: `cpu_bound_task()` (sum of squares) vs `io_bound_task()` (time.sleep)
  - Test: Verify both run correctly
  - AI Prompt: "Why does time.sleep() release the GIL, but the sum calculation doesn't? Explain what's happening in the CPython interpreter for each task."

### CoLearning Element Tasks

- [ ] T035 Integrate 4 💬 AI Colearning Prompts:
  - Prompt 1: After GIL definition ("What does 'mutex' mean in the context of the GIL?")
  - Prompt 2: After why GIL exists ("Ask AI: 'Could Python have been designed without a GIL? What would be different?'")
  - Prompt 3: After CPU vs I/O (CRITICAL) ("Give me 5 real-world examples of CPU-bound and 5 of I/O-bound tasks in AI applications")
  - Prompt 4: After workarounds ("Explain how multiprocessing avoids the GIL. What's the cost?")

- [ ] T036 [P] Integrate 3 🎓 Instructor Commentaries:
  - Commentary 1: After GIL prevents parallelism ("Understanding WHY the GIL exists is more valuable than memorizing what it prevents")
  - Commentary 2: After CPU vs I/O distinction ("This distinction is CRITICAL for Lesson 5's decision framework—master it now")
  - Commentary 3: Before workarounds ("Multiprocessing was Python's answer for 30 years. Lesson 4 changes everything.")

- [ ] T037 [P] Integrate 2 🚀 CoLearning Challenges:
  - Challenge 1: After CPU vs I/O ("Ask AI to analyze your recent project: classify each function as CPU-bound or I/O-bound")
  - Challenge 2: Before Try With AI ("Generate a flowchart: How does CPython decide when to release the GIL during execution?")

- [ ] T038 [P] Integrate 2 ✨ Teaching Tips:
  - Tip 1: When showing CPU vs I/O code ("Use Claude Code to experiment: 'Modify this code to show GIL behavior with threading.Thread'")
  - Tip 2: After workarounds ("Ask your AI: 'Show me a benchmark comparing threading vs multiprocessing for the same CPU task'")

### Lesson Closure Task

- [ ] T039 Write "Try With AI" section (4 prompts):
  - Prompt 1 (Remember): "Explain in your own words what the GIL is and why it exists. Ask AI to validate."
  - Prompt 2 (Understand): "Ask your AI: 'Why does the GIL release during I/O but not during CPU work? Explain the interpreter mechanics.'"
  - Prompt 3 (Apply): "Tell your AI: 'Help me classify these 10 tasks as CPU-bound or I/O-bound: [list tasks]. Explain your reasoning.'"
  - Prompt 4 (Analyze/Synthesize): "Ask your AI: 'How does the traditional GIL limit multi-agent AI systems? Preview what Lesson 4's free-threading solves.'"

**Acceptance Criteria (Lesson 3)**:
- ✅ 8 concepts covered (B1-B2 tier allows more depth)
- ✅ Reading level: B1-B2
- ✅ CPU vs I/O distinction emphasized (CRITICAL for Lesson 5)
- ✅ All CoLearning elements present
- ✅ Code Example 3 tested and working
- ✅ Sets up Lesson 4 (GIL as problem → free-threading as solution)

---

## Phase 5: Lesson 4 - Free-Threaded Python (3.14+ Production Ready)

**CEFR Proficiency**: B1-B2 (PRIMARY LESSON - most concepts)
**Estimated Time**: 120 minutes (student), 9 hours (implementation)
**Learning Objectives**: LO-2 (GIL evolution - PRIMARY), LO-4 (free-threading application)
**Success Evals**: EVAL-002 (GIL evolution 80%+), EVAL-004 (detection code 85%+)

### Content Tasks

- [ ] T040 Write Lesson 4 introduction (hook: "The GIL is now optional - Python's biggest change in 30 years")
- [ ] T041 Write Section 1: The GIL Removal Paradigm Shift:
  - Three-phase roadmap (3.13 experimental 40% → 3.14 supported 5-10% → 3.15+ default)
  - PEP 703 (free-threading implementation) + PEP 779 (criteria for support)
  - What "free-threaded" means (true parallelism on multi-core)
- [ ] T042 Write Section 2: How Free-Threading Works:
  - Per-thread state (not global interpreter state)
  - Lock-free data structures
  - Thread-safe built-ins (dict, list, set with internal locks)
  - Specializing adaptive interpreter (PEP 659) enabled
- [ ] T043 Write Section 3: Installing Free-Threaded Python:
  - macOS/Windows: Official installers with free-threading option
  - Linux: Build with `./configure --disable-gil`
  - Docker: `python:3.14t` image tag
- [ ] T044 Write Section 4: Detection and Runtime Control:
  - Detection code (`sys._is_gil_enabled()`)
  - Runtime control (`PYTHON_GIL` environment variable)
  - Build verification
- [ ] T045 Write Section 5: Performance Characteristics:
  - Single-threaded overhead: 5-10% (vs 40% in 3.13)
  - Multi-threaded gains: 2-10x on CPU-bound tasks (real benchmarks)
  - When overhead matters vs when gains dominate
- [ ] T046 Write Section 6: Thread Safety Considerations:
  - Built-in types have internal locks
  - Still use threading.Lock for complex operations
  - When to trust built-ins vs when to add locks
- [ ] T047 Write Section 7: AI-Native Context:
  - Multi-agent systems can now reason in parallel
  - Example: 4 agents on 4 cores = 2-4x throughput
  - Preview: Kubernetes resource utilization (Parts 11-13)

### Code Example Tasks

- [ ] T048 [P] Implement Example 2: Free-threading detection code in `04-free-threaded-python.md`
  - Code: `check_free_threading()` function with type hints, dict return
  - Test: Verify detects build support correctly
  - AI Prompt: "Walk me through this code line by line. Why does `gil_enabled is not None` check if the build supports free-threading?"

- [ ] T049 [P] Implement Example 6: Benchmark - free-threaded Python in `04-free-threaded-python.md`
  - Code: `benchmark_free_threaded()` with CPU task, threading
  - Test: Verify runs (may show "not supported" on non-free-threaded builds)
  - AI Prompt: "How does this compare to traditional threading and multiprocessing? When would you choose free-threaded Python over multiprocessing?"

### CoLearning Element Tasks

- [ ] T050 Integrate 5 💬 AI Colearning Prompts (most critical lesson):
  - Prompt 1: After paradigm shift ("Why did it take 30 years to make the GIL optional? What technical challenges existed?")
  - Prompt 2: After how it works ("Explain per-thread state vs global interpreter state. Draw a diagram.")
  - Prompt 3: After installation ("Which installation method should I use for development vs production? Why?")
  - Prompt 4: After performance ("When does the 5-10% overhead matter? When do multi-threaded gains dominate?")
  - Prompt 5: After thread safety ("If built-ins have internal locks, why do I still need threading.Lock sometimes?")

- [ ] T051 [P] Integrate 3 🎓 Instructor Commentaries:
  - Commentary 1: After paradigm shift ("Free-threading isn't just a feature—it's a fundamental shift in how Python executes")
  - Commentary 2: After performance ("Benchmarks show 2-10x gains, but YOUR workload may differ. Always measure.")
  - Commentary 3: After AI-native context ("This is why Chapter 29 matters: true parallel AI reasoning is now possible in Python")

- [ ] T052 [P] Integrate 3 🚀 CoLearning Challenges:
  - Challenge 1: After installation ("Set up a free-threaded Python environment with your AI's help. Document each step.")
  - Challenge 2: After detection ("Write a script that checks 5 different Python environments and reports their free-threading status")
  - Challenge 3: Before Try With AI ("Design a multi-agent system architecture that leverages free-threading. Ask AI to critique your design.")

- [ ] T053 [P] Integrate 2 ✨ Teaching Tips:
  - Tip 1: When showing detection code ("Use your AI to explore: 'What happens if I try to disable GIL on a build that doesn't support it?'")
  - Tip 2: After performance ("Ask Claude Code: 'Generate a benchmark showing overhead vs gains. Plot results as ASCII chart.'")

### Lesson Closure Task

- [ ] T054 Write "Try With AI" section (4 prompts):
  - Prompt 1 (Remember): "Summarize the three-phase GIL removal roadmap. Where are we now (Python 3.14)? Ask AI to validate."
  - Prompt 2 (Understand): "Ask your AI: 'Explain how per-thread state enables free-threading. Why couldn't Python do this before 3.13?'"
  - Prompt 3 (Apply): "Tell your AI: 'Help me write code to: (a) detect free-threading, (b) enable/disable GIL at runtime, (c) verify it worked.'"
  - Prompt 4 (Analyze/Synthesize): "Ask your AI: 'Design a 3-agent AI system where each agent processes data in parallel. How does free-threading make this practical? Connect to Kubernetes deployment (Part 11 preview).'"

**Acceptance Criteria (Lesson 4)**:
- ✅ 10 concepts covered (max for B1-B2 tier - justified as PRIMARY LESSON)
- ✅ Reading level: B1-B2
- ✅ All CoLearning elements present (5 prompts justified for depth)
- ✅ Code Examples 2 and 6 tested and working
- ✅ Free-threading positioned as paradigm shift (not incremental improvement)
- ✅ Lesson ends with "Try With AI" ONLY
- ✅ AI-native context integrated (multi-agent systems, production preview)

---

## Phase 6: Lesson 5 - Choosing Your Concurrency Approach

**CEFR Proficiency**: B1-B2 (practical decision-making)
**Estimated Time**: 90 minutes (student), 8 hours (implementation)
**Learning Objectives**: LO-3 (concurrency decisions - PRIMARY), LO-4 (benchmarking)
**Success Evals**: EVAL-003 (decision-making 70%+), EVAL-005 (benchmarking 75%+)

### Content Tasks

- [ ] T055 Write Lesson 5 introduction (hook: "Four concurrency approaches - when to use each")
- [ ] T056 Write Section 1: Decision Framework (CORE TABLE):
  | Use Case | Best Approach | Why |
  | I/O-bound | Asyncio or threading | GIL releases during I/O |
  | CPU-bound (single task) | Single thread | No overhead |
  | CPU-bound (parallel tasks) | Free-threaded Python | True parallelism + shared memory |
  | CPU-bound (isolation needed) | Multiprocessing | Separate memory spaces |
  | Mixed workload | Free-threaded + asyncio | Best of both |
- [ ] T057 Write Section 2: Benchmarking Methodology:
  - Design CPU-bound task (prime number calculation)
  - Test 1: Single-threaded baseline
  - Test 2: Traditional threading (GIL-constrained)
  - Test 3: Multiprocessing (separate processes)
  - Test 4: Free-threaded Python (true parallelism)
  - Test 5: Asyncio (for I/O comparison)
- [ ] T058 Write Section 3: Interpreting Results:
  - Execution time comparison
  - CPU utilization metrics
  - Memory usage analysis
  - When to choose which approach
- [ ] T059 Write Section 4: Python 3.14 Asyncio Improvements:
  - CLI introspection tools (`python -m asyncio ps`, `pstree`)
  - Thread-safe event loop (works with free-threading)
  - 10-20% faster single-threaded
  - Linear scaling with threads
- [ ] T060 Write Section 5: Multiprocessing Changes in 3.14:
  - Default `forkserver` on Unix (safer than `fork`)
  - `Process.interrupt()` for graceful shutdown
  - Context inheritance

### Code Example Tasks

- [ ] T061 [P] Implement Example 4: Benchmark - traditional threading in `05-choosing-concurrency.md`
  - Code: `benchmark_threading()` with CPU task
  - Test: Verify shows ~same time as single-threaded (GIL prevents speedup)
  - AI Prompt: "Why don't we see a 4x speedup with 4 threads here? Explain what the GIL is doing during this benchmark."

- [ ] T062 [P] Implement Example 5: Benchmark - multiprocessing in `05-choosing-concurrency.md`
  - Code: `benchmark_multiprocessing()` with same CPU task
  - Test: Verify shows 2-4x speedup (true parallelism)
  - AI Prompt: "Compare this to the threading example. Why is multiprocessing faster for CPU-bound tasks? What are the tradeoffs (memory, startup time, communication)?"

- [ ] T063 [P] Implement Example 7: Asyncio CLI introspection in `05-choosing-concurrency.md`
  - Code: async_example.py with 3 agents
  - Commands: `python -m asyncio ps <PID>`, `pstree <PID>`
  - Test: Verify tools show task hierarchy
  - AI Prompt: "Explain what these CLI tools show. How do they help debug async programs? When would you use `ps` vs `pstree`?"

### CoLearning Element Tasks

- [ ] T064 Integrate 4 💬 AI Colearning Prompts:
  - Prompt 1: After decision framework ("Walk me through 10 real-world scenarios. For each, which concurrency approach should I use and why?")
  - Prompt 2: After benchmarking ("What factors besides execution time matter when choosing a concurrency approach? (memory, complexity, debugging)")
  - Prompt 3: After asyncio improvements ("How do asyncio and free-threading complement each other? When would you use both?")
  - Prompt 4: After multiprocessing changes ("Why is `forkserver` safer than `fork`? What can go wrong with `fork`?")

- [ ] T065 [P] Integrate 3 🎓 Instructor Commentaries:
  - Commentary 1: After decision framework ("Memorizing this table won't help. Understanding the WHY behind each choice makes you production-ready.")
  - Commentary 2: After benchmarking ("Benchmarks lie. Your workload is unique. Always measure on YOUR data.")
  - Commentary 3: After asyncio CLI ("Python 3.14 gives you production-grade debugging tools. Use them.")

- [ ] T066 [P] Integrate 3 🚀 CoLearning Challenges:
  - Challenge 1: After benchmarking methodology ("Implement all 5 benchmarks for a different CPU task (matrix multiplication). Compare results.")
  - Challenge 2: After interpreting results ("Ask AI to generate a decision tree: input use case → output recommended approach")
  - Challenge 3: Before Try With AI ("Design a hybrid system: free-threaded for CPU work + asyncio for I/O. Ask AI to implement scaffolding.")

- [ ] T067 [P] Integrate 2 ✨ Teaching Tips:
  - Tip 1: When showing benchmarks ("Use your AI to add memory profiling: 'Show me how to measure memory usage for each approach'")
  - Tip 2: After asyncio CLI ("Ask Claude Code: 'Set up a test async program and show me what `ps` and `pstree` output look like'")

### Lesson Closure Task

- [ ] T068 Write "Try With AI" section (4 prompts):
  - Prompt 1 (Remember): "List the 5 concurrency approaches and their use cases. Ask AI to validate your list."
  - Prompt 2 (Understand): "Ask your AI: 'For each approach, explain the memory model (shared vs separate) and when it matters.'"
  - Prompt 3 (Apply): "Tell your AI: 'I have a web scraper that downloads 100 pages and processes text (CPU-intensive). Which approach should I use? Implement a benchmark to prove it.'"
  - Prompt 4 (Analyze/Synthesize): "Ask your AI: 'Design a production AI system with 10 agents: 5 do NLP (CPU-bound), 5 fetch data (I/O-bound). Which concurrency approaches for each group? Why? Connect to Kubernetes horizontal scaling (Part 11 preview).'"

**Acceptance Criteria (Lesson 5)**:
- ✅ 9 concepts covered (within B1-B2 limit)
- ✅ Reading level: B1-B2
- ✅ Decision framework table is CLEAR and actionable
- ✅ All CoLearning elements present
- ✅ Code Examples 4, 5, 7 tested and working
- ✅ Connects to Lessons 3-4 (applies GIL knowledge)
- ✅ Lesson ends with "Try With AI" ONLY

---

## Phase 7: Lesson 6 - Capstone: Multi-Agent Concurrency System

**CEFR Proficiency**: B2 (advanced integration/synthesis)
**Estimated Time**: 150-180 minutes (student), 8 hours (implementation)
**Learning Objectives**: LO-4 (AI-native systems - PRIMARY), LO-3 (apply decisions), LO-5 (meta-skill)
**Success Evals**: EVAL-006 (multi-agent 60%+), EVAL-005 (benchmark 75%+), EVAL-010 (production context 70%+)

### Content Tasks

- [ ] T069 Write Lesson 6 introduction (hook: "Build a production-ready multi-agent system demonstrating true parallelism")
- [ ] T070 Write Section 1: Project Vision:
  - Part A: Multi-Agent AI System (3-4 agents, CPU-bound reasoning, free-threaded)
  - Part B: Benchmarking Dashboard (compare all approaches, visual output)
  - Integration: Multi-agent system IS the benchmark workload
- [ ] T071 Write Section 2: Architecture Design:
  - AIAgent class (base agent with reasoning method)
  - Shared state management (thread-safe data structures)
  - Benchmarking harness (timing, metrics, comparison)
- [ ] T072 Write Section 3: Implementation Guide:
  - Step 1: Set up free-threading environment
  - Step 2: Implement AIAgent class (with type hints, dataclasses)
  - Step 3: Create multi-agent runner (threading + free-threading check)
  - Step 4: Add benchmarking dashboard (ASCII charts or tables)
  - Step 5: Error handling (thread safety, exceptions)
- [ ] T073 Write Section 4: Testing & Validation:
  - Verify agents run in parallel (check CPU usage)
  - Validate performance gains (free-threaded vs traditional)
  - Test thread safety (shared state correctness)
- [ ] T074 Write Section 5: Production Context:
  - "In Part 11 (Kubernetes), you'll deploy this as pods..."
  - "In Part 14 (Dapr Actors), you'll scale this pattern to virtual actors..."
  - Free-threading maximizes CPU utilization in production

### Code Example Tasks

- [ ] T075 [P] Implement Example 8: Multi-agent system foundation in `06-capstone-multi-agent.md`
  - Code: AIAgent class, run_multi_agent_system(), AgentResult dataclass
  - Test: Verify 4 agents run, produce results
  - AI Prompt: "How would you expand this into the full capstone project? What benchmarking features should I add? How can I make this production-ready for deployment in Kubernetes (Part 11)?"

### CoLearning Element Tasks

- [ ] T076 Integrate 3 💬 AI Colearning Prompts:
  - Prompt 1: When introducing project ("Help me design the architecture for a multi-agent system. What classes do I need? What methods? How should agents communicate?")
  - Prompt 2: During implementation ("I want to add 2 more agent types: sentiment analysis + keyword extraction. Generate code and explain integration.")
  - Prompt 3: After testing ("My agents aren't running in parallel. Here's the code [paste]. What's wrong? How do I verify free-threading is active?")

- [ ] T077 [P] Integrate 2 🎓 Instructor Commentaries:
  - Commentary 1: After foundation code ("Professional AI developers describe architecture, AI generates scaffolding, then they validate and extend. This is AI-native workflow.")
  - Commentary 2: After production context ("Free-threading isn't just academic—it's how you'll build production systems in Parts 11-14")

- [ ] T078 [P] Integrate 2 🚀 CoLearning Challenges:
  - Challenge 1: During implementation ("Add benchmarking: compare threading vs multiprocessing vs free-threaded for your agent workload")
  - Challenge 2: Before Try With AI ("Ask AI to suggest 3 real-world AI applications where this multi-agent pattern would excel")

- [ ] T079 [P] Integrate 1 ✨ Teaching Tip:
  - Tip 1: When debugging ("Use your AI to debug thread issues: 'My agents share a dict. Is this thread-safe in free-threaded Python? Should I add locks?'")

### Lesson Closure Task

- [ ] T080 Write "Try With AI" section (4 prompts):
  - Prompt 1 (Remember): "Summarize what you learned in Lessons 1-5. Then ask your AI: 'What are the 3 most important concepts for building this capstone project?'"
  - Prompt 2 (Understand): "Ask your AI: 'Explain step-by-step how the multi-agent system achieves true parallelism. What would break if I used traditional threading?'"
  - Prompt 3 (Apply): "Tell your AI: 'Help me implement the benchmarking dashboard. I want to compare 4 concurrency approaches with ASCII bar charts. Generate the code.'"
  - Prompt 4 (Analyze/Synthesize): "Ask your AI: 'How would I deploy this multi-agent system in Kubernetes (Part 11)? What changes for production? How does free-threading help with resource utilization?'"

**Acceptance Criteria (Lesson 6)**:
- ✅ 8 concepts covered (B2 synthesis allows integration of prior knowledge)
- ✅ Reading level: B1-B2
- ✅ All CoLearning elements present
- ✅ Code Example 8 tested and working (capstone foundation)
- ✅ Capstone project is DOABLE in 2-3 hours (scaffolding provided)
- ✅ Production context integrated (Parts 11-14 preview)
- ✅ Lesson ends with "Try With AI" ONLY

---

## Phase 8: Final Validation & Quality Assurance

### Validation Tasks

- [ ] T081 [P] Run reading level analysis on all 6 lessons (verify Flesch-Kincaid Grade 9-11 for B1-B2)
- [ ] T082 [P] Test all 8 code examples on Python 3.14.0+ (verify working, no errors)
- [ ] T083 [P] Validate CoLearning element count (50+ prompts total across 6 lessons: ✅ 💬🎓🚀✨)
- [ ] T084 Verify lesson closure compliance (all 6 lessons end with "Try With AI" ONLY, no summaries after)
- [ ] T085 Check proficiency progression (validate B1 → B1-B2 → B2, no zigzag)
- [ ] T086 Validate no forward references (Lesson N only uses concepts from Lessons 1 to N-1)

**Acceptance Criteria (Final Validation)**:
- ✅ All code examples run on Python 3.14.0+
- ✅ Reading level: B1-B2 (all lessons)
- ✅ 50+ CoLearning prompts present
- ✅ All lessons end with "Try With AI" ONLY
- ✅ Proficiency progression smooth (no zigzag)
- ✅ No forward references within chapter
- ✅ All 11 success evals supported by lessons

---

## Implementation Notes

### Parallel Execution Opportunities

**Within Each Lesson**:
- Code examples can be written concurrently (marked with [P])
- CoLearning element integration can be parallelized (💬🎓🚀✨ tasks)
- Content sections are sequential (dependencies exist)

**Example** (Lesson 4):
- T048 (Example 2) + T049 (Example 6) can be written in parallel
- T050-T053 (CoLearning elements) can be written in parallel
- T040-T047 (content sections) must be sequential

### MVP Scope Recommendation

**Minimum Viable Chapter** (Lessons 1-4):
- Lesson 1: CPython fundamentals
- Lesson 2: Python 3.14 improvements
- Lesson 3: Traditional GIL understanding
- Lesson 4: Free-threaded Python (PRIMARY)

**Rationale**: Lessons 1-4 provide complete understanding of CPython + GIL evolution + free-threading. Students can stop here and still have foundational knowledge. Lessons 5-6 add practical application but aren't required for conceptual understanding.

**Full Chapter** (Lessons 1-6):
- Add Lesson 5: Concurrency decision-making (practical)
- Add Lesson 6: Capstone project (integration)

**Rationale**: Lessons 5-6 transform understanding into application. Students learn to CHOOSE and BUILD, not just understand. Essential for production readiness.

### Policy Note for Lesson Authors

**"Try With AI" Section Context**:
- **Before Part 1 (AI tools not yet taught)**: Use ChatGPT web interface in "Try With AI" prompts
- **After Part 1 (Claude Code/Gemini CLI taught)**: Instruct students to "Use your AI companion (Claude Code or Gemini CLI)" with optional web alternative
- **Chapter 29 (Part 4, after tools taught)**: ALL prompts assume students have Claude Code or Gemini CLI available
- **Format**: Always end with "Try With AI" section ONLY (no "Key Takeaways", "Summary", or "What's Next" after)

### Constitutional Compliance

**All tasks respect**:
- ✅ AI-Native Learning pattern (describe intent → explore → validate → learn from errors)
- ✅ Graduated Teaching Pattern (Book teaches foundational, AI handles complex, AI orchestrates scale)
- ✅ Cognitive Load Limits (B1: max 10 concepts)
- ✅ CoLearning Pedagogy (💬🎓🚀✨ throughout, not just at end)
- ✅ Lesson Closure Rule (Try With AI ONLY)
- ✅ No Forward References (Part 4 language, no SDD terminology)
- ✅ Python 3.14+ (modern type hints, free-threading production-ready)

---

## Task Completion Tracking

**As tasks are completed, mark them with `[x]` and update the summary**:

- Phase 1 (Setup): 0/3 complete
- Phase 2 (Lesson 1): 0/12 complete
- Phase 3 (Lesson 2): 0/12 complete
- Phase 4 (Lesson 3): 0/12 complete
- Phase 5 (Lesson 4): 0/15 complete
- Phase 6 (Lesson 5): 0/14 complete
- Phase 7 (Lesson 6): 0/12 complete
- Phase 8 (Validation): 0/6 complete

**Total Progress**: 0/86 tasks complete (0%)

---

**END OF TASKS**

**Next Step**: Begin implementation with Phase 1 (Setup). Then proceed sequentially through Lessons 1-6 with validation gates after each lesson.
