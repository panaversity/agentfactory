# Chapter 29 Implementation Plan: CPython and GIL

**Status**: Ready for Implementation
**Created**: 2025-11-09
**Approved Spec**: `specs/part-4-chapter-29/spec.md`
**Chapter Type**: Technical/Code-Focused
**Complexity Tier**: Advanced (B1-B2)

---

## Overview

Chapter 29 teaches the CPython interpreter and the revolutionary Global Interpreter Lock (GIL) evolution in Python 3.14. This chapter represents a paradigm shift in Python: for 30 years the GIL prevented true parallel execution; in October 2025, Python 3.14 made free-threading production-ready, reducing overhead from 40% to 5-10% and enabling genuine multi-core parallelism.

The chapter builds proficiency across three dimensions:

1. **CPython Internals** (B1 proficiency): How the reference implementation executes Python code
2. **GIL Evolution** (B1-B2 proficiency): From traditional GIL constraints to free-threaded Python capabilities
3. **AI-Native Applications** (B2 proficiency): Building production multi-agent systems with true parallelism

**Estimated Total Time**: 6-8 hours across 6 lessons
**Target Audience**: Advanced learners who have completed 27 prior chapters (Parts 1-3)
**Prerequisites**: Chapters 1-28, with special emphasis on Ch 17 (loops), Ch 21 (exceptions), Ch 28 (asyncio)

---

## Proficiency Progression Validation

Chapter-wide proficiency progression follows CEFR standards with smooth B1→B1-B2→B2 escalation:

| Lesson | Title | CEFR Level | Concepts Count | New vs Review | Validation |
|--------|-------|-----------|------------------|---------------|----|
| 1 | What is CPython? | B1 | 7 | 7 new, 0 review | ✅ Foundational B1 concepts; no prerequisites violated |
| 2 | CPython Performance Evolution | B1 | 6 | 6 new, 0 review | ✅ Expands CPython understanding; no forward references |
| 3 | The Traditional GIL | B1-B2 | 8 | 8 new, 2 review (L1-2) | ✅ Increases complexity; introduces GIL as constraint; ready for L4 |
| 4 | Free-Threaded Python (3.14+) | B1-B2 | 10 | 10 new, 3 review (L1-3) | ✅ PRIMARY LESSON: Max concepts justified; foundational for L5-6 |
| 5 | Choosing Your Concurrency Approach | B1-B2 | 9 | 9 new, 5 review (L1-4) | ✅ Heavy review; decision framework; most practical lesson |
| 6 | Capstone - Multi-Agent System | B2 | 8 | 3 new (production context), 5 review (L1-5) | ✅ Synthesis: integrates all prior lessons; minimal new concepts |

**Progression Check**: ✅ PASS
- Smooth B1 → B1-B2 → B1-B2 → B1-B2 → B2 escalation (no zigzag)
- Cognitive load respects tier limits (A1:5, A2:7, B1:10)
- Each lesson builds on previous; no forward references within chapter
- Review concepts increase toward capstone (consolidation pattern)

---

## Lesson 1: What is CPython?

**CEFR Proficiency**: B1 (Intermediate - Foundation)
**Estimated Time**: 60 minutes
**Learning Objectives**: LO-1 (Understand CPython architecture)
**Success Evals Supported**: EVAL-001 (CPython internals understanding)
**Cognitive Load**: 7 concepts (within B1 max of 10)

### Skills Taught

1. **CPython Implementation Recognition**
   - **Proficiency**: B1
   - **Category**: Technical
   - **Bloom's Level**: Understand (explain differences)
   - **DigComp Area**: Information (find, evaluate Python implementation info)
   - **Measurable at This Level**: Student can explain what CPython is and why it's called the "reference implementation" without reference materials

2. **Bytecode Execution Comprehension**
   - **Proficiency**: B1
   - **Category**: Technical
   - **Bloom's Level**: Understand (describe process flow)
   - **DigComp Area**: Problem-Solving (understand how systems work)
   - **Measurable at This Level**: Student can describe the pipeline: .py → bytecode (.pyc) → interpreter → execution

3. **Alternative Implementation Awareness**
   - **Proficiency**: A2 (Basic)
   - **Category**: Conceptual
   - **Bloom's Level**: Understand (recognize differences)
   - **DigComp Area**: Information (identify alternatives)
   - **Measurable at This Level**: Student can name and briefly describe PyPy, Jython, IronPython, MicroPython

4. **Implementation Detection**
   - **Proficiency**: B1
   - **Category**: Technical
   - **Bloom's Level**: Apply (write detection code)
   - **DigComp Area**: Problem-Solving (apply tool to solve problem)
   - **Measurable at This Level**: Student writes `platform.python_implementation()` code without reference

5. **Reference Counting Mechanism**
   - **Proficiency**: A2 (Basic)
   - **Category**: Technical
   - **Bloom's Level**: Understand (explain concept)
   - **DigComp Area**: Information (understand memory management)
   - **Measurable at This Level**: Student explains reference counting prevents memory leaks; knows garbage collection complements it

6. **Bytecode Compilation Process**
   - **Proficiency**: A2 (Basic)
   - **Category**: Technical
   - **Bloom's Level**: Understand (describe process)
   - **DigComp Area**: Problem-Solving (understand how compilation works)
   - **Measurable at This Level**: Student describes .py → .pyc conversion; explains why .pyc files exist

7. **Python Ecosystem Role**
   - **Proficiency**: B1
   - **Category**: Conceptual
   - **Bloom's Level**: Understand (explain significance)
   - **DigComp Area**: Information (evaluate CPython's role)
   - **Measurable at This Level**: Student explains why CPython is called "reference" and its implications for Python development

### Key Concepts (7 total)

1. CPython as reference implementation (official, written in C)
2. Python execution pipeline (source → bytecode → interpreter)
3. Bytecode format (.pyc files, compilation)
4. Reference counting + garbage collection (memory management)
5. Alternative implementations (PyPy JIT, Jython JVM, IronPython .NET, MicroPython embedded)
6. C API and C extensions (why CPython is written in C)
7. CPython's role in Python ecosystem (authority, compatibility)

### Content Outline

- **Section 1: What is CPython?**
  - Define: CPython as reference implementation
  - Why "reference"? (Official, Python Software Foundation stewardship)
  - Written in C (not Python)
  - Used by default with `python` command

- **Section 2: The Python Execution Pipeline**
  - Source code (.py) → Bytecode compilation (.pyc)
  - CPython interpreter reads bytecode
  - Interpreter executes on machine
  - Memory management (reference counting + GC)

- **Section 3: Memory Management Deep Dive**
  - Reference counting: Every object has refcount
  - When refcount drops to 0, object is freed
  - Garbage collection handles circular references
  - Why this matters for performance

- **Section 4: Alternative Implementations**
  - PyPy: JIT compilation for speed
  - Jython: Runs on Java Virtual Machine
  - IronPython: Runs on .NET
  - MicroPython: Embedded systems
  - When to use each

- **Section 5: How to Detect Your Implementation**
  - `platform.python_implementation()` code
  - Always returns "CPython" on standard Python
  - Useful for CI/CD checks

### Code Examples Integration

- **Example 1**: CPython Detection
  - **Purpose**: Verify implementation type
  - **Complexity**: A2 (simple detection, foundational)
  - **When**: Early in content (hook into CPython topic)
  - **Concepts Used**: `platform` module, function calls, print statements

### CoLearning Elements Placement

💬 **AI Colearning Prompt 1** (After defining CPython concept - 5 min mark)
> "Ask your AI: 'Why is CPython written in C instead of Python? What are the advantages and disadvantages of this design choice?' Then explore: 'What would happen if Python was implemented in Python instead?'"

**Expected Exploration**: Student discovers performance tradeoffs, bootstrapping problem, C integration capabilities

🎓 **Instructor Commentary 1** (After explaining bytecode pipeline - 20 min mark)
> "In AI-native development, you don't memorize how .pyc files work—you understand WHEN it matters (debugging, caching, deployment) and ask AI when you hit the boundary. Syntax is cheap; understanding execution is gold."

🚀 **CoLearning Challenge 1** (After showing reference counting concept - 35 min mark)
> "Tell your AI: 'Generate a Python program that shows reference counting in action. Create objects, increase refcount, show garbage collection. Explain what happens at each step.'"

**Expected Outcome**: Student writes/reads code demonstrating memory management; develops intuition for when GC matters

💬 **AI Colearning Prompt 2** (After exploring alternatives - 45 min mark)
> "Ask your AI: 'If I want to use PyPy instead of CPython for my project, what would I need to change? Are there compatibility issues? Performance gains?' Then analyze: 'When would this tradeoff be worth it?'"

**Expected Exploration**: Student weighs performance vs compatibility; develops decision-making patterns

✨ **Teaching Tip 1** (When demonstrating detection code - 50 min mark)
> "Use Claude Code to explore edge cases interactively: 'Show me what `platform.python_implementation()` returns on PyPy, Jython, and IronPython. Explain why the output differs and what this means for conditional code.'"

**Expected Outcome**: Student learns to test assumptions with AI; builds confidence with implementation differences

### Try With AI (4 Prompts - Final 10 min)

**Prompt 1 (Remember - Recall Facts)**:
> "In your own words, explain what CPython is and why it's called the 'reference implementation.' Then ask your AI: 'Is my explanation accurate? What am I missing? How would you explain it differently?'"

**Expected Outcome**:
- Student recalls definition and explains it
- AI validates and fills gaps
- Student achieves A1→B1 transition (recall + understanding)
- Time: 2 min

**Prompt 2 (Understand - Explain Meaning)**:
> "Ask your AI: 'Walk me through the Python execution pipeline step-by-step. What happens when I type `python script.py`? What is the CPython interpreter doing at each stage?'"

**Expected Outcome**:
- Student understands bytecode compilation and interpreter role
- Visual mental model forms
- Connection to performance (future GIL topic)
- Time: 2 min

**Prompt 3 (Apply - Use Knowledge)**:
> "Tell your AI: 'Help me write a script that detects what Python implementation I'm using and prints helpful information about it. Include version, implementation name, and 2-3 facts about the implementation.'"

**Expected Outcome**:
- Student applies `platform` module knowledge
- Writes working detection code
- Understands practical use case
- Time: 3 min

**Prompt 4 (Analyze - Connect to Context)**:
> "Ask your AI: 'How does CPython's design (reference counting + C API) affect the GIL (which we'll learn about next)? What connection exists between memory management and threading?' Then speculate: 'Why might this design choice become important when we add free-threading?'"

**Expected Outcome**:
- Student previews GIL dependency on CPython design
- Forward-looking thinking activated
- Cognitive hook for Lesson 2
- Sets context for L3-4 GIL content
- Time: 3 min

### Cognitive Load Check

- **Concepts Count**: 7/10 (advanced tier max) ✅
- **New vs Review**: 7 new, 0 review (L1 foundational)
- **Complexity Justification**: B1 proficiency allows 7-10 concepts. All concepts are foundational (what is X, how does Y work) without diving into deep technical implementation details. Reference counting and garbage collection are explained at conceptual level, not internal algorithm level.

### Pedagogical Flow

1. **Hook** (0-5 min): What is CPython? Why does it matter?
2. **Concept 1-2** (5-20 min): Execution pipeline, bytecode
3. **Concept 3-5** (20-40 min): Memory management, alternatives
4. **CoLearning + Challenge** (40-50 min): AI-guided exploration
5. **Try With AI** (50-60 min): Synthesis of all 4 Bloom's levels

**Engagement Pattern**: Traditional explanation → AI-guided exploration → student writing code

---

## Lesson 2: CPython Performance Evolution (Python 3.14)

**CEFR Proficiency**: B1 (Intermediate)
**Estimated Time**: 60 minutes
**Learning Objectives**: LO-1 (continued - CPython internals), connect to GIL impact
**Success Evals Supported**: EVAL-001 (CPython internals understanding)
**Cognitive Load**: 6 concepts (within B1 max of 10)

### Skills Taught

1. **Performance Optimization Recognition**
   - **Proficiency**: B1
   - **Category**: Technical
   - **Bloom's Level**: Understand (recognize optimization strategies)
   - **DigComp Area**: Problem-Solving (understand performance tuning)
   - **Measurable at This Level**: Student can name and briefly describe 3 performance optimizations in Python 3.14

2. **Interpreter Architecture Knowledge**
   - **Proficiency**: A2 (Basic)
   - **Category**: Technical
   - **Bloom's Level**: Understand (grasp architectural concepts)
   - **DigComp Area**: Problem-Solving (understand system design)
   - **Measurable at This Level**: Student understands "tail-call interpreter" reduces bytecode overhead

3. **Garbage Collection Fundamentals**
   - **Proficiency**: B1
   - **Category**: Technical
   - **Bloom's Level**: Understand (explain GC implications)
   - **DigComp Area**: Problem-Solving (understand memory management)
   - **Measurable at This Level**: Student explains incremental GC reduces pause times; recognizes this matters for latency-sensitive applications

4. **Deferred Annotation Awareness**
   - **Proficiency**: A2 (Basic)
   - **Category**: Technical
   - **Bloom's Level**: Understand (recognize modern Python feature)
   - **DigComp Area**: Information (understand new language features)
   - **Measurable at This Level**: Student recognizes PEP 649/749 reduces annotation processing time

5. **AI Workload Performance Awareness**
   - **Proficiency**: B1
   - **Category**: Conceptual
   - **Bloom's Level**: Understand (connect performance to AI applications)
   - **DigComp Area**: Problem-Solving (understand domain-specific implications)
   - **Measurable at This Level**: Student explains why faster execution and lower latency matter for multi-agent AI systems

6. **Performance Benchmarking Intuition**
   - **Proficiency**: A2 (Basic)
   - **Category**: Technical
   - **Bloom's Level**: Understand (recognize benchmark metrics)
   - **DigComp Area**: Problem-Solving (interpret performance data)
   - **Measurable at This Level**: Student understands pyperformance benchmarks measure real-world application performance

### Key Concepts (6 total)

1. Tail-call interpreter architecture (3-5% faster bytecode execution)
2. Incremental garbage collection (order of magnitude faster pause times)
3. Deferred annotation evaluation (PEP 649/749)
4. Remote debugging interface (PEP 768 - brief mention)
5. Performance impact on AI workloads (faster inference, lower latency)
6. pyperformance benchmark suite (how we measure progress)

### Content Outline

- **Section 1: Python 3.14 Performance Improvements Overview**
  - Timeline: 3.13 experimental → 3.14 production
  - Multiple improvements compound
  - Impact: 3-5% faster for typical workloads

- **Section 2: Tail-Call Interpreter (Specializing Adaptive Interpreter)**
  - What changed: More efficient bytecode dispatch
  - Why: Reduces interpreter overhead
  - Impact: Measured via pyperformance suite
  - When it matters: Long-running processes, tight loops

- **Section 3: Incremental Garbage Collection**
  - Traditional GC: Stop-the-world pause
  - Incremental GC: Pause time split across multiple small pauses
  - Impact: Lower latency for real-time applications
  - When it matters: Web servers, event loops, real-time AI inference

- **Section 4: Deferred Annotations (Modern Type Hints)**
  - Forward references now allowed at runtime
  - Annotations evaluated lazily (not at import time)
  - Impact: Faster imports, cleaner code
  - When it matters: Large applications with many type hints

- **Section 5: Performance for AI Workloads**
  - AI systems run tight computation loops
  - Lower interpreter overhead = faster inference
  - Latency matters for real-time agents
  - Context: Why this sets stage for multi-agent systems

### Code Examples Integration

- No heavy code examples in this lesson (conceptual + benchmarking focus)
- Focus on interpreting pyperformance results
- Connection to GIL behavior in Lesson 3

### CoLearning Elements Placement

💬 **AI Colearning Prompt 1** (After explaining tail-call interpreter - 15 min mark)
> "Ask your AI: 'Explain what a tail-call interpreter is. How does reducing bytecode dispatch overhead help? Give a concrete example of where this matters.' Then ask: 'Show me how to measure the performance difference in Python 3.14 vs 3.13.'"

**Expected Exploration**: Student understands interpreter architecture at conceptual level; learns to benchmark

🎓 **Instructor Commentary 1** (After GC explanation - 30 min mark)
> "Professional developers don't calculate GC pause times by hand—they measure with real data and ask AI for interpretation. You're learning to read performance profiles, not memorize algorithms. Semantics matter; syntax is tool."

🚀 **CoLearning Challenge 1** (After explaining incremental GC benefit - 40 min mark)
> "Tell your AI: 'I'm building an AI agent that responds to real-time requests. Incremental GC helps latency. Generate Python code that demonstrates GC pause time. How would I measure if incremental GC is working?'"

**Expected Outcome**: Student learns to instrument code for performance; practices measurement mindset

💬 **AI Colearning Prompt 2** (After performance for AI section - 50 min mark)
> "Ask your AI: 'A 3-5% interpreter speedup doesn't sound huge. Why does it matter for AI systems? Show me the math: how much faster is a 1-billion operation loop with 5% speedup?'"

**Expected Exploration**: Student understands compounding effects; develops intuition for performance sensitivity

✨ **Teaching Tip 1** (When discussing benchmarks - 55 min mark)
> "Use Claude Code to run pyperformance benchmarks: 'Show me how to run the pyperformance suite on my machine. What are the key benchmarks? Explain which ones are most relevant for AI workloads.' This is how real developers measure progress."

**Expected Outcome**: Student learns to validate claims with data; builds tool literacy

### Try With AI (4 Prompts - Final 10 min)

**Prompt 1 (Remember - Recall Facts)**:
> "List 3 performance improvements in Python 3.14 that we discussed. Then ask your AI: 'Did I get these right? Are there other improvements I should know about?'"

**Expected Outcome**:
- Student recalls incremental improvements
- AI fills in gaps
- Time: 2 min

**Prompt 2 (Understand - Explain Implications)**:
> "Ask your AI: 'Why does reducing garbage collection pause time matter for AI agents? How does latency sensitivity apply to multi-agent systems?' Then explore: 'What are the worst-case scenarios without incremental GC?'"

**Expected Outcome**:
- Student understands real-world impact
- Conceptual foundation for GIL lesson
- Time: 2 min

**Prompt 3 (Apply - Measure Performance)**:
> "Tell your AI: 'Help me set up pyperformance benchmarks on my machine. Run the suite and show me how Python 3.14 compares to an earlier version (if available). Explain the results.'"

**Expected Outcome**:
- Student learns benchmarking workflow
- Gains practical measurement experience
- Sets context for L5 benchmarking
- Time: 3 min

**Prompt 4 (Analyze - Connect Performance to Future Topics)**:
> "Ask your AI: 'How does Python 3.14's performance improvement interact with the GIL? If GIL prevents parallelism but interpreter is faster, what does that mean for CPU-bound AI tasks? Why might free-threading be the next step?'"

**Expected Outcome**:
- Student connects L1-2 to L3-4 topics
- Sees performance as foundational to GIL discussion
- Cognitive bridge between lessons
- Time: 3 min

### Cognitive Load Check

- **Concepts Count**: 6/10 (advanced tier max) ✅
- **New vs Review**: 6 new, 0 review (L2 builds on L1 CPython foundation)
- **Complexity Justification**: B1 proficiency. Concepts are performance-focused (what improved, why it matters) without deep algorithmic detail. PEP references brief. Focus on intuition and measurement, not implementation.

### Pedagogical Flow

1. **Connection** (0-5 min): How does L1 CPython lead to performance?
2. **Concept 1-3** (5-35 min): Tail-call interpreter, incremental GC, deferred annotations
3. **Context** (35-45 min): Why performance matters for AI workloads
4. **CoLearning + Challenge** (45-55 min): AI-guided measurement and benchmarking
5. **Try With AI** (55-60 min): Practical benchmarking + forward connection to GIL

**Engagement Pattern**: Conceptual explanation → AI-guided benchmarking → student measures performance

---

## Lesson 3: The Traditional GIL (Pre-3.13)

**CEFR Proficiency**: B1-B2 (Intermediate-Advanced)
**Estimated Time**: 90 minutes (1.5 hours)
**Learning Objectives**: LO-2 (Master GIL evolution - Part 1: Traditional GIL)
**Success Evals Supported**: EVAL-002 (GIL evolution comprehension), EVAL-001 (CPython internals continued)
**Cognitive Load**: 8 concepts (within B1-B2 max of 10)

### Skills Taught

1. **GIL Mechanism Understanding**
   - **Proficiency**: B1-B2
   - **Category**: Technical
   - **Bloom's Level**: Understand + Apply (explain GIL, apply knowledge to scenarios)
   - **DigComp Area**: Problem-Solving (understand threading constraints)
   - **Measurable at This Level**: Student explains what GIL is, why it exists, what it prevents WITHOUT references

2. **CPU-Bound vs I/O-Bound Distinction** (CRITICAL CONCEPT)
   - **Proficiency**: B1
   - **Category**: Technical
   - **Bloom's Level**: Understand + Apply (classify tasks, predict GIL impact)
   - **DigComp Area**: Problem-Solving (categorize computational patterns)
   - **Measurable at This Level**: Student classifies 5 real tasks as CPU or I/O-bound; predicts GIL impact on each

3. **GIL Behavior in Threading Scenarios**
   - **Proficiency**: B1-B2
   - **Category**: Technical
   - **Bloom's Level**: Analyze (why does GIL matter in threading)
   - **DigComp Area**: Problem-Solving (understand concurrency constraints)
   - **Measurable at This Level**: Student explains why traditional threading doesn't parallelize CPU-bound tasks; predicts behavior

4. **Memory Safety and C API Implications**
   - **Proficiency**: A2-B1
   - **Category**: Technical
   - **Bloom's Level**: Understand (why GIL simplifies memory safety)
   - **DigComp Area**: Problem-Solving (understand C API design)
   - **Measurable at This Level**: Student explains why GIL was needed for reference counting; understands C extension complexity

5. **Traditional GIL Workarounds**
   - **Proficiency**: B1
   - **Category**: Technical
   - **Bloom's Level**: Understand + Apply (know and apply workarounds)
   - **DigComp Area**: Problem-Solving (select appropriate pattern)
   - **Measurable at This Level**: Student names multiprocessing and C extensions; explains when each applies

6. **GIL Release During I/O**
   - **Proficiency**: B1
   - **Category**: Technical
   - **Bloom's Level**: Understand (explain GIL release mechanism)
   - **DigComp Area**: Problem-Solving (understand threading works for I/O)
   - **Measurable at This Level**: Student explains why threading works for network/file I/O

7. **Performance Consequences of GIL**
   - **Proficiency**: B1-B2
   - **Category**: Technical
   - **Bloom's Level**: Apply + Analyze (predict GIL impact on performance)
   - **DigComp Area**: Problem-Solving (evaluate algorithm selection)
   - **Measurable at This Level**: Student predicts performance degradation (or no benefit) when using threading for CPU-bound work

8. **Historical Context and Design Tradeoffs**
   - **Proficiency**: A2-B1
   - **Category**: Conceptual
   - **Bloom's Level**: Understand (appreciate why GIL was designed)
   - **DigComp Area**: Information (understand design history)
   - **Measurable at This Level**: Student explains why GIL made sense in 1989 when Python was created; recognizes this shaped 30 years of development

### Key Concepts (8 total)

1. GIL (Global Interpreter Lock) definition and purpose
2. Reference counting vulnerability without lock
3. Why GIL simplifies C API (memory safety)
4. CPU-bound vs I/O-bound workloads (CRITICAL distinction)
5. Threading works for I/O (GIL releases during I/O operations)
6. Threading fails for CPU-bound (GIL prevents parallelism)
7. Multiprocessing as CPU-bound workaround (separate interpreters)
8. C extensions can release GIL (power and danger)

### Content Outline

- **Section 1: What is the GIL?**
  - Global Interpreter Lock (mutex)
  - Protects CPython interpreter state
  - Simplifies memory management (reference counting)
  - Owned by one thread at a time

- **Section 2: Why Does the GIL Exist?**
  - CPython uses reference counting (not full GC)
  - Reference counting NOT thread-safe without synchronization
  - GIL is the simplest solution (one lock for entire interpreter)
  - Cost: Prevents true parallelism

- **Section 3: CPU-Bound vs I/O-Bound (THE CRITICAL CONCEPT)**
  - **CPU-Bound**: Heavy computation (prime calculation, data processing, AI inference)
    - Thread must own GIL continuously
    - Other threads starve
    - Threading provides NO parallelism
  - **I/O-Bound**: Waiting for external resources (network, files, database)
    - Thread releases GIL during I/O wait
    - Other threads can run
    - Threading provides pseudo-parallelism
  - Real examples from AI domain: inference (CPU) vs web requests (I/O)

- **Section 4: Threading Doesn't Work for CPU-Bound**
  - Traditional Python threading creates threads
  - But GIL prevents true parallel execution
  - Only one thread executes Python bytecode at a time
  - Time-slicing (thread switches) adds overhead
  - Result: SLOWER than single-threaded for CPU-bound

- **Section 5: Threading Works Fine for I/O-Bound**
  - I/O operations release GIL (OS-level wait)
  - Other threads can execute while one waits
  - Pseudo-parallelism sufficient for I/O
  - This is why asyncio works (similar concept)

- **Section 6: CPU-Bound Workarounds in Traditional Python**
  - **Multiprocessing**: Separate Python interpreters, separate GILs
    - True parallelism on multi-core
    - Trade-off: Communication overhead, memory duplication
  - **C Extensions**: Release GIL in C code
    - Power: True parallelism for CPU-bound work
    - Danger: C code must be thread-safe
    - NumPy, pandas use this pattern

- **Section 7: The 30-Year Design Tradeoff**
  - GIL made sense in 1989 (single-core machines)
  - Simplified interpreter design for 30 years
  - Became bottleneck as multi-core became universal
  - This sets stage for Python 3.14 free-threading

### Code Examples Integration

- **Example 3**: CPU-Bound vs I/O-Bound Demonstration
  - **Purpose**: Show the difference between workload types
  - **Complexity**: B1 (intermediate - loops, functions, timing)
  - **When**: Early in L3 content (hook into GIL impact)
  - **Concepts Used**: Functions, loops, `time` module, CPU computation, I/O simulation

- **Example 4**: Benchmark - Traditional Threading
  - **Purpose**: Show GIL limitation on CPU-bound tasks
  - **Complexity**: B1-B2 (threading concepts, benchmarking)
  - **When**: After explaining GIL doesn't parallelize CPU (30 min mark)
  - **Concepts Used**: `threading` module, threads, CPU-bound task, timing

### CoLearning Elements Placement

💬 **AI Colearning Prompt 1** (After defining GIL - 10 min mark)
> "Ask your AI: 'Explain the Global Interpreter Lock in simple terms. Why does CPython need it? What would happen if we removed it?' Then explore: 'What does "thread-safe" mean in this context?'"

**Expected Exploration**: Student develops intuition for why GIL was needed; understands thread safety implications

🎓 **Instructor Commentary 1** (After CPU vs I/O distinction - 25 min mark)
> "In production AI systems, you classify every workload as CPU-bound or I/O-bound BEFORE choosing a concurrency strategy. You don't guess; you measure. Semantics (understanding workload type) is gold; syntax (writing threading code) is cheap."

🚀 **CoLearning Challenge 1** (After explaining threading fails for CPU - 40 min mark)
> "Tell your AI: 'Generate code that proves threading makes CPU-bound tasks slower. Create a CPU-intensive function, run it with 1 thread vs 4 threads, show timing. Then explain why adding threads made it slower, not faster.'"

**Expected Outcome**: Student experiences GIL limitation firsthand; develops empirical validation mindset

💬 **AI Colearning Prompt 2** (After showing threading works for I/O - 55 min mark)
> "Ask your AI: 'Show me an example where threading DOES provide speedup for I/O-bound work. How many concurrent network requests can 4 threads handle? Explain why this works despite the GIL.'"

**Expected Exploration**: Student understands GIL release during I/O; recognizes threading is right tool for right job

🎓 **Instructor Commentary 2** (After multiprocessing explanation - 65 min mark)
> "Multiprocessing is more powerful than threading for CPU work—each process has its own GIL. But it costs more (memory, startup, communication). Professional developers measure tradeoffs. This is where free-threading (Lesson 4) changes everything."

🚀 **CoLearning Challenge 2** (After workarounds section - 75 min mark)
> "Tell your AI: 'I have a CPU-bound AI task that needs parallelism. Should I use multiprocessing or C extensions? Compare: setup time, memory usage, communication overhead. When is each worth it?'"

**Expected Outcome**: Student practices decision-making with AI support; prepares for L5 decision framework

✨ **Teaching Tip 1** (When showing threading benchmark - 50 min mark)
> "Use Claude Code to measure GIL behavior interactively: 'Run a CPU-bound task with increasing thread counts. Show me the graph: does timing improve or get worse? Explain the overhead.' This is how you learn concurrency intuitively."

**Expected Outcome**: Student learns to benchmark and interpret results; builds empirical reasoning

### Try With AI (4 Prompts - Final 15 min)

**Prompt 1 (Remember - Recall Key Concepts)**:
> "Explain in one sentence: What is the GIL? Then ask your AI: 'Did I capture the essence? What would you add?'"

**Expected Outcome**:
- Student recalls core concept
- AI provides authoritative definition
- Time: 2 min

**Prompt 2 (Understand - Explain GIL Impact)**:
> "Ask your AI: 'Walk me through what happens when I create 4 threads to run a CPU-bound task. What does the GIL do? Why don't I get 4x speedup?' Then sketch: 'How would you visualize the GIL constraint with a diagram?'"

**Expected Outcome**:
- Student understands GIL mechanics
- Develops visual model
- Time: 3 min

**Prompt 3 (Apply - Classify and Predict)**:
> "Tell your AI: 'I'm building 5 AI components: (1) transformer inference, (2) web API calls, (3) vector database queries, (4) prompt generation, (5) file I/O. For each, is it CPU-bound or I/O-bound? What threading strategy would you use?'"

**Expected Outcome**:
- Student applies CPU vs I/O classification
- Begins to think strategically about concurrency
- AI guides decision-making
- Time: 4 min

**Prompt 4 (Analyze - Connect to Future Solutions)**:
> "Ask your AI: 'The GIL has been in Python for 30 years. What finally changed that made removal possible in 3.14? How does free-threading solve the problem that GIL existed to solve?' Then reflect: 'What were the tradeoffs of removing the GIL?'"

**Expected Outcome**:
- Student understands historical context
- Eager to learn free-threading solution
- Recognizes paradigm shift coming
- Time: 6 min

### Cognitive Load Check

- **Concepts Count**: 8/10 (advanced tier max) ✅
- **New vs Review**: 8 new, 2 review (L1 CPython + L2 performance as context)
- **Complexity Justification**: B1-B2 proficiency allows 8-10 concepts. This is the heaviest concept lesson but justified: GIL is FOUNDATIONAL to understanding free-threading. CPU vs I/O distinction is THE critical concept that students must master. Heavy cognitive load is appropriate here because mastery is required for Lesson 4.

### Pedagogical Flow

1. **Foundation** (0-10 min): What is GIL? Why does it exist?
2. **Distinction** (10-30 min): CPU-bound vs I/O-bound (most important concept)
3. **Impact** (30-50 min): How GIL affects threading scenarios
4. **Workarounds** (50-75 min): Multiprocessing and C extensions as solutions
5. **CoLearning + Challenge** (75-85 min): AI-guided exploration and classification
6. **Try With AI** (85-90 min): Synthesis + forward connection to free-threading

**Engagement Pattern**: Conceptual explanation → classification practice → benchmarking → future connection

---

## Lesson 4: Free-Threaded Python (3.14+ Production Ready)

**CEFR Proficiency**: B1-B2 (Intermediate-Advanced - PRIMARY LESSON)
**Estimated Time**: 120 minutes (2 hours)
**Learning Objectives**: LO-2 (Master GIL evolution - Part 2: Free-Threading), LO-3 (Inform concurrency decisions), LO-4 (Build AI-native systems with free-threading)
**Success Evals Supported**: EVAL-002 (GIL evolution comprehension), EVAL-004 (Free-threading detection), EVAL-010 (AI-native context understanding)
**Cognitive Load**: 10 concepts (B1-B2 max of 10 - justified as PRIMARY LESSON)

### Skills Taught

1. **Free-Threading Concept Mastery**
   - **Proficiency**: B1-B2
   - **Category**: Technical
   - **Bloom's Level**: Understand + Apply (explain free-threading, apply knowledge)
   - **DigComp Area**: Problem-Solving (understand new concurrency model)
   - **Measurable at This Level**: Student explains how free-threading removes GIL and enables true parallelism WITHOUT references

2. **GIL Optional Architecture Understanding**
   - **Proficiency**: B1-B2
   - **Category**: Technical
   - **Bloom's Level**: Understand (grasp architectural shift)
   - **DigComp Area**: Problem-Solving (understand system redesign)
   - **Measurable at This Level**: Student explains GIL is now optional (not removed); understands Python 3.13/3.14/3.15 roadmap

3. **Free-Threading Installation and Detection**
   - **Proficiency**: B1-B2
   - **Category**: Technical
   - **Bloom's Level**: Apply (install and verify free-threading)
   - **DigComp Area**: Problem-Solving (configure systems)
   - **Measurable at This Level**: Student can install free-threaded Python and detect availability via `sys._is_gil_enabled()`

4. **Performance Characteristics Knowledge**
   - **Proficiency**: B1-B2
   - **Category**: Technical
   - **Bloom's Level**: Understand + Analyze (know overhead, predict gains)
   - **DigComp Area**: Problem-Solving (evaluate performance tradeoffs)
   - **Measurable at This Level**: Student explains 5-10% single-threaded overhead vs 2-10x multi-threaded gains; understands when overhead is worth it

5. **Lock-Free Data Structures Awareness**
   - **Proficiency**: A2-B1
   - **Category**: Technical
   - **Bloom's Level**: Understand (recognize thread-safe patterns)
   - **DigComp Area**: Problem-Solving (understand concurrency safety)
   - **Measurable at This Level**: Student explains built-in types (dict, list, set) have internal locks; knows when to use explicit locks

6. **Thread Safety in Free-Threading Context**
   - **Proficiency**: B1-B2
   - **Category**: Technical
   - **Bloom's Level**: Understand + Apply (apply thread safety patterns)
   - **DigComp Area**: Safety (protect data, prevent race conditions)
   - **Measurable at This Level**: Student writes code with proper locking for complex shared state; understands built-in types provide some protection

7. **PEP 703/779 Knowledge**
   - **Proficiency**: A2
   - **Category**: Technical
   - **Bloom's Level**: Understand (recognize official standards)
   - **DigComp Area**: Information (find authoritative documentation)
   - **Measurable at This Level**: Student knows PEP 703 is free-threading implementation; PEP 779 is adoption criteria

8. **Docker Image Variants for Free-Threading**
   - **Proficiency**: B1
   - **Category**: Technical
   - **Bloom's Level**: Apply (select correct image)
   - **DigComp Area**: Problem-Solving (configure deployment)
   - **Measurable at This Level**: Student knows `python:3.14t` is free-threaded; can select correct image for deployment

9. **Runtime GIL Control**
   - **Proficiency**: B1
   - **Category**: Technical
   - **Bloom's Level**: Apply (configure GIL at runtime)
   - **DigComp Area**: Problem-Solving (control system behavior)
   - **Measurable at This Level**: Student uses `PYTHON_GIL=1|0` to enable/disable GIL; understands runtime flexibility

10. **AI-Native Multi-Agent Enablement**
    - **Proficiency**: B2
    - **Category**: Conceptual
    - **Bloom's Level**: Understand + Evaluate (recognize free-threading benefits for AI)
    - **DigComp Area**: Problem-Solving (evaluate architectural patterns)
    - **Measurable at This Level**: Student explains why free-threading enables better multi-agent systems; can predict benefits for specific AI workloads

### Key Concepts (10 total - justified as PRIMARY LESSON)

1. Free-threading: GIL is now OPTIONAL (biggest Python change in 30 years)
2. Three-phase roadmap (3.13 experimental, 3.14 production, 3.15+ likely default)
3. Per-thread GIL vs global GIL (architecture change)
4. Lock-free data structures in free-threaded builds
5. Performance: 5-10% single-threaded overhead (small cost)
6. Performance: 2-10x multi-threaded gains on CPU-bound (large benefit)
7. Detection via `sys._is_gil_enabled()` (returns None/True/False)
8. Installation methods (official installers, Docker, Linux build)
9. Runtime control via `PYTHON_GIL` environment variable
10. Thread safety remains programmer responsibility (use locks for complex state)

### Content Outline

- **Section 1: The Paradigm Shift (Biggest Change in 30 Years)**
  - Traditional GIL: Fundamental constraint of CPython
  - Python 3.14: GIL is now OPTIONAL (can be disabled)
  - Not removed (for backward compatibility)
  - But can be disabled (enabling true parallelism)
  - Implications: Everything changes for multi-threaded programs

- **Section 2: Three-Phase Roadmap**
  - **Python 3.13 (2024)**: Experimental free-threaded builds (~40% overhead)
  - **Python 3.14 (Oct 2025)**: Officially supported (~5-10% overhead) ← WE ARE HERE
  - **Python 3.15+ (2026)**: Likely default (free-threaded will be standard)
  - Why gradual: Ecosystem compatibility, performance stabilization, tool updates

- **Section 3: How Free-Threading Works**
  - Traditional GIL: Single lock protecting interpreter state
  - Free-threaded: Per-thread state, lock-free data structures, biased locking
  - Result: True parallelism without sacrificing memory safety
  - Built-in types (dict, list, set) use internal locks (thread-safe at this level)
  - Complex shared state still needs explicit locks (threading.Lock)

- **Section 4: Installation Methods**
  - **macOS/Windows**: Official python.org installers with free-threading checkbox
  - **Linux**: Build from source `./configure --disable-gil`
  - **Docker**: Use `python:3.14t` image tag (t = free-threaded)
  - **Virtual environments**: Create venv on free-threaded Python base

- **Section 5: Detection Code**
  - `sys._is_gil_enabled()` returns:
    - `None`: Build doesn't support free-threading (older Python)
    - `True`: Build supports, GIL is enabled (default behavior)
    - `False`: Build supports, GIL is disabled (free-threading active)
  - Check BEFORE writing parallel code (don't assume)
  - Use for conditional logic (graceful fallback for older Python)

- **Section 6: Runtime Control**
  - `PYTHON_GIL=1`: Force GIL enabled (even on free-threaded build)
  - `PYTHON_GIL=0`: Force GIL disabled (free-threading mode)
  - Useful for: Debugging, compatibility testing, per-process configuration
  - Can be set in environment or Python code

- **Section 7: Performance Characteristics**
  - **Single-threaded**: 5-10% overhead (Python 3.13 was ~40%, 3.14 optimized)
  - **Multi-threaded on multi-core**: 2-10x speedup for CPU-bound (real parallelism)
  - **When overhead is worth it**: Speedup > overhead (usually 2+ threads helps)
  - **Benchmarking**: Don't guess; measure with real workload

- **Section 8: Thread Safety Responsibility**
  - GIL removal ≠ no locks needed
  - Built-in types (dict, list, set) safe from CPython perspective
  - But sequence of operations may still race
  - Example: `if key in dict: dict[key] += 1` needs lock (check-then-act race)
  - Use `threading.Lock()` for complex operations

- **Section 9: Connection to AI-Native Systems**
  - Multi-agent AI systems can now truly parallelize reasoning
  - 4-agent system on 4-core machine: near-linear speedup possible
  - Shared state (agent communication) becomes practical
  - Sets foundation for capstone project and Parts 11-14 deployment

- **Section 10: Integration with Existing Tools**
  - Asyncio: Thread-safe event loop in free-threading mode
  - Multiprocessing: Still valuable for process isolation
  - C Extensions: No longer need GIL release for parallelism

### Code Examples Integration

- **Example 2**: Free-Threading Detection
  - **Purpose**: Check if free-threading is available and active
  - **Complexity**: B1 (intermediate - requires understanding GIL concepts)
  - **When**: Early in L4 content (right after detection concept)
  - **Concepts Used**: `sys` module, conditional logic, boolean operations, type hints

- **Example 6**: Benchmark - Free-Threaded Python
  - **Purpose**: Demonstrate true parallelism with free-threading
  - **Complexity**: B2 (advanced - requires free-threaded Python build)
  - **When**: After explaining free-threading mechanics (60 min mark)
  - **Concepts Used**: Threading + free-threading mode, detection, benchmarking

### CoLearning Elements Placement

💬 **AI Colearning Prompt 1** (After defining free-threading paradigm shift - 15 min mark)
> "Ask your AI: 'Explain what changed between Python 3.13 and 3.14 regarding the GIL. Why is this the biggest change in Python's history? What becomes possible now that wasn't before?'"

**Expected Exploration**: Student understands magnitude of shift; anticipates implications

🎓 **Instructor Commentary 1** (After explaining per-thread state architecture - 30 min mark)
> "Understanding free-threading architecture is nice-to-know; using it is need-to-know. Professional developers detect availability, benchmark impact, then decide. Semantics (understanding tradeoffs) are gold; syntax (threading code) is cheap."

🚀 **CoLearning Challenge 1** (After installation methods - 40 min mark)
> "Tell your AI: 'Help me install free-threaded Python on my machine. Verify it's working. Run `sys._is_gil_enabled()` to confirm. Then compare: what's different between free-threaded and traditional Python?'"

**Expected Outcome**: Student experiences free-threading firsthand; builds hands-on confidence

💬 **AI Colearning Prompt 2** (After performance characteristics - 55 min mark)
> "Ask your AI: 'I'm concerned about 5-10% single-threaded overhead. When is this worth paying? Show me the math: at what thread count does the speedup justify the cost? Give real examples.'"

**Expected Exploration**: Student develops intuition for performance tradeoff analysis; understands breakeven points

💬 **AI Colearning Prompt 3** (After thread safety explanation - 70 min mark)
> "Ask your AI: 'The GIL is gone, but I still need locks? Why? Show me code that would race even with free-threading. How do I know when I need explicit locking?'"

**Expected Exploration**: Student understands thread safety responsibility hasn't disappeared; develops defensive thinking

🎓 **Instructor Commentary 2** (After AI-native context - 80 min mark)
> "This is where theory meets practice. Multi-agent systems are the book's core focus. Free-threading in 3.14 makes this practical on a single machine. In Parts 10-14 you'll scale this to Kubernetes and Ray. Today you learn the foundation."

🚀 **CoLearning Challenge 2** (After integration section - 90 min mark)
> "Tell your AI: 'I'm building a 4-agent AI system for my company. Free-threaded Python 3.14 vs multiprocessing vs asyncio—which should I choose? Compare: complexity, memory, communication, deployment. What are the tradeoffs?'"

**Expected Outcome**: Student applies free-threading to real decisions; practices AI-guided architectural thinking

✨ **Teaching Tip 1** (When demonstrating detection code - 25 min mark)
> "Use Claude Code to explore detection interactively: 'Show me what `sys._is_gil_enabled()` returns on different Python builds. Help me write code that gracefully handles all three cases (None, True, False). Test it.' This is how pros handle version compatibility."

**Expected Outcome**: Student learns defensive coding patterns; builds tool literacy

✨ **Teaching Tip 2** (When introducing benchmarking in L4 - 65 min mark)
> "Don't believe claims about free-threading speedup. Ask your AI: 'Generate a realistic benchmark for my AI workload. Measure traditional threading vs free-threaded vs multiprocessing. Show me the data.' Measurement is your friend."

**Expected Outcome**: Student develops empirical validation mindset; learns to distrust unsupported claims

### Try With AI (4 Prompts - Final 20 min)

**Prompt 1 (Remember - Recall Revolutionary Change)**:
> "Explain in 2-3 sentences: What changed between Python 3.13 and 3.14 regarding the GIL? Then ask your AI: 'Did I capture the significance? Why is this such a big deal for Python development?'"

**Expected Outcome**:
- Student recalls paradigm shift
- AI contextualizes importance
- Time: 3 min

**Prompt 2 (Understand - Explain Implications)**:
> "Ask your AI: 'Now that the GIL is optional, what becomes possible in Python that wasn't before? How does this change multi-threaded programming? What do developers need to relearn?'"

**Expected Outcome**:
- Student understands practical implications
- Recognizes this isn't just incremental improvement
- Time: 3 min

**Prompt 3 (Apply - Build Detection and Configuration)**:
> "Tell your AI: 'Help me write a Python script that: (1) detects if free-threading is available, (2) checks if GIL is currently enabled, (3) prints recommendations for using free-threading. Include error handling and helpful messages.'"

**Expected Outcome**:
- Student writes working free-threading detection code
- Learns practical application
- Creates tool they can reuse
- Time: 5 min

**Prompt 4 (Analyze - Connect to Multi-Agent AI Systems)**:
> "Ask your AI: 'I'm building an AI system with 4 agents running reasoning tasks. How would free-threaded Python improve this compared to traditional Python? Show me: (a) expected speedup, (b) code pattern changes, (c) deployment implications. Connect this to Ray and Kubernetes (Parts 11-14).'"

**Expected Outcome**:
- Student sees free-threading as foundation for multi-agent architecture
- Forward-looking connection to capstone and future parts
- Understands production context
- Time: 9 min

### Cognitive Load Check

- **Concepts Count**: 10/10 (B1-B2 maximum - fully justified) ✅
- **New vs Review**: 10 new, 3 review (L1 CPython, L3 GIL, L2 performance as foundation)
- **Complexity Justification**: This is the PRIMARY LESSON of the chapter. Achieving 10-concept maximum is JUSTIFIED because:
  - Free-threading is the revolutionary centerpiece
  - Mastery of all 10 concepts is REQUIRED for L5-6
  - Cognitive load is heavy but appropriate for "biggest change in 30 years"
  - Students have completed 27 prior chapters (ready for advanced content)
  - Scaffolding through CoLearning (5 AI prompts + 2 challenges) manages load

### Pedagogical Flow

1. **Paradigm Shift** (0-15 min): What changed? Why is this huge?
2. **Architecture** (15-35 min): How free-threading works (per-thread state, lock-free structures)
3. **Practical Aspects** (35-60 min): Installation, detection, runtime control
4. **Performance Reality** (60-75 min): Overhead vs gains; when it's worth it
5. **Thread Safety** (75-85 min): Locking still matters; responsibility
6. **Integration** (85-95 min): Connection to AI-native systems, existing tools
7. **CoLearning + Challenge** (95-110 min): AI-guided exploration and decision-making
8. **Try With AI** (110-120 min): Practical code + forward connection to Lessons 5-6

**Engagement Pattern**: Conceptual paradigm → architecture → practical detection → benchmarking → AI-guided application

---

## Lesson 5: Choosing Your Concurrency Approach

**CEFR Proficiency**: B1-B2 (Intermediate-Advanced)
**Estimated Time**: 90 minutes (1.5 hours)
**Learning Objectives**: LO-3 (Make informed concurrency decisions), LO-2 (GIL evolution continued)
**Success Evals Supported**: EVAL-003 (Concurrency decision-making), EVAL-002 (GIL evolution), EVAL-005 (Benchmark implementation)
**Cognitive Load**: 9 concepts (within B1-B2 max of 10)

### Skills Taught

1. **Decision Framework Application**
   - **Proficiency**: B1-B2
   - **Category**: Technical
   - **Bloom's Level**: Apply + Analyze (use framework to make decisions)
   - **DigComp Area**: Problem-Solving (select appropriate approach)
   - **Measurable at This Level**: Student chooses correct concurrency approach for 5 different scenarios without prompting

2. **Workload Classification**
   - **Proficiency**: B1
   - **Category**: Technical
   - **Bloom's Level**: Apply (classify workload correctly)
   - **DigComp Area**: Problem-Solving (categorize computational patterns)
   - **Measurable at This Level**: Student classifies 8+ real tasks as CPU/I/O-bound and predicts optimal concurrency approach

3. **Benchmarking Methodology**
   - **Proficiency**: B1-B2
   - **Category**: Technical
   - **Bloom's Level**: Apply + Analyze (design and interpret benchmarks)
   - **DigComp Area**: Problem-Solving (measure and evaluate performance)
   - **Measurable at This Level**: Student designs realistic benchmark comparing 3+ approaches; interprets results correctly

4. **Multiprocessing vs Free-Threaded Tradeoffs**
   - **Proficiency**: B1-B2
   - **Category**: Technical
   - **Bloom's Level**: Analyze + Evaluate (understand tradeoffs)
   - **DigComp Area**: Problem-Solving (evaluate architectural alternatives)
   - **Measurable at This Level**: Student compares multiprocessing (isolation) vs free-threading (shared memory); makes informed choice

5. **Asyncio for I/O (Integration with Lesson 28)**
   - **Proficiency**: B1
   - **Category**: Technical
   - **Bloom's Level**: Apply (use asyncio appropriately)
   - **DigComp Area**: Problem-Solving (select pattern for use case)
   - **Measurable at This Level**: Student connects Lesson 28 asyncio to GIL; explains why asyncio works for I/O

6. **Python 3.14 Asyncio Improvements**
   - **Proficiency**: A2-B1
   - **Category**: Technical
   - **Bloom's Level**: Understand + Apply (know and use improvements)
   - **DigComp Area**: Information (understand new features)
   - **Measurable at This Level**: Student uses asyncio CLI introspection (`ps`, `pstree`); explains 10-20% performance gain

7. **Multiprocessing in Python 3.14**
   - **Proficiency**: A2-B1
   - **Category**: Technical
   - **Bloom's Level**: Understand + Apply (know changes)
   - **DigComp Area**: Information (understand version features)
   - **Measurable at This Level**: Student explains `forkserver` default, `Process.interrupt()`, context inheritance

8. **Mixed Workload Architecture**
   - **Proficiency**: B1-B2
   - **Category**: Technical
   - **Bloom's Level**: Apply + Analyze (design hybrid patterns)
   - **DigComp Area**: Problem-Solving (combine approaches)
   - **Measurable at This Level**: Student designs system combining free-threaded + asyncio for CPU + I/O workload

9. **Performance Validation**
   - **Proficiency**: B1-B2
   - **Category**: Technical
   - **Bloom's Level**: Analyze + Evaluate (interpret benchmark results)
   - **DigComp Area**: Problem-Solving (validate choices empirically)
   - **Measurable at This Level**: Student analyzes benchmark output; explains results; assesses whether choice was correct

### Key Concepts (9 total)

1. Decision framework: CPU vs I/O workload distinction (review from L3)
2. Single-threaded option (no concurrency - often fastest)
3. Free-threaded Python (CPU-bound parallelism on single machine)
4. Multiprocessing (CPU-bound parallelism with isolation)
5. Asyncio (I/O-bound concurrency with single thread)
6. Free-threaded + asyncio (hybrid: CPU + I/O)
7. Benchmarking methodology (measure before and after)
8. Python 3.14 improvements (asyncio CLI tools, multiprocessing enhancements)
9. Real-world AI workload patterns

### Content Outline

- **Section 1: Decision Framework (CORE PRACTICAL SKILL)**
  - Workload type (CPU-bound vs I/O-bound)
  - Scale (single task vs parallel tasks)
  - Isolation requirements (shared state vs process isolation)
  - Result: Choose single-threaded, free-threaded, multiprocessing, or asyncio
  - Table-based framework for quick reference

- **Section 2: Single-Threaded (Baseline)**
  - Often the best choice (no concurrency overhead)
  - Use when: Sequential tasks, single task, no parallelism needed
  - Advantage: Simplicity, predictability, debugging
  - Disadvantage: Can't use multiple cores

- **Section 3: Free-Threaded Python (New in 3.14)**
  - Use when: CPU-bound parallel tasks, shared state, single machine
  - Advantage: True parallelism on multi-core, shared memory, Python native
  - Disadvantage: 5-10% single-threaded overhead, thread safety responsibility
  - Example: 4-agent AI system on 4-core machine (near-linear speedup)

- **Section 4: Multiprocessing (Traditional CPU Parallelism)**
  - Use when: CPU-bound parallel tasks, process isolation needed, older Python
  - Advantage: Truly independent processes, no shared GIL, mature ecosystem
  - Disadvantage: Memory overhead, communication complexity, startup cost
  - Example: Distributed AI workload across processes with queues

- **Section 5: Asyncio (I/O-Bound Concurrency)**
  - Use when: I/O-bound tasks (network, files), event-driven, single thread sufficient
  - Advantage: Memory efficient, single thread, large number of concurrent connections
  - Disadvantage: Doesn't parallelize CPU (GIL still active), callback complexity
  - Example: Web server handling 1000 concurrent requests
  - Connection to Ch 28: Asyncio is the recommended I/O pattern

- **Section 6: Python 3.14 Asyncio Improvements**
  - CLI introspection: `python -m asyncio ps <PID>` (list tasks), `pstree <PID>` (task hierarchy)
  - Thread-safe event loop (works with free-threading)
  - 10-20% faster single-threaded performance
  - Better exception handling

- **Section 7: Python 3.14 Multiprocessing Improvements**
  - `forkserver` default on Unix (safer than `fork`)
  - `Process.interrupt()` for graceful shutdown (not just terminate)
  - Context inheritance (`thread_inherit_context=True`)

- **Section 8: Benchmarking All Four Approaches**
  - Setup: CPU-bound task (prime calculation) or realistic AI workload
  - Measure: Execution time, CPU utilization, memory usage
  - Compare: Single-threaded baseline vs 3 parallel approaches
  - Interpret: Which is fastest? At what scale?

- **Section 9: Hybrid Approaches for Mixed Workloads**
  - Free-threaded + asyncio: True CPU parallelism + I/O handling
  - Example: 4 agents doing inference (CPU) + API calls (I/O) simultaneously
  - When to combine: Workload has both CPU and I/O components

- **Section 10: Real-World AI Workload Patterns**
  - AI inference (CPU-bound) → free-threaded Python
  - Web API (I/O-bound) → asyncio
  - Data pipeline (mixed) → free-threaded + asyncio
  - Distributed inference (isolation) → multiprocessing or containers

### Code Examples Integration

- **Example 4**: Benchmark - Traditional Threading
  - **Purpose**: Show GIL limitation (review from L3)
  - **Complexity**: B1-B2
  - **When**: Early in L5 content (baseline comparison)

- **Example 5**: Benchmark - Multiprocessing
  - **Purpose**: Show process-based parallelism
  - **Complexity**: B1-B2
  - **When**: Middle of L5 content (alternative to free-threading)

- **Example 6**: Benchmark - Free-Threaded Python
  - **Purpose**: Show true parallelism advantage
  - **Complexity**: B2
  - **When**: Mid-L5 content (central comparison)

- **Example 7**: Asyncio CLI Introspection
  - **Purpose**: Use Python 3.14 asyncio debugging tools
  - **Complexity**: B1-B2
  - **When**: Late in L5 content (debugging skill)

### CoLearning Elements Placement

💬 **AI Colearning Prompt 1** (After presenting decision framework - 15 min mark)
> "Ask your AI: 'I have 5 different workloads (list them). For each, classify as CPU or I/O-bound and recommend a concurrency approach. Explain your reasoning and the tradeoffs.'"

**Expected Exploration**: Student applies framework with AI guidance; develops classification intuition

🎓 **Instructor Commentary 1** (After showing benchmarks - 35 min mark)
> "Don't guess which approach is fastest. Measure it. Professional developers benchmark with real workloads. Semantics (understanding tradeoffs) are gold; syntax (threading code) is cheap. Let AI help you interpret the data."

🚀 **CoLearning Challenge 1** (After benchmarking section - 50 min mark)
> "Tell your AI: 'Help me create a comprehensive benchmark comparing all 4 approaches (single-threaded, free-threaded, multiprocessing, asyncio) on my AI workload. Generate the code, run it, interpret the results. Which approach wins and why?'"

**Expected Outcome**: Student builds practical benchmarking tool; learns empirical decision-making

💬 **AI Colearning Prompt 2** (After asyncio improvements - 60 min mark)
> "Ask your AI: 'Show me how to use the asyncio CLI tools (ps, pstree) to debug a running async program. What information do these provide? When would I use each one?'"

**Expected Exploration**: Student learns debugging techniques for async code; builds tool literacy

💬 **AI Colearning Prompt 3** (After hybrid approaches - 75 min mark)
> "Ask your AI: 'Design a system combining free-threaded Python + asyncio. I need 4 agents running inference (CPU) while also making API calls (I/O). How would you architect this? Show code.'"

**Expected Exploration**: Student practices advanced architectural thinking; applies multiple concepts

✨ **Teaching Tip 1** (When introducing benchmarking - 40 min mark)
> "Use Claude Code to benchmark interactively: 'Here's my workload. Compare all approaches. Show me code for each. Run them and show me results. Help me interpret: which should I use and why?' This is the professional workflow."

**Expected Outcome**: Student learns collaborative benchmarking with AI; builds confidence with empirical reasoning

✨ **Teaching Tip 2** (When discussing decision-making - 25 min mark)
> "The decision framework isn't a rigid rule—it's a starting point. Ask your AI: 'I'm not sure if my workload is CPU or I/O-bound. How do I test this? What tools help me measure?' Learning to characterize workloads is a professional skill."

**Expected Outcome**: Student learns to validate assumptions; develops exploratory mindset

### Try With AI (4 Prompts - Final 15 min)

**Prompt 1 (Remember - Recall Decision Framework)**:
> "Create a decision matrix: workload type (CPU/I/O), scale (single/parallel), and your recommended approach (single-threaded/free-threaded/multiprocessing/asyncio). Then ask your AI: 'Did I get these right? Any edge cases I'm missing?'"

**Expected Outcome**:
- Student recalls framework
- Internalizes decision patterns
- Time: 3 min

**Prompt 2 (Understand - Explain Tradeoffs)**:
> "Ask your AI: 'I want to move from multiprocessing to free-threaded Python. What benefits do I get? What do I lose? Is this upgrade worth it for my use case?'"

**Expected Outcome**:
- Student understands tradeoff analysis
- Develops nuanced thinking
- Time: 3 min

**Prompt 3 (Apply - Benchmark and Choose)**:
> "Tell your AI: 'Benchmark my workload using all 4 approaches. Run the code, show me timing for each. Based on results, recommend the best approach. Explain: (a) why it wins, (b) at what scale it starts winning, (c) any caveats.'"

**Expected Outcome**:
- Student makes empirical decision
- Learns to trust measurement over intuition
- Time: 5 min

**Prompt 4 (Analyze - Connect to Capstone and Production)**:
> "Ask your AI: 'I'm building a multi-agent AI system for production. Concurrency is important. Walk me through: (1) How I'd characterize my workload, (2) Which approach I'd choose, (3) How I'd benchmark to validate, (4) How this connects to Kubernetes deployment (Part 11).'"

**Expected Outcome**:
- Student integrates L5 knowledge into capstone context
- Sees connection to production deployment
- Anticipates future learning
- Time: 4 min

### Cognitive Load Check

- **Concepts Count**: 9/10 (advanced tier max) ✅
- **New vs Review**: 9 new (4 approaches, decision framework, benchmarking, Python 3.14 improvements), 5 review (CPU/I/O from L3-4, asyncio from L28, thread safety from L4)
- **Complexity Justification**: B1-B2 proficiency. Heavy review of prior content helps manage cognitive load. Decision-making is the heart of this lesson, not new concepts. Students are synthesizing knowledge from L1-4 plus Ch 28.

### Pedagogical Flow

1. **Decision Framework** (0-20 min): CPU vs I/O, workload classification
2. **Four Approaches** (20-50 min): Single-threaded, free-threaded, multiprocessing, asyncio with pros/cons
3. **Benchmarking** (50-70 min): Methodology, real examples, interpretation
4. **Python 3.14 Improvements** (70-80 min): Asyncio CLI tools, multiprocessing enhancements
5. **CoLearning + Challenge** (80-90 min): AI-guided framework application and benchmarking
6. **Try With AI** (85-90 min): Practical decision-making and production context

**Engagement Pattern**: Framework presentation → four approach comparison → benchmarking practice → AI-guided application

---

## Lesson 6: Capstone - Multi-Agent Concurrency System

**CEFR Proficiency**: B2 (Vantage - Advanced Integration)
**Estimated Time**: 150-180 minutes (2.5-3 hours)
**Learning Objectives**: LO-4 (Build AI-native systems with free-threading), LO-3 (Apply concurrency decisions), LO-5 (Apply AI-Native Learning pattern - Meta-skill)
**Success Evals Supported**: EVAL-006 (Multi-agent system building), EVAL-005 (Benchmark implementation), EVAL-003 (Concurrency decision-making), EVAL-010 (AI-native context), EVAL-011 (Forward-looking preparation for Parts 10-13)
**Cognitive Load**: 8 concepts (integration project - review-heavy)

### Skills Taught

1. **Multi-Agent Architecture Design**
   - **Proficiency**: B2
   - **Category**: Technical
   - **Bloom's Level**: Create (design novel system)
   - **DigComp Area**: Problem-Solving (design complex systems)
   - **Measurable at This Level**: Student designs functional multi-agent system with agents, shared state, communication

2. **Thread-Safe Shared State Management**
   - **Proficiency**: B2
   - **Category**: Technical
   - **Bloom's Level**: Apply + Analyze (implement thread-safe patterns)
   - **DigComp Area**: Safety (protect data from race conditions)
   - **Measurable at This Level**: Student implements proper locking/synchronization for complex shared state

3. **Benchmarking Dashboard Creation**
   - **Proficiency**: B1-B2
   - **Category**: Technical
   - **Bloom's Level**: Create (design and build dashboard)
   - **DigComp Area**: Problem-Solving (create performance measurement tool)
   - **Measurable at This Level**: Student builds working dashboard comparing concurrency approaches; generates valid performance metrics

4. **Performance Measurement and Interpretation**
   - **Proficiency**: B1-B2
   - **Category**: Technical
   - **Bloom's Level**: Analyze + Evaluate (measure and assess)
   - **DigComp Area**: Problem-Solving (evaluate system performance)
   - **Measurable at This Level**: Student measures execution time, CPU%, memory; interprets results correctly

5. **AI-Native Architecture Pattern Recognition**
   - **Proficiency**: B2
   - **Category**: Conceptual
   - **Bloom's Level**: Understand + Evaluate (recognize and assess AI-native patterns)
   - **DigComp Area**: Problem-Solving (apply domain-specific patterns)
   - **Measurable at This Level**: Student explains how free-threading enables AI-native parallelism; connects to book vision

6. **Production Preview (Kubernetes/Ray Context)**
   - **Proficiency**: B2
   - **Category**: Conceptual
   - **Bloom's Level**: Understand + Evaluate (forward-looking readiness)
   - **DigComp Area**: Problem-Solving (evaluate production deployment)
   - **Measurable at This Level**: Student explains how capstone system would scale in Kubernetes (Parts 10-13); anticipates deployment patterns

7. **Type Hints and Modern Python (Python 3.14 Style)**
   - **Proficiency**: B1-B2
   - **Category**: Technical
   - **Bloom's Level**: Apply (use modern syntax)
   - **DigComp Area**: Content Creation (write idiomatic code)
   - **Measurable at This Level**: Student writes all code with modern type hints (dict[str, int], list[AgentResult], X | None)

8. **Error Handling in Concurrent Systems**
   - **Proficiency**: B1-B2
   - **Category**: Technical
   - **Bloom's Level**: Apply (implement error handling)
   - **DigComp Area**: Safety (handle failures gracefully)
   - **Measurable at This Level**: Student implements try/except for thread safety; handles agent failures gracefully

### Key Concepts (8 total - integration project, mostly review)

1. Multi-agent system architecture (agents as concurrent units)
2. Shared state management (thread-safe data structures)
3. Agent communication patterns (how agents coordinate)
4. Performance metrics (execution time, CPU%, memory)
5. Benchmarking comparison (free-threaded vs traditional vs multiprocessing)
6. Error resilience (handling agent failures)
7. Production readiness patterns (preview for Parts 10-13)
8. AI-native reasoning parallelism (core book vision)

### Content Outline

- **Section 1: Multi-Agent System Architecture**
  - What is an agent? (Independent reasoning unit)
  - Multi-agent system? (Multiple concurrent agents)
  - Parallel reasoning on shared problem
  - Why free-threading helps (true parallelism on multi-core)
  - Example: 4-agent AI system analyzing data in parallel

- **Section 2: Agent Implementation**
  - Agent as class with reasoning method
  - Reasoning task: CPU-intensive (text analysis, calculations)
  - Results collection: Thread-safe
  - Agent lifecycle: Start, reason, complete, report

- **Section 3: Shared State Management**
  - Results accumulator (thread-safe collection)
  - Agent identification (unique per agent)
  - Timing (measure individual and total performance)
  - Locking pattern (threading.Lock for complex operations)

- **Section 4: Benchmark Comparison**
  - Part A: Run system with traditional threading (GIL-constrained)
  - Part B: Run system with free-threaded Python (true parallelism)
  - Part C: Run system with multiprocessing (comparison baseline)
  - Measure: Execution time, CPU utilization, memory usage

- **Section 5: Benchmarking Dashboard**
  - Collect metrics from all approaches
  - Visual output (ASCII tables or terminal graphs)
  - Compare results
  - Practical tool students can reuse

- **Section 6: Production Preview - Deployment Context**
  - Part 11 (Kubernetes): Deploy multi-agent system as pods
  - Part 14 (Dapr Actors): Scale pattern to distributed virtual actors
  - Resource utilization: How free-threading improves CPU efficiency
  - Ray integration: Distributed agent execution (Part 14 topic)

- **Section 7: Error Handling and Resilience**
  - Agent failures don't crash system
  - Try/except in agent reasoning
  - Failed agent results tracked
  - System continues with remaining agents

- **Section 8: Capstone Synthesis**
  - Integrate: CPython (L1) → Performance (L2) → GIL (L3) → Free-threading (L4) → Decisions (L5) → Build (L6)
  - Demonstrate: Knowledge application in realistic project
  - Validate: Proficiency across all learning objectives

### Code Examples Integration

- **Example 8**: Simple Multi-Agent System (Foundation Code)
  - **Purpose**: Scaffolding for capstone project
  - **Complexity**: B2 (integrates multiple concepts)
  - **When**: Start of L6 content (students extend this)
  - **Concepts Used**: Free-threading, classes, type hints, threading, shared state, dataclasses

- Students expand Example 8 into full capstone

### CoLearning Elements Placement

💬 **AI Colearning Prompt 1** (At project start - 15 min mark)
> "Ask your AI: 'I'm building a multi-agent AI system from scratch. Walk me through the architecture. What classes do I need? What methods? How should agents communicate? Show me the blueprint before I code.'"

**Expected Exploration**: Student designs architecture with AI guidance; practices specification thinking (Part 5 preview)

🎓 **Instructor Commentary 1** (After showing foundation code - 40 min mark)
> "Professional developers don't write every line. They ask AI to generate scaffolding, then extend and validate. You're learning the AI-native workflow: (1) Describe what you want, (2) AI generates draft, (3) You improve and test. This is how 2025 AI developers work."

🚀 **CoLearning Challenge 1** (During implementation - 60 min mark)
> "Tell your AI: 'Add 2 more agent types to the system. First, sketch what you want (CSV-to-database agent, sentiment analyzer). Then ask AI: Generate the code and explain how they integrate with the existing system. Test it.'"

**Expected Outcome**: Student practices extension pattern; learns system thinking; applies AI-native workflow

💬 **AI Colearning Prompt 2** (During benchmarking - 90 min mark)
> "Ask your AI: 'Run the benchmark comparing free-threaded vs traditional vs multiprocessing. Analyze the results. For my 4-agent system, which is fastest? At what thread count does the advantage appear? Why?'"

**Expected Exploration**: Student interprets real data; makes empirical decisions; validates L5 learning

🎓 **Instructor Commentary 2** (When discussing production context - 120 min mark)
> "This capstone system is a single-machine prototype. In Parts 10-14 you'll scale this to Kubernetes (multiple pods) and Ray (distributed virtual actors). The concurrency patterns you learn here apply everywhere. Semantics (understanding parallelism) transcend the technology."

🚀 **CoLearning Challenge 2** (During error handling - 130 min mark)
> "Tell your AI: 'Make the system resilient. If an agent crashes, the system should log it and continue. Add exception handling. Generate the code and show me test cases where agents fail.'"

**Expected Outcome**: Student learns production thinking; practices error resilience; extends system robustness

✨ **Teaching Tip 1** (When starting implementation - 10 min mark)
> "Use Claude Code for this capstone. Describe what you want, ask AI to generate, validate the code, then extend. This is how professional developers work with AI tools. Build the habit now."

**Expected Outcome**: Student learns professional AI-native workflow; gains confidence with Claude Code

✨ **Teaching Tip 2** (When troubleshooting - 100 min mark)
> "When your benchmark shows unexpected results, ask your AI: 'Why is free-threaded slower than I expected? What could I be missing? Help me debug.' AI is excellent at performance analysis—use it."

**Expected Outcome**: Student learns collaborative debugging; builds problem-solving skills

### Try With AI (4 Prompts - Final 20 min)

**Prompt 1 (Remember - Recall Your System)**:
> "Describe the multi-agent system you built. What agents do you have? How do they communicate? Then ask your AI: 'Did I capture the key design? What should I emphasize?'"

**Expected Outcome**:
- Student recalls architecture
- Solidifies learning through articulation
- Time: 3 min

**Prompt 2 (Understand - Explain Design Choices)**:
> "Ask your AI: 'Why did I choose free-threaded Python over multiprocessing for this system? Walk through the tradeoffs. Did I make the right choice?'"

**Expected Outcome**:
- Student justifies architectural decisions
- Reflects on learning
- Time: 3 min

**Prompt 3 (Apply + Evaluate - Benchmark Results)**:
> "Tell your AI: 'Show me the benchmarking results. Analyze them: (a) which approach is fastest, (b) why, (c) at what scale, (d) what's the overhead of free-threading in my case. Did my prediction match reality?'"

**Expected Outcome**:
- Student compares expectations vs reality
- Learns from empirical results
- Validates understanding
- Time: 6 min

**Prompt 4 (Analyze + Synthesize - Production Context and Book Vision)**:
> "Ask your AI: 'This capstone system works on one machine. How would I:(1) Deploy it to Kubernetes (Part 11) as multiple pods, (2) Scale it with Ray (Part 14) as distributed actors, (3) Add persistence with databases (Part 13), (4) Monitor in production. How does what you learned in Chapter 29 connect to production AI systems?'"

**Expected Outcome**:
- Student connects current learning to future parts
- Anticipates deployment complexity
- Understands why free-threading matters in production
- Recognizes book's narrative arc
- Time: 8 min

### Cognitive Load Check

- **Concepts Count**: 8/10 (integration project, review-heavy) ✅
- **New vs Review**: 3 new (production preview, benchmarking dashboard, AI-native patterns), 5 review (multi-agent architecture, thread safety, error handling, performance measurement, decision-making from L1-5)
- **Complexity Justification**: B2 proficiency - SYNTHESIS. This lesson integrates all prior lessons. Minimal new concepts; maximum application. Students apply knowledge to realistic project, not learn new foundations. Cognitive load is high due to project scope, not concept count.

### Pedagogical Flow

1. **Project Vision** (0-15 min): What are we building? Why?
2. **Architecture** (15-45 min): Agent design, shared state, communication
3. **Implementation** (45-100 min): Build system, test components, integrate
4. **Benchmarking** (100-130 min): Measure approaches, interpret results
5. **Error Handling** (130-145 min): Resilience, exception management
6. **Production Context** (145-160 min): Kubernetes/Ray/Deployment preview
7. **Try With AI** (160-180 min): Reflection and forward-looking synthesis

**Engagement Pattern**: Project-based learning with AI scaffolding → implementation → validation → production context

---

## Implementation Checklist

After planning, before implementation (/sp.tasks):

- [ ] All 6 lessons have CEFR proficiency levels assigned (B1, B1, B1-B2, B1-B2, B1-B2, B2)
- [ ] Proficiency progression validated (smooth B1 → B1-B2 → B2, no zigzag) ✅
- [ ] Cognitive load checked for each lesson (max 10 concepts, justified) ✅
- [ ] Skills proficiency mapped (CEFR + Bloom's + assessment method) ✅
- [ ] CoLearning elements specified (💬🎓🚀✨ placement clear in all lessons) ✅
- [ ] "Try With AI" 4-prompt structure defined for all lessons ✅
- [ ] Code examples integrated from spec (8 examples across 6 lessons) ✅
- [ ] Success evals supported (which lessons validate which evals) ✅
- [ ] Prerequisites satisfied (no forward references within chapter) ✅
- [ ] AI-native context threaded throughout (multi-agent systems, production preview) ✅
- [ ] No SDD terminology (uses Part 4 appropriate language: "describe intent") ✅
- [ ] Reading level maintained (B1-B2: Grade 9-11, verified via Flesch-Kincaid) ✅
- [ ] Type hints used throughout (Python 3.14 modern syntax) ✅
- [ ] Lesson closure appropriate (each ends with "Try With AI" ONLY, no summaries) ✅

---

## Skills Proficiency Summary by Lesson

| Lesson | Primary Skills (CEFR) | Bloom's Focus | Category | Assessment Method |
|--------|----------------------|---------------|---------|--------------------|
| 1 | CPython Recognition (B1), Implementation Detection (B1), Alternative Awareness (A2) | Understand-Apply | Technical | Written explanation + code detection |
| 2 | Performance Optimization (B1), GC Knowledge (B1), AI Workload Awareness (B1) | Understand-Apply | Technical | Benchmark interpretation |
| 3 | GIL Mechanism (B1-B2), CPU/IO Distinction (B1), Threading Behavior (B1-B2) | Understand-Apply-Analyze | Technical | Scenario classification + coding |
| 4 | Free-Threading Mastery (B1-B2), Detection (B1-B2), Thread Safety (B1-B2) | Understand-Apply-Analyze | Technical | Detection code + benchmarking |
| 5 | Decision Framework (B1-B2), Workload Classification (B1), Benchmarking (B1-B2) | Apply-Analyze-Evaluate | Technical | Decision matrix + benchmark design |
| 6 | Multi-Agent Architecture (B2), Shared State (B2), Benchmarking Dashboard (B1-B2) | Create-Evaluate | Technical | Capstone project + production context |

---

## Risks & Mitigations

**Risk 1**: Lesson 4 may exceed cognitive load at 10 concepts
- **Mitigation**: Monitor during implementation. CoLearning prompts (5 AI prompts + 2 challenges) provide scaffolding. If students struggle, split into Lessons 4A and 4B (becomes 7-lesson chapter).

**Risk 2**: Free-threading unavailable on all student systems
- **Mitigation**: Lesson 4 includes detection code. Students without free-threading can still learn concepts, run traditional benchmarks, understand architecture. Provides fallback patterns.

**Risk 3**: Capstone project too ambitious (2.5-3 hours)
- **Mitigation**: Provide solid foundation code (Example 8). Students extend, not build from scratch. Clear scaffolding via CoLearning prompts guides implementation.

**Risk 4**: Students may plateau at B1 (not progress to B1-B2/B2)
- **Mitigation**: CoLearning challenges and "Try With AI" Prompt 4 (Analyze/Synthesize) explicitly target higher Bloom's levels. Lesson 6 capstone requires B2 synthesis.

**Risk 5**: Python 3.14 features may not match documentation as book published
- **Mitigation**: All free-threading information from October 2025 release. Maintenance trigger: Review PEP 703/779 annually; update if behavior changes.

**Risk 6**: Benchmarking results may vary significantly by hardware
- **Mitigation**: Lessons emphasize measurement methodology, not absolute speedup numbers. Students learn to benchmark their own workloads, not memorize claimed gains.

---

## Next Steps

1. ✅ **Planning complete** → User reviews and approves plan
2. ⏭️ **Generate tasks.md** → Run `/sp.tasks part-4-chapter-29` to create actionable task checklist
3. ⏭️ **Implement lessons** → Run `/sp.implement part-4-chapter-29` with chapter-planner subagent
4. ⏭️ **Technical review** → Automatic validation + PHR creation

---

**Status**: ✅ PLANNING PHASE COMPLETE

Plan is ready for task generation and implementation. All proficiency levels, cognitive load, CoLearning elements, and learning objectives are specified and validated.

**Files Ready for Next Phase**:
- `specs/part-4-chapter-29/spec.md` (approved specification)
- `specs/part-4-chapter-29/plan.md` (this file - detailed lesson plan)
- Ready for: `/sp.tasks part-4-chapter-29`

