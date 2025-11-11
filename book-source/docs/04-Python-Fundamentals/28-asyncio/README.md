---
title: "Chapter 28: Asyncio"
chapter: 28
sidebar_position: 28
---

# Chapter 28: Asyncio — Concurrent I/O and CPU-Parallel Workloads

Master modern Python 3.14+ asyncio patterns for building high-performance concurrent systems that handle both I/O operations and CPU-intensive work efficiently.

## Overview

This chapter teaches you to build production-ready asynchronous systems using Python 3.14's latest asyncio features. You'll learn when asyncio helps (I/O-bound tasks), when it doesn't (CPU-bound work), and how to combine both patterns for optimal performance in AI-native applications.

By the end of this chapter, you'll design hybrid systems that fetch data concurrently from multiple sources and process it in parallel—the exact pattern used in modern AI agents that combine API calls with inference workloads.

## What You'll Learn

### Core Asyncio Concepts
- Event loop abstraction and how `asyncio.run()` manages it automatically
- Coroutines (`async def`) and the `await` keyword for cooperative multitasking
- I/O-bound vs CPU-bound task distinction and when asyncio applies
- Concurrency (task switching) vs parallelism (multi-core execution)

### Modern Patterns (Python 3.14+)
- `asyncio.TaskGroup()` for structured concurrency (fail-fast, automatic cleanup)
- `asyncio.timeout()` context manager for timeout controls
- `asyncio.gather()` for best-effort result collection
- `InterpreterPoolExecutor` for true CPU parallelism (new in Python 3.14)

### Production Techniques
- Error handling strategies (TimeoutError, CancelledError, partial failures)
- Resilience patterns (retries, exponential backoff, circuit breakers)
- Resource limiting with Semaphores
- Performance benchmarking and bottleneck identification

### AI-Native Workloads
- Combining I/O concurrency with CPU parallelism
- Batch processing and pipeline patterns
- Multi-service concurrent API fetching
- Parallel inference processing simulation

## Prerequisites

Before starting this chapter, you should have completed:

- **Chapter 20**: Module and Functions (function composition, type hints)
- **Chapter 21**: Exception Handling (try/except, error propagation)
- **Chapters 24-27**: Object-Oriented Programming, Data Classes, Pydantic (class design, type systems)
- **Chapters 1-11**: AI-Driven Development mindset and tools (Claude Code / Gemini CLI)

**Python Knowledge Assumed**:
- Comfortable writing functions with type hints
- Understand exception handling patterns
- Familiar with basic OOP concepts

**Technical Requirements**:
- Python 3.14+ installed (Chapter 12: UV package manager)
- `httpx` library for async HTTP (`uv pip install httpx`)
- AI companion tool configured (Claude Code, Gemini CLI, or ChatGPT)

## Chapter Structure

This chapter contains **6 progressive lessons** (12-14 hours total):

### Lesson 1: Asyncio Foundations — Event Loop and Coroutines (2-2.5h)
**Proficiency**: B1 (can apply asyncio patterns to new I/O problems)

Learn the event loop concept, write basic async functions with `async def` and `await`, distinguish I/O-bound from CPU-bound tasks, and understand when asyncio helps.

**Key Topics**: Event loop, coroutines, `asyncio.run()`, I/O vs CPU, concurrency vs parallelism

---

### Lesson 2: Concurrent Tasks — create_task(), gather(), TaskGroup() (2-2.5h)
**Proficiency**: B1 (can write structured concurrent code)

Schedule multiple coroutines concurrently, collect results with `gather()`, and apply modern `TaskGroup()` for structured concurrency with automatic error handling.

**Key Topics**: Task scheduling, `gather()` vs `TaskGroup()`, error propagation, performance benchmarking

---

### Lesson 3: Advanced Patterns — Timeouts, Futures, Error Handling (2-2.5h)
**Proficiency**: B1-B2 (can build resilient async systems)

Apply timeout controls with `asyncio.timeout()`, understand Futures conceptually, handle exceptions gracefully, and build resilience patterns like retry logic and exponential backoff.

**Key Topics**: `asyncio.timeout()`, TimeoutError, Futures, exception handling, resilience patterns

---

### Lesson 4: CPU-Bound Work and GIL — InterpreterPoolExecutor (2-2.5h)
**Proficiency**: B1-B2 (can apply CPU parallelism solutions)

Understand the GIL limitation briefly (2-3 sentences), use Python 3.14's `InterpreterPoolExecutor` for true CPU parallelism, measure performance differences, and bridge CPU work into async code.

**Key Topics**: GIL brief intro, `InterpreterPoolExecutor`, threading vs parallelism, `run_in_executor()`

**Forward Reference**: Chapter 29 (CPython and GIL) covers deep GIL internals; this lesson teaches practical solutions only.

---

### Lesson 5: Hybrid Workloads — Combining I/O and CPU (2-2.5h)
**Proficiency**: B2 (can design and optimize hybrid systems)

Combine `asyncio.TaskGroup()` (I/O concurrency) with `InterpreterPoolExecutor` (CPU parallelism) for optimal performance. Learn batch processing, pipeline patterns, and resource limiting for realistic AI workloads.

**Key Topics**: Hybrid patterns, batch processing, pipeline architecture, AI workload simulation, Semaphores

---

### Lesson 6: Capstone — AI Agent System (2.5-3h)
**Proficiency**: B2 (can integrate all concepts into production systems)

Build a complete multi-service AI agent that fetches data concurrently from 3+ sources (Weather, News, Knowledge Base) and processes results in parallel with simulated LLM inference. Integrate all patterns from Lessons 1-5.

**Key Topics**: System architecture, concurrent fetching, parallel processing, error recovery, performance optimization

---

## Learning Approach

### AI-Native Pedagogy

This chapter uses **AI-Native Learning** methodology:

- **Book teaches foundational concepts** clearly (event loop, coroutines, I/O vs CPU)
- **AI companion helps with complex execution** (generating code, exploring edge cases)
- **AI orchestrates scaling tasks** (parallel implementations, performance testing)

Each lesson includes:
- **CoLearning Elements** (💬🎓🚀✨) positioned throughout to guide AI collaboration
- **"Try With AI" closure** with 4 progressive prompts (Understand → Apply → Analyze → Create/Evaluate)
- **Part 4 appropriate language**: "Describe your intent to AI" (not formal SDD terminology)

### Performance Focus

All lessons include measurable timing comparisons:
- Lesson 1: 5x speedup (sequential 5s → concurrent 1s)
- Lesson 2: Concurrent task execution validated with benchmarks
- Lesson 4: 3-4x speedup with `InterpreterPoolExecutor` on 4-core machines
- Lesson 5: 40%+ improvement in hybrid I/O+CPU systems
- Lesson 6: Capstone demonstrates 2.5x+ speedup vs sequential

### Modern Standards Only

**Python 3.14+ patterns exclusively**:
- ✅ `asyncio.run()` entry point
- ✅ `asyncio.TaskGroup()` structured concurrency
- ✅ `asyncio.timeout()` context manager
- ✅ `InterpreterPoolExecutor` for CPU parallelism
- ✅ 100% type hints (`dict[str, Any]`, `list[str]`, `X | None`)

**Zero deprecated APIs**:
- ❌ No `asyncio.get_event_loop()`
- ❌ No manual event loop management
- ❌ No `nest_asyncio` (Jupyter workarounds)
- ❌ No child watcher APIs

## Success Criteria

By the end of this chapter, you will:

1. **Identify I/O vs CPU-bound tasks** correctly in 90%+ of cases
2. **Write async code** that achieves measurable concurrency (time ≈ longest operation, not sum)
3. **Apply modern patterns** (`TaskGroup()`, `timeout()`, `InterpreterPoolExecutor`) appropriately
4. **Demonstrate performance improvements** with benchmarks (4x CPU speedup, 40%+ hybrid improvement)
5. **Build hybrid systems** that combine I/O concurrency and CPU parallelism
6. **Debug async errors** using AI as co-reasoning partner
7. **Complete the capstone** AI Agent System with all patterns integrated

## Key Differentiators

### Why This Chapter Matters for AI Development

Modern AI applications are **inherently hybrid workloads**:
- **I/O operations**: Fetching from multiple LLM APIs, databases, vector stores concurrently
- **CPU operations**: Running inference, embeddings, data processing in parallel

This chapter teaches the exact patterns used in production AI systems:
- Concurrent multi-source data fetching (TaskGroup)
- Parallel inference processing (InterpreterPoolExecutor)
- Graceful error handling and timeouts
- Resource limiting and optimization

### Python 3.14+ Killer Feature

`InterpreterPoolExecutor` (new in Python 3.14) is a **game-changer** for AI workloads:
- True CPU parallelism without process overhead
- Separate interpreters = separate GILs
- 3-4x speedup on multi-core machines
- Perfect for AI inference, embeddings, data processing

This is the first book chapter to teach this pattern comprehensively.

## Next Steps

After completing Chapter 28:

- **Chapter 29**: CPython and GIL (deep GIL internals, free-threaded mode, C extensions)
- **Part 5**: Spec-Driven Development (apply asyncio patterns in formal SDD workflow)
- **Part 6**: AI Native Software Development (build production AI agents using these patterns)

## Technical Notes

### Environment Setup

```bash
# Verify Python 3.14+
python3 --version  # Should show 3.14.0 or higher

# Install dependencies
uv pip install httpx  # Async HTTP library

# Optional: Install asyncio development tools
uv pip install aiofiles asyncpg  # Async file I/O, PostgreSQL
```

### Code Examples

All code examples in this chapter:
- Run successfully on Python 3.14.0
- Include full type hints and docstrings
- Are cross-platform (Windows, Mac, Linux)
- Use only standard library + `httpx`

### Maintenance Notes

**Python 3.14+ features may evolve**:
- `InterpreterPoolExecutor` is new in 3.14; verify API stability with each Python release
- `asyncio.timeout()` and `TaskGroup()` are stable (Python 3.11+)
- Monitor Python release notes for asyncio changes

**Recommended**: Review this chapter before each major Python release (3.15, 3.16, etc.) to ensure API compatibility.

---

**Chapter 28 Duration**: 12-14 hours total
**Proficiency Progression**: B1 → B1-B2 → B2
**Cognitive Load**: Graduated (7-9 concepts per lesson, appropriate for advanced learners)
**Code Examples**: 29+ across all lessons
**"Try With AI" Prompts**: 24 (4 per lesson, Bloom's progression)

Ready to master modern Python asyncio? Start with [Lesson 1: Asyncio Foundations](./01-asyncio-foundations.md).
