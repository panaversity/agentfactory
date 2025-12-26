---
sidebar_position: 31
title: "Chapter 31: Asyncio — Concurrent I/O and CPU-Parallel Workloads"
slides:
  source: "https://pub-80f166e40b854371ac7b05053b435162.r2.dev/books/ai-native-dev/static/slides/chapter-31-slides.pdf"
  title: "Chapter 31: Asyncio — Concurrent I/O and CPU-Parallel Workloads"
  height: 700
---

# Chapter 31: Asyncio — Concurrent I/O and CPU-Parallel Workloads

Modern applications don't wait—they fetch from multiple APIs concurrently, process data in parallel, and combine results efficiently. An AI agent making sequential API calls wastes time. A data pipeline processing items one-by-one underutilizes CPU cores. **Asyncio** enables concurrent I/O operations, and **InterpreterPoolExecutor** enables true CPU parallelism.

This chapter teaches you to build production-ready asynchronous systems using Python 3.14's latest asyncio features. You'll learn when asyncio helps (I/O-bound tasks), when it doesn't (CPU-bound work), and how to combine both patterns for optimal performance in AI-native applications. Modern AI applications are inherently hybrid workloads—fetching from multiple LLM APIs, databases, and vector stores concurrently while running inference, embeddings, and data processing in parallel.

By the end of this chapter, you'll design hybrid systems that fetch data concurrently from multiple sources and process it in parallel—the exact pattern used in modern AI agents. You'll build a complete multi-service AI agent that demonstrates 2.5x+ speedup vs sequential approaches.

## 🎯 Before You Begin
---
## What You'll Learn
- **Master core asyncio concepts** — Understand event loop abstraction, write coroutines with `async def` and `await`, distinguish I/O-bound from CPU-bound tasks, differentiate concurrency (task switching) from parallelism (multi-core execution)
- **Apply modern Python 3.14+ patterns** — Use `asyncio.TaskGroup()` for structured concurrency with fail-fast and automatic cleanup, apply `asyncio.timeout()` context manager for timeout controls, collect results with `asyncio.gather()`, leverage `InterpreterPoolExecutor` for true CPU parallelism
- **Implement production techniques** — Handle errors gracefully (TimeoutError, CancelledError, partial failures), build resilience patterns (retries, exponential backoff, circuit breakers), limit resources with Semaphores, benchmark and identify bottlenecks
- **Build AI-native workloads** — Combine I/O concurrency with CPU parallelism for optimal performance, implement batch processing and pipeline patterns, fetch from multi-service APIs concurrently, process parallel inference workloads
- **Achieve measurable performance gains** — Demonstrate 5x speedup for I/O concurrency, achieve 3–4x speedup with InterpreterPoolExecutor on multi-core machines, build hybrid systems with 40%+ improvement, create complete AI Agent System with 2.5x+ speedup vs sequential
