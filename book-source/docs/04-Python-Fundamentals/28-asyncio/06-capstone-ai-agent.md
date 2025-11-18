---
title: "Capstone: AI Agent System for Multi-Source Data Processing"
chapter: 28
lesson: 6
duration_minutes: 180
learning_objectives:
  - objective: "Integrate asyncio TaskGroup, InterpreterPoolExecutor, error handling, and timeout management into a cohesive system"
    proficiency_level: "B2"
    bloom_level: "Synthesize"
    assessment_method: "Complete working capstone system with all patterns integrated"

  - objective: "Design concurrent API fetching with parallel processing for realistic AI workloads"
    proficiency_level: "B2"
    bloom_level: "Create"
    assessment_method: "Multi-service agent architecture handling 3+ concurrent sources"

  - objective: "Implement graceful error handling and timeout controls for production resilience"
    proficiency_level: "B2"
    bloom_level: "Evaluate"
    assessment_method: "System handles partial failures and continues with best-effort results"

  - objective: "Measure and optimize hybrid I/O+CPU system for performance (40%+ faster than sequential)"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Performance benchmarks demonstrating concurrent and parallel execution benefits"

  - objective: "Explain architectural decisions and system behavior when questioned by AI"
    proficiency_level: "B2"
    bloom_level: "Understand"
    assessment_method: "Clear explanations of how TaskGroup and InterpreterPoolExecutor coordinate"

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "Async Architecture Design"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student designs multi-service async system with proper structured concurrency"

  - name: "Error Handling in Concurrent Systems"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Evaluate"
    digcomp_area: "Safety"
    measurable_at_this_level: "Student implements graceful degradation and partial failure recovery"

  - name: "Hybrid I/O+CPU Performance Optimization"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student benchmarks and optimizes system achieving 2-3x speedup vs sequential"

  - name: "AI System Architecture"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student explains how concurrent fetching and parallel inference interact in production AI systems"

  - name: "System Integration"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Synthesize"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student combines TaskGroup, InterpreterPoolExecutor, timeouts, and error handling into working system"

cognitive_load:
  new_concepts: 0
  assessment: "Capstone integrates all prior concepts—zero new concepts introduced ✓ Appropriate for B2 synthesis level"

differentiation:
  extension_for_advanced: "Add distributed caching across sources; implement adaptive timeout strategies; extend to 5+ concurrent sources"
  remedial_for_struggling: "Start with 2 sources instead of 3; scaffold error handling patterns step-by-step; use skeleton code liberally"

# Generation metadata
generated_by: "content-implementer v1.0.0"
source_spec: "specs/001-part-4-chapter-28/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Lesson 6: Capstone — AI Agent System for Multi-Source Data Processing

## Opening Hook: Real AI Systems Are Hybrid

You've learned how asyncio handles concurrency, how InterpreterPoolExecutor enables true parallelism, and how to combine them. But you've studied these patterns in isolation—single API calls, simple parallel tasks.

Now imagine building a real AI system. A production language model agent that needs to:

1. **Fetch context from multiple sources concurrently** — Weather API, News API, knowledge base (all at the same time, not sequentially)
2. **Process each response in parallel** — Simulate LLM inference on each result using multiple CPU cores
3. **Handle failures gracefully** — If one source times out, continue with others
4. **Aggregate results** — Combine partial results into a cohesive response
5. **Measure performance** — Prove that hybrid execution is 2-3x faster than doing everything sequentially

That's what this capstone builds. You'll create a production-style AI agent system that integrates **every concept from Lessons 1-5** into one working example. This isn't a learning exercise—it's a pattern you'll use in real applications.

By the end of this lesson, you'll have built a system that:

- Fetches from 3+ sources concurrently using `TaskGroup()` (Lesson 2)
- Processes results in parallel using `InterpreterPoolExecutor` (Lesson 4)
- Applies timeout controls using `asyncio.timeout()` (Lesson 3)
- Handles errors gracefully with try/except patterns (Lesson 3)
- Demonstrates measurable performance improvement (Lesson 5)

---

## Project Overview: The Multi-Source AI Agent

### What You're Building

An AI Agent System that simulates how real AI applications work:

```
┌─────────────────────────────────────────────────────────────┐
│           User Query: "What's happening now?"                │
└────────┬──────────────────────────────────────────────────┬──┘
         │                                                  │
    ┌────▼─────┐  ┌──────────────┐  ┌──────────────────┐  │
    │  Weather │  │    News      │  │   Knowledge Base │  │
    │   API    │  │    API       │  │   Query          │  │
    │(2s delay)│  │ (1.5s delay) │  │ (3s delay)       │  │
    └────┬─────┘  └──────┬───────┘  └────────┬─────────┘  │
         │                │                    │            │
         └────────────────┼────────────────────┘            │
                          │                                 │
                    ┌─────▼─────────┐                       │
                    │   TaskGroup    │ Concurrent Fetching  │
                    │   (All 3 at    │ Total: ~3s           │
                    │    once)       │ (not 6.5s!)          │
                    └─────┬─────────┘                       │
                          │                                 │
            ┌─────────────┼────────────────┐               │
            │             │                │               │
        ┌───▼─┐       ┌──▼──┐        ┌───▼──┐             │
        │LLM  │       │LLM  │        │LLM   │             │
        │Infer│       │Infer│        │Infer │             │
        │(2s) │       │(2s) │        │(2s)  │             │
        └───┬─┘       └──┬──┘        └───┬──┘             │
            │            │               │                │
            └────────┬───┴───────────────┘                │
                     │                                     │
            ┌────────▼──────────┐                         │
            │ InterpreterPool   │ Parallel Processing     │
            │ Executor (3 cores │ Total: ~2s              │
            │ in parallel)      │ (not 6s!)               │
            └────────┬──────────┘                         │
                     │                                     │
        ┌────────────▼─────────────┐                      │
        │   Aggregate Results      │                      │
        │   Return unified response│                      │
        └──────────────────────────┘                      │
                     │                                     │
                     └─────────────────────────────────────┘

TOTAL TIME: ~5 seconds (with timeouts)
Sequential would take: ~6.5 seconds just for fetching + 6 seconds for inference = 12.5 seconds!
Hybrid saves ~150% time through I/O concurrency + CPU parallelism
```

### System Components

**Concurrent Fetchers** (TaskGroup handles these):
- `fetch_weather(city)` — Simulates API call with 2s latency
- `fetch_news(query)` — Simulates API call with 1.5s latency
- `query_knowledge_base(question)` — Simulates API call with 3s latency

**Parallel Processor** (InterpreterPoolExecutor handles this):
- `simulate_llm_inference(text)` — CPU-intensive text analysis (2s per result)

**Main Agent** (orchestrates everything):
- `ai_agent_query(user_question)` — Coordinates concurrent fetch + parallel processing

---

## Part 1: System Requirements and Architecture

### Functional Requirements

Your system must:

1. **Fetch from 3 concurrent sources** using `TaskGroup()` with structured concurrency
2. **Process each result** with simulated LLM inference in parallel via `InterpreterPoolExecutor`
3. **Apply timeouts**:
   - Individual API calls: 5 seconds each
   - Overall system: 15 seconds total
4. **Handle partial failures**: If one source times out, continue with others (graceful degradation)
5. **Return aggregated results** containing:
   - Successful results from all working sources
   - Error messages from failed sources
   - Total execution time
   - Count of parallel cores used
6. **Demonstrate performance**: Complete 2-3x faster than sequential execution

### Non-Functional Requirements

- All code uses Python 3.14+ patterns (type hints, modern syntax)
- Error handling is explicit (no silent failures)
- Logging captures execution flow for debugging
- System is resilient (handles timeouts and exceptions gracefully)

### Architecture Principles

**Use the Graduated Teaching Pattern from Lesson 5:**

1. **Manual Foundation (Lesson 1):** Understand sequential execution first
2. **Concurrent I/O (Lessons 2-3):** Replace sequential with TaskGroup
3. **Parallel Processing (Lesson 4):** Add CPU work in InterpreterPoolExecutor
4. **Integration (Lesson 5):** Combine both patterns
5. **System Design (Lesson 6 - THIS LESSON):** Real-world architecture with error handling and optimization

Your capstone applies this progression in one integrated system.

---

## Part 2: Code Structure and Implementation Approach

### Skeleton Code: Understanding the Pattern

Here's the structure of the system students implement (with AI guidance):

```python
# File: ai_agent.py
import asyncio
from concurrent.futures import Executor, ProcessPoolExecutor
from typing import Any
import logging
import time

# Configure logging for visibility
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# ============================================================================
# PART 1: Simulated Data Sources (Students implement these first)
# ============================================================================

async def fetch_weather(city: str, *, timeout: float = 5.0) -> dict[str, Any]:
    """
    Fetch weather data for a city.

    Simulates API call with 2-second latency.

    Args:
        city: City name to fetch weather for
        timeout: Maximum wait time in seconds

    Returns:
        Dictionary with weather data or error message
    """
    try:
        async with asyncio.timeout(timeout):
            logger.info(f"Fetching weather for {city}...")
            await asyncio.sleep(2)  # Simulate 2s API latency
            return {
                "source": "weather",
                "city": city,
                "temp": 72,
                "condition": "sunny",
                "humidity": 45
            }
    except asyncio.TimeoutError:
        logger.warning(f"Weather API timeout for {city}")
        return {"source": "weather", "error": "timeout"}


async def fetch_news(query: str, *, timeout: float = 5.0) -> list[dict[str, Any]]:
    """
    Fetch news articles matching query.

    Simulates API call with 1.5-second latency.

    Args:
        query: Search query for news
        timeout: Maximum wait time in seconds

    Returns:
        List of news articles or error dict
    """
    try:
        async with asyncio.timeout(timeout):
            logger.info(f"Fetching news for '{query}'...")
            await asyncio.sleep(1.5)  # Simulate 1.5s API latency
            return [
                {
                    "title": f"Article 1 about {query}",
                    "source": "news",
                    "snippet": "Breaking updates...",
                    "url": f"https://news.example.com/1"
                },
                {
                    "title": f"Article 2 about {query}",
                    "source": "news",
                    "snippet": "Latest developments...",
                    "url": f"https://news.example.com/2"
                }
            ]
    except asyncio.TimeoutError:
        logger.warning(f"News API timeout for '{query}'")
        return [{"source": "news", "error": "timeout"}]


async def query_knowledge_base(
    question: str,
    *,
    timeout: float = 5.0
) -> str:
    """
    Query knowledge base for information.

    Simulates knowledge base lookup with 3-second latency.

    Args:
        question: Question to search knowledge base for
        timeout: Maximum wait time in seconds

    Returns:
        Knowledge base result or error message
    """
    try:
        async with asyncio.timeout(timeout):
            logger.info(f"Querying knowledge base for '{question}'...")
            await asyncio.sleep(3)  # Simulate 3s knowledge base latency
            return (
                f"Knowledge base result for '{question}': "
                f"This is contextual information from our knowledge base "
                f"containing domain-specific expertise."
            )
    except asyncio.TimeoutError:
        logger.warning(f"Knowledge base query timeout for '{question}'")
        return "Knowledge base: timeout"


# ============================================================================
# PART 2: CPU-Intensive Processing (Runs in InterpreterPoolExecutor)
# ============================================================================

def simulate_llm_inference(data: str) -> str:
    """
    Simulate LLM inference on input data.

    This is CPU-intensive processing that runs in a separate interpreter
    via InterpreterPoolExecutor. Each call simulates ~2 seconds of inference.

    Args:
        data: Input text for the LLM to process

    Returns:
        Simulated LLM analysis result
    """
    start_time = time.time()

    # Simulate CPU-intensive work
    # (In real application, this would be tokenization + forward pass)
    while time.time() - start_time < 2:
        # CPU-bound work that actually uses multiple cores with InterpreterPoolExecutor
        _ = sum(i**2 for i in range(100000))

    # Return simulated inference result
    return f"[LLM Analysis] Processed: {data[:80]}... Result: Summary of key insights."


# ============================================================================
# PART 3: Main Agent Orchestration (Students implement this - THE KEY PART)
# ============================================================================

async def ai_agent_query(
    user_question: str,
    executor: Executor | None = None
) -> dict[str, Any]:
    """
    Main AI agent function that orchestrates concurrent fetching and parallel processing.

    This is the capstone implementation that integrates:
    1. TaskGroup for concurrent API fetching
    2. InterpreterPoolExecutor for parallel inference
    3. Error handling and timeouts
    4. Result aggregation

    **Students implement this function with AI guidance using pattern:**

    1. Create executor if not provided
    2. Start overall system timeout (15 seconds)
    3. Use TaskGroup to fetch from all 3 sources concurrently
    4. For each successful result, submit to executor for LLM inference
    5. Wait for all inference tasks to complete
    6. Aggregate and return results

    Args:
        user_question: Question from user to process
        executor: Optional executor for CPU-bound work (created if not provided)

    Returns:
        Dictionary with aggregated results, errors, and metadata
    """
    # Student implementation starts here...
    #
    # Pseudocode (students fill in real implementation):
    #
    # start_time = perf_counter()
    # loop = asyncio.get_running_loop()
    #
    # if executor is None:
    #     executor = ProcessPoolExecutor(max_workers=os.cpu_count())
    #
    # async with asyncio.timeout(15):  # Overall system timeout
    #     async with asyncio.TaskGroup() as tg:
    #         # Fetch from all 3 sources concurrently
    #         weather_task = tg.create_task(fetch_weather("New York"))
    #         news_task = tg.create_task(fetch_news(user_question))
    #         kb_task = tg.create_task(query_knowledge_base(user_question))
    #
    #     # Process each result with LLM inference in parallel
    #     inference_tasks = [
    #         loop.run_in_executor(executor, simulate_llm_inference, str(result))
    #         for result in [weather_task.result(), news_task.result(), kb_task.result()]
    #     ]
    #
    #     inference_results = await asyncio.gather(*inference_tasks, return_exceptions=True)
    #
    #     # Aggregate results...
    #
    pass  # Placeholder for student implementation


# ============================================================================
# PART 4: Performance Benchmarking
# ============================================================================

async def benchmark_sequential(user_question: str) -> tuple[dict[str, Any], float]:
    """
    Baseline: Sequential execution (fetch one at a time, process one at a time).

    This shows why concurrency and parallelism matter.
    """
    start = time.perf_counter()

    # Sequential fetching
    weather = await fetch_weather("New York")
    news = await fetch_news(user_question)
    kb = await query_knowledge_base(user_question)

    # Sequential processing
    executor = ProcessPoolExecutor(max_workers=1)  # Only 1 worker = sequential
    loop = asyncio.get_running_loop()

    result1 = await loop.run_in_executor(executor, simulate_llm_inference, str(weather))
    result2 = await loop.run_in_executor(executor, simulate_llm_inference, str(news))
    result3 = await loop.run_in_executor(executor, simulate_llm_inference, str(kb))

    elapsed = time.perf_counter() - start

    return {
        "approach": "Sequential",
        "results": [result1, result2, result3],
        "errors": []
    }, elapsed


async def benchmark_hybrid(user_question: str) -> tuple[dict[str, Any], float]:
    """
    Optimized: Hybrid execution (concurrent fetching + parallel processing).

    This is what students build.
    """
    start = time.perf_counter()

    result = await ai_agent_query(user_question)
    elapsed = time.perf_counter() - start

    return result, elapsed


async def run_benchmarks(user_question: str = "What's happening right now?") -> None:
    """
    Compare sequential vs hybrid execution.

    Students run this to validate that their hybrid system is faster.
    """
    print("\n" + "="*70)
    print("ASYNCIO CAPSTONE: PERFORMANCE BENCHMARK")
    print("="*70)
    print(f"Question: {user_question}\n")

    # Run sequential baseline
    print("Running SEQUENTIAL baseline (fetch one at a time, process one at a time)...")
    seq_result, seq_time = await benchmark_sequential(user_question)
    print(f"Sequential execution: {seq_time:.2f} seconds")

    # Run hybrid optimized
    print("\nRunning HYBRID execution (concurrent fetch + parallel processing)...")
    hybrid_result, hybrid_time = await benchmark_hybrid(user_question)
    print(f"Hybrid execution: {hybrid_time:.2f} seconds")

    # Calculate improvement
    speedup = seq_time / hybrid_time
    improvement_pct = (1 - hybrid_time / seq_time) * 100

    print("\n" + "-"*70)
    print(f"Speedup: {speedup:.2f}x")
    print(f"Time saved: {seq_time - hybrid_time:.2f} seconds ({improvement_pct:.1f}%)")
    print("-"*70)

    # Validation
    if speedup >= 1.4:  # Target: 40%+ improvement
        print("✓ PERFORMANCE TARGET MET: Hybrid is 40%+ faster than sequential")
    else:
        print("⚠ Performance below target. Review executor configuration and task distribution.")

    print("="*70 + "\n")


# ============================================================================
# ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    # Run the benchmark
    asyncio.run(run_benchmarks("What is AI-native development?"))
```

#### 💬 AI Colearning Prompt

> "Walk me through how the `ai_agent_query()` function coordinates TaskGroup and InterpreterPoolExecutor. Which part runs concurrently (TaskGroup)? Which part runs in parallel (InterpreterPoolExecutor)? Draw a timeline showing when each stage executes."

---

## Part 3: Implementation Walkthrough

### Step 1: Understand the Skeleton

Before implementing, study the provided skeleton code above. Notice:

1. **Three data sources** with built-in timeouts
2. **One CPU-intensive function** (`simulate_llm_inference`)
3. **Main orchestrator** (`ai_agent_query`) — this is where you work
4. **Benchmark functions** to prove your system is faster

Your job: **Implement `ai_agent_query()` function** using TaskGroup and InterpreterPoolExecutor.

#### 🎓 Expert Insight

> In AI-native development, you don't code from scratch. You understand a pattern, see a skeleton, and ask AI to help fill the gaps. The `ai_agent_query()` function is that gap—it's where all concepts combine. You're not memorizing how to use TaskGroup and InterpreterPoolExecutor in isolation. You're architecting a system where both work together, optimally using I/O concurrency and CPU parallelism.

### Step 2: Implement Concurrent Fetching with TaskGroup

The first task: fetch from all 3 sources *at the same time* using structured concurrency.

#### 🚀 CoLearning Challenge

Ask your AI:

> "I need to fetch from 3 async sources concurrently and handle timeouts gracefully. The fetchers are: fetch_weather(), fetch_news(), query_knowledge_base(). Show me how to use TaskGroup to fetch all 3 in parallel. If one times out, the system should continue with the others. Explain the TaskGroup pattern step-by-step."

**Expected Output**: AI shows code using `async with asyncio.TaskGroup() as tg:` with proper error handling.

**Your Validation**: Run the fetching code and verify:
- All 3 sources start fetching (logs should show starting almost simultaneously)
- Total fetch time is ~3 seconds (limited by longest source), not 6.5 seconds
- TimeoutError from any source doesn't crash the system

### Step 3: Implement Parallel Processing with InterpreterPoolExecutor

Once you have all results, process each with LLM inference in parallel.

#### ✨ Teaching Tip

> Use your AI to explore the bridge between async and parallel execution: "How do I use `loop.run_in_executor()` to call `simulate_llm_inference()` from async code? Why do I need an executor instead of just calling the function directly?"

**Pattern you're building**:

```
TaskGroup fetches 3 sources concurrently (~3s)
    ↓
Results returned (weather, news, KB answer)
    ↓
InterpreterPoolExecutor processes each (~2s per source, in parallel across cores)
    ↓
All inference results available
    ↓
Aggregate and return
```

Your implementation should:

1. Get the event loop: `loop = asyncio.get_running_loop()`
2. Create executor: `executor = ProcessPoolExecutor(max_workers=os.cpu_count())`
3. Submit each inference: `await loop.run_in_executor(executor, simulate_llm_inference, data)`
4. Wait for all: `results = await asyncio.gather(...)`

### Step 4: Add Error Handling and Timeouts

Real systems fail. Your agent must handle:

- **Individual API timeouts**: Apply 5-second timeout to each fetch
- **Overall system timeout**: Apply 15-second timeout to entire operation
- **Partial success**: If 2 sources work and 1 times out, return the 2 results with error info for the 3rd
- **Logging**: Log all operations for debugging

**Pattern**:

```python
async with asyncio.timeout(15):  # Overall system timeout
    # If anything inside takes >15s, TimeoutError is raised
    # Your code must handle this gracefully
    try:
        # Fetching and processing
    except asyncio.TimeoutError:
        # Return whatever you have collected so far
```

### Step 5: Aggregate and Return Results

Collect all results (successful and failed) into a comprehensive response:

```python
result = {
    "query": user_question,
    "success": True/False,
    "sources": {
        "weather": { ... successful results or error },
        "news": { ... },
        "knowledge_base": { ... }
    },
    "inferences": [
        "LLM analysis of weather...",
        "LLM analysis of news...",
        "LLM analysis of KB result..."
    ],
    "errors": [],  # Any errors encountered
    "execution_time": total_time,
    "cores_used": os.cpu_count()
}
```

---

## Part 4: Testing and Validation

### Test 1: Verify Concurrent Fetching

**What to check**: All 3 sources start nearly simultaneously (not sequentially).

```python
# Run with logging enabled
asyncio.run(ai_agent_query("Test query"))

# Expected logs (notice timestamps are close):
# INFO - Fetching weather for New York...
# INFO - Fetching news for 'Test query'...
# INFO - Querying knowledge base for 'Test query'...
# (All start within milliseconds of each other)

# Expected total fetch time: ~3 seconds (longest source)
# NOT 6.5 seconds (sum of all)
```

### Test 2: Verify Parallel Processing

**What to check**: Multiple cores are used for inference (CPU spikes).

```bash
# Terminal 1: Run your agent
python ai_agent.py

# Terminal 2: Watch CPU usage
# For Linux/Mac:
top -o %CPU

# For Windows:
tasklist /V /FI "IMAGENAME eq python.exe"

# You should see:
# - CPU jumps to 200%+ (multiple cores active)
# - Not just 100% (single core)
```

### Test 3: Handle Timeout Gracefully

**What to check**: System continues if one source times out.

```python
# Modify fetch_weather to always timeout:
async def fetch_weather(...):
    await asyncio.sleep(6)  # Exceeds 5s timeout

# Expected result:
# {"source": "weather", "error": "timeout"}
#
# System still returns results from news and knowledge_base

# Total execution should be ~5 seconds (timeout applied)
# Not crash or hang
```

### Test 4: Run Benchmark

**What to check**: Hybrid execution is 2-3x faster than sequential.

```python
asyncio.run(run_benchmarks())

# Expected output:
# Sequential execution: ~12.5 seconds (6.5 fetch + 6 inference)
# Hybrid execution: ~5 seconds (3 concurrent fetch + 2 parallel inference)
# Speedup: 2.5x
```

---

## Part 5: Extension Challenges

Once your capstone works, push further:

### Challenge 1: Add Caching

Modify `ai_agent_query()` to cache results by question. If the same question is asked within 60 seconds, return cached results instead of re-fetching.

```python
# Ask your AI: "How do I implement a simple in-memory cache that expires results after 60 seconds? Should I use a dict with timestamps?"
```

### Challenge 2: Implement Retry Logic

If a source times out, retry up to 2 times before giving up.

```python
# Ask your AI: "How do I retry a timed-out request? Should the retry timeout be shorter or longer?"
```

### Challenge 3: Add More Sources

Extend to 5+ sources. Verify that performance scales linearly with TaskGroup concurrency.

```python
# Ask your AI: "I want to add 2 more async data sources. How should I structure TaskGroup to handle 5 sources?"
```

### Challenge 4: Simulate Real AI Processing

Replace `simulate_llm_inference()` with a more realistic simulation that:
- Tokenizes input (split into words)
- Computes embeddings (calculate vector for each token)
- Ranks top results
- Returns structured JSON

---

## Part 6: Architecture Review with AI

Once your capstone is complete, validate it with your AI:

#### 💬 AI Colearning Prompt

> "I built an AI agent that fetches from 3 sources concurrently and processes each with LLM inference in parallel. Here's my code: [paste your ai_agent_query() function]. Can you:
> 1. Explain how TaskGroup ensures structured concurrency (what happens if one fetch fails?)
> 2. Explain how InterpreterPoolExecutor achieves true parallelism (why can't threading do this?)
> 3. Identify any error handling gaps or improvements"

**Expected Outcome**: AI reviews your architecture and provides concrete feedback on structured concurrency, error recovery, and GIL implications.

---

## Key Learnings Summary

This capstone demonstrates:

1. **TaskGroup** provides structured concurrency — all tasks must complete before moving on, and failure in one cancels others (intentional design)
2. **InterpreterPoolExecutor** provides true parallelism — multiple Python interpreters with separate GILs let CPU-bound work run on multiple cores
3. **Hybrid workloads** combine both patterns:
   - Use TaskGroup for I/O-bound work (API calls)
   - Use InterpreterPoolExecutor for CPU-bound work (inference)
   - Overlap them: fetch while processing, process while fetching
4. **Error handling** is critical — graceful degradation (continue with partial results) is more valuable than failing completely
5. **Timeouts** prevent hang states — always apply timeouts to external operations
6. **Performance measurement** proves the pattern works — measure sequential vs hybrid to quantify the benefit

---

## Challenge 6: The Complete AI Agent Capstone (5-Part)

This is a **5-part bidirectional learning challenge** where you complete, evaluate, and reflect on your production AI agent system.

### Part 1: Verify and Test Your Implementation (Student as Quality Engineer)
**Your Challenge**: Ensure your built system actually works as intended.

**Verification Checklist**:
1. Run your complete AI agent system from Part 4 of the main lesson
2. Measure actual execution time (should be ~5-6 seconds, not 9+ seconds sequential)
3. Verify concurrent fetching: all 3 sources start within 100ms of each other
4. Verify parallel processing: CPU work happens on multiple cores simultaneously
5. Test error handling: simulate one source timing out; system should continue
6. Document timing: `{fetch_time, process_time, total_time, speedup_vs_sequential}`

**Expected Behavior**:
- Concurrent fetch: 3 seconds (longest single source, not sum)
- Parallel process: 2 seconds (3 tasks on multiple cores)
- Total: 5-6 seconds (stages overlap, not sequential)
- Speedup: 1.5-2x vs sequential (~10 seconds)

**Deliverable**: Create `/tmp/capstone_verification.md` documenting:
- Actual measured timings
- Proof of concurrency (execution timeline)
- Proof of parallelism (core utilization)
- Error scenarios tested
- Performance improvement quantified

---

### Part 2: Discover Optimization Opportunities (Student as Scientist)
**Your Challenge**: Identify bottlenecks without AI help first.

**Analysis Tasks**:
1. Run with different number of fetch sources (2, 3, 4, 5) — how does total time scale?
2. Run with different process times (1s, 2s, 3s) — what becomes the bottleneck?
3. Add timing instrumentation: log exactly when each fetch starts/completes, when each process starts/completes
4. Create a timeline visualization showing overlap (or lack thereof)
5. Identify the critical path: which task/stage limits overall execution?

**Expected Observations**:
- Adding more sources increases fetch time (more tasks)
- Processing time affects when results are ready
- Critical path is: max(fetch_times) + max(process_times), not sum
- Pipeline is NOT fully optimal—there's opportunity for improvement

**Self-Validation**:
- Can you explain why adding a 4th source didn't increase time much?
- What would happen if processing was much slower than fetching?
- How would you fix a bottlenecked pipeline?

---

### Part 3: AI as Optimization Coach (AI Teaching You Production Patterns)
**Your AI Prompt**:
> "I built an AI agent that fetches from 3 sources (2-3s each) and processes results (2s each). Total time is ~5 seconds. I added timing instrumentation and identified that fetch is the bottleneck. Teach me about optimization strategies: 1) Early timeout to fail fast, 2) Caching to avoid refetching, 3) Request prioritization (fetch fast sources first), 4) Adaptive batch sizing. Show me code examples for each. Which would help most in my system?"

**AI's Role**: Explain production optimization patterns, show code for each strategy, help you reason about tradeoffs (latency vs memory, throughput vs latency).

**Interactive Moment**: Ask a clarifying question:
> "You showed me timeout for fast-fail. But what if a source is occasionally slow (97% of time 0.5s, but sometimes 5s)? How do I balance reliability (waiting for slow source) with latency (timing out)? What's the right timeout value?"

**Expected Outcome**: AI clarifies that optimization is contextual—you optimize for your constraints (latency SLO, error budget, resource limits). You learn to make principled engineering decisions, not just chase speedup.

---

### Part 4: You as Architect (Correcting and Extending the System)
**Setup**: AI generates an optimized version of your system. Your job is to test it, find issues, and teach AI about production realities.

**AI's Initial Code** (ask for this):
> "Show me an optimized version of the AI agent that: 1) Implements per-source timeout (fail fast on slow sources), 2) Uses asyncio.as_completed() to process results as they arrive (not wait for all), 3) Adds exponential backoff for retries. Explain why this is better than my current version."

**Your Task**:
1. Run the optimized version. Measure timing.
2. Compare to your original: is it faster? By how much?
3. Identify potential issues:
   - Does early timeout help or hurt? (Might fail unnecessarily)
   - Does as_completed() improve UX or create race conditions?
   - Is retry logic robust or does it hang on certain failures?
4. Teach AI:
> "Your optimized version is 10% faster but sometimes times out sources that would have succeeded in 3.5 seconds. The timeout you chose (3s) is too aggressive for my use case. How do I determine the right timeout for each source? Should it be static or adaptive based on historical latency?"

**Your Edge Case Discovery**: Ask AI:
> "What if I want to return 'best effort' results as soon as 2 out of 3 sources complete, instead of waiting for all 3? How would that change the architecture? What problems might that create?"

**Expected Outcome**: You discover that optimization has tradeoffs—faster timeouts mean more failures, early returns mean incomplete results. You learn to think about requirements first, then optimize.

---

### Part 5: Reflect and Synthesize (Student as Thoughtful Engineer)
**Your Challenge**: Synthesize everything you've learned about asyncio into principle-based thinking.

**Reflection Tasks**:
1. **Conceptual Mapping**: Draw a diagram showing how Lessons 1-5 concepts connect:
   - Event loops (Lesson 1) enable TaskGroup (Lesson 2)
   - await/pause points (Lesson 1) create opportunities for timeouts (Lesson 3)
   - TaskGroup (Lesson 2) can't help CPU work, so InterpreterPoolExecutor (Lesson 4) needed
   - All together form the hybrid system (Lesson 5)

2. **Principles Documentation**: Write 3-5 principles you've learned:
   - Example: "I/O-bound work benefits from concurrency (TaskGroup), CPU-bound work needs parallelism (InterpreterPoolExecutor)"
   - Example: "Timeouts are defensive—without them, one slow source hangs the whole system"
   - Example: "Error handling must distinguish transient (retry) vs permanent (fail fast) failures"

3. **Production Checklist**: Create a checklist for building production asyncio systems:
   - [ ] Event loop management (asyncio.run at top level)
   - [ ] Structured concurrency (TaskGroup, not raw create_task)
   - [ ] Timeout controls (prevent infinite waits)
   - [ ] Error handling (distinguish error types)
   - [ ] Resource cleanup (executor shutdown, queue drains)
   - [ ] Performance measurement (baseline vs optimized)

4. **AI Conversation**: Talk to AI about your system as if you were explaining it to a colleague:
> "Our AI agent works like this: [describe your architecture]. The key insights are: [principles]. I'd optimize by [your optimization strategy]. What are the risks I haven't considered? What production issues might I hit?"

**Expected Outcome**: AI identifies edge cases you missed (concurrency bugs, resource exhaustion, corner cases). You learn from production experience vicariously.

**Deliverable**: Save to `/tmp/capstone_reflection.md`:
- Concept map (text or ASCII diagram)
- 5 Core principles you've learned
- Production checklist
- Summary of architecture and optimization choices
- Identified risks and mitigation strategies

---

### Chapter Synthesis: From Concepts to Production

**You've now mastered**:
- Layer 1 (Foundations): Understanding event loops and coroutines
- Layer 2 (Collaboration): TaskGroup, gather(), timeout patterns
- Layer 3 (Intelligence): Hybrid I/O+CPU systems, bottleneck analysis
- Layer 4 (Integration): Production-grade system with error handling, timeouts, optimization

**You can now**:
- Build systems that scale to 1000+ concurrent I/O operations
- Parallelize CPU work across cores despite the GIL
- Handle real-world failures gracefully
- Optimize based on bottleneck analysis
- Explain architectural decisions to colleagues

---

**Time Estimate**: 50-65 minutes (10 min verification, 10 min discovery, 12 min coach interaction, 12 min optimization, 6-15 min reflection)

**Key Takeaway**: Asyncio isn't just syntax—it's a mindset. Event loops, structured concurrency, careful error handling, and measurement are how real systems are built. You've learned the architecture, not just the code.
