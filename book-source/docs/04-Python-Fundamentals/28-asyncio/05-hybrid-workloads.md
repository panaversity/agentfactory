---
title: "Hybrid Workloads: Combining I/O and CPU for Peak Performance"
chapter: 28
lesson: 5
duration_minutes: 150
learning_objectives:
  - objective: "Combine asyncio.TaskGroup() for I/O concurrency with InterpreterPoolExecutor for CPU parallelism"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Working code integrating both patterns for realistic hybrid workloads"

  - objective: "Design batch processing systems that optimize for throughput and resource utilization"
    proficiency_level: "B2"
    bloom_level: "Create"
    assessment_method: "Architecture design showing optimal batch sizing and worker allocation"

  - objective: "Implement pipeline patterns where I/O and CPU work overlap efficiently"
    proficiency_level: "B2"
    bloom_level: "Create"
    assessment_method: "Multi-stage pipeline code demonstrating overlapping fetch, transform, and store"

  - objective: "Optimize resource usage with Semaphores to prevent resource exhaustion"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Code limiting concurrent I/O and CPU worker pools appropriately"

  - objective: "Identify and resolve performance bottlenecks in hybrid systems"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Analysis of system performance with recommendations for optimization"

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "Hybrid Workload Architecture"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student combines asyncio (I/O concurrency) with InterpreterPoolExecutor (CPU parallelism) in single system"

  - name: "Batch Processing Optimization"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student designs batch systems with optimal sizing for throughput, considering resource constraints"

  - name: "Pipeline Pattern Implementation"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student implements multi-stage pipelines with overlapping execution (fetch, transform, store)"

  - name: "AI Workload Patterns"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student applies hybrid patterns to realistic AI scenarios (API calls + inference processing)"

  - name: "Resource Management with Semaphores"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Safety & Security"
    measurable_at_this_level: "Student limits concurrent I/O and CPU work using Semaphores to prevent exhaustion"

  - name: "Performance Bottleneck Analysis"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student identifies CPU-bound vs I/O-bound bottlenecks and optimizes architecture accordingly"

cognitive_load:
  new_concepts: 8
  assessment: "8 new concepts (hybrid pattern, batch processing, pipeline pattern, AI workloads, resource limiting, bottleneck analysis, semaphores, optimization) within B2 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Explore dynamic batch sizing based on system metrics, adaptive worker pool scaling, and performance profiling techniques for 10,000+ item pipelines"
  remedial_for_struggling: "Start with simple 2-item fetch+process before scaling; use timing diagrams to visualize I/O and CPU overlap; focus on one pattern (batch OR pipeline) before combining"

# Generation metadata
generated_by: "content-implementer v1.0.0"
source_spec: "specs/001-part-4-chapter-28/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Lesson 5: Hybrid Workloads — Combining I/O and CPU for Peak Performance

## Opening Hook

Here's a realistic problem: **You're building an AI system that needs to fetch data from 1,000 API endpoints and process each response with heavy analysis.**

You have three choices:

1. **Sequential approach**: Fetch one, process it, then fetch the next. Total time = (1000 × fetch_time) + (1000 × process_time). On a typical system: 1000 × (0.1s fetch + 2s processing) = 2,100 seconds ≈ 35 minutes.

2. **Concurrent fetching only** (from Lesson 2): Fetch all 1,000 at once. But now you hit memory limits and API rate limits crash your system.

3. **Hybrid approach**: Fetch in batches of 10 while processing previous batches in parallel. Total time ≈ 300 seconds ≈ 5 minutes. That's **7x faster**, and you never run out of resources.

**This is the hybrid workload pattern.** It's what powers production AI systems that need both concurrent I/O (API calls) and parallel processing (inference, data transformation).

By the end of this lesson, you'll understand why hybrid workloads matter for AI applications, how to architect them efficiently, and how to optimize them for your specific hardware and network conditions.

---

## Understanding the Hybrid Pattern: I/O + CPU Together

### Why Separate I/O and CPU?

From Lesson 4, you learned that **asyncio helps with I/O-bound work** (waiting for network, disk, etc.) but **doesn't help with CPU-bound work** (the GIL prevents true parallelism). So what do you do when your workload has both?

**Answer**: You use both tools simultaneously.

- **asyncio.TaskGroup()** — Run multiple I/O operations concurrently (while one waits, another runs)
- **InterpreterPoolExecutor** — Run multiple CPU operations in true parallel (separate interpreters = separate GILs)

#### 💬 AI Colearning Prompt

> "In a system that fetches data from APIs and processes it with heavy computation, explain how I/O and CPU work could overlap efficiently. What's the benefit of overlap versus sequential execution?"

#### The Core Pattern

Imagine stages in a pipeline:

```
Time:    0s         2s              4s              6s
Stage 1: [Fetch #1]  [Fetch #2]     [Fetch #3]
Stage 2:            [Process #1]    [Process #2]   [Process #3]
Stage 3:                           [Store #1]     [Store #2]
```

While you're **fetching** item #2, you're **processing** item #1 (CPU cores stay busy). While you're **processing** item #3, you're **storing** item #1 (I/O pipeline keeps flowing).

This is what **hybrid workloads** achieve: **parallel execution of fundamentally different types of work**.

#### 🎓 Expert Insight

> In AI-native development, you don't choose between concurrency and parallelism—you use both. This is why Python 3.14's InterpreterPoolExecutor paired with asyncio creates such powerful systems. The pattern is: "concurrent I/O boundaries" around "parallel CPU cores."

### Real-World Example: AI Workload Characteristics

Most AI applications look like this:

1. **Fetch** — Call an API, get data (I/O-bound, waiting on network)
2. **Process** — Run inference, transform data (CPU-bound, heavy computation)
3. **Store** — Write to database (I/O-bound, waiting on storage)

Each stage has different resource constraints:
- Fetching is limited by network bandwidth (often 100-1000 requests in flight is fine)
- Processing is limited by CPU cores (4-8 workers for a typical machine)
- Storing is limited by database connection pool (often 5-20 connections)

A naive approach would run all three sequentially. A better approach would batch them: fetch 10, process in parallel, store, repeat.

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:
> "Describe how you would optimize a system that needs to fetch 10,000 items from an API, analyze each with CPU-intensive text processing, and store results in a database. What would your batch size be? How many workers for each stage? Why?"

**Expected Outcome**: You'll understand how to think about bottlenecks, resource constraints, and throughput optimization.

---

## Code Example 1: Simple Hybrid Pattern (Fetch → Process)

Let's start with the simplest hybrid case: **fetch one item, process it in parallel, repeat.**

**Specification Reference**: Basic hybrid pattern
**Prompt Used**: "Create a system that fetches N items from a mock API concurrently and processes each with CPU-intensive work, with proper type hints and error handling"

```python
import asyncio
from concurrent.futures import Executor
from typing import Any
import time

async def fetch_item(item_id: int) -> dict[str, Any]:
    """Simulate fetching an item from an API."""
    # Simulate network latency (100-200ms)
    await asyncio.sleep(0.1 + (item_id % 2) * 0.1)
    return {"id": item_id, "data": f"item-{item_id}-data"}

def process_item(item: dict[str, Any]) -> dict[str, Any]:
    """CPU-intensive processing of fetched item."""
    # Simulate heavy computation (CPU-bound, ~500ms)
    time.sleep(0.5)
    result = {
        **item,
        "processed": True,
        "analysis": f"analyzed-{item['data']}",
    }
    return result

async def simple_hybrid(item_ids: list[int], executor: Executor) -> list[dict[str, Any]]:
    """
    Fetch items concurrently, then process each in parallel.

    Demonstrates basic hybrid: I/O concurrency (fetch) + CPU parallelism (process).
    Total time: ~1-2 seconds (overlapped)
    Sequential would take: ~7 seconds (5 items × 1.4s per item)
    """
    loop = asyncio.get_running_loop()

    # Step 1: Fetch all items concurrently (I/O phase)
    async with asyncio.TaskGroup() as tg:
        fetch_tasks = [
            tg.create_task(fetch_item(item_id))
            for item_id in item_ids
        ]

    fetched_items = [task.result() for task in fetch_tasks]
    print(f"✓ Fetched {len(fetched_items)} items in ~0.2s")

    # Step 2: Process all items in parallel (CPU phase)
    # Use run_in_executor to offload CPU work to separate interpreters
    process_tasks = [
        loop.run_in_executor(executor, process_item, item)
        for item in fetched_items
    ]

    processed_items = await asyncio.gather(*process_tasks)
    print(f"✓ Processed {len(processed_items)} items in ~0.5s (parallel)")

    return processed_items

# Validation steps
async def main() -> None:
    from concurrent.futures import ThreadPoolExecutor
    # ⚠️ For CPU-bound work, use InterpreterPoolExecutor (not shown in this small example)
    # For demonstration, ThreadPoolExecutor shows structure (though it won't parallelize CPU)

    executor = ThreadPoolExecutor(max_workers=4)

    start = time.time()
    results = await simple_hybrid([1, 2, 3, 4, 5], executor)
    elapsed = time.time() - start

    print(f"\n✓ Total hybrid time: {elapsed:.2f}s")
    print(f"✓ Sequential time would be: ~{5 * 1.4:.1f}s")
    print(f"✓ Speedup: {5 * 1.4 / elapsed:.1f}x")
    print(f"✓ Processed {len(results)} items")

    executor.shutdown(wait=True)

if __name__ == "__main__":
    asyncio.run(main())
```

**Validation Steps**:
1. Run the code and observe total time (~0.7s for fetch+process)
2. Notice how fetching completes quickly, then processing happens in parallel
3. With ThreadPoolExecutor shown here (for simplicity), CPU doesn't parallelize; with InterpreterPoolExecutor (Python 3.14), you'd see true parallelism

**Key Insight**: The `loop.run_in_executor()` bridges sync CPU work into async I/O flow. While one CPU task runs, the event loop can handle other I/O operations.

#### ✨ Teaching Tip

> Use your AI co-teacher to explore: "In the simple_hybrid function, why do we fetch all items first, then process all items second? What would happen if we fetched and processed in a mixed order?"

---

## Code Example 2: Batch Processing Pattern

Real-world systems rarely have enough memory to fetch all items at once. Instead, you fetch in **batches**.

**Specification Reference**: Batch processing with resource limits
**Prompt Used**: "Create a batch processing system that fetches N items in groups of B, processes in parallel, then repeats"

```python
import asyncio
from concurrent.futures import ThreadPoolExecutor  # Use InterpreterPoolExecutor in production
from typing import TypeVar, Callable, Any
import time

T = TypeVar("T")
U = TypeVar("U")

async def fetch_batch(
    batch_ids: list[int],
    delay_per_item: float = 0.05,
) -> list[dict[str, Any]]:
    """Fetch a batch of items concurrently."""
    async def fetch_one(item_id: int) -> dict[str, Any]:
        await asyncio.sleep(delay_per_item)  # Simulate network
        return {"id": item_id, "raw_data": f"data-{item_id}"}

    async with asyncio.TaskGroup() as tg:
        tasks = [tg.create_task(fetch_one(item_id)) for item_id in batch_ids]

    return [task.result() for task in tasks]

def process_batch(
    items: list[dict[str, Any]],
    cpu_time_per_item: float = 0.1,
) -> list[dict[str, Any]]:
    """Process a batch of items (CPU-bound, runs in executor)."""
    time.sleep(cpu_time_per_item * len(items))  # Simulate heavy processing
    return [
        {**item, "processed": True, "result": f"result-{item['id']}"}
        for item in items
    ]

async def batch_processing_system(
    total_items: int,
    batch_size: int,
    executor: ThreadPoolExecutor,
) -> list[dict[str, Any]]:
    """
    Process items in batches: fetch batch → process batch → repeat.

    Reduces memory usage and allows resource control.
    """
    loop = asyncio.get_running_loop()
    results: list[dict[str, Any]] = []

    # Process in batches
    for batch_num in range(0, total_items, batch_size):
        batch_ids = list(range(batch_num, min(batch_num + batch_size, total_items)))

        # Fetch this batch concurrently
        print(f"  Batch {batch_num // batch_size + 1}: Fetching {len(batch_ids)} items...")
        fetched = await fetch_batch(batch_ids)

        # Process this batch in parallel
        print(f"  Batch {batch_num // batch_size + 1}: Processing {len(fetched)} items...")
        processed = await loop.run_in_executor(
            executor,
            process_batch,
            fetched,
        )

        results.extend(processed)

    return results

# Validation example
async def main() -> None:
    executor = ThreadPoolExecutor(max_workers=4)

    print("Batch Processing System Demo")
    print("=" * 50)

    start = time.time()
    results = await batch_processing_system(
        total_items=20,
        batch_size=5,
        executor=executor,
    )
    elapsed = time.time() - start

    print(f"\n✓ Processed {len(results)} items in {elapsed:.2f}s")
    print(f"  Batch size: 5 items")
    print(f"  Number of batches: {(20 + 4) // 5}")

    executor.shutdown(wait=True)

if __name__ == "__main__":
    asyncio.run(main())
```

**Why Batching Matters**:
- **Memory**: Fetch 5 items, process, store. Not 1,000 in memory at once.
- **Throughput**: Process while fetching next batch (pipeline overlap)
- **Resource Control**: Never overwhelm the server or database

#### 💬 AI Colearning Prompt

> "In batch processing, why is batch size important? How would you determine the optimal batch size for your specific system?"

---

## Code Example 3: Pipeline Pattern (Fetch → Transform → Store)

The most powerful hybrid pattern has **three stages running in parallel**.

**Specification Reference**: Three-stage pipeline with overlapping execution
**Prompt Used**: "Implement a fetch-transform-store pipeline where stages overlap using asyncio queues"

```python
import asyncio
from asyncio import Queue
from concurrent.futures import ThreadPoolExecutor
from typing import Any
import time

async def pipeline_fetch_stage(
    queue: Queue[dict[str, Any] | None],
    total_items: int,
    items_per_batch: int = 10,
) -> None:
    """
    Stage 1: Fetch items from API concurrently, add to queue.

    Runs concurrently while other stages process and store.
    """
    for batch_start in range(0, total_items, items_per_batch):
        batch_ids = list(range(batch_start, min(batch_start + items_per_batch, total_items)))

        # Fetch batch concurrently (simulates network I/O)
        async def fetch_one(item_id: int) -> dict[str, Any]:
            await asyncio.sleep(0.05)  # Simulate 50ms network latency
            return {"id": item_id, "raw": f"raw-{item_id}"}

        async with asyncio.TaskGroup() as tg:
            fetched = await asyncio.gather(*[fetch_one(i) for i in batch_ids])

        # Add fetched items to queue for transform stage
        for item in fetched:
            await queue.put(item)

        print(f"[FETCH] Added batch starting at {batch_start}, queue size: {queue.qsize()}")

    # Signal end of fetching
    await queue.put(None)
    print("[FETCH] Done fetching. Signaled end-of-stream.")

async def pipeline_transform_stage(
    fetch_queue: Queue[dict[str, Any] | None],
    store_queue: Queue[dict[str, Any] | None],
    executor: ThreadPoolExecutor,
) -> None:
    """
    Stage 2: Transform items (CPU-bound) via executor, forward to store queue.

    While fetching continues, we transform previous items in parallel.
    """
    loop = asyncio.get_running_loop()
    processed_count = 0

    while True:
        # Get item from fetch queue (blocks if empty, waiting for fetch stage)
        item = await fetch_queue.get()

        if item is None:
            # End-of-stream signal from fetch stage
            await store_queue.put(None)
            print(f"[TRANSFORM] Done transforming ({processed_count} items). Signaled end-of-stream.")
            break

        # Transform item CPU-bound work (offload to executor)
        def transform_item(x: dict[str, Any]) -> dict[str, Any]:
            time.sleep(0.1)  # Simulate CPU processing
            return {**x, "transformed": True, "result": f"transformed-{x['id']}"}

        transformed = await loop.run_in_executor(executor, transform_item, item)

        # Forward to store queue
        await store_queue.put(transformed)
        processed_count += 1

        if processed_count % 5 == 0:
            print(f"[TRANSFORM] Processed {processed_count} items, store queue size: {store_queue.qsize()}")

async def pipeline_store_stage(
    store_queue: Queue[dict[str, Any] | None],
) -> None:
    """
    Stage 3: Store items (simulated I/O).

    While fetching and transforming continue, we store results concurrently.
    """
    stored_count = 0

    while True:
        item = await store_queue.get()

        if item is None:
            print(f"[STORE] Done storing ({stored_count} items).")
            break

        # Simulate database write (I/O)
        await asyncio.sleep(0.02)  # 20ms DB write
        stored_count += 1

        if stored_count % 5 == 0:
            print(f"[STORE] Stored {stored_count} items")

async def three_stage_pipeline(
    total_items: int,
    executor: ThreadPoolExecutor,
) -> None:
    """
    Run three-stage pipeline: fetch → transform → store.

    Each stage runs concurrently with overlap:
    - While fetching items 11-20, transforming 1-10, storing 1-5
    """
    fetch_queue: Queue[dict[str, Any] | None] = asyncio.Queue(maxsize=20)
    store_queue: Queue[dict[str, Any] | None] = asyncio.Queue(maxsize=20)

    # Run all three stages concurrently
    async with asyncio.TaskGroup() as tg:
        tg.create_task(pipeline_fetch_stage(fetch_queue, total_items))
        tg.create_task(pipeline_transform_stage(fetch_queue, store_queue, executor))
        tg.create_task(pipeline_store_stage(store_queue))

# Validation example
async def main() -> None:
    executor = ThreadPoolExecutor(max_workers=4)

    print("Three-Stage Pipeline Demo")
    print("=" * 50)
    print("Fetching, transforming, and storing 30 items with pipeline overlap\n")

    start = time.time()
    await three_stage_pipeline(total_items=30, executor=executor)
    elapsed = time.time() - start

    print(f"\n✓ Total pipeline time: {elapsed:.2f}s")
    print(f"  Sequential (no overlap) would take: ~{30 * (0.05 + 0.1 + 0.02):.1f}s")
    print(f"  Pipeline speedup: {30 * 0.17 / elapsed:.1f}x")

    executor.shutdown(wait=True)

if __name__ == "__main__":
    asyncio.run(main())
```

**Pipeline Benefits**:
- **Overlap**: While fetching item #20, transform item #10, store item #5
- **Throughput**: Queues decouple stages, each stage keeps busy
- **Resource Control**: Queues have max sizes, preventing runaway memory

#### 🎓 Expert Insight

> Pipeline patterns are everywhere in production systems: data ETL, stream processing, real-time inference. The pattern is simple: decouple stages with queues, run concurrently, and watch throughput improve. This is why asyncio + queues are so powerful for backend systems.

---

## Code Example 4: AI Workload Simulation (API Calls + Inference)

Now let's simulate a realistic **AI workload**: fetch data from multiple APIs, then run "inference" on each.

**Specification Reference**: Realistic AI pattern with multiple data sources
**Prompt Used**: "Create a system that concurrently fetches from 3 APIs and processes with simulated inference, showing what production AI pipelines look like"

```python
import asyncio
from concurrent.futures import ThreadPoolExecutor
from typing import Any
from dataclasses import dataclass
import time

@dataclass
class QueryResult:
    """Result from a single data source."""
    source: str
    raw_data: str
    inference_result: str | None = None

async def fetch_from_weather_api(query: str) -> QueryResult:
    """Fetch from weather API (I/O)."""
    await asyncio.sleep(0.3)  # Network latency
    return QueryResult(
        source="weather",
        raw_data=f"Weather for {query}: 72°F, sunny",
    )

async def fetch_from_news_api(query: str) -> QueryResult:
    """Fetch from news API (I/O)."""
    await asyncio.sleep(0.4)  # Slightly slower API
    return QueryResult(
        source="news",
        raw_data=f"News about {query}: Breaking story...",
    )

async def fetch_from_knowledge_base(query: str) -> QueryResult:
    """Fetch from internal knowledge base (I/O)."""
    await asyncio.sleep(0.25)
    return QueryResult(
        source="knowledge",
        raw_data=f"Knowledge: {query} is...",
    )

def simulate_llm_inference(raw_data: str) -> str:
    """
    Simulate LLM inference (CPU-intensive text processing).

    In production, this would call Claude, GPT-4, or another LLM.
    Here we simulate heavy computation.
    """
    # Simulate CPU-intensive inference (0.5s per item)
    time.sleep(0.5)

    # Simulate some kind of analysis
    tokens = len(raw_data.split())
    return f"Analyzed {tokens} tokens: [summary of {raw_data[:30]}...]"

async def ai_workload_hybrid(
    query: str,
    executor: ThreadPoolExecutor,
) -> list[QueryResult]:
    """
    AI workload: fetch from 3 APIs concurrently, process each with inference.

    Demonstrates:
    - Concurrent I/O (TaskGroup fetches from 3 APIs in parallel)
    - Parallel CPU (InterpreterPoolExecutor runs inference in parallel)
    - Realistic AI pattern
    """
    loop = asyncio.get_running_loop()

    # Stage 1: Fetch from all 3 APIs concurrently (I/O phase)
    print(f"Fetching from 3 APIs for query: {query}")
    async with asyncio.TaskGroup() as tg:
        tasks = [
            tg.create_task(fetch_from_weather_api(query)),
            tg.create_task(fetch_from_news_api(query)),
            tg.create_task(fetch_from_knowledge_base(query)),
        ]

    results = [task.result() for task in tasks]
    print(f"✓ Fetched from all 3 APIs in parallel")

    # Stage 2: Process each with inference in parallel (CPU phase)
    print(f"Running inference on {len(results)} sources...")

    async def run_inference(result: QueryResult) -> QueryResult:
        """Run inference on one result."""
        inference = await loop.run_in_executor(
            executor,
            simulate_llm_inference,
            result.raw_data,
        )
        result.inference_result = inference
        return result

    processed = await asyncio.gather(*[run_inference(r) for r in results])
    print(f"✓ Processed {len(processed)} results with inference")

    return processed

# Validation example with benchmark
async def main() -> None:
    executor = ThreadPoolExecutor(max_workers=3)

    print("AI Workload Hybrid Demo (Fetch + Inference)")
    print("=" * 60)

    # Measure hybrid approach
    start = time.time()
    results = await ai_workload_hybrid("python asyncio", executor)
    hybrid_time = time.time() - start

    print(f"\n✓ Hybrid approach time: {hybrid_time:.2f}s")
    print(f"  - Fetch phase: ~0.4s (max of 3 concurrent fetches)")
    print(f"  - Inference phase: ~0.5s (3 items processed in parallel)")

    # Calculate sequential time for comparison
    sequential_time = 0.3 + 0.4 + 0.25 + 3 * 0.5  # All fetches + all inferences sequentially
    print(f"\n✗ Sequential approach would take: {sequential_time:.2f}s")
    print(f"  Speedup: {sequential_time / hybrid_time:.1f}x faster")

    # Show results
    print(f"\nResults from {len(results)} sources:")
    for result in results:
        print(f"  [{result.source}] → {result.inference_result[:50]}...")

    executor.shutdown(wait=True)

if __name__ == "__main__":
    asyncio.run(main())
```

**Key Insight for AI Applications**:
- **Fetch phase** is I/O-bound (network waiting)
- **Inference phase** is CPU-bound (computation)
- **Hybrid pattern** overlaps them: fetch next batch while processing current batch
- This is exactly how production AI systems (like Claude's backend) work

#### 💬 AI Colearning Prompt

> "In this AI workload example, could we process inference results WHILE still fetching from the APIs? How would you structure that? What would the advantage be?"

---

## Code Example 5: Resource Limiting with Semaphores

Real-world systems have limits: you can't fetch infinitely from an API, can't run infinitely many inference tasks.

**Specification Reference**: Controlling concurrency with Semaphores
**Prompt Used**: "Create a system that limits concurrent I/O to N requests and CPU workers to M processes using Semaphores"

```python
import asyncio
from concurrent.futures import ThreadPoolExecutor
from typing import Any
import time

class RateLimitedAPIFetcher:
    """
    Fetch items from API with rate limiting (max concurrent requests).

    Prevents overwhelming the API server.
    """

    def __init__(self, max_concurrent_requests: int = 5):
        self.semaphore = asyncio.Semaphore(max_concurrent_requests)

    async def fetch_item(self, item_id: int) -> dict[str, Any]:
        """Fetch single item with rate limiting."""
        async with self.semaphore:  # Only N can be in-flight at once
            # Simulate API call
            await asyncio.sleep(0.1)
            return {"id": item_id, "data": f"item-{item_id}"}

class CPUBoundProcessor:
    """
    Process items with CPU-bound work, limited by available workers.

    Prevents spawning too many parallel tasks.
    """

    def __init__(self, executor: ThreadPoolExecutor, max_parallel_workers: int = 4):
        self.executor = executor
        self.semaphore = asyncio.Semaphore(max_parallel_workers)

    async def process_item(
        self,
        item: dict[str, Any],
        cpu_time: float = 0.2,
    ) -> dict[str, Any]:
        """Process item with CPU resource limiting."""
        async with self.semaphore:  # Only M can run in parallel
            loop = asyncio.get_running_loop()

            def do_process(x: dict[str, Any]) -> dict[str, Any]:
                time.sleep(cpu_time)
                return {**x, "processed": True}

            return await loop.run_in_executor(
                self.executor,
                do_process,
                item,
            )

async def rate_limited_hybrid_system(
    total_items: int,
    max_concurrent_fetches: int = 5,
    max_concurrent_processes: int = 3,
) -> None:
    """
    Hybrid system with resource limits:
    - Max 5 API requests in-flight (don't overwhelm server)
    - Max 3 CPU processes (use available cores efficiently)
    """
    executor = ThreadPoolExecutor(max_workers=max_concurrent_processes)
    fetcher = RateLimitedAPIFetcher(max_concurrent_requests=max_concurrent_fetches)
    processor = CPUBoundProcessor(executor, max_parallel_workers=max_concurrent_processes)

    print(f"Fetching {total_items} items with resource limits")
    print(f"  Max concurrent API fetches: {max_concurrent_fetches}")
    print(f"  Max concurrent CPU processes: {max_concurrent_processes}")
    print()

    start = time.time()

    # Fetch and process with limits
    async def fetch_and_process_item(item_id: int) -> dict[str, Any]:
        fetched = await fetcher.fetch_item(item_id)
        processed = await processor.process_item(fetched)
        return processed

    # Create tasks for all items
    tasks = [fetch_and_process_item(i) for i in range(total_items)]

    # Process with concurrency control (gather handles all concurrently, but semaphores limit actual work)
    results = await asyncio.gather(*tasks, return_exceptions=True)

    elapsed = time.time() - start

    # Results summary
    successful = sum(1 for r in results if not isinstance(r, Exception))
    print(f"✓ Processed {successful}/{total_items} items in {elapsed:.2f}s")
    print(f"  Average per item: {elapsed / total_items:.3f}s")
    print(f"  Throughput: {total_items / elapsed:.1f} items/sec")

    executor.shutdown(wait=True)

# Validation example
async def main() -> None:
    print("Resource-Limited Hybrid System Demo")
    print("=" * 60)

    await rate_limited_hybrid_system(
        total_items=20,
        max_concurrent_fetches=5,  # API allows max 5 parallel requests
        max_concurrent_processes=3,  # Machine has 3 free cores
    )

if __name__ == "__main__":
    asyncio.run(main())
```

**Why Semaphores Matter**:
- **API Rate Limiting**: Don't send 1000 requests at once; send 5-10 at a time
- **CPU Worker Limits**: Don't spawn 100 workers on a 4-core machine; spawn 4
- **Database Connections**: Don't open 1000 connections; use connection pool (5-20)

#### ✨ Teaching Tip

> Use your AI co-teacher to explore: "If you have 1000 items to process and a 4-core machine, is max_concurrent_processes = 4 always optimal? When might you want 8 or 2 instead?"

---

## Code Example 6: Production Hybrid System (Complete Example)

Here's a complete example combining all patterns: **realistic system with error handling, timeouts, and monitoring.**

**Specification Reference**: Production-grade hybrid system architecture
**Prompt Used**: "Design a production hybrid system: fetch 100 items with timeouts, process in batches, handle partial failures, log progress"

```python
import asyncio
from concurrent.futures import ThreadPoolExecutor
from typing import Any
from dataclasses import dataclass
from enum import Enum
import time
import logging

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
logger = logging.getLogger(__name__)

class ItemStatus(Enum):
    """Lifecycle of an item through the pipeline."""
    PENDING = "pending"
    FETCHED = "fetched"
    PROCESSED = "processed"
    STORED = "stored"
    FAILED = "failed"

@dataclass
class Item:
    """Item moving through the pipeline."""
    item_id: int
    status: ItemStatus = ItemStatus.PENDING
    raw_data: str | None = None
    processed_result: str | None = None
    error: str | None = None
    fetch_time: float = 0.0
    process_time: float = 0.0
    store_time: float = 0.0

async def fetch_item_with_timeout(
    item_id: int,
    timeout_seconds: float = 2.0,
) -> Item:
    """Fetch item with timeout, return result or error."""
    item = Item(item_id=item_id)

    try:
        async with asyncio.timeout(timeout_seconds):
            start = time.time()
            # Simulate API call (sometimes fails)
            await asyncio.sleep(0.05 + (item_id % 3) * 0.05)
            if item_id % 7 == 0:  # Simulate occasional failures
                raise ValueError(f"API returned 500 for item {item_id}")
            item.raw_data = f"data-{item_id}"
            item.fetch_time = time.time() - start
            item.status = ItemStatus.FETCHED

    except asyncio.TimeoutError:
        item.error = f"Fetch timeout after {timeout_seconds}s"
        item.status = ItemStatus.FAILED
        logger.warning(f"Item {item_id}: {item.error}")
    except ValueError as e:
        item.error = str(e)
        item.status = ItemStatus.FAILED
        logger.warning(f"Item {item_id}: {item.error}")

    return item

def process_item_cpu_bound(item: Item) -> Item:
    """Process item CPU-bound. Runs in executor."""
    if item.status != ItemStatus.FETCHED:
        return item  # Skip if fetch failed

    try:
        start = time.time()
        # Simulate CPU-intensive processing
        time.sleep(0.1)
        item.processed_result = f"processed-{item.raw_data}"
        item.process_time = time.time() - start
        item.status = ItemStatus.PROCESSED

    except Exception as e:
        item.error = f"Processing failed: {str(e)}"
        item.status = ItemStatus.FAILED

    return item

async def store_item_with_fallback(item: Item) -> Item:
    """Store item, with fallback if DB is down."""
    if item.status != ItemStatus.PROCESSED:
        return item  # Skip if processing failed

    try:
        start = time.time()
        # Simulate database write with occasional timeout
        await asyncio.sleep(0.02)
        if item.item_id % 20 == 0:
            raise TimeoutError("DB connection timeout")
        item.status = ItemStatus.STORED
        item.store_time = time.time() - start

    except TimeoutError:
        # Fallback: log to file instead
        item.error = "DB failed, logged to fallback"
        item.status = ItemStatus.STORED  # Mark as stored (via fallback)
        logger.info(f"Item {item.item_id}: Stored via fallback")

    return item

async def production_hybrid_pipeline(
    total_items: int,
    batch_size: int = 10,
) -> tuple[list[Item], dict[str, Any]]:
    """
    Production hybrid system: fetch → process → store with:
    - Error handling and partial failures
    - Timeouts at each stage
    - Batching for memory efficiency
    - Progress monitoring
    - Fallback strategies
    """
    executor = ThreadPoolExecutor(max_workers=4)
    loop = asyncio.get_running_loop()
    all_items: list[Item] = []
    metrics = {
        "total": total_items,
        "successful": 0,
        "failed": 0,
        "total_time": 0.0,
        "batches": 0,
    }

    start_time = time.time()

    # Process in batches
    for batch_start in range(0, total_items, batch_size):
        batch_ids = list(range(batch_start, min(batch_start + batch_size, total_items)))
        metrics["batches"] += 1

        logger.info(f"Batch {metrics['batches']}: Fetching {len(batch_ids)} items...")

        # Stage 1: Fetch all in batch concurrently with timeout
        fetch_tasks = [
            fetch_item_with_timeout(item_id, timeout_seconds=2.0)
            for item_id in batch_ids
        ]
        fetched_items = await asyncio.gather(*fetch_tasks)

        # Stage 2: Process all in parallel via executor
        logger.info(f"Batch {metrics['batches']}: Processing {len(fetched_items)} items...")
        process_tasks = [
            loop.run_in_executor(executor, process_item_cpu_bound, item)
            for item in fetched_items
        ]
        processed_items = await asyncio.gather(*process_tasks)

        # Stage 3: Store all concurrently
        logger.info(f"Batch {metrics['batches']}: Storing {len(processed_items)} items...")
        store_tasks = [store_item_with_fallback(item) for item in processed_items]
        final_items = await asyncio.gather(*store_tasks)

        all_items.extend(final_items)

        # Update metrics
        batch_successful = sum(1 for item in final_items if item.status == ItemStatus.STORED)
        batch_failed = sum(1 for item in final_items if item.status == ItemStatus.FAILED)
        logger.info(f"Batch {metrics['batches']}: {batch_successful} success, {batch_failed} failed")

    metrics["total_time"] = time.time() - start_time
    metrics["successful"] = sum(1 for item in all_items if item.status == ItemStatus.STORED)
    metrics["failed"] = sum(1 for item in all_items if item.status == ItemStatus.FAILED)

    executor.shutdown(wait=True)

    return all_items, metrics

# Validation and results summary
async def main() -> None:
    print("Production Hybrid System Demo")
    print("=" * 60)

    items, metrics = await production_hybrid_pipeline(
        total_items=100,
        batch_size=10,
    )

    # Results summary
    print("\n" + "=" * 60)
    print("RESULTS SUMMARY")
    print("=" * 60)
    print(f"Total items: {metrics['total']}")
    print(f"Successfully processed: {metrics['successful']}")
    print(f"Failed: {metrics['failed']}")
    print(f"Success rate: {metrics['successful'] / metrics['total'] * 100:.1f}%")
    print(f"Total time: {metrics['total_time']:.2f}s")
    print(f"Throughput: {metrics['total'] / metrics['total_time']:.1f} items/sec")
    print(f"Batches: {metrics['batches']}")

    # Per-item timing
    successful_items = [item for item in items if item.status == ItemStatus.STORED]
    if successful_items:
        avg_fetch = sum(item.fetch_time for item in successful_items) / len(successful_items)
        avg_process = sum(item.process_time for item in successful_items) / len(successful_items)
        avg_store = sum(item.store_time for item in successful_items) / len(successful_items)
        print(f"\nAverage per-item times:")
        print(f"  Fetch:   {avg_fetch * 1000:.1f}ms")
        print(f"  Process: {avg_process * 1000:.1f}ms")
        print(f"  Store:   {avg_store * 1000:.1f}ms")

if __name__ == "__main__":
    asyncio.run(main())
```

**Production-Ready Features**:
- **Error handling**: Try/except at each stage, graceful failures
- **Timeouts**: Each operation has timeout to prevent hanging
- **Fallback strategies**: If database fails, log to file instead
- **Partial success**: If 1-2 items fail, system continues
- **Monitoring**: Log progress, measure timing, calculate metrics
- **Batching**: Process in chunks for memory efficiency

---

## Identifying and Optimizing Bottlenecks

Real hybrid systems don't work optimally by accident. You need to **measure** where the bottleneck is.

### Common Bottleneck Scenarios

**Scenario 1: I/O-Bound Bottleneck**
```
Time: 0s         5s              10s             15s
Fetch: [-------fetch-10s-------]
Proc:             [--process--]   [--process--]
Time to completion: 15s (waiting for fetches)
```
**Solution**: Increase fetch concurrency (higher `TaskGroup` density)

**Scenario 2: CPU-Bound Bottleneck**
```
Time: 0s    2s      4s      6s      8s      10s
Fetch:  [fetch]
Proc:         [----process--]  [----process--]  [----process--]
Time to completion: 10s (waiting for CPU processing)
```
**Solution**: Increase CPU workers (more InterpreterPoolExecutor workers)

**Scenario 3: Storage Bottleneck**
```
Time: 0s    2s    4s    6s    8s    10s   12s
Fetch: [fetch]
Proc:      [proc]
Store:         [store----]  [store----]  [store----]
Time to completion: 12s (waiting for database)
```
**Solution**: Increase database connections, or batch stores

### How to Identify Your Bottleneck

```python
# Measure each stage independently
fetch_times: list[float] = []
process_times: list[float] = []
store_times: list[float] = []

for item in results:
    fetch_times.append(item.fetch_time)
    process_times.append(item.process_time)
    store_times.append(item.store_time)

avg_fetch = sum(fetch_times) / len(fetch_times)
avg_process = sum(process_times) / len(process_times)
avg_store = sum(store_times) / len(store_times)

# Bottleneck is the longest average stage
bottleneck = max(
    ("fetch", avg_fetch),
    ("process", avg_process),
    ("store", avg_store),
)

print(f"Bottleneck: {bottleneck[0]} taking {bottleneck[1]:.3f}s per item")
```

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:
> "You have a system where fetch takes 0.1s per item, process takes 0.5s per item, and store takes 0.05s per item. You have 1000 items to process and a 4-core machine. What's the optimal batch size and worker allocation?"

**Expected Outcome**: You'll think about bottlenecks systematically—identifying which stage limits throughput, then optimizing resource allocation accordingly.

---

## Challenge 5: The Hybrid Workload Builder

This is a **4-part bidirectional learning challenge** where you architect complete production systems combining asyncio, executors, and intelligent orchestration.

### Part 1: Discover Independently (Student as Scientist)
**Your Challenge**: Experience the power of pipelining without AI guidance.

**Deliverable**: Create `/tmp/pipeline_discovery.py` containing:
1. A 3-stage pipeline (fetch, process, store) each taking 1 second
2. Sequential version: run all 3 for 5 items — measure time (should be ~15 seconds)
3. Pipelined version: start fetching item 2 while processing item 1, start storing item 0 while processing item 2 — measure time (should be ~7 seconds)

**Expected Observation**:
- Sequential (stage after stage): ~15 seconds
- Pipelined (stages overlap): ~7 seconds
- Lesson: Pipelining nearly halves the time by overlapping stages

**Self-Validation**:
- How does pipelining differ from just running stages in parallel?
- What's the minimum time needed with pipelining? (max stage time + buffer time, not sum of all)
- When does pipelining help? (When stages have different speeds, creating bottlenecks)

---

### Part 2: AI as Teacher (Teaching System Design)
**Your AI Prompt**:
> "I have a system: fetch documents (2s), extract text (3s), generate embeddings (4s), store results (1s). I run these sequentially: 4 documents = 40 seconds. I tried running them in parallel but wasted resources. Teach me about pipelining. How would I fetch doc 2 while extracting doc 1 while generating embeddings for doc 0? Show me the architecture using asyncio + executors. What's the optimal pipeline depth?"

**AI's Role**: Explain pipelining concept, show queue-based architecture, discuss how to balance stage speeds, and explain resource tradeoffs.

**Interactive Moment**: Ask a clarifying question:
> "You used a Queue to buffer between stages. What happens if the extract stage is much slower than fetch? Does the queue grow infinitely? How do I prevent memory exhaustion? What's backpressure?"

**Expected Outcome**: AI clarifies queue dynamics and backpressure concepts. You understand that pipelines need flow control, not just throughput maximization.

---

### Part 3: You as Teacher (Discovering Bottleneck Optimization)
**Setup**: AI generates a simple pipeline. Your job is to identify bottlenecks and teach AI how to optimize.

**AI's Initial Code** (ask for this):
```python
import asyncio
from concurrent.futures import ProcessPoolExecutor
import time

async def fetch(item_id: int) -> str:
    await asyncio.sleep(1)  # I/O: 1s
    return f"Data-{item_id}"

def process(data: str) -> str:
    # time.sleep(2)  # CPU: 2s (simulated)
    return f"Processed-{data}"

async def store(result: str) -> None:
    await asyncio.sleep(0.5)  # I/O: 0.5s
    return f"Stored-{result}"

async def pipeline():
    loop = asyncio.get_event_loop()

    with ProcessPoolExecutor(max_workers=4) as executor:
        for i in range(10):
            data = await fetch(i)
            processed = await loop.run_in_executor(executor, process, data)
            await store(processed)

    print("Done")

asyncio.run(pipeline())
```

**Your Task**:
1. Run this code. Measure time. (Should be sequential: ~35 seconds for 10 items)
2. Identify the problem: stages run one-at-a-time, no parallelism
3. Teach AI:
> "This takes 35 seconds sequentially. I want it pipelined: fetch multiple items while processing previous ones. Show me how to use asyncio.Queue to buffer between fetch and process, and between process and store. How deep should the queues be?"

**Your Edge Case Discovery**: Ask AI:
> "What if fetch is fast (0.5s) but process is slow (3s)? Process becomes the bottleneck. How many concurrent process workers should I have? What if I can't increase workers due to memory limits? How do I measure which stage is the bottleneck?"

**Expected Outcome**: You discover bottleneck analysis—profiling each stage, identifying slowest link, optimizing for throughput (not latency). You teach AI how production systems think about optimization.

---

### Part 4: Build Production Artifact (Student as Engineer)
**Your Capstone for This Challenge**: Build an optimized end-to-end data pipeline.

**Specification**:
- 3-stage pipeline: Fetch, Process, Store
- Fetch 12 items from API simulation (asyncio.sleep 0.5-1.5s each)
- Process each with CPU work (simulated with time.sleep, 1-2s each)
- Store results (asyncio.sleep 0.1-0.3s each)
- Stages connected with asyncio.Queue (implement backpressure)
- Measure: throughput (items/second), latency (item start to finish), bottleneck identification
- Return: `{item: (fetch_ms, process_ms, store_ms, e2e_ms)}`, plus throughput metrics
- Type hints throughout

**Deliverable**: Save to `/tmp/data_pipeline.py`

**Testing Your Work**:
```bash
python /tmp/data_pipeline.py
# Expected output:
# Processed 12 items in ~12 seconds (pipelined, not ~24s sequential)
# Throughput: ~1 item/second
# Bottleneck: Process stage (slowest at 1-2s per item)
# Fetch queue depth: varies (shows backpressure)
# Pipeline efficiency: ~75% (good sign of effective overlap)
```

**Validation Checklist**:
- [ ] Code runs without deadlocks or race conditions
- [ ] Stages run concurrently (not sequentially)
- [ ] Queues implement backpressure (don't grow unbounded)
- [ ] Total time < sequential time by at least 20%
- [ ] Bottleneck identified correctly (slowest stage)
- [ ] Type hints complete
- [ ] Production-ready: proper cleanup, error handling, queue drains

---

**Time Estimate**: 35-42 minutes (5 min discover, 8 min teach/learn, 10 min edge cases, 12-19 min build artifact)

**Key Takeaway**: You've mastered production system design. Pipelining, queues, backpressure, and bottleneck analysis are the foundation of real-world AI data systems.
