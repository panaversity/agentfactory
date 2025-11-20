---
sidebar_position: 7
title: "Chapter 29: Asyncio Quiz"
---

# Chapter 29: Asyncio — Concurrent I/O and CPU-Parallel Workloads Quiz

Test your understanding of asynchronous programming with asyncio, concurrent I/O operations, CPU parallelism with InterpreterPoolExecutor, and hybrid workload patterns for modern AI applications.

<Quiz
  title="Chapter 29: Asyncio Assessment"
  questions={[    {
      question: "You're building a weather dashboard that fetches from 10 APIs, each taking 2 seconds. Using asyncio.gather(), how long will the total fetch take compared to sequential execution?",
      options: [
        "Sequential: 20s, Concurrent: ~2s (max delay of all tasks)",
        "Sequential: 20s, Concurrent: 20s (GIL prevents speedup)",
        "Sequential: 20s, Concurrent: ~10s (half the time)",
        "Sequential: 20s, Concurrent: varies based on CPU cores"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. With asyncio.gather(), all 10 API calls run concurrently (overlapping in time). The total time is approximately the longest single operation (2s), not the sum (20s). This is because I/O-bound tasks can pause with await, allowing the event loop to switch between them. Sequential execution would wait for each call to complete before starting the next (2+2+2...=20s). The GIL doesn't prevent concurrency for I/O-bound work (option 2 is wrong). Option 3's 'half the time' has no basis. Option 4 is incorrect because I/O concurrency doesn't depend on CPU cores. This demonstrates the fundamental asyncio value proposition from Lesson 1: Asyncio Foundations.",
      source: "Lesson 1: Asyncio Foundations — Event Loop and Coroutines"
    },
    {
      question: "What happens when you call an async function without using await or asyncio.run()?",
      options: [
        "Executes immediately and returns the result",
        "Returns a coroutine object but doesn't execute the function",
        "Raises SyntaxError because async requires await",
        "Automatically schedules on default event loop"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 1. Calling an async function without await returns a coroutine object—a promise of future execution—but doesn't actually run the function. You'll see output like '<coroutine object func_name at 0x...>' and get a RuntimeWarning: 'coroutine was never awaited.' To execute, you must either await it (inside another async function) or use asyncio.run() (from sync context). Option 2 is wrong—async functions never execute immediately without await. Option 3 is wrong—it's a runtime warning, not a syntax error. Option 4 is wrong—there's no 'default event loop' that auto-schedules. This is a common beginner mistake covered in Lesson 1: Asyncio Foundations.",
      source: "Lesson 1: Asyncio Foundations — Event Loop and Coroutines"
    },
    {
      question: "Your AI agent processes text with heavy NLP analysis (CPU-intensive). You wrap it in async def and use asyncio.gather() to run 4 analyses concurrently. What performance do you expect?",
      options: [
        "2x speedup due to partial parallelism",
        "4x speedup because 4 tasks run in parallel",
        "No speedup; CPU work doesn't benefit from asyncio",
        "Performance worse due to event loop overhead"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 1. Asyncio provides concurrency (task switching) for I/O-bound work, not parallelism for CPU-bound work. CPU-intensive NLP analysis has no I/O wait points where await can yield control. The GIL (Global Interpreter Lock) prevents multiple threads from executing Python bytecode simultaneously, so wrapping CPU work in async def doesn't help. All 4 tasks still run sequentially on one CPU core. Option 4 is partially true (overhead exists) but the key issue is no parallelism. Option 2 is wrong—asyncio doesn't parallelize CPU work. Option 3 is wrong for the same reason. To parallelize CPU work, use InterpreterPoolExecutor (Lesson 4) or ProcessPoolExecutor. This fundamental limitation is explained in Lesson 1: I/O-Bound vs CPU-Bound Tasks.",
      source: "Lesson 1: Asyncio Foundations — I/O vs CPU Task Recognition"
    },
    {
      question: "What is the key difference between concurrency (asyncio) and parallelism (InterpreterPoolExecutor)?",
      options: [
        "Concurrency uses threads; Parallelism uses processes",
        "Concurrency requires async def; Parallelism works with def",
        "Concurrency is for I/O; Parallelism is for CPU (both are correct but secondary)",
        "Concurrency: tasks switch on one core; Parallelism: tasks run on multiple cores simultaneously"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 1. Concurrency means multiple tasks make progress by switching execution on one CPU core (cooperative multitasking). When task A hits await, task B runs. Parallelism means multiple tasks literally run at the same time on different CPU cores. Asyncio provides concurrency (event loop switches tasks). InterpreterPoolExecutor provides parallelism (separate interpreters on separate cores). Option 2 is a syntax distinction, not the conceptual difference. Option 3 is true (I/O vs CPU) but describes the use case, not the core distinction. Option 4 is wrong—asyncio doesn't use threads (it uses one thread with an event loop), and InterpreterPoolExecutor uses separate interpreters (not processes). This critical distinction is taught in Lesson 1: Concurrency vs Parallelism.",
      source: "Lesson 1: Asyncio Foundations — Concurrency vs Parallelism"
    },
    {
      question: "You need to fetch user data from 5 services concurrently. What's the correct pattern using asyncio.create_task()?",
      options: [
        "Schedule all tasks first, then await results",
        "Await each task immediately after creating it",
        "Create tasks inside a for loop with await",
        "Use create_task() but don't await the results"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. The pattern is: create all tasks (schedule them), THEN await results. This allows all tasks to start running concurrently. Example: 'tasks = [asyncio.create_task(fetch(i)) for i in range(5)]' (all start immediately), then 'results = [await t for t in tasks]' (collect results). Option 2 defeats concurrency—awaiting immediately blocks, so tasks run sequentially. Option 3 has the same problem (await inside loop = sequential). Option 4 creates tasks but never gets results (memory leak, RuntimeWarning). The key insight: create_task() schedules immediately, await collects later. This pattern is demonstrated in Lesson 2: Understanding Task Scheduling with create_task().",
      source: "Lesson 2: Concurrent Tasks — create_task() Pattern"
    },
    {
      question: "When should you use asyncio.gather() instead of asyncio.create_task() for multiple concurrent operations?",
      options: [
        "When you need fine-grained control over task cancellation",
        "When collecting all results at once and you don't need individual task references",
        "When tasks have different error handling requirements",
        "When you need to inspect task state during execution"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 1. asyncio.gather() is a convenience wrapper for 'schedule multiple coroutines and collect all results.' It's concise: 'results = await asyncio.gather(fetch(1), fetch(2), fetch(3))' vs creating and awaiting individual tasks. Use gather() when you want all results together and don't need task references. Use create_task() when you need: fine-grained control (cancel specific tasks), task inspection (task.done(), task.result()), or different handling per task. Options 2-4 describe scenarios where create_task() is BETTER (individual task references needed). Gather is for simplicity; create_task is for control. This tradeoff is explained in Lesson 2: Collecting Results with asyncio.gather().",
      source: "Lesson 2: Concurrent Tasks — gather() vs create_task() Comparison"
    },
    {
      question: "What is the key advantage of asyncio.TaskGroup() (Python 3.11+) over asyncio.gather()?",
      options: [
        "TaskGroup() allows collecting results from exceptions",
        "TaskGroup() runs faster than gather() for many tasks",
        "Fail-fast behavior: if one task fails, all others are automatically cancelled",
        "TaskGroup() works with synchronous functions, gather() requires async"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 1. TaskGroup implements structured concurrency with fail-fast semantics. If any task in the group raises an exception, TaskGroup immediately cancels all other tasks and raises ExceptionGroup. This is all-or-nothing execution. In contrast, gather(return_exceptions=True) continues running other tasks even when one fails (best-effort). Option 2 is wrong—performance is similar. Option 3 is backwards—gather() can collect exceptions with return_exceptions=True. Option 4 is wrong—both require async functions. Use TaskGroup when atomicity matters (all succeed or all fail). Use gather() when partial success is acceptable (resilient systems). This pattern is taught in Lesson 2: TaskGroup: Modern Structured Concurrency.",
      source: "Lesson 2: Concurrent Tasks — TaskGroup() Structured Concurrency"
    },
    {
      question: "You're using asyncio.gather(return_exceptions=True) to fetch from 10 APIs. Three APIs fail with ConnectionError. What do you receive?",
      options: [
        "First ConnectionError raised immediately, stopping other tasks",
        "ExceptionGroup containing all 3 failures",
        "Only the 7 successful results (failed ones omitted)",
        "A list of 10 items: 7 successful results and 3 ConnectionError exceptions"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 1. With gather(return_exceptions=True), the result is a list matching input order: successful results as values, failed tasks as Exception objects. You get: [result, result, ConnectionError(...), result, ...] with 10 total items. You must manually check 'isinstance(item, Exception)' to identify failures. Option 2 is wrong—ExceptionGroup is from TaskGroup, not gather. Option 3 is wrong—failed tasks remain in the list as exceptions, not omitted. Option 4 describes default gather() behavior (return_exceptions=False), not the True case. This resilience pattern is key for production systems where partial success is acceptable (backup services, multi-source aggregation). Covered in Lesson 2: Error Handling with gather(return_exceptions=True).",
      source: "Lesson 2: Concurrent Tasks — gather() Error Handling"
    },
    {
      question: "Your async API client occasionally hangs waiting for responses. You add asyncio.timeout(5) context manager. What exception do you catch when timeout occurs?",
      options: [
        "asyncio.CancelledError",
        "asyncio.TimeoutError",
        "TimeoutError (built-in)",
        "asyncio.TimeoutException"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 1. The asyncio.timeout() context manager (Python 3.11+) raises asyncio.TimeoutError when the time limit is exceeded. Pattern: 'async with asyncio.timeout(5): await slow_operation()' raises TimeoutError if it takes >5s. This is different from asyncio.CancelledError (option 2), which occurs when a task is explicitly cancelled (task.cancel() or TaskGroup cancellation). Option 3 (built-in TimeoutError) exists but asyncio uses asyncio.TimeoutError specifically. Option 4 doesn't exist. This defensive pattern is critical for production systems to prevent indefinite hangs. Always catch and handle timeouts gracefully (retry, fallback, or fail explicitly). Taught in Lesson 3: Timeout Control with asyncio.timeout().",
      source: "Lesson 3: Advanced Patterns — Timeout Control"
    },
    {
      question: "What's the difference between asyncio.TimeoutError (from timeout context) and asyncio.CancelledError (from task cancellation)?",
      options: [
        "TimeoutError: operation exceeded time limit; CancelledError: task cancelled externally",
        "Both are identical; asyncio uses them interchangeably",
        "TimeoutError for I/O operations; CancelledError for CPU operations",
        "TimeoutError is recoverable; CancelledError always crashes the program"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. TimeoutError indicates the operation took too long (you set a timeout and it expired). CancelledError indicates external cancellation (another task called task.cancel(), or TaskGroup cancelled due to another task's failure). They require different handling: TimeoutError might warrant retry (transient network slowness); CancelledError usually means cleanup time (system is shutting down or coordinated cancellation). Option 2 is wrong—they're distinct exceptions. Option 3 is wrong—both apply to any async operation. Option 4 is wrong—both are recoverable (you can catch them). Understanding this distinction helps design resilient systems that respond appropriately to different failure modes. Explained in Lesson 3: Exception Handling in Async Code.",
      source: "Lesson 3: Advanced Patterns — TimeoutError vs CancelledError"
    },
    {
      question: "You wrote async def fetch(): result = asyncio.sleep(1) and see RuntimeWarning: coroutine 'sleep' was never awaited. What's wrong?",
      options: [
        "async functions can't call sleep without yield",
        "Should use time.sleep(1) instead of asyncio.sleep(1)",
        "Forgot to import asyncio module",
        "Missing await keyword before asyncio.sleep(1)"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 1. asyncio.sleep() is a coroutine (async function). You must await it: 'await asyncio.sleep(1)'. Without await, you create a coroutine object but never execute it—it never pauses the function. The corrected code: 'async def fetch(): result = await asyncio.sleep(1)'. Option 2 is dangerous—time.sleep() blocks the entire event loop (no other tasks run). Option 3 is wrong—you'd get ImportError, not RuntimeWarning. Option 4 is nonsense (yield isn't relevant here). This is the most common async beginner mistake. The RuntimeWarning tells you: 'You created an awaitable but didn't await it.' Always await coroutines! Covered in Lesson 3: Common Mistakes — Never-Awaited Coroutines.",
      source: "Lesson 3: Advanced Patterns — Common Async Pitfalls"
    },
    {
      question: "Why is using time.sleep(2) inside an async function a problem?",
      options: [
        "time.sleep() is slower than asyncio.sleep()",
        "time.sleep() raises SyntaxError in async functions",
        "Blocks the event loop; no other tasks can run during sleep",
        "time.sleep() only works with threading, not asyncio"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 1. time.sleep() is a blocking call—it freezes the entire thread, including the event loop. During time.sleep(2), NO other async tasks can run (they're all blocked). This defeats asyncio's concurrency. Use await asyncio.sleep(2) instead—it yields control to the event loop, allowing other tasks to run. Option 2 is wrong—no syntax error, just bad practice. Option 3 is wrong—speed is similar, the issue is blocking. Option 4 is wrong—time.sleep() works anywhere but blocks. This mistake is subtle because the code 'works' (doesn't crash) but kills performance. A system with 100 concurrent tasks using time.sleep becomes sequential. Always use await asyncio.sleep() in async code! Explained in Lesson 3: Common Mistake: Blocking the Event Loop.",
      source: "Lesson 3: Advanced Patterns — Blocking the Event Loop"
    },
    {
      question: "Your async API client retries failed requests. What pattern implements exponential backoff correctly?",
      options: [
        "Delay doubles after each retry: 1s, 2s, 4s, 8s (with max cap)",
        "Delay decreases after each retry: 4s, 2s, 1s, 0.5s",
        "Delay stays constant: 2s between each retry",
        "Delay increases linearly: 1s, 2s, 3s, 4s"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. Exponential backoff means delay DOUBLES after each failure: retry 1 waits 1s, retry 2 waits 2s, retry 3 waits 4s, etc. This gives failing services increasing time to recover while avoiding 'thundering herd' (all clients retrying simultaneously). Best practice: add jitter (random variance) and max cap (prevent infinite growth to hours). Pattern: 'delay = min(initial_delay * (2 ** attempt), max_delay)'. Option 2 is backwards (decreasing makes no sense—give more time, not less). Option 3 (constant delay) is better than nothing but doesn't give escalating recovery time. Option 4 (linear) is less effective than exponential for transient failures. Exponential backoff is the industry standard for resilient systems. Taught in Lesson 3: Retry Logic with Exponential Backoff.",
      source: "Lesson 3: Advanced Patterns — Resilience Patterns"
    },
    {
      question: "What is a Future in asyncio, and when do you manually create one?",
      options: [
        "Replacement for async def functions in Python 3.10+",
        "Awaitable placeholder for a result; rarely created manually (asyncio creates them)",
        "Way to schedule CPU-bound work in async code",
        "Context manager for timeout control"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 1. A Future is an awaitable object representing a result that isn't available yet—a promise. You await it, and when the value arrives, the await completes. In practice, asyncio creates Futures internally: create_task() returns Task (Future subclass), gather() returns Future, executors return Futures. You rarely create Futures manually (asyncio.Future()). Option 2 is wrong—Futures don't replace async def. Option 3 is wrong—run_in_executor() returns a Future, but you don't create it yourself. Option 4 describes asyncio.timeout(), not Futures. Understanding Futures helps debug ('why is task.result() raising InvalidStateError?') but you primarily interact with them indirectly. Covered in Lesson 3: Understanding Futures.",
      source: "Lesson 3: Advanced Patterns — Futures Conceptualization"
    },
    {
      question: "Python's Global Interpreter Lock (GIL) prevents true parallelism for which type of work?",
      options: [
        "Only work using threading; asyncio bypasses GIL",
        "I/O-bound work (network calls, file reads)",
        "Both I/O and CPU work equally",
        "CPU-bound work (calculations, data processing)"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 1. The GIL allows only one thread to execute Python bytecode at a time. For CPU-bound work (math, parsing, analysis), all threads are always ready to run, so the GIL becomes a bottleneck—only one runs while others wait. For I/O-bound work, threads naturally wait for I/O (network, disk), so the GIL matters less (one thread waits while another runs). Option 2 is wrong—I/O work can still benefit from threading/asyncio (one waits, another runs). Option 3 is wrong—impact is greater for CPU work. Option 4 is wrong—asyncio uses the same GIL, just one thread with an event loop. To escape GIL for CPU work: use InterpreterPoolExecutor (separate interpreters = separate GILs) or ProcessPoolExecutor. Explained in Lesson 4: What Is the GIL?",
      source: "Lesson 4: CPU-Bound Work — GIL Limitation"
    },
    {
      question: "You benchmark CPU-intensive work with ThreadPoolExecutor (4 workers) and sequential execution. What do you expect?",
      options: [
        "Threading 2x faster due to partial parallelism",
        "Threading 4x faster because 4 threads run in parallel",
        "Threading slower than sequential due to GIL contention and context switching",
        "Both perform identically (same time)"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 1. For CPU-bound work, ThreadPoolExecutor is SLOWER than sequential execution. Why? The GIL forces 4 threads to compete for access to the single interpreter. Only one runs at a time, but now you pay context-switching overhead (OS switching between threads). Benchmark shows: Sequential 4.5s, Threading 6.1s (slower!). Option 2 is the intuitive expectation, but wrong due to GIL. Option 3 is wrong for the same reason. Option 4 is wrong—threading adds overhead. This counterintuitive result demonstrates why understanding GIL matters. For CPU parallelism, use InterpreterPoolExecutor (Python 3.14+) which achieves ~4x speedup on 4 cores. Demonstrated in Lesson 4: Threading Benchmark (Shows the Problem).",
      source: "Lesson 4: CPU-Bound Work — Why Threading Fails"
    },
    {
      question: "What is the key advantage of InterpreterPoolExecutor over ThreadPoolExecutor for CPU-bound work?",
      options: [
        "Separate interpreters with separate GILs enable true parallelism on multiple cores",
        "InterpreterPoolExecutor uses less memory than ThreadPoolExecutor",
        "InterpreterPoolExecutor is faster for I/O-bound work",
        "InterpreterPoolExecutor works on older Python versions (3.7+)"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. InterpreterPoolExecutor (Python 3.14+) creates a pool of independent Python interpreters, each with its own GIL. This means each worker can run CPU-bound code simultaneously on different cores—true parallelism. ThreadPoolExecutor uses one interpreter shared by multiple threads, so the GIL serializes execution. Benchmarks show: Sequential 4.5s, Threading 6.1s, InterpreterPoolExecutor 1.15s (~4x speedup on 4 cores). Option 2 is wrong—memory overhead is similar. Option 3 is wrong—both work for I/O, but asyncio is better. Option 4 is backwards—InterpreterPoolExecutor requires Python 3.14+. This is the solution to Python's long-standing CPU parallelism problem. Explained in Lesson 4: InterpreterPoolExecutor: The Solution.",
      source: "Lesson 4: CPU-Bound Work — InterpreterPoolExecutor"
    },
    {
      question: "How do you run a synchronous CPU-bound function inside an async program without blocking the event loop?",
      options: [
        "Wrap the function with async def and use await",
        "Use await loop.run_in_executor(executor, sync_function, args)",
        "Call the function directly; asyncio handles it automatically",
        "Use asyncio.create_task(sync_function())"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 1. loop.run_in_executor() bridges sync functions into async code. Pattern: 'loop = asyncio.get_running_loop(); result = await loop.run_in_executor(executor, cpu_function, data)'. The executor (InterpreterPoolExecutor or ThreadPoolExecutor) runs the sync function in a separate context, returning a Future that you await. The event loop remains responsive. Option 2 is wrong—wrapping def as async def doesn't make it non-blocking (still blocks the loop). Option 3 is wrong—calling sync functions directly blocks. Option 4 is wrong—create_task() requires async functions (coroutines), not sync functions. This pattern is critical for hybrid workloads (I/O + CPU). You fetch data with asyncio, process with CPU-bound work via executor. Taught in Lesson 4: Bridging CPU Work into Async Code.",
      source: "Lesson 4: CPU-Bound Work — Async Executor Integration"
    },
    {
      question: "What is the key tradeoff between InterpreterPoolExecutor and ProcessPoolExecutor for CPU-bound work?",
      options: [
        "InterpreterPoolExecutor: for I/O; ProcessPoolExecutor: for CPU",
        "InterpreterPoolExecutor: slower; ProcessPoolExecutor: faster",
        "InterpreterPoolExecutor: lightweight, shared memory; ProcessPoolExecutor: heavyweight, isolated memory",
        "InterpreterPoolExecutor: requires Python 3.7+; ProcessPoolExecutor: requires 3.14+"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 1. InterpreterPoolExecutor (Python 3.14+) uses separate interpreters (lightweight, fast startup, shared memory namespace). ProcessPoolExecutor uses separate processes (heavyweight, slow startup, isolated memory requiring pickle serialization). For typical CPU work with Python objects, InterpreterPoolExecutor is faster and more efficient. For long-running isolated tasks or work requiring full process isolation, ProcessPoolExecutor might be preferred. Option 2 is wrong—both achieve similar CPU parallelism. Option 3 is wrong—both are for CPU work (asyncio handles I/O). Option 4 is backwards—ProcessPoolExecutor is older, InterpreterPoolExecutor is new (3.14+). The choice depends on memory sharing needs and overhead tolerance. Covered in Lesson 4: ProcessPoolExecutor: An Alternative.",
      source: "Lesson 4: CPU-Bound Work — Executor Comparison"
    },
    {
      question: "Your AI system fetches from 100 APIs and processes each response with CPU-intensive analysis. What's the optimal architecture?",
      options: [
        "Use only InterpreterPoolExecutor for both fetch and process",
        "Fetch all 100 concurrently, then process all sequentially",
        "Process each item as it arrives (no batching)",
        "Batch fetch 10 concurrently (asyncio), process batch in parallel (InterpreterPoolExecutor), repeat"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 1. This is a hybrid workload (I/O + CPU) requiring both asyncio (concurrent fetch) and InterpreterPoolExecutor (parallel processing). Batching prevents memory exhaustion (not all 100 in memory) and rate limit violations (controlled concurrent requests). Pattern: fetch batch → process batch in parallel → store → fetch next batch. Option 2 is problematic (100 items in memory, long wait before processing starts). Option 3 is inefficient (no pipelining—fetch #2 waits for process #1 to finish). Option 4 is wrong—InterpreterPoolExecutor doesn't help I/O (use asyncio for that). Batch size is tunable: small batches = low memory, high overhead; large batches = high memory, low overhead. This pattern is fundamental to production AI systems. Taught in Lesson 5: Batch Processing Pattern.",
      source: "Lesson 5: Hybrid Workloads — Batch Processing"
    },
    {
      question: "What is the primary benefit of pipeline patterns (fetch → transform → store) over batch processing?",
      options: [
        "Pipelines use less memory than batching",
        "Overlapping execution: while fetching batch 2, processing batch 1 and storing batch 0",
        "Pipelines automatically scale to unlimited workers",
        "Pipelines work with synchronous code; batching requires async"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 1. Pipeline patterns overlap stages: while stage 1 fetches new data, stage 2 processes previous data, stage 3 stores earlier results—all happening concurrently. This maximizes throughput by keeping all resources busy (network, CPU, database). Batch processing fetches all, then processes all, then stores all (sequential stages). Pipelines achieve higher throughput at the cost of complexity (managing queues between stages). Option 2 is wrong—memory usage depends on queue depth, not the pattern choice. Option 3 is wrong—pipelines require manual worker tuning. Option 4 is wrong—both patterns work with async code. Pipelines are the optimal pattern for high-throughput systems where latency of individual items matters less than total system throughput. Explained in Lesson 5: Pipeline Pattern.",
      source: "Lesson 5: Hybrid Workloads — Pipeline Patterns"
    },
    {
      question: "You're using asyncio.Semaphore(10) to limit concurrent API calls. What does this protect against?",
      options: [
        "Overwhelming the API server with too many simultaneous requests (rate limiting)",
        "Running out of CPU cores for processing",
        "Python's GIL contention",
        "Memory leaks in async code"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. asyncio.Semaphore(10) limits concurrency to 10 simultaneous operations. Pattern: 'async with semaphore: await fetch_api()' ensures max 10 requests in flight at once. This prevents overwhelming servers (respecting rate limits), exhausting connection pools, or hitting OS resource limits (file descriptors). When 10 tasks are running, additional tasks wait at the semaphore until one completes. Option 2 is wrong—Semaphores don't manage CPU cores (use executor max_workers for that). Option 3 is wrong—Semaphores don't affect GIL. Option 4 is wrong—Semaphores are for resource limiting, not memory leak prevention. This pattern is essential for production systems: API providers rate-limit, and you must respect those limits or face bans. Covered in Lesson 5: Resource Management with Semaphores.",
      source: "Lesson 5: Hybrid Workloads — Resource Limiting"
    },
    {
      question: "When building AI agents that call multiple LLM APIs concurrently, what resource constraint typically becomes the bottleneck first?",
      options: [
        "Memory for storing intermediate results",
        "CPU cores for processing responses",
        "API rate limits (requests per second) from LLM providers",
        "Network bandwidth for request/response data"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 1. LLM API providers (OpenAI, Anthropic, etc.) enforce strict rate limits (e.g., 60 requests/minute, 10 concurrent). If you exceed these, requests fail with 429 errors. Before hitting CPU, memory, or bandwidth limits, you'll hit API rate limits. Solution: use Semaphore to limit concurrency (e.g., max 5 concurrent LLM calls), implement retry with exponential backoff for 429 errors, and batch requests where possible. Option 2 is rarely the bottleneck (LLM processing happens server-side). Option 3 can be an issue for large context windows but is secondary to rate limits. Option 4 is almost never the bottleneck (request/response data is typically <1MB). Understanding API constraints is critical for production AI systems. Explained in Lesson 5: AI-Native Workloads.",
      source: "Lesson 5: Hybrid Workloads — AI Workload Patterns"
    },
    {
      question: "Your async system processes 1,000 items. Fetching is fast (0.1s each), processing is slow (2s each). Where's the bottleneck, and how do you optimize?",
      options: [
        "Bottleneck: event loop; Solution: use multiple event loops",
        "Bottleneck: I/O fetching; Solution: increase concurrent fetch count",
        "Bottleneck: memory; Solution: reduce batch size",
        "Bottleneck: CPU processing; Solution: increase InterpreterPoolExecutor workers"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 1. Processing takes 20x longer than fetching (2s vs 0.1s), making CPU the bottleneck. Even with concurrent fetching, you'll spend most time waiting for processing. Solution: increase InterpreterPoolExecutor workers (e.g., from 4 to 8 if you have 8 cores) to parallelize processing. With 8 workers, processing time drops from 2s to 0.25s per batch of 8 items. Option 2 is wrong—fetch is already fast. Option 3 is wrong—memory isn't the constraint described. Option 4 is nonsense—you don't use multiple event loops in one program. This analysis (identify slowest stage, optimize that) is fundamental to performance tuning. Measure each stage, find the bottleneck, scale that resource. Covered in Lesson 5: Performance Bottleneck Analysis.",
      source: "Lesson 5: Hybrid Workloads — Bottleneck Analysis"
    },
    {
      question: "What pattern best describes AI agent systems that fetch from multiple services (weather, news, stock) and combine results?",
      options: [
        "Fan-out (concurrent fetch all sources) then fan-in (combine results)",
        "Sequential fetch and combine",
        "Pipeline (fetch → transform → fetch → transform)",
        "Round-robin (fetch one, fetch another, alternate)"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. Fan-out/fan-in is the standard pattern for multi-service aggregation. Fan-out: launch concurrent tasks for all services simultaneously ('tasks = [fetch_weather(), fetch_news(), fetch_stock()]'). Fan-in: wait for all to complete and combine ('results = await asyncio.gather(*tasks); combined = merge(results)'). This maximizes parallelism (all services queried at once) and minimizes latency (total time = slowest service, not sum of all). Option 2 wastes time (sequential). Option 3 doesn't match the scenario (no multi-stage transformation). Option 4 doesn't make sense (why alternate?). Fan-out/fan-in is ubiquitous in distributed systems and AI agents (query multiple knowledge sources, combine into unified response). Taught in Lesson 5: AI Workload Patterns.",
      source: "Lesson 5: Hybrid Workloads — Multi-Service Patterns"
    },
    {
      question: "You're designing a text processing pipeline: fetch 1000 documents → analyze with NLP → store results. What's the optimal stage-to-stage communication mechanism?",
      options: [
        "Global list that all stages append to",
        "asyncio.Queue for each stage transition (fetch→queue→process→queue→store)",
        "Database table as intermediate storage",
        "Direct function calls between stages"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 1. asyncio.Queue provides async-safe, ordered, backpressure-aware communication between pipeline stages. Pattern: fetch_stage puts items into queue1, process_stage gets from queue1 and puts into queue2, store_stage gets from queue2. Queues naturally handle different stage speeds (if processing is slow, queue1 fills, fetch_stage waits—backpressure). Option 2 is problematic (no backpressure, memory grows unbounded, race conditions). Option 3 adds latency and I/O overhead (database reads/writes for every item). Option 4 creates tight coupling and doesn't allow overlap (stage 2 waits for stage 1 to finish). Queues are the standard pattern for producer-consumer pipelines in async systems. Explained in Lesson 5: Pipeline Pattern Implementation.",
      source: "Lesson 5: Hybrid Workloads — Pipeline Communication"
    },
    {
      question: "When describing a concurrent API aggregator to an AI, which prompt produces the most useful code?",
      options: [
        "Implement async functions for API calls",
        "Use asyncio.gather to call APIs",
        "Make the API fetcher fast and concurrent",
        "Fetch from 5 APIs concurrently with 2s timeout per API, handle partial failures, return available results"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 1. This prompt specifies: WHAT (5 APIs), HOW (concurrent), CONSTRAINTS (2s timeout each), ERROR HANDLING (partial failures OK), and OUTCOME (return available results). AI can generate: asyncio.gather(return_exceptions=True), asyncio.timeout(2) context, result filtering. Option 2 is too vague (gather is the tool, not the full requirement). Option 3 uses vague terms ('fast', 'concurrent') without specifics. Option 4 describes syntax, not requirements. Good AI prompts answer: what data, what operations, what constraints, what error cases, what success looks like. Describe intent and constraints, not implementation syntax. This principle applies to all AI-assisted development: clear requirements yield better code. This is an AI workflow question testing Lesson 2-3 concepts.",
      source: "Lessons 2-3: AI Collaboration for Async Patterns"
    },
    {
      question: "After generating async code with AI, it uses time.sleep() instead of await asyncio.sleep(). What should you request?",
      options: [
        "Change import to 'from asyncio import sleep as time.sleep'",
        "Add async keyword before time.sleep()",
        "Replace time.sleep() with await asyncio.sleep() to avoid blocking the event loop",
        "Use threading.sleep() instead"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 1. The request should be: 'Replace time.sleep() with await asyncio.sleep() because time.sleep() blocks the event loop, preventing other tasks from running.' This teaches AI the specific async pattern and explains why. Option 2 doesn't fix anything (time.sleep is still sync/blocking). Option 3 is a hacky workaround that doesn't address the real issue. Option 4 makes it worse (threading.sleep has the same blocking problem). Good AI collaboration involves: identifying the issue (blocking call), explaining why it's wrong (event loop blocked), specifying the correct pattern (await asyncio.sleep). This iterative refinement is how AI learns your requirements. This tests AI workflow understanding from Lesson 3: Common Async Pitfalls.",
      source: "Lesson 3: AI Collaboration — Correcting Async Mistakes"
    },
    {
      question: "You ask AI to implement retry logic. It generates a while loop with immediate retry (no delay). What's missing, and how do you prompt for it?",
      options: [
        "Just add 'time.sleep(1)' before each retry",
        "Request exponential backoff: 'Add delay between retries that doubles each attempt (1s, 2s, 4s) with max 32s cap'",
        "Ask AI to make the retry loop faster",
        "Request circuit breaker pattern instead"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 1. The missing piece is exponential backoff (increasing delay between retries). Without delay, you hammer the failing service, worsening the problem (thundering herd). The prompt specifies: PATTERN (exponential backoff), BEHAVIOR (doubles each attempt), SPECIFIC VALUES (1s→2s→4s), and CONSTRAINT (max cap 32s). AI can implement: 'delay = min(initial * (2**attempt), max_cap); await asyncio.sleep(delay)'. Option 2 adds delay but it's constant (not exponential). Option 3 is counterproductive (retries should slow down, not speed up). Option 4 is a different pattern (circuit breaker stops retrying after threshold; exponential backoff spaces out retries). This demonstrates teaching AI the correct resilience pattern with specific implementation details. From Lesson 3: Retry Logic with Exponential Backoff.",
      source: "Lesson 3: AI Collaboration — Resilience Patterns"
    },
    {
      question: "When reviewing AI-generated async code, what's the FIRST thing to check for correctness?",
      options: [
        "All coroutines are awaited (no 'coroutine was never awaited' warnings)",
        "Type hints are complete on all functions",
        "Variable names follow PEP 8 conventions",
        "Docstrings explain every function"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. The most critical async correctness issue is unawaited coroutines—code that compiles but doesn't execute. Check: every async function call has 'await', asyncio.create_task(), or asyncio.gather(). Run the code and look for RuntimeWarning about never-awaited coroutines. This is a functional error (code doesn't work). Options 2-4 are style/documentation issues (important but secondary to correctness). The review priority: 1) Correctness (does it work?), 2) Performance (blocking calls, proper concurrency), 3) Error handling (timeouts, exceptions), 4) Style/docs. When teaching AI to generate async code, emphasize: 'Always await coroutines.' This is the #1 async bug. From Lesson 3: Common Async Pitfalls.",
      source: "Lesson 3: AI Collaboration — Code Review Priorities"
    },
    {
      question: "You describe a hybrid workload to AI: 'Fetch 100 items concurrently, process each with CPU work.' AI generates asyncio.gather() for both fetch AND process. What's wrong?",
      options: [
        "Needs more workers for concurrency",
        "Should use TaskGroup instead of gather",
        "CPU work won't parallelize with asyncio; need InterpreterPoolExecutor for processing",
        "Should fetch sequentially, process concurrently"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 1. The issue: AI used asyncio (concurrency) for CPU work, which doesn't provide parallelism due to GIL. Fetch should use asyncio.gather() (I/O concurrency). Process should use InterpreterPoolExecutor with loop.run_in_executor() (CPU parallelism). The corrected prompt: 'Fetch 100 items concurrently with asyncio.gather(). For each item, run CPU processing using InterpreterPoolExecutor with 4 workers. Use loop.run_in_executor() to bridge the sync CPU function into async code.' This teaches AI the I/O vs CPU distinction. Option 2 doesn't solve the CPU parallelism issue. Option 3 doesn't address the pattern mismatch. Option 4 is backwards (fetch should be concurrent). This tests understanding of hybrid patterns from Lesson 5.",
      source: "Lesson 5: AI Collaboration — Hybrid Workload Patterns"
    },
    {
      question: "What's the correct way to combine asyncio.timeout() with asyncio.gather() for multiple concurrent operations?",
      options: [
        "Use asyncio.wait_for(gather(...), timeout=5)",
        "Wrap each task in separate timeout contexts before gather",
        "Pass timeout as parameter to gather: asyncio.gather(..., timeout=5)",
        "Wrap gather in timeout context: async with asyncio.timeout(5): await asyncio.gather(...)"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 1. To timeout the entire gather operation, wrap it in asyncio.timeout() context manager: 'async with asyncio.timeout(5): results = await asyncio.gather(...)'. If the group takes >5s total, TimeoutError is raised. Option 2 creates per-task timeouts (different use case—each task has its own limit). Option 3 is wrong—gather() doesn't have a timeout parameter. Option 4 works but asyncio.timeout() is the modern pattern (Python 3.11+). Note: global timeout (option 1) vs per-task timeout (option 2) are different strategies. Global: 'all tasks together must finish in 5s'. Per-task: 'each task must finish in 5s individually'. Choose based on requirements. This combines Lesson 3 (timeouts) with Lesson 2 (gather). Explained in Lesson 3: Timeout Control.",
      source: "Lesson 3: Advanced Patterns — Timeout with gather()"
    },
    {
      question: "When would you use TaskGroup over gather() for fetching from multiple unreliable APIs?",
      options: [
        "When one API failure should stop all others (fail-fast, transactional behavior)",
        "When you want partial results even if some APIs fail",
        "When APIs return different data types",
        "When you need the fastest possible performance"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. TaskGroup implements fail-fast: if one task fails, all others are immediately cancelled. Use this when operations are transactional (all must succeed or none matter). Example: fetching required configuration from multiple sources—if one fails, the config is incomplete, so stop everything. gather(return_exceptions=True) is for partial success (option 2)—fetch from 5 backup servers, use whatever succeeds. Option 3 doesn't determine the choice (both handle different types). Option 4 is wrong—performance is similar. The decision: Do you need all results (TaskGroup) or best-effort (gather)? Are failures atomic (TaskGroup) or independent (gather)? This is an architectural decision, not a syntax choice. From Lesson 2: Choosing TaskGroup vs gather().",
      source: "Lesson 2: Concurrent Tasks — Pattern Selection"
    },
    {
      question: "Your AI system calls 10 LLM APIs concurrently. 8 succeed, 2 timeout. Using gather(return_exceptions=True), how do you extract successful results?",
      options: [
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. gather(return_exceptions=True) returns a list with mixed types: successful results as their return values, failed tasks as Exception objects. You must manually filter: 'successes = [r for r in results if not isinstance(r, Exception)]' gives you the 8 successful calls. For failures: 'failures = [r for r in results if isinstance(r, Exception)]'. Option 2 is wrong—results include exceptions. Option 3 is wrong—no such method exists. Option 4 is wrong—return_exceptions=True prevents exceptions from being raised (they're returned instead). This pattern is essential for resilient multi-service systems: gather all results, inspect each, use what succeeded, log what failed. From Lesson 2: gather() Error Handling.",
      source: "Lesson 2: Concurrent Tasks — Partial Failure Handling"
    },
    {
      question: "What's the relationship between asyncio.create_task() and asyncio.gather() in terms of task execution?",
      options: [
        "create_task is for single tasks; gather is for multiple only",
        "create_task schedules immediately; gather schedules when awaited",
        "Both schedule tasks immediately; gather also waits for all results",
        "create_task returns Task objects; gather returns coroutines"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 1. Both schedule tasks immediately for execution. create_task(coro) returns a Task object that's already running. gather(coro1, coro2) schedules all coroutines immediately and returns an awaitable that completes when all finish. The difference: create_task gives you individual Task references (for cancellation, inspection). gather gives you a single combined awaitable (convenience for bulk collection). Option 2 is wrong—both schedule immediately. Option 3 is wrong—create_task works for any count (create multiple tasks). Option 4 is wrong—gather doesn't return coroutines. Use create_task when you need control per task. Use gather when you just want all results. From Lesson 2: Understanding the Relationship.",
      source: "Lesson 2: Concurrent Tasks — create_task vs gather"
    },
    {
      question: "Why is asyncio.TaskGroup() considered 'structured concurrency' compared to gather()?",
      options: [
        "TaskGroup provides better performance for many tasks",
        "TaskGroup structures tasks into a hierarchy; gather uses flat list",
        "TaskGroup automatically retries failed tasks; gather doesn't",
        "TaskGroup ensures all tasks are cleaned up (cancelled) if any fail; gather continues running others"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 1. Structured concurrency means: explicit task lifetime management with automatic cleanup. TaskGroup guarantees: when you exit the context (normally or via exception), ALL tasks are either completed or cancelled—no orphaned tasks. If one task fails, all others are cancelled immediately (fail-fast). gather() continues running other tasks even when one fails (unless you manually cancel). Option 2 is wrong—both can handle any task count/structure. Option 3 is wrong—neither automatically retries. Option 4 is wrong—performance is similar. Structured concurrency prevents resource leaks and ensures predictable cleanup. It's the modern best practice pattern. From Lesson 2: TaskGroup Structured Concurrency.",
      source: "Lesson 2: Concurrent Tasks — Structured Concurrency"
    },
    {
      question: "When benchmarking async code, you measure 'wall-clock time' vs 'CPU time.' For I/O-bound async work, what do you expect?",
      options: [
        "Wall-clock time equals sum of individual task times (sequential execution)",
        "Wall-clock time much lower than sum of individual task times (concurrency works)",
        "CPU time matches wall-clock time (CPU always busy)",
        "Wall-clock time higher than sequential (overhead)"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 1. For I/O-bound work with concurrency, wall-clock time approaches the MAXIMUM single task duration, not the sum. Example: 10 API calls at 2s each = 20s sequential, ~2s concurrent. This proves concurrency worked (tasks overlapped). CPU time will be LOW (mostly waiting for I/O, not computing). Option 2 indicates concurrency failed (ran sequentially). Option 3 is wrong for I/O work (CPU is mostly idle during network waits). Option 4 is possible with poor implementation but not expected. Benchmark pattern: time.perf_counter() before and after to measure wall-clock time. If concurrent time ≈ max(task_times), concurrency succeeded. If ≈ sum(task_times), concurrency failed. From Lesson 2: Performance Comparison.",
      source: "Lesson 2: Concurrent Tasks — Performance Benchmarking"
    },
    {
      question: "You're using asyncio.Semaphore(5) to limit concurrent API calls. 10 tasks are created. What happens?",
      options: [
        "5 tasks run immediately; remaining 5 wait at semaphore until first 5 complete",
        "All 10 tasks run concurrently, semaphore has no effect",
        "Only 5 tasks run total; remaining 5 are cancelled",
        "Tasks run in groups of 5 sequentially with no overlap"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. Semaphore(5) allows max 5 concurrent acquisitions. Pattern: 'async with semaphore: await api_call()'. First 5 tasks acquire the semaphore and run. Tasks 6-10 wait at 'async with semaphore' line. When task 1 completes and exits the context, task 6 acquires the semaphore and runs. This creates continuous flow: always 5 tasks running (until <5 remain). Option 2 is wrong—semaphore enforces the limit. Option 3 is wrong—tasks wait, not cancelled. Option 4 is wrong—there IS overlap (as tasks complete, new ones start). Semaphores are critical for rate limiting (API constraints, connection pools, resource limits). From Lesson 5: Semaphores for Resource Limiting.",
      source: "Lesson 5: Hybrid Workloads — Semaphore Behavior"
    },
    {
      question: "What's the PRIMARY reason to use batching (process N items at a time) instead of concurrent processing (process all items at once)?",
      options: [
        "Batching works on older Python versions",
        "Batching is always faster than concurrent processing",
        "Batching requires less code complexity",
        "Prevent memory exhaustion and respect resource constraints (rate limits, connection pools)"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 1. Batching controls resource usage. Processing 10,000 items concurrently means 10,000 in memory, 10,000 API calls in flight, 10,000 database connections—often impossible or prohibited. Batching (e.g., 100 at a time) limits memory (max 100 items), respects rate limits (max 100 concurrent requests), and prevents resource exhaustion. Option 2 is wrong—batching is often SLOWER than unlimited concurrency (but sustainable). Option 3 is arguable (batching adds loop complexity). Option 4 is irrelevant (batching is a pattern, not a version feature). The tradeoff: small batches = low memory, high overhead; large batches = high memory, low overhead. Tune batch size to system constraints. From Lesson 5: Batch Processing Pattern.",
      source: "Lesson 5: Hybrid Workloads — Batching Rationale"
    },
    {
      question: "In a fetch→process→store pipeline with asyncio.Queue, the process stage is slower than fetch. What happens to the queue between fetch and process?",
      options: [
        "Process stage speeds up to match fetch rate",
        "Queue grows unbounded causing memory exhaustion",
        "Queue fills up; fetch stage blocks when queue reaches max size (backpressure)",
        "Fetch stage slows down automatically"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 1. asyncio.Queue(maxsize=N) implements backpressure. When the queue fills (fetch producing faster than process consuming), queue.put() blocks, pausing the fetch stage until process consumes items. This prevents unbounded memory growth (option 2). The queue acts as a buffer: it absorbs temporary rate differences, but sustained imbalance triggers backpressure. Option 3 is wishful thinking (stages don't auto-scale). Option 4 describes the backpressure effect but states it's automatic (it's via queue blocking, not magic). This self-regulating behavior is why queues are the standard for pipeline stage communication. Set maxsize based on memory constraints and desired buffering. From Lesson 5: Pipeline Pattern with Queues.",
      source: "Lesson 5: Hybrid Workloads — Queue Backpressure"
    },
    {
      question: "Your async API client gets RuntimeWarning: coroutine was never awaited. The code is: async def fetch(): return await api_call(). What's the most likely cause in the CALLING code?",
      options: [
        "The fetch() function is missing return statement",
        "Caller is calling fetch() without await: result = fetch() instead of result = await fetch()",
        "api_call() is not an async function",
        "Missing asyncio.run() at the module level"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 1. The warning 'coroutine was never awaited' means someone called an async function without await or asyncio.run(). The fetch() function is correct (async def with await). The bug is in the CALLER: 'result = fetch()' creates a coroutine object but doesn't execute it. Fix: 'result = await fetch()' (if inside async function) or 'result = asyncio.run(fetch())' (if in sync context). Option 2 doesn't cause this warning (would cause different error). Option 3 would cause TypeError at await, not RuntimeWarning. Option 4 is a possibility but less direct than option 1. This is the #1 async beginner mistake: forgetting await when calling async functions. Always: async function call = await. From Lesson 3: Never-Awaited Coroutines.",
      source: "Lesson 3: Advanced Patterns — Common Async Bugs"
    },
    {
      question: "What's the difference between asyncio.timeout(5) context manager and setting a 5-second timeout on an HTTP client?",
      options: [
        "asyncio.timeout applies to entire operation block; HTTP timeout applies per request",
        "Both are identical; asyncio.timeout is just a wrapper",
        "asyncio.timeout is for CPU work; HTTP timeout is for I/O",
        "HTTP timeout is faster and more efficient"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. asyncio.timeout(5) wraps ANY async operations in a context: 'async with asyncio.timeout(5): await fetch(); await process(); await store()' gives 5s total for all three. HTTP client timeout (e.g., httpx.AsyncClient(timeout=5)) applies PER REQUEST: each fetch has 5s limit. Use asyncio.timeout for overall operation deadlines (entire workflow must finish in 5s). Use HTTP timeout for individual request limits (no single request takes >5s). Often you use both: HTTP timeout prevents individual hangs, asyncio.timeout enforces overall SLA. Option 2 is wrong—they're different scopes. Option 3 is wrong—both are for I/O. Option 4 is wrong—performance is similar. From Lesson 3: Timeout Control.",
      source: "Lesson 3: Advanced Patterns — Timeout Scope"
    },
    {
      question: "When would you choose ProcessPoolExecutor over InterpreterPoolExecutor for CPU-bound work?",
      options: [
        "When memory usage must be minimized",
        "When you need faster startup time",
        "When you need full process isolation or working with non-thread-safe C extensions",
        "When working with pure Python code only"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 1. ProcessPoolExecutor provides FULL isolation (separate processes, separate memory spaces). Use it when: 1) C extensions are not thread-safe (even with separate interpreters), 2) complete memory isolation is required (security, stability), or 3) you're already using multiprocessing patterns. InterpreterPoolExecutor is better for: fast startup (option 2—interpreters start faster than processes), lower memory overhead (option 3—shared memory space), and pure Python CPU work (option 4). The tradeoff: ProcessPoolExecutor = isolation + pickle overhead; InterpreterPoolExecutor = shared memory + faster startup. For most Python CPU work, InterpreterPoolExecutor (Python 3.14+) is the better choice. From Lesson 4: Executor Comparison.",
      source: "Lesson 4: CPU-Bound Work — Executor Selection"
    },
    {
      question: "You're optimizing a hybrid system: fetch (100ms each), process (500ms each), 100 items. You have 4 CPU cores. What's the optimal configuration?",
      options: [
        "Batch size 100 to minimize batches",
        "Fetch all 100 concurrently, then process all sequentially",
        "Process with 1 worker to minimize overhead",
        "Batch size 8-16, InterpreterPoolExecutor with 4 workers, concurrent fetch within batch"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 1. With 4 CPU cores, use 4 InterpreterPoolExecutor workers (1 worker per core). Batch size 8-16 balances memory (not all 100 in RAM) and overhead (not too many batches). Fetch within each batch is concurrent (I/O work). Processing is parallel (CPU work). Total time ≈ (100/16) batches × (100ms fetch + 500ms/4 process) ≈ 6.25 × 225ms ≈ 1.4s. Option 2 has 100 items in memory and long wait before processing starts. Option 3 underutilizes CPU (3 cores idle). Option 4 risks memory exhaustion and rate limits. This demonstrates hybrid optimization: match batch size to memory/rate limits, match workers to CPU cores. From Lesson 5: Optimization Strategies.",
      source: "Lesson 5: Hybrid Workloads — Performance Optimization"
    },
    {
      question: "What does 'await loop.run_in_executor(executor, sync_func, arg)' actually do under the hood?",
      options: [
        "Converts sync_func to async function automatically",
        "Submits sync_func to executor, returns Future, awaits Future completion without blocking event loop",
        "Runs sync_func in a separate thread pool always",
        "Blocks event loop until sync_func completes"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 1. Here's the flow: 1) loop.run_in_executor calls executor.submit(sync_func, arg), which returns a concurrent.futures.Future. 2) The asyncio event loop wraps this Future as an awaitable. 3) await pauses the current coroutine until the Future completes. 4) During the await, the event loop is FREE to run other tasks (not blocked). When the executor finishes, the result is returned. Option 2 is wrong—it doesn't change the function, just runs it elsewhere. Option 3 is wrong—it uses whatever executor you pass (could be InterpreterPoolExecutor). Option 4 is wrong—the event loop is NOT blocked (that's the whole point). This bridge pattern is how you integrate blocking/CPU work into async programs. From Lesson 4: Async Executor Integration.",
      source: "Lesson 4: CPU-Bound Work — run_in_executor Mechanics"
    },
    {
      question: "Your AI agent's performance: fetch 50ms (I/O), embedding generation 200ms (CPU), storage 30ms (I/O). You process 1000 items. What's the theoretical best total time with optimal architecture?",
      options: [
        "Approximately 200ms × (1000 / num_cpu_cores) if CPU bottleneck is parallelized",
        "Exactly 280ms (50+200+30) for all 1000 items total",
        "50ms + 200ms + 30ms = 280ms per item × 1000 = 280 seconds",
        "Depends on network speed only (fetch is the bottleneck)"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. With optimal architecture: 1) Fetch is I/O (concurrent, ~50ms for entire batch with asyncio). 2) Embedding is CPU (bottleneck at 200ms each; parallelize with InterpreterPoolExecutor). 3) Storage is I/O (concurrent, ~30ms for entire batch). With 4 CPU cores, embedding time is 200ms × 1000 / 4 = 50 seconds. Total ≈ 50s (all other stages overlap with embedding). Option 2 is fantasy (can't process 1000 items in 280ms). Option 3 is sequential (no optimization). Option 4 is wrong—CPU (embedding) is the bottleneck, not fetch. This analysis (identify slowest stage, parallelize it) is fundamental to hybrid system design. The bottleneck determines total time. From Lesson 5: Bottleneck Analysis.",
      source: "Lesson 5: Hybrid Workloads — Performance Calculation"
    },
    {
      question: "When teaching AI to implement circuit breaker pattern, what key states should you specify in your prompt?",
      options: [
        "Connected, Disconnected, Reconnecting",
        "Active, Inactive, Retry",
        "Success, Failure, Timeout",
        "Closed (normal), Open (failing, reject requests), Half-Open (testing recovery)"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 1. Circuit breaker has three states: 1) CLOSED: normal operation, requests flow through. 2) OPEN: too many failures detected, reject all requests immediately (fail-fast, give service time to recover). 3) HALF-OPEN: after timeout period, allow ONE test request; if successful → CLOSED, if failed → OPEN again. Prompt should specify: failure threshold (e.g., 5 consecutive failures), open duration (e.g., 60s), success threshold to close (e.g., 1 success). Option 2 is too vague. Option 3 describes outcomes, not states. Option 4 describes connection, not circuit breaker. Circuit breaker prevents cascading failures by stopping requests to failing services. From Lesson 3: Resilience Patterns.",
      source: "Lesson 3: Advanced Patterns — Circuit Breaker"
    },
    {
      question: "What's the PRIMARY benefit of using asyncio for AI agent systems that call multiple LLM APIs?",
      options: [
        "asyncio provides better error handling than sync code",
        "asyncio makes API calls faster (lower latency per call)",
        "Concurrent API calls reduce total latency; while waiting for API A, call API B",
        "asyncio uses less memory than sequential calls"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 1. Asyncio's value is CONCURRENCY: while awaiting response from Claude API (2s), simultaneously call OpenAI API (2s). Total time ~2s instead of 4s. This is pure latency reduction through overlap. Option 2 is wrong—asyncio doesn't make individual API calls faster (network speed is unchanged). Option 3 is wrong—error handling quality depends on your code, not sync vs async. Option 4 is wrong—memory usage is similar (might be slightly higher with many concurrent tasks). The win is: total wall-clock time approaches max(individual_calls), not sum(individual_calls). For AI agents querying multiple sources, this is dramatic: 10 APIs at 2s each = 20s sequential, ~2s concurrent. From Lesson 5: AI-Native Workloads.",
      source: "Lesson 5: Hybrid Workloads — AI System Benefits"
    },
    {
      question: "You're reviewing AI-generated async code. It creates 1000 tasks with asyncio.create_task() but never awaits them. What's the problem?",
      options: [
        "Memory leak and RuntimeWarning; tasks are scheduled but results never collected",
        "Code will crash immediately with RuntimeError",
        "Tasks won't run at all (create_task requires await)",
        "Performance will be slow due to too many tasks"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. create_task() DOES schedule tasks (they start running), but if you never await them: 1) You can't get results (they're lost). 2) Tasks might still be running when program exits (resources not cleaned up). 3) You'll get RuntimeWarning: 'Task was destroyed but it is pending!' 4) Potential memory leak (task objects not garbage collected). The fix: store task references and await them: 'tasks = [create_task(f(i)) for i in range(1000)]; results = await asyncio.gather(*tasks)' or use TaskGroup. Option 2 is wrong—no immediate crash. Option 3 is wrong—tasks DO run (just results are lost). Option 4 is wrong—1000 tasks is fine. This is a subtle bug: code 'works' but leaks resources. From Lesson 2: Task Lifecycle.",
      source: "Lesson 2: Concurrent Tasks — Task Management"
    },
    {
      question: "When optimizing a pipeline (fetch→transform→store), which stage should have the MOST workers if transform is CPU-intensive and fetch/store are I/O?",
      options: [
        "Fetch stage (first stage should be fastest)",
        "Transform stage (CPU bottleneck needs parallelism)",
        "Store stage (last stage prevents backlog)",
        "Equal workers across all stages"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 1. The bottleneck stage needs the most resources. If transform is CPU-intensive (e.g., 500ms each), it determines total throughput. Allocate most workers there (e.g., 4-8 InterpreterPoolExecutor workers). Fetch/store are I/O (fast with concurrency, need fewer workers or just asyncio concurrency). Example: fetch with 10 concurrent tasks (asyncio), transform with 8 workers (InterpreterPoolExecutor), store with 5 concurrent tasks (asyncio). Option 2 is wrong—fetch being fast is GOOD (not the bottleneck). Option 3 is wrong—store being fast is GOOD. Option 4 wastes resources (I/O stages don't need workers). The principle: match resources to bottleneck stage. From Lesson 5: Pipeline Optimization.",
      source: "Lesson 5: Hybrid Workloads — Worker Allocation"
    },
    {
      question: "What's the correct way to add jitter to exponential backoff retry logic?",
      options: [
        "Add random variance: delay = base_delay * (2 ** attempt) + random.uniform(0, base_delay * 0.1)",
        "Multiply delay by random factor: delay = base_delay * random.random()",
        "Use random delay each time: delay = random.randint(1, 10)",
        "Jitter isn't needed; exponential backoff alone is sufficient"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. Jitter adds small random variance to prevent thundering herd (all clients retrying at exactly the same time). Pattern: calculate exponential delay (1s, 2s, 4s), then add jitter (0-10% random). Example: 'jitter = random.uniform(0, delay * 0.1); actual_delay = min(delay + jitter, max_cap)'. This spreads retries over time. Option 2 is too chaotic (random 0-100% defeats exponential pattern). Option 3 abandons exponential backoff entirely. Option 4 is wrong—without jitter, all failed clients retry simultaneously (amplifies server load). Jitter is subtle but critical for distributed system resilience. Covered in Lesson 3: Exponential Backoff with Jitter.",
      source: "Lesson 3: Advanced Patterns — Retry Jitter"
    },
    {
      question: "You're building a multi-agent AI system where 5 agents work independently but results are combined. One agent fails. Should you use TaskGroup or gather()?",
      options: [
        "gather(return_exceptions=True): agents are independent, partial results are valuable",
        "TaskGroup: if one fails, cancel all others immediately",
        "Neither; use separate asyncio.run() calls for each agent",
        "Use create_task() and manual cancellation logic"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. The key phrase: 'work independently' and 'results are combined.' If agents are independent, one failure shouldn't stop others (Option 2's fail-fast is wrong). Use gather(return_exceptions=True) to collect all results and exceptions: 'results = await asyncio.gather(agent1(), agent2(), ..., return_exceptions=True)'. Then filter: successes = [r for r in results if not isinstance(r, Exception)]. Combine available results. Option 3 is wrong—separate asyncio.run() calls can't run concurrently. Option 4 is possible but gather() is cleaner. The decision: independent operations with valuable partial results → gather(). Atomic operations where all must succeed → TaskGroup(). From Lesson 2: Pattern Selection for Multi-Agent Systems.",
      source: "Lesson 2: Concurrent Tasks — Multi-Agent Architecture"
    },
    {
      question: "What happens to pending tasks when the Python interpreter exits with tasks still running?",
      options: [
        "RuntimeWarning: Task was destroyed but it is pending (tasks are cancelled, resources may leak)",
        "Tasks complete normally in background",
        "Python waits for all tasks to finish before exiting",
        "Tasks are automatically cancelled with proper cleanup"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. If your program exits (asyncio.run() completes or sys.exit()) with pending tasks: 1) Python issues RuntimeWarning: 'Task was destroyed but it is pending!' 2) Tasks are cancelled ungracefully (no cleanup code runs). 3) Resources may leak (files open, connections active). Prevention: ensure all created tasks are awaited before exit. Use TaskGroup (automatic cleanup) or gather() to wait for all tasks. Option 2 is wrong—background execution doesn't exist in asyncio. Option 3 is wrong—Python does NOT wait. Option 4 is wrong—cancellation is ungraceful. This is why TaskGroup is recommended: it guarantees cleanup. From Lesson 2: Task Lifecycle Management.",
      source: "Lesson 2: Concurrent Tasks — Program Exit Handling"
    },
    {
      question: "When describing error handling requirements to AI for an async API client, which prompt is most complete?",
      options: [
        "Handle TimeoutError (retry 3×), ConnectionError (fail), 429 RateLimitError (exponential backoff), return partial results on other exceptions",
        "Add try/except for errors",
        "Make it handle exceptions properly",
        "Implement error handling for network calls"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. This prompt specifies: WHICH exceptions (TimeoutError, ConnectionError, 429), HOW to handle each (retry vs fail vs backoff), and overall strategy (partial results). AI can generate: 'try: ... except asyncio.TimeoutError: retry_with_backoff() except ConnectionError: return None except RateLimitError: exponential_backoff()'. Option 2 is too vague (what errors? how to handle?). Option 3 is even vaguer ('properly' is undefined). Option 4 specifies domain (network) but not behavior. Good AI prompts enumerate specific exceptions and specific handling logic. Generic 'handle errors' produces generic try/except that might catch too much or too little. From Lesson 3: AI Collaboration for Error Handling.",
      source: "Lesson 3: Advanced Patterns — AI Error Handling Prompts"
    }
  ]}
/>
