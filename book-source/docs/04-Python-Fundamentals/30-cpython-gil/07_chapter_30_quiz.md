---
sidebar_position: 7
title: "Chapter 30: CPython and GIL Quiz"
---

# Chapter 30: CPython and GIL Quiz

Test your understanding of CPython architecture, the Global Interpreter Lock, free-threading revolution in Python 3.14, and concurrency decision-making for AI-native development.

<Quiz
  title="Chapter 30: CPython and GIL Assessment"
  questions={[    {
      question: "You're building an AI agent that processes 1M records with NumPy matrix operations. Each operation releases the GIL during computation. Which concurrency approach provides the best performance?",
      options: [
        "Traditional threading with Python threads for parallelism",
        "Free-threaded Python to utilize multiple CPU cores",
        "Multiprocessing to avoid GIL contention completely",
        "Single-threaded execution since NumPy releases the GIL"
      ],
      correctOption: 3,
      explanation: "Single-threaded execution is best because NumPy's C extensions already release the GIL during matrix operations, allowing true parallelism without Python threading overhead. Option 1 (traditional threading) adds unnecessary context switching when NumPy operations already run in parallel. Option 2 (free-threading) introduces 5-10% overhead when NumPy already handles parallelism efficiently through C code. Option 4 (multiprocessing) adds process creation and IPC overhead when GIL isn't a bottleneck. The key insight: when C extensions release the GIL, you get parallelism for free. This appears in Lesson 3: Traditional GIL, where C extension GIL release is explained.",
      source: "Lesson 3: The Traditional GIL (Pre-3.13)"
    },
    {
      question: "What is the primary reason CPython uses reference counting for memory management instead of garbage collection alone?",
      options: [
        "Deterministic memory release without unpredictable pause times",
        "Lower overall memory consumption than garbage collection",
        "Better compatibility with multithreaded Python code execution",
        "Faster object creation and deletion in Python"
      ],
      correctOption: 0,
      explanation: "Reference counting provides deterministic memory release—objects are freed immediately when their count reaches zero, without waiting for a garbage collector sweep. This predictability matters for real-time systems and AI inference workloads where latency spikes are unacceptable. Option 2 is incorrect; reference counting can use MORE memory (each object stores a count). Option 3 is backwards; reference counting actually complicates threading (it's why the GIL exists). Option 4 is partially true but not the PRIMARY reason; determinism is the key advantage. Garbage collection handles cycles (the weakness of reference counting alone), which is why CPython uses both. This concept is covered in Lesson 1: What is CPython?",
      source: "Lesson 1: What is CPython?"
    },
    {
      question: "A colleague asks AI to generate code for processing API responses. The AI produces `for i in range(len(items)): process(items[i])`. What should you recommend?",
      options: [
        "Use while loop for better performance characteristics",
        "Replace with direct iteration over items collection",
        "Add enumerate for accessing both index values",
        "Convert to list comprehension for faster execution"
      ],
      correctOption: 1,
      explanation: "Direct iteration (`for item in items:`) is more Pythonic than manual indexing. The AI used C-style iteration which is technically correct but non-idiomatic. Option 1 (while loop) would make code worse—for loops are correct here, just poorly written. Option 3 (enumerate) is only needed if you actually use the index; otherwise it adds unnecessary overhead. Option 4 (list comprehension) changes semantics from iteration-with-side-effects to list-building, which may not be appropriate if `process()` has side effects. When working with AI, describing 'iterate directly over collection' produces better code than accepting C-style patterns. This pattern recognition is covered in Lesson 1 when discussing Pythonic iteration.",
      source: "Lesson 1: What is CPython?"
    },
    {
      question: "Python 3.14's incremental garbage collection reduces pause times by splitting collection work into smaller chunks. When does this optimization provide the MOST benefit?",
      options: [
        "Batch data processing with gigabytes of memory",
        "Scientific computing with long-running numerical calculations",
        "Real-time chatbot responding to user messages instantly",
        "File I/O operations reading large datasets sequentially"
      ],
      correctOption: 2,
      explanation: "Real-time chatbots benefit most because they're latency-sensitive—users notice 50ms pauses as sluggishness. Incremental GC spreads collection across tiny chunks, keeping response times consistent. Option 1 (batch processing) cares about throughput, not latency; one 100ms pause is fine. Option 3 (scientific computing) runs for hours; GC pauses are negligible compared to total runtime. Option 4 (file I/O) is already I/O-bound; computation pauses don't matter when disk reads dominate. The distinction: latency-sensitive applications (chatbots, real-time inference, interactive systems) benefit from pause reduction, while throughput-oriented workloads don't care about individual pause duration. This is explained in Lesson 2: CPython Performance Evolution.",
      source: "Lesson 2: CPython Performance Evolution (Python 3.14)"
    },
    {
      question: "You're profiling a tight loop running 1 billion iterations. Python 3.14's tail-call interpreter optimization shows 4% speedup compared to Python 3.13. Why does this small percentage matter?",
      options: [
        "It improves garbage collection pause time characteristics",
        "It reduces memory usage for large datasets",
        "It enables better multithreading performance gains",
        "It compounds across billions of operations significantly"
      ],
      correctOption: 3,
      explanation: "4% speedup across 1 billion operations translates to substantial time savings—if your loop takes 10 hours, you save 24 minutes. Interpreter optimizations compound because they apply to EVERY bytecode instruction. Option 2 (memory usage) is unrelated; tail-call optimization affects execution speed, not memory. Option 3 (multithreading) is incorrect; interpreter speedup is single-threaded (GIL behavior unchanged). Option 4 (GC pauses) is a separate optimization. The professional insight: don't dismiss small percentages at scale. In AI workloads processing billions of operations, 3-5% improvements are significant. This compounding effect is explained in Lesson 2 when discussing performance at scale.",
      source: "Lesson 2: CPython Performance Evolution (Python 3.14)"
    },
    {
      question: "Which statement correctly describes the relationship between CPython and the Global Interpreter Lock?",
      options: [
        "GIL is CPython implementation detail protecting reference counts",
        "GIL is Python language requirement for thread safety",
        "GIL exists because Python doesn't support multithreading",
        "GIL prevents race conditions in all Python code"
      ],
      correctOption: 0,
      explanation: "The GIL is a CPython implementation detail, not a Python language requirement. Other implementations like Jython don't have a GIL because they use different memory management (JVM garbage collection instead of reference counting). Option 1 is wrong; PyPy and Jython are Python implementations without a GIL. Option 3 is backwards; Python supports multithreading, but CPython's GIL limits parallelism. Option 4 is dangerously wrong; the GIL only protects CPython's internals (like reference counts), NOT your application code—you still need locks for thread-safe data structures. Understanding this distinction is critical: the GIL solves CPython's memory safety problem, not general concurrency safety. Covered in Lesson 3: Traditional GIL.",
      source: "Lesson 3: The Traditional GIL (Pre-3.13)"
    },
    {
      question: "A function downloads 100 URLs concurrently using threading. Each request takes 500ms. The total time is approximately 550ms instead of 50 seconds. Why does threading provide speedup here?",
      options: [
        "Threads execute simultaneously on multiple CPU cores",
        "Python GIL releases during network I/O operations",
        "Threading eliminates all context switching overhead completely",
        "Network operations bypass the GIL protection mechanism"
      ],
      correctOption: 1,
      explanation: "The GIL releases during I/O operations (network calls, file reads), allowing other threads to execute while one waits for network responses. This creates pseudo-parallelism—not true simultaneous execution, but concurrent waiting. Option 1 is wrong; with the GIL, threads DON'T execute simultaneously for Python code. Option 3 is incorrect; context switching still exists (it's just less impactful than I/O wait times). Option 4 misrepresents the mechanism; network operations don't 'bypass' the GIL—they explicitly release it during blocking calls. The key insight: threading works for I/O-bound tasks because waiting time >> context switching overhead. For CPU-bound work, threading provides no benefit (GIL held continuously). Explained in Lesson 3: Traditional GIL.",
      source: "Lesson 3: The Traditional GIL (Pre-3.13)"
    },
    {
      question: "You're running 4 CPU-bound threads on a 4-core machine with traditional Python. What performance do you observe compared to single-threaded execution?",
      options: [
        "4x speedup utilizing all CPU cores equally",
        "2x speedup from partial parallelism with GIL",
        "Same speed or slightly slower from overhead",
        "Significant speedup only on first two cores"
      ],
      correctOption: 2,
      explanation: "With traditional Python (GIL enabled), CPU-bound threading provides NO speedup and often runs SLOWER due to context switching overhead. The GIL prevents true parallelism—only one thread executes Python bytecode at a time. Option 1 (4x speedup) is what you'd expect on free-threaded Python, not traditional. Option 2 (partial parallelism) doesn't happen; it's all or nothing with the GIL. Option 4 (two cores) is arbitrary; the GIL isn't core-specific. The harsh reality: threading + CPU-bound work + GIL = wasted effort. Use multiprocessing (separate processes, separate GILs) or free-threaded Python instead. This failure mode is demonstrated with benchmarks in Lesson 3.",
      source: "Lesson 3: The Traditional GIL (Pre-3.13)"
    },
    {
      question: "What is the most significant architectural change in Python 3.14 regarding concurrency compared to all previous Python versions?",
      options: [
        "GIL can now be disabled at runtime",
        "Threading performance improved by incremental garbage collection",
        "Asyncio supports CLI debugging with ps commands",
        "Multiprocessing uses forkserver by default safely"
      ],
      correctOption: 0,
      explanation: "Making the GIL optional is the biggest change in Python's 30-year history. For the first time, you can disable the GIL entirely (via free-threaded Python builds), enabling true multi-core parallelism for CPU-bound work. This fundamentally changes Python's capabilities. Option 2 (incremental GC) is important but incremental, not revolutionary. Option 3 (asyncio CLI) is a debugging improvement, not architectural. Option 4 (forkserver) is a safety improvement but doesn't change fundamental capabilities. The paradigm shift: Python was defined by GIL limitations for 30 years. Python 3.14 makes those limitations optional, unlocking multi-core hardware for CPU-bound workloads. This is the centerpiece of Lesson 4: Free-Threaded Python.",
      source: "Lesson 4: Free-Threaded Python (3.14+ Production Ready)"
    },
    {
      question: "Free-threaded Python 3.14 has 5-10% single-threaded overhead compared to traditional Python. When is this overhead justified?",
      options: [
        "Any workload benefiting from cleaner code structure",
        "Small scripts running for less than seconds",
        "I/O-bound workload with network requests waiting time",
        "CPU-bound workload achieving 2x or greater speedup"
      ],
      correctOption: 3,
      explanation: "The 5-10% overhead is justified when multi-threaded speedup exceeds the single-threaded cost. For CPU-bound workloads on multi-core hardware, 2-4x speedup easily outweighs the 10% overhead (net gain: 2x becomes 1.8x, still significant). Option 1 (code structure) doesn't justify performance cost; use traditional Python if not parallelizing. Option 3 (I/O-bound) doesn't benefit from free-threading—the GIL already releases during I/O in traditional Python, so you get concurrency without overhead. Option 4 (small scripts) makes overhead worse; short runtimes mean overhead percentage is higher relative to total time. Professional decision-making: benchmark first, adopt free-threading if parallel gains > overhead cost. Covered in Lesson 4 when discussing performance characteristics.",
      source: "Lesson 4: Free-Threaded Python (3.14+ Production Ready)"
    },
    {
      question: "What does `sys._is_gil_enabled()` returning `False` indicate about your Python environment?",
      options: [
        "GIL temporarily released during I/O operations currently",
        "GIL disabled at runtime using environment variables",
        "Python built with free-threading and GIL disabled",
        "Threading module not available in this installation"
      ],
      correctOption: 2,
      explanation: "`sys._is_gil_enabled() == False` means you're running a free-threaded Python build with the GIL disabled. This is a compile-time decision (built with `--disable-gil`), not runtime state. Option 1 confuses GIL release (temporary, during I/O) with GIL disabled (permanent, free-threaded build). Option 3 misunderstands; while `PYTHON_GIL=0` can force GIL off, `_is_gil_enabled()` returns the CURRENT state—if it's False, GIL is disabled regardless of how. Option 4 is unrelated; threading availability is separate from GIL status. The return values: `True` = GIL enabled, `False` = GIL disabled (free-threading active), `None` = no free-threading support. This detection API is covered in Lesson 4.",
      source: "Lesson 4: Free-Threaded Python (3.14+ Production Ready)"
    },
    {
      question: "You have a hybrid workload: fetch data from APIs (I/O), process with CPU-intensive calculations, then store results (I/O). What concurrency approach is most appropriate?",
      options: [
        "Multiprocessing for isolation across all three stages",
        "Free-threaded Python with asyncio for I/O layers",
        "Asyncio alone for handling all I/O efficiently",
        "Traditional threading since I/O releases the GIL"
      ],
      correctOption: 1,
      explanation: "Hybrid workloads need hybrid solutions: asyncio for I/O (fetching, storing) and free-threaded Python for CPU processing. This combination handles both efficiently without overhead. Option 1 (multiprocessing) works but adds process overhead unnecessarily—free-threading provides CPU parallelism without IPC costs. Option 2 (asyncio alone) fails for CPU work; asyncio is single-threaded, so CPU processing blocks the event loop. Option 4 (traditional threading) works for I/O but fails for CPU (GIL prevents parallelism). The professional pattern: use the right tool for each stage. Python 3.14 officially supports combining free-threaded execution with asyncio. Covered in Lesson 5: Choosing Your Concurrency Approach.",
      source: "Lesson 5: Choosing Your Concurrency Approach"
    },
    {
      question: "When benchmarking concurrency approaches, why must you measure single-threaded performance FIRST before adding concurrency?",
      options: [
        "Identifies optimal thread count for parallel execution",
        "Prevents race conditions when comparing multiple approaches",
        "Ensures garbage collection runs consistently across tests",
        "Establishes baseline to determine if overhead justified"
      ],
      correctOption: 3,
      explanation: "Single-threaded performance is your baseline—any concurrency approach must beat this to justify its overhead. Without a baseline, you can't tell if threading/multiprocessing actually helped or hurt. Option 2 (race conditions) is unrelated to measurement methodology; race conditions are bugs, not measurement issues. Option 3 (GC consistency) matters but isn't the PRIMARY reason for baseline measurement. Option 4 (thread count) is determined AFTER establishing baseline, not before. The professional workflow: (1) measure baseline, (2) add concurrency, (3) compare, (4) decide if overhead is justified by speedup. Many developers skip step 1 and wonder why their 'optimized' code is slower. Covered in Lesson 5's decision framework.",
      source: "Lesson 5: Choosing Your Concurrency Approach"
    },
    {
      question: "Python 3.14 adds `python -m asyncio ps <PID>` CLI debugging. What does this command reveal about a running asyncio application?",
      options: [
        "Currently running tasks with execution time data",
        "Memory usage per async task allocated currently",
        "CPU core assignments for each concurrent task",
        "Network connections opened by asyncio event loop"
      ],
      correctOption: 0,
      explanation: "`python -m asyncio ps <PID>` shows currently running tasks, their names, and how long they've been executing—critical for debugging deadlocks and hangs. If a task has been running for minutes when it should take seconds, you've found your bottleneck. Option 2 (memory) isn't provided by `ps`; you'd use profiling tools like `tracemalloc`. Option 3 (CPU cores) is meaningless for asyncio—it's single-threaded, running on one core. Option 4 (network connections) is OS-level, not asyncio-specific (use `netstat` or `lsof`). The companion command `pstree` shows task hierarchy. These CLI tools make production asyncio debugging feasible without modifying code. Introduced in Lesson 5 as a Python 3.14 improvement.",
      source: "Lesson 5: Choosing Your Concurrency Approach"
    },
    {
      question: "A multi-agent system with 4 agents processes independent reasoning tasks. Each agent runs for 10 seconds. On a 4-core machine with free-threaded Python, what's the expected total runtime?",
      options: [
        "Approximately 10 seconds with true parallel execution",
        "Approximately 40 seconds since GIL prevents parallelism",
        "Approximately 12 seconds accounting for overhead only",
        "Approximately 20 seconds with partial parallel execution"
      ],
      correctOption: 2,
      explanation: "With free-threaded Python on 4 cores, the 4 agents run in parallel, taking ~10 seconds base time + 5-10% overhead = ~11-12 seconds total. This assumes all cores are available and workload is purely CPU-bound. Option 1 (exactly 10s) ignores free-threading's overhead; true parallel doesn't mean zero overhead. Option 2 (40s) describes traditional GIL behavior (sequential execution), not free-threaded. Option 3 (20s) suggests only 2x speedup on 4 cores, which would indicate bottlenecks or non-CPU work. Real-world results: 2-4x speedup on 4 cores (closer to 3.5x typically). The overhead is the cost of thread-safe operations without the GIL. Demonstrated in Lesson 6's capstone project.",
      source: "Lesson 6: Capstone - Multi-Agent Concurrency System"
    },
    {
      question: "Why do multi-agent systems built with free-threaded Python need explicit locks for shared data structures even though the GIL is disabled?",
      options: [
        "GIL removal creates new race conditions automatically",
        "Multiple threads access shared data simultaneously now",
        "Built-in types are no longer thread-safe automatically",
        "Free-threading requires manual synchronization for shared state"
      ],
      correctOption: 1,
      explanation: "With the GIL disabled, multiple threads can ACTUALLY run simultaneously on different CPU cores, accessing shared data at the same time. This creates race conditions if data isn't protected by locks. The GIL previously prevented this by ensuring only one thread ran at a time (automatic serialization). Option 1 is misleading; GIL removal doesn't 'create' race conditions—it exposes them by removing automatic protection. Option 2 is correct semantically but doesn't explain WHY. Built-in types (Option 3) ARE thread-safe in free-threaded Python (internal locks protect them), but YOUR shared data structures aren't automatically protected. Professional practice: treat free-threaded code like any multi-threaded language (Java, C++)—protect shared mutable state. Covered in Lesson 6's thread-safe result collector.",
      source: "Lesson 6: Capstone - Multi-Agent Concurrency System"
    },
    {
      question: "What distinguishes CPython from PyPy in terms of execution model and performance characteristics?",
      options: [
        "CPython uses JIT compilation for faster code",
        "PyPy interprets bytecode while CPython compiles directly",
        "PyPy requires C extensions while CPython doesn't",
        "CPython compiles to bytecode; PyPy uses JIT"
      ],
      correctOption: 3,
      explanation: "CPython compiles Python to bytecode and interprets it. PyPy also compiles to bytecode but adds JIT (Just-In-Time) compilation—it detects hot code paths and compiles them to machine code for much faster execution (2-10x speedup). Option 1 reverses them; CPython is the interpreter-only implementation. Option 2 is backwards; both compile to bytecode first (PyPy adds JIT afterward). Option 4 is wrong; PyPy's C extension support is LIMITED (many C extensions don't work), while CPython has full C API support. Tradeoffs: CPython = compatibility and startup speed, PyPy = execution speed for long-running programs. Understanding implementation differences helps choose the right Python for your workload. Covered in Lesson 1: What is CPython?",
      source: "Lesson 1: What is CPython?"
    },
    {
      question: "When would you choose MicroPython over CPython for a production system?",
      options: [
        "Embedded systems with limited memory and constraints",
        "High-performance numerical computing requiring speed and precision",
        "Web applications handling thousands of requests concurrently",
        "Data processing pipelines with gigabytes of data"
      ],
      correctOption: 0,
      explanation: "MicroPython is designed for embedded systems (IoT devices, microcontrollers) with kilobytes of RAM, not gigabytes. It sacrifices features for a tiny footprint. Option 2 (numerical computing) needs full NumPy/SciPy support, which MicroPython lacks—use CPython. Option 3 (web apps) needs mature frameworks (Django, FastAPI) unavailable on MicroPython—use CPython. Option 4 (data processing) needs pandas/Dask, which require full CPython. MicroPython's subset supports ~70% of Python language; it omits advanced features to fit in <256KB. Use when hardware constraints force minimalism (Arduino, Raspberry Pi Pico, ESP32 chips). Implementation choice is hardware-driven here, not workload-driven. Explained in Lesson 1's alternative implementations comparison.",
      source: "Lesson 1: What is CPython?"
    },
    {
      question: "A developer uses `platform.python_implementation()` and gets 'CPython'. What does this tell them about their runtime environment?",
      options: [
        "Their code uses C extensions for performance",
        "They have the official reference Python implementation",
        "The GIL is enabled in this environment",
        "They're running Python on a C compiler"
      ],
      correctOption: 1,
      explanation: "`platform.python_implementation()` returns 'CPython' when running the official reference implementation (the standard python.org download). This is the most common Python, accounting for 95%+ of installations. Option 2 (C extensions) is unrelated; you CAN use C extensions in CPython, but this function doesn't indicate whether you ARE. Option 3 (GIL enabled) is likely true but not guaranteed; free-threaded CPython builds return 'CPython' but may have GIL disabled (check `sys._is_gil_enabled()` separately). Option 4 misunderstands; CPython is WRITTEN in C, but you're not 'running on a C compiler'—you're running the interpreter compiled from C source. Other return values: 'PyPy', 'Jython', 'IronPython'. Use this for implementation-aware code. Demonstrated in Lesson 1's detection examples.",
      source: "Lesson 1: What is CPython?"
    },
    {
      question: "Why is bytecode compilation to `.pyc` files an optimization in CPython?",
      options: [
        "Bytecode executes faster than Python source directly",
        "Bytecode uses less memory than source files",
        "Bytecode is cached avoiding recompilation on imports",
        "Bytecode enables cross-platform deployment and portability"
      ],
      correctOption: 2,
      explanation: "Bytecode caching in `__pycache__/*.pyc` files avoids recompilation. The first import compiles `.py` → `.pyc`; subsequent imports load cached bytecode directly, skipping parsing and compilation—significantly faster for large projects. Option 1 is misleading; bytecode doesn't 'execute faster' than source (they're the same once compiled)—the speedup is avoiding compilation, not execution. Option 3 is wrong; `.pyc` files can be LARGER than source due to metadata. Option 4 (portability) is partially true but not the PRIMARY optimization; bytecode is platform-independent but must match Python version. When you wonder 'why isn't my code change working?', check if bytecode is stale—delete `__pycache__` to force recompilation. Explained in Lesson 1's execution pipeline.",
      source: "Lesson 1: What is CPython?"
    },
    {
      question: "What problem does Python 3.14's deferred annotation evaluation solve for large applications?",
      options: [
        "Fixes compatibility issues with forward type references",
        "Improves type checking accuracy for complex types",
        "Enables runtime type validation without performance cost",
        "Reduces import time by evaluating annotations lazily"
      ],
      correctOption: 3,
      explanation: "Deferred annotations (PEP 649/749) delay type hint evaluation until needed, drastically reducing import time for modules with hundreds of complex type hints. Previously, `def func(x: dict[str, list[int]])` evaluated the annotation immediately on import. Now it's stored as a string and evaluated only when introspection tools request it. Option 2 (type checking accuracy) is unchanged; type checkers like mypy work the same. Option 3 (runtime validation) isn't what deferred annotations provide—libraries like Pydantic do runtime validation separately. Option 4 (forward references) was solved earlier with `from __future__ import annotations`. The impact: large AI applications with heavy type hinting see import times drop from seconds to milliseconds. Covered in Lesson 2's performance improvements.",
      source: "Lesson 2: CPython Performance Evolution (Python 3.14)"
    },
    {
      question: "An AI inference loop runs 10 million iterations in 100 seconds on Python 3.13. On Python 3.14 with tail-call optimization, it runs in 96 seconds. Is the 4% speedup significant?",
      options: [
        "Yes, 4% compounds significantly at this scale",
        "No, 4 seconds saved is negligible overhead",
        "No, measurement variance likely explains the difference",
        "Yes, but only for production deployment scenarios"
      ],
      correctOption: 0,
      explanation: "4% speedup at scale is significant. If this loop runs hourly in production, you save 4 seconds × 24 hours × 365 days = 35,040 seconds (9.7 hours) per year per instance. Multiply by fleet size (100 servers) = 970 hours saved annually. Option 1 dismisses percentage gains at scale—professional mistake. Option 3 (measurement variance) would be valid if the difference was <1%, but 4% is above noise threshold (run multiple trials to confirm). Option 4 (production only) is arbitrary; development benefits too (faster testing, iteration). The mindset shift: small percentages × large scale = substantial absolute time. This thinking separates amateur from professional performance analysis. Explained in Lesson 2 when discussing why small optimizations matter.",
      source: "Lesson 2: CPython Performance Evolution (Python 3.14)"
    },
    {
      question: "Why did CPython use a global lock instead of per-object locks for reference counting?",
      options: [
        "Global locks are faster than fine-grained locking",
        "Per-object locks aren't supported in C language",
        "Avoids deadlock risk and reduces memory overhead significantly",
        "Global approach enables better garbage collection performance"
      ],
      correctOption: 2,
      explanation: "Per-object locks create deadlock risk (Thread A locks obj1 then wants obj2; Thread B locks obj2 then wants obj1 → deadlock) and memory overhead (millions of objects each need a lock). A single global lock is simpler, safer, and uses constant memory. Option 1 is misleading; global locks can be slower due to contention, but simplicity/safety outweighed this. Option 3 is absurd; C fully supports per-object locks (mutexes in structs). Option 4 reverses causation; GC design is affected BY the GIL, not why the GIL exists. The tradeoff: global lock = safe but no parallelism; per-object locks = complex but parallel. CPython chose safety. Free-threading uses biased locking (middle ground). Explained in Lesson 3: Traditional GIL.",
      source: "Lesson 3: The Traditional GIL (Pre-3.13)"
    },
    {
      question: "A CPU-bound task runs in 10s single-threaded. With 4 threads on traditional Python, it takes 11s. Why is threading slower?",
      options: [
        "GIL contention causes threads to wait excessively",
        "Context switching overhead exceeds any parallelism benefit",
        "Threading implementation has bugs in Python version",
        "CPU-bound work requires multiprocessing not threads"
      ],
      correctOption: 1,
      explanation: "Context switching overhead (saving/restoring thread state) exceeds any benefit because the GIL prevents actual parallelism. You get all the costs (switching) without benefits (parallel execution). Option 2 (GIL contention) is semantically similar but misses the point—the GIL isn't 'contested' (implying competition), it's sequentially held (no parallelism at all). Option 3 (bugs) is wrong; this is expected behavior, not a bug. Option 4 is prescriptive (correct advice) but doesn't explain WHY threading is slower. The insight: concurrency without parallelism is overhead without benefit. For CPU work with GIL, threading makes things WORSE. Use free-threaded Python or multiprocessing instead. Demonstrated with benchmarks in Lesson 3.",
      source: "Lesson 3: The Traditional GIL (Pre-3.13)"
    },
    {
      question: "You're classifying workloads for concurrency strategy. Which of these is I/O-bound?",
      options: [
        "Database query processing returning 1M rows iteratively",
        "Matrix multiplication for machine learning training data",
        "Image resizing with PIL for batch processing",
        "Prime number calculation using Sieve of Eratosthenes"
      ],
      correctOption: 0,
      explanation: "Database queries are I/O-bound—your code waits for the database server to process the query and return results. During this wait, the GIL releases, allowing other threads to execute. Option 1 (matrix multiplication) is CPU-bound (computation). Option 3 (image resizing) is CPU-bound (pixel manipulation calculations). Option 4 (prime calculation) is obviously CPU-bound (pure computation). The distinction: I/O-bound = waiting for external resources (network, disk, database); CPU-bound = doing calculations. For I/O-bound workloads, threading or asyncio work well even with the GIL because waiting time >> execution time. This classification is THE critical decision-making skill from Lesson 3 and Lesson 5.",
      source: "Lesson 3: The Traditional GIL (Pre-3.13)"
    },
    {
      question: "When does multiprocessing provide better performance than free-threaded Python for CPU-bound work?",
      options: [
        "When GIL overhead exceeds process creation cost",
        "When shared memory usage exceeds available RAM",
        "When workload requires more than four CPU cores",
        "When processes need complete isolation for safety"
      ],
      correctOption: 3,
      explanation: "Multiprocessing is better when you need process isolation—one process crash doesn't kill others, and processes can't corrupt each other's memory. For fault-tolerant systems or untrusted code execution, isolation justifies overhead. Option 2 (shared memory) is backwards; multiprocessing DUPLICATES memory (worse for large datasets), while free-threading SHARES memory (better). Option 3 (core count) is arbitrary; free-threading scales to any core count. Option 4 (GIL overhead) is nonsensical in this context—free-threading doesn't have GIL overhead (GIL is disabled). The decision: free-threading for shared-state CPU parallelism; multiprocessing for isolation or when pre-3.14 compatibility required. Both achieve parallelism; isolation is the differentiator. From Lesson 5's decision framework.",
      source: "Lesson 5: Choosing Your Concurrency Approach"
    },
    {
      question: "What is the three-phase roadmap for free-threading adoption in Python?",
      options: [
        "3.13 beta, 3.14 stable, 3.15 removes GIL",
        "3.13 production, 3.14 optimized, 3.15 mandatory requirement",
        "3.13 experimental, 3.14 production-ready, 3.15+ likely default",
        "3.13 disabled, 3.14 optional, 3.15 removes overhead"
      ],
      correctOption: 2,
      explanation: "Python 3.13 (2024) introduced free-threading as experimental with 40% overhead. Python 3.14 (Oct 2025) made it production-ready with 5-10% overhead. Python 3.15+ (2026+) will likely make free-threading the default (GIL becomes opt-in). Option 2 mischaracterizes 3.13 (it was experimental, not production). Option 3's language (beta/stable) isn't how Python versions are described, and GIL isn't being 'removed' (made optional). Option 4 is wrong; 3.13 had free-threading available (not disabled), and overhead won't be fully 'removed' (tradeoffs persist). The strategy: gradual rollout lets ecosystem adapt, performance stabilize, and developers migrate on their timeline. Forcing too fast breaks compatibility. Covered in Lesson 4's roadmap section.",
      source: "Lesson 4: Free-Threaded Python (3.14+ Production Ready)"
    },
    {
      question: "How does free-threading achieve thread safety without the global lock?",
      options: [
        "Disables reference counting in favor of garbage collection",
        "Per-thread state with fine-grained object-level locks internally",
        "Restricts threading to I/O-bound operations only carefully",
        "Requires programmers to manually synchronize all object access"
      ],
      correctOption: 1,
      explanation: "Free-threading uses per-thread interpreter state and fine-grained locks on shared objects (dicts, lists) to protect internal state without a global lock. Built-in types are thread-safe through internal locking. Option 2 is wrong; free-threading still uses reference counting (modified with biased/deferred techniques). Option 3 is nonsense; free-threading works for ALL operations, not just I/O. Option 4 is partially true for APPLICATION data but misleading—built-in types are automatically thread-safe; programmers only lock THEIR shared state. The architecture: each thread operates independently; when accessing shared objects, fine-grained locks prevent corruption. This is how Jython/IronPython always worked (JVM/.NET handle it). Explained in Lesson 4's architecture section.",
      source: "Lesson 4: Free-Threaded Python (3.14+ Production Ready)"
    },
    {
      question: "You're debugging an asyncio application that appears stuck. Which Python 3.14 command helps diagnose hanging tasks?",
      options: [
        "python -m concurrent.futures status to show threads",
        "python -m threading inspect to show state",
        "python -m multiprocessing debug to show processes",
        "python -m asyncio ps to show tasks"
      ],
      correctOption: 3,
      explanation: "`python -m asyncio ps <PID>` (new in Python 3.14) shows all running asyncio tasks, how long they've been executing, and their current state—perfect for identifying stuck tasks. If one task shows 300 seconds of execution when it should take 5 seconds, you've found your hang. Option 2 (`threading inspect`) doesn't exist as a module command. Option 3 (`multiprocessing debug`) isn't a real command. Option 4 (`concurrent.futures status`) also doesn't exist. The companion command `python -m asyncio pstree` shows task hierarchy (which task spawned which). These CLI tools are production-focused debugging additions in 3.14. Covered in Lesson 5's asyncio improvements.",
      source: "Lesson 5: Choosing Your Concurrency Approach"
    },
    {
      question: "What does Python 3.14's forkserver default for multiprocessing improve compared to fork?",
      options: [
        "Safer process creation avoiding threading state deadlocks",
        "Faster process startup reducing overall initialization time",
        "Better memory sharing between parent and processes",
        "Automatic load balancing across CPU cores evenly"
      ],
      correctOption: 0,
      explanation: "Forkserver creates a clean server process at startup, then spawns workers from this clean state. This avoids inheriting complex threading state from the parent (which causes deadlocks). Fork copies the ENTIRE parent process, including locks held by threads—if a thread held a lock when fork happened, the child inherits a permanently locked mutex (deadlock). Option 2 (faster startup) is wrong; forkserver is actually SLOWER (extra indirection). Option 3 (memory sharing) is better with fork (copy-on-write), not forkserver. Option 4 (load balancing) is unrelated to fork method. Python 3.14 makes forkserver default for safety. Trade speed for correctness. Explained in Lesson 5's multiprocessing improvements.",
      source: "Lesson 5: Choosing Your Concurrency Approach"
    },
    {
      question: "A multi-agent system collects results in a shared list without locks. Under free-threaded Python, what problem occurs?",
      options: [
        "GIL prevents writes causing deadlock scenarios immediately",
        "Race conditions corrupting shared list structure silently",
        "Memory leaks from duplicate entries in list",
        "Performance degradation from excessive context switching overhead"
      ],
      correctOption: 1,
      explanation: "Without locks, multiple threads appending simultaneously can corrupt the list's internal structure (lost updates, crashes, garbled data). With free-threading, threads ACTUALLY run in parallel, making race conditions real hazards. Option 2 is backwards; there IS no GIL in free-threaded Python (that's the point), so GIL can't prevent anything. Option 3 (memory leaks) isn't the primary issue; corruption (lost results, crashes) is worse. Option 4 (context switching) is unrelated to data corruption. The solution: use locks (as shown in Lesson 6's ThreadSafeResultCollector) or built-in thread-safe structures (queue.Queue). Free-threading removes automatic protection—you must synchronize manually. From Lesson 6's thread-safety section.",
      source: "Lesson 6: Capstone - Multi-Agent Concurrency System"
    },
    {
      question: "Why is agent independence important for multi-agent systems using free-threaded Python?",
      options: [
        "Simplifies debugging by centralizing execution logic flow",
        "Reduces memory usage by sharing state data",
        "Independent agents enable true parallel execution benefits",
        "Enables better garbage collection pause time management"
      ],
      correctOption: 2,
      explanation: "Independent agents (no shared mutable state, minimal coordination) can execute in parallel without contention. If agents constantly share data, they'll spend time waiting for locks, losing parallelism benefits. Option 2 (memory usage) is backwards; independence often INCREASES memory (duplicated data), but enables parallelism. Option 3 (debugging) is tangential; independence simplifies reasoning but isn't the PRIMARY benefit. Option 4 (GC pauses) is unrelated to agent independence. The design principle: maximize agent autonomy, minimize shared state. This unlocks linear scaling (4 agents on 4 cores = ~4x speedup). When agents are interdependent (frequent synchronization), speedup diminishes. From Lesson 6's architecture patterns.",
      source: "Lesson 6: Capstone - Multi-Agent Concurrency System"
    },
    {
      question: "When asking AI to implement concurrent data processing, which prompt produces the most appropriate code for Python 3.14?",
      options: [
        "Classify workload first, then choose concurrency approach appropriately",
        "Use threading for all concurrent operations immediately",
        "Default to multiprocessing for maximum performance always",
        "Implement asyncio for modern concurrent Python patterns"
      ],
      correctOption: 0,
      explanation: "The professional approach is workload classification FIRST (CPU vs I/O), THEN choose the appropriate concurrency model. Prescribing a solution before understanding the problem leads to mismatches. Option 1 (threading for all) fails for CPU-bound work in traditional Python. Option 3 (multiprocessing always) adds unnecessary overhead for I/O-bound or single-threaded workloads. Option 4 (asyncio default) only works for I/O-bound; it's single-threaded (bad for CPU work). Better prompt: 'I have a [describe workload]. Help me classify it as CPU-bound or I/O-bound, then recommend the best concurrency approach for Python 3.14.' This activates AI's reasoning about tradeoffs. From Lesson 5's decision framework.",
      source: "Lesson 5: Choosing Your Concurrency Approach"
    },
    {
      question: "You profile a data processing pipeline and find 70% time in NumPy operations, 20% in pure Python loops, 10% in file I/O. What concurrency approach optimizes this?",
      options: [
        "Free-threaded Python for NumPy parallelism enhancement opportunities",
        "Asyncio for I/O with threading for loops",
        "Multiprocessing to parallelize all three pipeline stages",
        "Single-threaded execution since NumPy releases GIL already"
      ],
      correctOption: 3,
      explanation: "NumPy operations already release the GIL and run in parallel (via optimized C code). Adding threading/multiprocessing adds overhead without benefit for the dominant 70% of work. The 20% pure Python could benefit from free-threading, but optimizing 20% isn't worth 5-10% overhead on 100%. Option 1 (free-threading) adds overhead when NumPy already parallelizes. Option 3 (multiprocessing) adds process overhead when parallelism already exists. Option 4 (asyncio+threading) over-complicates for 10% I/O time. Professional decision: measure first, optimize if needed. Here, single-threaded is fastest because bottleneck (NumPy) is already optimized. From Lesson 5's benchmarking methodology.",
      source: "Lesson 5: Choosing Your Concurrency Approach"
    },
    {
      question: "What does CPython's use of reference counting imply for multi-threaded code safety?",
      options: [
        "Automatic garbage collection handles concurrency safety completely automatically",
        "Reference counts must be protected from simultaneous modification",
        "Multi-threading is automatically safe for all objects",
        "Reference counting eliminates need for synchronization primitives"
      ],
      correctOption: 1,
      explanation: "Reference counts are NOT thread-safe without protection. If two threads simultaneously increment/decrement an object's count, the final value can be wrong (race condition), leading to premature deallocation or memory leaks. This is WHY the GIL exists—it protects reference count operations. Option 2 (GC handles it) is wrong; garbage collection is separate from reference counting. Option 3 (automatically safe) is dangerously false; thread safety requires explicit synchronization (GIL or fine-grained locks). Option 4 reverses reality; reference counting REQUIRES synchronization. In free-threaded Python, biased reference counting and deferred counting reduce synchronization overhead while maintaining safety. From Lesson 3's explanation of why GIL exists.",
      source: "Lesson 3: The Traditional GIL (Pre-3.13)"
    },
    {
      question: "A developer benchmarks code and finds free-threaded Python is slower than traditional Python for their workload. What's the most likely cause?",
      options: [
        "Hardware lacks sufficient CPU cores for benefits",
        "Free-threading implementation has performance bugs currently compiled",
        "Workload is single-threaded suffering from overhead penalty",
        "Python version installed incorrectly missing optimizations"
      ],
      correctOption: 2,
      explanation: "Free-threaded Python has 5-10% single-threaded overhead. If the workload doesn't use multiple threads or isn't CPU-bound, you pay the overhead without gaining parallelism—net loss. Option 2 (bugs) is unlikely; free-threading is production-ready in 3.14 (thoroughly tested). Option 3 (hardware) doesn't matter for single-threaded workloads; even 1 core shows the overhead. Option 4 (installation) is possible but rare; verify with `sys._is_gil_enabled()` first. The lesson: don't adopt free-threading blindly. Benchmark your ACTUAL workload. If you're not parallelizing CPU-bound work, stick with traditional Python (avoid overhead). From Lesson 4's performance characteristics discussion.",
      source: "Lesson 4: Free-Threaded Python (3.14+ Production Ready)"
    },
    {
      question: "How does biased locking in free-threaded Python reduce synchronization overhead?",
      options: [
        "Predicts future access patterns using machine learning",
        "Distributes locks evenly across CPU cores automatically",
        "Eliminates all locking when reference counts stable",
        "Optimizes for single-thread access avoiding lock acquisition costs"
      ],
      correctOption: 3,
      explanation: "Biased locking assumes most objects are accessed by a single thread repeatedly. Instead of acquiring/releasing locks each time, the lock 'biases' toward that thread (fast path). If another thread accesses the object, the bias is revoked and slower locking activates. This optimization keeps the common case (single-thread access) fast. Option 2 (distributes locks) misunderstands; biasing is about reducing lock operations, not distributing them. Option 3 (eliminates locking) is wrong; locks are still present, just optimized away for the common case. Option 4 (machine learning) is absurd. The pattern: most objects are short-lived and single-threaded; biased locking exploits this. From Lesson 4's free-threading architecture.",
      source: "Lesson 4: Free-Threaded Python (3.14+ Production Ready)"
    },
    {
      question: "Why is incremental garbage collection particularly beneficial for real-time AI agents?",
      options: [
        "Reduces latency spikes from stop-the-world collection pauses",
        "Improves overall throughput for batch processing workloads",
        "Decreases total memory usage for long-running processes",
        "Enables faster allocation of new objects consistently"
      ],
      correctOption: 0,
      explanation: "Real-time AI agents care about latency—users notice 50-100ms pauses as delays. Incremental GC splits collection work into tiny chunks (3ms instead of 100ms), preventing perceptible pauses. Option 2 (throughput) is unrelated; incremental GC may DECREASE throughput slightly (more total work) but improves latency. Option 3 (memory usage) isn't affected; total memory usage is the same, just collection is spread out. Option 4 (faster allocation) is tangential; allocation speed isn't the primary benefit. The use case: chatbots, voice assistants, real-time inference systems where response time consistency matters more than raw throughput. From Lesson 2's performance improvements.",
      source: "Lesson 2: CPython Performance Evolution (Python 3.14)"
    },
    {
      question: "You're building a web scraper downloading 200 pages concurrently. Each page takes 2 seconds. What's the best concurrency approach?",
      options: [
        "Multiprocessing to avoid GIL contention issues completely",
        "Free-threaded Python to utilize multiple cores fully",
        "Asyncio for handling I/O concurrency efficiently here",
        "Traditional threading since I/O releases GIL automatically"
      ],
      correctOption: 2,
      explanation: "Asyncio is ideal for high-concurrency I/O workloads—it handles hundreds of concurrent connections in a single thread with minimal overhead. Web scraping is I/O-bound (waiting for HTTP responses), perfect for asyncio. Option 2 (free-threading) works but adds unnecessary overhead; asyncio is single-threaded with no GIL overhead. Option 3 (multiprocessing) adds process overhead when asyncio handles concurrency efficiently without it. Option 4 (traditional threading) works but asyncio scales better (200 threads has overhead; 200 async tasks doesn't). Professional choice: asyncio for I/O-bound high-concurrency workloads. Save threads/processes for CPU-bound parallelism. From Lesson 5's decision framework.",
      source: "Lesson 5: Choosing Your Concurrency Approach"
    },
    {
      question: "What does the `__pycache__` directory contain and why does it matter for development?",
      options: [
        "Temporary objects created during Python program execution runtime",
        "Compiled bytecode files cached for faster subsequent imports",
        "Debug symbols for profiling and performance analysis",
        "Backup copies of source files before editing"
      ],
      correctOption: 1,
      explanation: "`__pycache__` stores compiled bytecode (.pyc files) to avoid recompiling on every import. First import: Python compiles .py → .pyc and caches it. Subsequent imports: load cached .pyc (much faster). Option 2 (temporary objects) is wrong; runtime objects are in memory, not on disk. Option 3 (debug symbols) is unrelated; profiling uses different tools. Option 4 (backups) is absurd; Python doesn't create source backups. Why it matters: if you edit code but changes don't appear, bytecode might be stale. Delete `__pycache__` to force recompilation. Also, .pyc files are platform-independent (same bytecode runs on Windows/Linux/Mac). From Lesson 1's execution pipeline.",
      source: "Lesson 1: What is CPython?"
    },
    {
      question: "When would you use Jython instead of CPython for a production system?",
      options: [
        "Integrating Python code with existing Java systems seamlessly",
        "Achieving maximum performance for numerical computation workloads",
        "Running Python on embedded devices with constraints",
        "Building high-performance web servers handling traffic"
      ],
      correctOption: 0,
      explanation: "Jython runs on the JVM, enabling seamless Java library access and deployment in Java-only environments. Use when you MUST integrate with Java codebases or infrastructure. Option 2 (performance) is wrong; Jython is SLOWER than CPython for most tasks (no NumPy/C extensions). Option 3 (embedded) describes MicroPython, not Jython (Jython needs full JVM). Option 4 (web servers) is better served by CPython (mature frameworks, better performance). Jython tradeoffs: gain Java ecosystem access, lose C extension support and performance. Niche use case: Python scripting in enterprise Java environments. From Lesson 1's alternative implementations.",
      source: "Lesson 1: What is CPython?"
    },
    {
      question: "An AI asks you to review code: `with multiprocessing.Pool(4) as pool: results = pool.map(func, range(100))`. For what workload is this appropriate?",
      options: [
        "Small quick operations completing in milliseconds each",
        "I/O-bound network requests fetching 100 URLs concurrently",
        "Mixed workload with both computation and network access",
        "CPU-bound function processing 100 independent items in parallel"
      ],
      correctOption: 3,
      explanation: "Multiprocessing is appropriate for CPU-bound work where each item is processed independently (embarrassingly parallel). Each of 4 processes handles ~25 items, running in true parallel on separate cores. Option 2 (I/O-bound) should use asyncio or threading (multiprocessing overhead is wasted for I/O). Option 3 (mixed) should use hybrid approach (free-threaded + asyncio). Option 4 (quick operations) pays high process creation overhead for minimal work; multiprocessing makes sense when `func` runs for seconds, not milliseconds. Rule of thumb: multiprocessing overhead is ~50-100ms per process. Justify this with work that takes 10x longer (1+ seconds per item). From Lesson 5's concurrency decision framework.",
      source: "Lesson 5: Choosing Your Concurrency Approach"
    },
    {
      question: "What is the primary advantage of using `dataclass` for agent results in multi-agent systems?",
      options: [
        "Built-in thread safety for concurrent access patterns",
        "Automatic generation of initialization and representation methods clearly",
        "Reduced memory usage compared to dictionaries significantly",
        "Faster attribute access than regular classes always"
      ],
      correctOption: 1,
      explanation: "`dataclass` auto-generates `__init__`, `__repr__`, `__eq__`, and other methods based on type annotations, reducing boilerplate. For `AgentResult(agent_id, result, duration)`, you get automatic initialization and readable representation. Option 2 (thread safety) is wrong; dataclasses provide NO thread safety (you still need locks). Option 3 (memory usage) is negligible; dataclasses are regular classes with generated methods. Option 4 (faster access) is false; attribute access speed is identical to regular classes. The benefit: clean code with less boilerplate. In Lesson 6's capstone, `AgentResult` dataclass made result handling cleaner than manual `__init__` writing. This is about code quality, not performance.",
      source: "Lesson 6: Capstone - Multi-Agent Concurrency System"
    },
    {
      question: "Why does Python 3.14 make free-threading optional rather than mandatory for all installations?",
      options: [
        "Reduces installation size for embedded Python systems",
        "Allows testing before full deployment in production",
        "Preserves compatibility and avoids overhead for unoptimized",
        "Enables gradual ecosystem migration over multiple years"
      ],
      correctOption: 2,
      explanation: "Making free-threading optional avoids forcing 5-10% overhead on users who don't need parallelism (single-threaded scripts, I/O-bound apps). Developers choose: traditional Python (no overhead) or free-threaded (overhead but parallelism). Option 2 (testing) is a side benefit, not the PRIMARY reason. Option 3 (installation size) is trivial; free-threading doesn't significantly affect size. Option 4 (ecosystem migration) is correct strategically but doesn't explain why OPTIONAL—it could be mandatory with migration period. The real reason: backward compatibility and performance. Don't penalize existing workloads for new features they won't use. This is explored in Lesson 4's paradigm shift section.",
      source: "Lesson 4: Free-Threaded Python (3.14+ Production Ready)"
    },
    {
      question: "What pattern should you follow when handling agent failures in multi-agent systems?",
      options: [
        "Retry failed agents indefinitely until eventual success",
        "Let exceptions propagate to crash entire system",
        "Disable error handling for performance optimization always",
        "Catch exceptions per agent, record failure, continue other"
      ],
      correctOption: 3,
      explanation: "Resilient systems catch exceptions per agent, record the failure (with error message), and allow other agents to continue. One agent's failure shouldn't crash the entire system. Option 2 (crash entire system) is the opposite of resilience; production systems must be fault-tolerant. Option 3 (disable error handling) trades correctness for unmeasurable performance gains (bad trade). Option 4 (retry indefinitely) can cause infinite loops; retries should be bounded (max 3 attempts). The pattern in Lesson 6's code: `try: result = agent.reason() except Exception as e: record_failure(e)`. This enables partial success (3 of 4 agents succeed → system continues) instead of all-or-nothing failure. Production-ready pattern.",
      source: "Lesson 6: Capstone - Multi-Agent Concurrency System"
    },
    {
      question: "How does free-threaded Python's 5-10% overhead manifest in production applications?",
      options: [
        "Thread-safe operations cost more than GIL protection",
        "Compilation time increases for all Python modules",
        "Memory usage doubles from duplicated interpreter state",
        "Startup time increases from additional initialization steps"
      ],
      correctOption: 0,
      explanation: "The overhead comes from fine-grained locks and atomic operations replacing the simple global lock. Each reference count update, object access, or built-in operation may acquire/release locks or use atomic instructions (slower than GIL's single check). Option 2 (compilation) is unchanged; bytecode compilation is identical. Option 3 (memory doubling) is wrong; per-thread state adds some memory but not 2x. Option 4 (startup time) is negligible; the overhead is in EXECUTION, not startup. The tradeoff: pay 5-10% for thread-safe operations, gain 2-10x from parallelism. Net benefit if you're parallelizing CPU-bound work. From Lesson 4's performance characteristics.",
      source: "Lesson 4: Free-Threaded Python (3.14+ Production Ready)"
    },
    {
      question: "You're asking AI to explain when to use threading vs multiprocessing. Which AI response demonstrates best understanding?",
      options: [
        "Syntax-focused: Threading is easier, use unless blocked",
        "Technology-first: Threading for speed, multiprocessing for safety",
        "Classification-driven: CPU vs I/O determines approach clearly",
        "Heuristic-based: If unsure, try both and measure"
      ],
      correctOption: 2,
      explanation: "The correct mental model is workload classification first (CPU vs I/O), then choose approach based on characteristics and constraints. Option 2 (technology-first) puts solution before problem—threading isn't 'for speed' (it's for I/O concurrency). Option 3 (syntax-focused) is wrong; threading is NOT 'easier' and can be WRONG choice (CPU-bound). Option 4 (try both) wastes time; classify first, then implement. The professional workflow: (1) classify workload, (2) apply decision framework, (3) implement chosen approach, (4) benchmark to validate. AI should guide you through this reasoning, not prescribe solutions. This is the core of Lesson 5's decision framework teaching.",
      source: "Lesson 5: Choosing Your Concurrency Approach"
    },
    {
      question: "What does `PYTHON_GIL=0` environment variable do in Python 3.14?",
      options: [
        "Disables garbage collection for performance testing only",
        "Forces GIL disabled at runtime if available",
        "Enables debug mode for threading issues detection",
        "Sets maximum thread count to zero threads"
      ],
      correctOption: 1,
      explanation: "`PYTHON_GIL=0` (or `PYTHON_GIL=0`) forces the GIL off at runtime IF you're running a free-threaded Python build. This allows testing both GIL-enabled and GIL-disabled behavior from the same binary. Option 2 (GC) is unrelated; GIL and garbage collection are separate systems. Option 3 (debug mode) is wrong; this controls GIL state, not debugging. Option 4 (thread count) is absurd; setting GIL=0 doesn't limit threads. Use case: benchmarking or debugging to compare GIL-on vs GIL-off performance without rebuilding Python. Check current state with `sys._is_gil_enabled()`. From Lesson 4's runtime GIL control section.",
      source: "Lesson 4: Free-Threaded Python (3.14+ Production Ready)"
    },
    {
      question: "Why is the CPU vs I/O distinction the most important factor in concurrency decision-making?",
      options: [
        "Influences whether to use classes or functions",
        "Affects programming language choice for project implementation",
        "Decides if application needs database or not",
        "Determines whether concurrency provides actual parallelism benefits clearly"
      ],
      correctOption: 3,
      explanation: "CPU vs I/O classification determines if concurrency helps. I/O-bound: threading/asyncio provide speedup (GIL releases during waits). CPU-bound: need multiprocessing or free-threading for parallelism (GIL blocks threading). Wrong classification → wrong approach → no speedup (or slowdown). Option 2 (language choice) is unrelated; this is about concurrency within Python. Option 3 (database) is tangential; I/O-bound might use databases, but classification doesn't determine this. Option 4 (classes vs functions) is completely unrelated to concurrency. Professional mistake: adding threading to CPU-bound work (no benefit). The decision framework in Lesson 5 makes classification the FIRST step—everything else follows from this. Get it wrong, and your entire approach fails.",
      source: "Lesson 5: Choosing Your Concurrency Approach"
    },
    {
      question: "When reviewing AI-generated concurrent code, what should you check first before evaluating correctness?",
      options: [
        "Workload classification matches concurrency approach chosen appropriately",
        "Variable naming follows Python conventions consistently throughout",
        "Type hints are complete for all functions",
        "Comments explain every line of complex code"
      ],
      correctOption: 0,
      explanation: "First check: does the concurrency approach match the workload type? If AI used threading for CPU-bound work (with GIL), the code will be slow regardless of correctness. Option 2 (naming) matters for maintainability but not performance. Option 3 (type hints) helps with code quality but doesn't affect concurrency choice. Option 4 (comments) aids understanding but doesn't validate approach. Professional code review workflow: (1) validate approach matches workload, (2) check correctness (race conditions, locks), (3) verify performance expectations, (4) review code quality. Getting approach wrong makes everything else irrelevant. From Lesson 5's decision framework application.",
      source: "Lesson 5: Choosing Your Concurrency Approach"
    }
  ]}
/>
