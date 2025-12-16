### Core Concept
Concurrency strategy depends on workload classification: CPU-bound vs I/O-bound. The decision framework is: measure single-threaded baseline, classify workload, apply the framework (single-threaded / free-threaded / multiprocessing / asyncio / hybrid), benchmark, compare to baseline. No concurrency is often faster than bad concurrency. Measuring beats guessing.

### Key Mental Models
**Workload Classification**: CPU-bound = computation-intensive (AI inference, matrix math); I/O-bound = waiting-intensive (network requests, file I/O). **Decision Framework Table**: Maps workload type → best approach → Python 3.14 notes. Single-threaded for sequential CPU work, free-threaded for parallel CPU with shared state, multiprocessing for parallel CPU with isolation, asyncio for I/O-bound concurrency, hybrid (free-threaded + asyncio) for mixed workloads. **Overhead-Benefit Analysis**: Concurrency adds overhead (thread/process creation, synchronization, IPC). Only use if benefit > overhead.

### Critical Patterns
**Baseline Rule**: Always measure single-threaded performance first—this is your baseline. Any concurrency must beat baseline accounting for overhead. **When Single-Threaded Wins**: Sequential tasks, single large task, small workloads where overhead dominates, memory-constrained environments. **Free-Threaded Decision**: Appropriate when CPU-bound, parallel, needs shared memory, workload large enough to justify 5–10% overhead. **Multiprocessing Decision**: Parallel CPU work where process isolation is necessary or when memory overhead is acceptable. **Asyncio Decision**: I/O-bound work with 10–100 concurrent tasks; Python 3.14 adds CLI debugging tools (asyncio ps, pstree).

### AI Collaboration Keys
Request AI to help classify 10 real scenarios: PDF extraction, web scraping, file compression, REST API, matrix multiplication, database queries, agent reasoning, stock price updates, batch image resizing, multi-agent negotiation. For each, ask: CPU or I/O bound? What's the best approach? Have AI design benchmarks comparing 2–3 approaches for same workload, showing why winner is faster.

### Common Mistakes
Choosing concurrency without measuring baseline (overhead exceeds benefit). Using threading for CPU-bound work expecting parallelism (GIL prevents it). Ignoring process isolation needs (using threading when multiprocessing required). Not accounting for synchronization overhead (locks can become bottleneck). Mixing asyncio with free-threaded threads without understanding their interaction.

### Connections
Applies GIL knowledge (Lesson 3): understanding GIL limitations drives decision framework. Leverages free-threading (Lesson 4): recognizes when free-threaded overhead is justified. Informs performance optimization (Lesson 2): benchmarking discipline ensures chosen approach is actually faster. Foundation for capstone (Lesson 6): decision framework applied to building production multi-agent system.
