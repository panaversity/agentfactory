### Core Concept
Asyncio enables concurrent I/O operations on a single thread through an event loop that switches between tasks at pause points (await), allowing network calls, file reads, and other I/O-bound operations to overlap in execution time rather than run sequentially.

### Key Mental Models
- **Event Loop as Coordinator**: Single-threaded manager that runs coroutines, switching between them when they hit `await` points, enabling concurrency without true parallelism
- **Coroutines as Pausable Functions**: Functions marked with `async def` that can suspend execution via `await`, yielding control to other tasks
- **Concurrency vs Parallelism**: Asyncio provides concurrency (tasks take turns, overlap in time) not parallelism (simultaneous execution on multiple cores)
- **I/O-Bound Recognition**: Asyncio helps when tasks wait for external resources (network, disk); useless for CPU-bound work where calculations block

### Critical Patterns
- **asyncio.run() Entry Point**: Modern Python 3.14+ pattern for starting async code from synchronous context
- **await as Yield Point**: Where event loop can pause and switch to other tasks, enabling overlap
- **asyncio.gather() for Concurrency**: Run multiple coroutines concurrently and collect results
- **Timing Rule**: Total time for concurrent I/O approaches longest operation, not sum of all operations

### AI Collaboration Keys
- Ask AI to explain event loop mechanics and when context switches occur during `await` statements
- Have AI show timelines comparing sequential vs concurrent execution visually
- Collaborate on identifying whether your workload is I/O-bound or CPU-bound

### Common Mistakes
- Forgetting `await` keyword (creates coroutine object but doesn't run it)
- Using `time.sleep()` instead of `await asyncio.sleep()` (blocks event loop for other tasks)
- Assuming asyncio helps CPU-bound work (it doesn't—GIL still blocks)

### Connections
- **Builds on**: Understanding functions, type hints, exception handling from Part 5 fundamentals
- **Leads to**: Task coordination patterns (create_task, gather, TaskGroup), timeout handling, and hybrid I/O+CPU workloads
