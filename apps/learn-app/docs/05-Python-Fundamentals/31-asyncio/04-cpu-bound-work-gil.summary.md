### Core Concept
The Global Interpreter Lock (GIL) prevents true parallelism in threaded code; Python 3.14's InterpreterPoolExecutor solves this by spawning separate interpreters (each with its own GIL), enabling 3-4x speedup on multi-core machines for CPU-bound work.

### Key Mental Models
- **GIL Limitation**: Only one thread executes Python bytecode at a time; threads fighting for single GIL causes context-switching overhead worse than sequential
- **Separate Interpreters ≠ Shared GIL**: InterpreterPoolExecutor creates independent Python interpreters; each has own GIL, enabling true parallelism
- **I/O vs CPU Distinction**: Asyncio (concurrency) solves I/O; InterpreterPoolExecutor (parallelism) solves CPU; GIL irrelevant for I/O because threads naturally yield
- **Executor Selection**: InterpreterPoolExecutor (lightweight, shared memory) preferable to ProcessPoolExecutor (heavyweight, serialized data)

### Critical Patterns
- **loop.run_in_executor()**: Bridge between async code and sync CPU functions
- **InterpreterPoolExecutor(max_workers=4)**: Create pool of 4 separate Python interpreters for true parallelism
- **CPU-Bound Recognition**: Heavy math, data analysis, cryptography—these block and need separate interpreters
- **Measurement**: Sequential (1 core) vs Threading (still 1 core due to GIL) vs InterpreterPoolExecutor (N cores)

### AI Collaboration Keys
- Ask AI to explain GIL's purpose (memory safety) and why it exists
- Have AI show benchmarks: sequential vs threading vs InterpreterPoolExecutor
- Collaborate on when ProcessPoolExecutor (processes) vs InterpreterPoolExecutor (interpreters) fits

### Common Mistakes
- Assuming threading helps CPU work (GIL prevents it; worse than sequential)
- Not measuring before optimizing (may not be CPU-bound bottleneck)
- Using ProcessPoolExecutor when InterpreterPoolExecutor is lighter

### Connections
- **Builds on**: Asyncio concurrency, task coordination, executor basics
- **Leads to**: Hybrid I/O+CPU systems, performance bottleneck analysis
