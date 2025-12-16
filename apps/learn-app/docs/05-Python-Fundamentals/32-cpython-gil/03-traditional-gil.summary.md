### Core Concept
The Global Interpreter Lock is a mutex protecting CPython's reference counting from race conditions. Only one thread executes Python bytecode at a time, preventing true parallelism on multi-core hardware. The GIL exists not as a design mistake but as a pragmatic choice: in 1989 (single-core machines), it simplified interpreter design and C API. In 2024 (multi-core standard), it became a visible constraint.

### Key Mental Models
**Reference Counting Thread Safety Problem**: If two threads modify the same object's reference count simultaneously, the final value depends on timing (race condition), causing premature memory release. **GIL Solution**: Protect ALL reference count operations with one global lock → thread-safe, but only one thread executes bytecode. **CPU vs I/O Distinction** (critical): CPU-bound work holds GIL continuously (no parallelism); I/O-bound work releases GIL during waits (pseudo-parallelism works).

### Critical Patterns
**Threading fails for CPU-bound**: Calculate sum of 50M numbers with 1, 2, 4, 8 threads → same time as sequential (or slower due to context switching overhead). **Threading succeeds for I/O-bound**: Make 4 network requests sequentially (2 seconds) vs with 4 threads (~500ms—concurrent waiting). **Multiprocessing workaround**: Separate processes get separate GILs → true parallelism but 50–100MB per process. **C extensions release GIL**: NumPy operations hold GIL minimally by computing in C.

### AI Collaboration Keys
Request code demonstrating threading fails for CPU work but succeeds for I/O work. Ask AI to explain why C extensions can release GIL safely (C code doesn't touch Python reference counts). Have AI classify 10 real workloads: CPU-bound or I/O-bound? What's the best concurrency approach? Explore multiprocessing tradeoffs: memory overhead vs true parallelism.

### Common Mistakes
Thinking threading always helps (only for I/O). Using multiprocessing for small tasks (overhead exceeds benefit). Assuming GIL is removable without redesigning reference counting. Confusing "concurrency" (switching between tasks) with "parallelism" (simultaneous execution)—GIL prevents parallelism but allows concurrency for I/O.

### Connections
Explains WHY the GIL exists: reference counting protection (Lesson 1). Shows PERFORMANCE IMPLICATIONS: threading overhead worse than single-threaded for CPU work. Motivates free-threading (Lesson 4): 30-year constraint finally solved through biased reference counting. Justifies decision framework (Lesson 5): understanding GIL is prerequisite to choosing between single-threaded, threading, multiprocessing, and free-threaded approaches.
