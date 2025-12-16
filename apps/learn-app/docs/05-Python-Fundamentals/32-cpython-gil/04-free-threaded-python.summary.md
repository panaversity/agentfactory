### Core Concept
Python 3.14 makes free-threading production-ready, reducing GIL overhead from 40% (experimental in 3.13) to 5–10%. The GIL is now optional—you choose whether to enable it. This enables true parallel Python threads on multi-core hardware for the first time in Python's 30-year history. A 4-agent system on 4 cores achieves 3–4x speedup instead of pseudo-concurrency.

### Key Mental Models
**Paradigm Shift**: Traditional Python (GIL always enabled) offers pseudo-parallelism for I/O-bound work, zero parallelism for CPU-bound work. Free-threaded Python offers true parallelism for both. **Three-Phase Rollout**: Phase 1 (3.13, experimental, 40% overhead) → Phase 2 (3.14, production, 5–10% overhead) → Phase 3 (3.15+, likely default). **Per-Thread State**: Free-threading uses biased reference counting (most objects have single reference, no lock needed) and deferred reference counting (batch updates, cheaper synchronization). **Optionality, Not Removal**: GIL still exists in traditional builds; you choose free-threaded build at install time.

### Critical Patterns
Install free-threaded Python alongside traditional build for comparison. Use `sys._is_gil_enabled()` to detect status (returns True/False/None). Set `PYTHON_GIL=0` environment variable to disable GIL at runtime. Benchmark: traditional Python + threading = 12 seconds (4 agents, no speedup); free-threaded = 3 seconds (4 agents, 4x speedup). Account for 5–10% single-threaded overhead when benefits are 2–10x multi-threaded gains.

### AI Collaboration Keys
Ask AI to explain biased reference counting without diving into C implementation details. Request benchmarking code comparing free-threaded vs traditional threading for CPU-bound multi-agent work. Have AI discuss backward compatibility: why can existing code run unchanged on both builds? Explore timeline: why did it take 30 years to solve (backward compatibility burden, ecosystem impact).

### Common Mistakes
Thinking GIL is removed (it's optional, not deleted—backward compatibility). Assuming all libraries work with free-threading (some C extensions may not—compatibility check needed). Ignoring 5–10% single-threaded overhead and choosing free-threaded build for single-threaded workloads. Not detecting GIL status at runtime (code should handle both scenarios).

### Connections
Solves the GIL problem stated in Lesson 3 through architectural innovation. Builds on performance improvements (Lesson 2): free-threading adds 5–10% overhead, but parallelism gains dwarf this cost. Enables new concurrency strategies (Lesson 5): CPU-bound parallel work is now viable on single machine. Foundation for multi-agent systems (Lesson 6): true parallel agents unlock production AI systems on commodity hardware.
