### Core Concept
Task scheduling transforms sequential awaiting into concurrent execution by scheduling multiple coroutines to run in the background and collecting results later, enabling 10 API calls with 2-second latency each to complete in ~2 seconds instead of 20 seconds.

### Key Mental Models
- **Scheduling vs Awaiting**: Schedule immediately (create_task) to run in background; await later to collect results, enabling overlap
- **Task Lifecycle**: Coroutines transition from pending → running → done states, queued and managed by event loop
- **gather() Convenience**: Schedules multiple coroutines internally, combines scheduling and collection into single call
- **TaskGroup Semantics**: All-or-nothing execution model; failure in one task cancels all others automatically (structured concurrency)

### Critical Patterns
- **asyncio.create_task()**: Schedule coroutine in background, get Task object to await later
- **asyncio.gather()**: Pass multiple coroutines, get results as list; handles scheduling internally
- **asyncio.TaskGroup()**: Python 3.11+ context manager for structured concurrency with automatic cleanup
- **Error Handling Tradeoff**: gather(return_exceptions=True) tolerates failures; TaskGroup fails-fast

### AI Collaboration Keys
- Ask AI to compare `create_task()` (fine-grained control) vs `gather()` (simpler syntax)
- Have AI explain why gather() with 100 tasks is easier than manually creating 100 tasks
- Collaborate on choosing between best-effort (gather) and atomic (TaskGroup) patterns

### Common Mistakes
- Awaiting immediately in loop (defeats concurrency; blocks other tasks)
- Not understanding gather() still needs await—it schedules but doesn't run tasks immediately
- Using TaskGroup when best-effort gathering is appropriate (premature fail-fast)

### Connections
- **Builds on**: Asyncio foundations, event loop mechanics, coroutine basics
- **Leads to**: Timeout handling, error handling patterns, hybrid CPU+I/O workloads
