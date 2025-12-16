### Core Concept
Production async systems require defensive patterns—timeouts prevent indefinite waiting, Futures are awaitable result placeholders (rarely created manually), and resilience patterns (retries, exponential backoff) handle transient failures gracefully.

### Key Mental Models
- **Timeout as Cancellation**: asyncio.timeout() context manager enforces upper bounds on operations; exceeding triggers automatic cancellation
- **Future as Abstract Result**: Placeholder for a value arriving asynchronously; create_task() and executors return Futures; rarely created manually
- **Resilience Layers**: Timeout, retry, and circuit-breaker patterns form defense-in-depth; each handles different failure modes
- **Error Categories**: TimeoutError (took too long), CancelledError (task stopped externally), and domain-specific errors need different handling

### Critical Patterns
- **asyncio.timeout(seconds)**: Context manager for time limits; raises TimeoutError on expiry
- **Exponential Backoff**: Double wait between retries (1s, 2s, 4s, 8s); prevents overwhelming failing services
- **Partial Failure Handling**: gather(return_exceptions=True) collects both successes and exceptions
- **Never-Awaited Detection**: RuntimeWarning signals coroutine created but never executed

### AI Collaboration Keys
- Ask AI to distinguish TimeoutError (operation slow) from CancelledError (task stopped)
- Have AI show retry patterns with exponential backoff and jitter
- Collaborate on circuit breaker design (Open → Half-Open → Closed states)

### Common Mistakes
- Blocking event loop with time.sleep() instead of await asyncio.sleep()
- Assuming timeout errors are permanent (usually transient; retryable)
- Not cleaning up resources in TimeoutError handlers

### Connections
- **Builds on**: Task coordination, concurrent execution patterns
- **Leads to**: Hybrid I/O+CPU workloads, production system design, monitoring
