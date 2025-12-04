### Core Concept
After catching an exception, the real decision is strategic: retry transient errors, use fallback values for permanent failures, degrade gracefully when non-critical features fail, and log all errors with context for debugging.

### Key Mental Models
- **Transient Errors**: Network timeouts, temporary unavailability—respond with retry logic with exponential backoff
- **Permanent Errors**: File not found, invalid data—respond with fallback values or graceful degradation
- **Graceful Degradation**: Skip invalid records, continue processing valid ones; partial results better than complete failure
- **Logging Strategy**: Include timestamp, error type, context data, and human-readable message for debugging without crashing

### Critical Patterns
- Retry with exponential backoff (1s, 2s, 4s, 8s) prevents overwhelming failing services
- Fallback values enable program continuation with sensible defaults (dark theme if config loads fails)
- Graceful degradation processes valid records, logs invalid ones, reports summary at end
- Logging includes context: what failed (error type), when (timestamp), where (line/function), why (error message), what values (data involved)
- Error handling strategy depends on error type—don't retry file-not-found errors; don't fallback network errors forever

### AI Collaboration Keys
- Ask AI to identify error types in a scenario and recommend recovery strategy for each
- Challenge AI: when to retry vs. when to fallback vs. when to degrade gracefully
- Have AI implement retry with exponential backoff; discuss why this is better than fixed-interval retry
- Review AI's logging patterns—does logged context enable meaningful debugging?

### Common Mistakes
- Retrying errors that won't resolve (file not found, permission denied)
- Using infinite retries without timeout or max-attempt limit
- Logging too little context (no data, no timestamp) making debugging impossible
- Not distinguishing between recoverable (retry) and non-recoverable (fallback) errors
- Crashing on first error instead of skipping that record and continuing

### Connections
- **Builds on**: Lessons 1-3 (catching, custom exceptions, exception types)
- **Leads to**: Lesson 5 (capstone—combining all strategies in realistic CSV parser project)
