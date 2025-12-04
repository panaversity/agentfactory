### Core Concept
The four-block structure (try/except/else/finally) provides complete control over program flow: try catches potential errors, except recovers from them, else runs only on success, finally guarantees cleanup regardless of outcome.

### Key Mental Models
- **Except Blocks**: Only run when matching exception occurs; specific exceptions checked before general ones
- **Else Block**: Runs only when try completes without exception; separates "might fail" code (try) from "success-only" code (else)
- **Finally Block**: Always runs, even after return statements in try/except/else; guarantees resource cleanup (file close, connection release)
- **Control Flow Rule**: try → except (if error) OR else (if no error) → finally (always)

### Critical Patterns
- Multiple except blocks for different exception types with appropriate recovery strategies
- Else block for logging success or post-processing that depends on no errors
- Finally block for guaranteed cleanup (closing files, releasing locks, cleanup operations)
- Return in finally is dangerous—it overrides return values from try/except/else blocks
- Nested try/except for handling errors at different levels (file-level vs. line-level errors)

### AI Collaboration Keys
- Ask AI to trace execution paths through all four blocks for different error scenarios
- Challenge AI with return statement edge cases—can finally block override a return?
- Have AI design file operation templates with appropriate except/else/finally usage
- Review AI's code to understand when each block executes and why

### Common Mistakes
- Putting success logic in try block when it belongs in else (confuses error source)
- Forgetting finally when working with resources (file leaks, unclosed connections)
- Using return in finally (dangerous—overwrites actual return value)
- Handling exceptions in except but forgetting cleanup (finally addresses this)

### Connections
- **Builds on**: Lesson 1 (exception types and catching), basic file I/O
- **Leads to**: Lesson 3 (raising custom exceptions), Lesson 4 (strategic error recovery)
