### Core Concept
Exceptions are objects that signal errors during program execution. Understanding exception types, reading tracebacks, and recognizing patterns enables defensive programming—anticipating problems before they crash your code.

### Key Mental Models
- **Exception Hierarchy**: BaseException → Exception → specific types (ValueError, TypeError, ZeroDivisionError)
- **Traceback Reading**: Read bottom-to-top: exception type/message (last line), call chain (middle lines), where error occurred (second-to-last line)
- **Recognition Patterns**: Different exceptions communicate different problems (wrong type vs. wrong value vs. math violation), enabling targeted recovery strategies

### Critical Patterns
- ValueError: Correct type, invalid value (e.g., `int("abc")`)
- TypeError: Wrong data type entirely (e.g., `5 + "10"`)
- ZeroDivisionError: Math violation (e.g., `10 / 0`)
- Multiple except blocks catch specific exceptions in priority order
- Exception chaining with `raise ... from e` preserves original error context

### AI Collaboration Keys
- Ask AI to explain exception hierarchy and show visual diagrams
- Have AI generate functions that raise multiple exception types; predict which exception occurs for different inputs
- Challenge AI to design recovery strategies for each exception type
- Validate your understanding by reviewing AI's code and explaining the exception flow

### Common Mistakes
- Catching too broad (catching Exception when you should catch ValueError)
- Ignoring exception messages—they contain debugging clues
- Not testing edge cases that trigger different exceptions
- Forgetting that `except` blocks execute in order; specific exceptions must come before general ones

### Connections
- **Builds on**: Understanding Python data types (Chapter 18), type hints (Chapter 19)
- **Leads to**: Lesson 2 (controlling exception flow with try/except/else/finally), Lesson 3 (raising custom exceptions), Lesson 4 (strategic error recovery)
