### Core Concept
Custom exceptions communicate domain-specific errors—failures that mean something in your business logic, not just generic Python errors. Custom exceptions enable callers to handle different failure types differently through the exception hierarchy.

### Key Mental Models
- **Domain Exceptions**: Inherit from Exception (or ValueError if input validation); represent business logic errors, not generic failures
- **Exception Hierarchy**: Base class catches all related errors; specific subclasses enable granular error handling
- **Custom `__init__`**: Store context attributes (username, attempt count, field names) for programmatic error recovery
- **Exception Chaining**: `raise NewException(...) from original_exception` preserves debugging trail while adding business context

### Critical Patterns
- Base exception class (e.g., AuthenticationError) with specific subclasses for different failure modes (UserNotFoundError, InvalidCredentialsError, AccountLockedError)
- Custom `__init__` with relevant parameters (username, attempt_number, locked_until) stored as instance attributes
- Callers catch specific errors for granular recovery OR base exception for category-level handling
- WeakPasswordError inherits from ValueError (semantically correct for input validation); domain errors inherit from Exception
- Exception chaining preserves original exception for debugging while adding business context

### AI Collaboration Keys
- Ask AI when to create custom exceptions vs. using built-in ValueError/TypeError/RuntimeError
- Have AI design exception hierarchies for multi-error domains (authentication, payment, orders)
- Challenge AI with exception chaining scenarios—show how original exception context is preserved
- Review AI's custom exception classes; discuss which attributes enable better error recovery

### Common Mistakes
- Creating custom exception for every error instead of grouping related errors under base class
- Forgetting to call `super().__init__(message)` in custom `__init__` methods
- Not storing relevant context as instance attributes (limiting caller's ability to respond)
- Using exception chaining for all errors instead of just when you're transforming error types

### Connections
- **Builds on**: Lesson 1 (exception types), Lesson 2 (raising exceptions), object-oriented programming (Chapter 28)
- **Leads to**: Lesson 4 (strategic error recovery with custom exceptions), Lesson 5 (capstone with custom exception hierarchy)
