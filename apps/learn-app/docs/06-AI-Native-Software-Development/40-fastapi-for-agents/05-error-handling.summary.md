### Core Concept
Error handling is communication—status codes tell clients what happened (success vs client error vs server error), and messages explain why. For agents, clear errors enable programmatic recovery: 404 means "try creating it," 5xx means "retry later," 400 means "fix your input."

### Key Mental Models
- **Status codes as contracts**: 2xx = success (proceed), 4xx = client error (don't retry same request), 5xx = server error (retry with backoff). Agents inspect status FIRST, then parse the body.
- **400 vs 422 distinction**: 422 = schema violation caught by Pydantic before your code runs. 400 = business logic violation you detect in your code (empty title, invalid status). Both mean "don't retry," but suggest different root causes to agents.
- **HTTPException as control flow**: `raise HTTPException(...)` stops execution immediately and returns the specified status. This is cleaner than checking errors and returning error responses manually.

### Critical Patterns
- Validate input BEFORE expensive operations (check task exists before calling agent)
- Use `status.HTTP_*` named constants instead of magic numbers (auto-complete, readability)
- Return 201 Created for POST, 204 No Content for DELETE, 200 OK for others
- Include specific context in error messages: `f"Task {task_id} not found"` instead of "not found"
- Handle 404 explicitly with HTTPException, not by returning None

### Common Mistakes
- Creating HTTPException but forgetting to raise it—the exception object does nothing without raise
- Returning {"error": "..."}  with status 200, confusing agents that check status codes first
- Using Python exceptions (ValueError, KeyError) instead of HTTPException, causing 500 errors
- Vague error messages like "Error" that don't help agents or users recover
- Missing status code specification, defaulting to 200 for errors

### Connections
- **Builds on**: Lesson 3's CRUD operations that need error handling
- **Leads to**: Lesson 5's dependency injection that makes error handling more maintainable
