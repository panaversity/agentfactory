### Core Concept
Integration lesson: build a robust CSV parser that demonstrates all exception handling patterns—try/except/else/finally blocks, custom exceptions, strategic error recovery strategies, and testing for edge cases—in a realistic file processing project.

### Key Mental Models
- **Specification-First**: Define validation rules and error-handling strategy before writing code
- **Layered Error Handling**: File-level errors (FileNotFoundError) stop processing; row-level errors (ValueError) skip record and continue
- **Strategy Selection**: Fatal errors (permissions, file not found) raise and propagate; data validation errors (invalid age) skip record with logging
- **Testing Edge Cases**: Test success path, each error type separately, combined errors, and empty/malformed input

### Critical Patterns
- Separate validation functions that raise exceptions rather than return error codes
- Two-level try/except: outer for file access (fatal), inner for row validation (recoverable)
- Graceful degradation: process all valid rows, log invalid ones, report summary at end
- Results dictionary tracks valid records, invalid records with reasons, total processed
- Test both success and failure paths intentionally to verify error handling works

### AI Collaboration Keys
- Ask AI to review your parser and identify edge cases you're not handling
- Challenge AI: for each validation error, should we skip the record or crash the program?
- Have AI design test cases that exercise each error scenario
- Validate your exception hierarchy—should all data errors be caught by same except block?

### Common Mistakes
- Catching too broadly (catching Exception) instead of specific error types
- Not distinguishing between fatal (file access) and recoverable (row validation) errors
- Forgetting to test edge cases: empty files, files with extra whitespace, missing fields
- Not providing sufficient context in error messages (which row? what value? what rule was violated?)
- Not creating comprehensive test data—mix of valid and invalid records, various error types

### Connections
- **Builds on**: All Lessons 1-4 (exception hierarchy, control flow, custom exceptions, error strategies)
- **Leads to**: Chapter 26 (file I/O with error handling), Chapters 28-29 (object-oriented error handling)
