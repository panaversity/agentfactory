### Core Concept
Loop control statements—`break`, `continue`, and loop-`else` clauses—provide precision over loop execution. `break` exits early when you find what you need, `continue` skips unwanted iterations, and `else` detects whether a loop completed naturally or was interrupted by `break`.

### Key Mental Models
- **Early Exit Pattern**: Use `break` when you discover the condition you're searching for and no longer need to continue
- **Skip Current Iteration**: Use `continue` when you want to filter out items and move to the next one
- **Completion Detection**: Loop-`else` executes only if the loop completes without `break`—perfect for search success/failure scenarios
- **Nested Loop Breaks**: `break` only exits the immediate loop, not outer loops (requires careful design for deeply nested structures)

### Critical Patterns
- **Search with Break**: Find an item in a list, exit immediately when found (efficient)
- **Filter with Continue**: Skip invalid/unwanted items while processing remaining ones
- **For-Else Search**: "Did I find it or exhaust all options?" pattern without needing a flag variable
- **While-Else Retry**: "Did user succeed within attempt limit or run out of tries?" pattern for validation loops

### AI Collaboration Keys
- Ask AI to trace nested loop execution, especially which loop a `break` exits
- Request loop-else patterns converted to flag-variable equivalents to see readability gains
- When combining `break` and `continue`, have AI trace execution with specific input values to verify behavior

### Common Mistakes
- **Misunderstanding Loop-Else**: Thinking it runs when loop has no items (false—it runs when loop completes without `break`)
- **Continue Outside Loops**: Using `continue` in `if` statements instead of loops causes `SyntaxError`
- **Nested Loop Break Confusion**: Forgetting that `break` only exits the immediate loop, not outer ones
- **Missing Default Cases**: Omitting `case _:` in match-case or `else:` in loops leaves edge cases unhandled

### Connections
- **Builds on**: Lesson 3 (for and while loops)
- **Leads to**: Lesson 5 (combining loop control with nested structures for complex filtering and validation)
