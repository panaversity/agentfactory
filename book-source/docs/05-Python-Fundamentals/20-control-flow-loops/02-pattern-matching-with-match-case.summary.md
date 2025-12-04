### Core Concept
Pattern matching with `match-case` provides a cleaner, more readable alternative to long `if-elif-else` chains when comparing one variable against multiple specific values. It reads like a structured decision table rather than sequential condition checking.

### Key Mental Models
- **Literal Pattern Matching**: Exact value comparison (matching status code 200, command "save", grade 5)
- **Wildcard Default Case**: The underscore `_` pattern catches any unmatched value, replacing the final `else` clause
- **One-Variable Focus**: `match-case` shines when you're checking ONE variable against MANY values (unlike `if-elif` which handles complex, multi-variable conditions)
- **Stop-at-First-Match**: Like `if-elif-else`, execution stops once a pattern matches; no fall-through or multiple matches

### Critical Patterns
- **Literal Value Matching**: `case 200:`, `case "save"`, `case 5` for exact value comparisons
- **Wildcard Pattern**: `case _:` handles all unmatched inputs (provides default behavior)
- **Menu/Status Code Routing**: Direct user input or status codes to different handlers based on value
- **Python 3.10+ Requirement**: Syntax unavailable in Python 3.9 and earlier versions

### AI Collaboration Keys
- Ask AI to convert existing `if-elif-else` chains to `match-case` to see readability improvements
- Request conversion back to `if-elif-else` for Python 3.9 compatibility
- When choosing between `match-case` and `if-elif`: ask AI to evaluate which is clearer for your specific logic

### Common Mistakes
- **Type Mismatches**: Comparing integer 200 with string "200"—patterns must match the variable type exactly
- **Unreachable Patterns**: Placing wildcard `_` before specific cases makes later cases unreachable
- **Missing Default Case**: Omitting `case _:` can leave valid inputs unhandled (risky for user input)
- **Python Version Errors**: Attempting to use `match-case` on Python 3.9 or earlier causes `SyntaxError`

### Connections
- **Builds on**: Lesson 1 (conditionals and decision-making fundamentals)
- **Leads to**: Lesson 5 (combining pattern matching with loops for filtered processing)
