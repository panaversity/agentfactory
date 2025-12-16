### Core Concept
Loops automate repetition by running code multiple times. `for` loops repeat a known number of times (definite iteration with `range()`); `while` loops repeat based on changing conditions (indefinite iteration). Together they transform tedious copy-paste code into concise, maintainable automation.

### Key Mental Models
- **For Loop with range()**: Generates number sequences; `range(start, stop, step)` never includes the stop value
- **Definite vs. Indefinite Iteration**: `for` when iteration count is known; `while` when repetition depends on a changing condition
- **Loop Variable Updates**: `while` loops MUST update their loop variable each iteration, or they become infinite
- **Zero-Based Indexing**: `range(5)` produces 0, 1, 2, 3, 4 (5 numbers starting at 0)

### Critical Patterns
- **Counting Loops**: `for i in range(10)` to repeat exactly 10 times
- **Step Sequences**: `range(0, 10, 2)` for even numbers; `range(10, 0, -1)` for countdown
- **Condition-Based Loops**: `while age < 18:` for validation or retry scenarios
- **Termination Logic**: Every `while` loop needs a condition that eventually becomes `False` to avoid infinite loops

### AI Collaboration Keys
- Ask AI to show the same task using both `for` and `while` loops; compare readability
- Request range() variations for specific sequences (even numbers, countdowns, step patterns)
- When you see infinite loops, ask AI to trace execution and identify missing loop variable updates

### Common Mistakes
- **Infinite Loops**: Forgetting to update the loop variable in `while` loops (`count += 1` must happen)
- **Off-by-One Errors**: Using `range(10)` when you meant `range(1, 11)` or expecting stop value to be included
- **Step of Zero**: `range(0, 10, 0)` causes `ValueError`—step must be positive or negative
- **Loop Variable Persistence**: Loop variables continue to exist after the loop with their last value

### Connections
- **Builds on**: Chapter 20 (operators for loop conditions)
- **Leads to**: Lesson 4 (loop control with break/continue), Lesson 5 (combining loops with conditionals for complex operations)
