# Console I/O and User Input Validation Summary

### Core Concept

Understand that `input()` always returns strings, requiring explicit type conversion before using data in calculations or comparisons. Input validation through try/except blocks prevents crashes from user errors.

### Key Mental Models

**String-First Principle**: Every user input begins as text. Converting "25" to the number 25 is a deliberate operation, not automatic. Think of `input()` as a translator that captures keystrokes and holds them as words until you tell Python what type they should be.

**Retry Loop Pattern**: Invalid input shouldn't crash the program or end it silently. A `while True` loop with `break` on valid input lets users correct mistakes without restarting the program.

### Critical Patterns

- `input()` → always `str` type, check with `type()`
- `int()` / `float()` → convert with error handling via `try/except ValueError`
- F-strings with format specs: `f"${price:.2f}"` for currency, `f"{value:05d}"` for padded integers
- `get_positive_integer(prompt)` function encapsulates validation logic reusably

### AI Collaboration Keys

Ask AI about validation strategies before coding: "Compare checking menu choices (range validation) vs numeric conversion (ValueError catching). Which error checking happens first?" Understand the design before syntax.

### Common Mistakes

- Forgetting `input()` returns strings ("28" ≠ 28)
- Missing newline characters in `input()` prompts (harder to read)
- Not handling `ValueError` when conversion fails (crashed program)
- Validation in wrong order (accept first, convert second causes unnecessary errors)

### Connections

Builds on Lesson 1's manual foundation (user interaction basics). Enables Lesson 2 (files need input paths). Feeds into Lesson 5 capstone (menu-driven app requires input validation at scale).
