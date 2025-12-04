### Core Concept

The `math` module provides specialized mathematical functions beyond built-in operations. Understanding when to use built-in functions (like `abs()`, `pow()`) versus module functions (like `sqrt()`, `sin()`) is fundamental to accurate computation. The key insight: **validation before operation**—domain errors occur when functions receive mathematically impossible inputs.

### Key Mental Models

**Built-in vs Module Functions**: Python separates lightweight core operations (built-ins) from specialized mathematics (module imports) to keep the language lean. This mental model helps you decide what to import.

**Domain Constraints**: Not all inputs are valid. Square roots require non-negative numbers, logarithms require positive numbers. Recognizing these constraints prevents runtime errors.

**Precision Matters**: Using `math.pi` instead of hardcoding `3.14159` compounds accuracy over large calculations. Small precision differences accumulate in scientific computing.

### Critical Patterns

**Rounding Strategies**: `round()` uses banker's rounding (round half to even), `ceil()` always rounds up, `floor()` always rounds down. Different applications need different strategies—financial calculations often require `ceil()` for safety.

**Validation Pattern**: Check inputs before operation (e.g., verify value ≥ 0 before `sqrt()`), return `None` for invalid cases, provide clear error messages.

**Type Hints for Math**: All mathematical functions should annotate parameter types (`float`, `int`) and return types to ensure clarity.

### AI Collaboration Keys

The AI-native approach: Don't memorize domain constraints or precision differences—understand the principle ("some operations are mathematically impossible") and ask AI to explain error messages. When you encounter `ValueError: math domain error`, AI helps you understand *why* the operation failed, transforming trial-and-error into learning.

### Common Mistakes

Students often hardcode constants like `3.14159` instead of using `math.pi`, losing precision silently. They also write code like `math.sqrt(-5)` without validation, causing runtime crashes. The pattern mistake: performing operations before validating that the operation is mathematically possible.

### Connections

This lesson builds **input validation patterns** used throughout programming (Chapter 26 preview), establishes **type hinting discipline** that improves code clarity, and introduces the mathematical foundation needed for Lessons 2-5 which build timing and calculation systems on this core.
