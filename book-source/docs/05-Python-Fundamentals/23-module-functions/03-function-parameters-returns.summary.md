### Core Concept
Functions accept data through parameters (required or optional) and return data via return values. Mastering parameter patterns (positional, keyword, default) and return patterns (multiple values via tuples, optional returns) enables flexible, reusable function design. Keyword arguments especially improve code clarity when many optional parameters exist.

### Key Mental Models
- **Parameter Order Rule**: Required parameters come BEFORE optional ones. Python enforces this because arguments map left-to-right.
- **Keyword Arguments for Clarity**: `function(title="Meeting", date="2025-11-15", time="2:00 PM")` is clearer than positional equivalents—readers know what each argument means.
- **Multiple Returns via Tuple**: Return several values as tuple, unpack into separate variables: `min_val, max_val, avg = analyze(data)`.
- **Optional Returns (Union Types)**: `Type | None` indicates a function might fail. Caller must check for None before using result.
- **Data Flow**: Return values from one function become input to the next. Understanding this flow is essential.

### Critical Patterns
- **Required then Optional**: `def func(required: str, optional: str = "default")`
- **Keyword Arguments**: Especially useful with 3+ optional parameters for readability.
- **Tuple Unpacking**: `a, b, c = function_returning_three_values()` vs. `result = function_returning_three_values()`.
- **Function Composition**: `result = format_price(add_tax(apply_discount(100.0, 20), 8))` chains return values through multiple functions.
- **Default Parameter Defaults**: `def func(items: list = [])` is a common bug—use `items: list | None = None` instead.

### AI Collaboration Keys
AI can generate complete function implementations from well-designed signatures with default parameters and type hints. AI especially excels at parameter ordering and tuple unpacking patterns.

### Common Mistakes
- Mutable defaults (`def func(items: list = [])` creates shared mutable state across calls)
- Overloading functions with too many optional parameters (consider design instead)
- Not checking for None when function returns `Type | None`

### Connections
- **Builds on**: Lessons 1-2 (function basics), Chapter 23 (tuples)
- **Leads to**: Lesson 4 (scope), understanding function architecture
