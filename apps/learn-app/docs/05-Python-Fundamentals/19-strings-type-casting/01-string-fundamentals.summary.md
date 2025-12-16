### Core Concept
Strings are sequences of characters treated as immutable data that cannot be modified after creation. Understanding their fundamental properties—indexing (0-based), length calculation, and the fact that operations create new strings rather than modifying originals—forms the foundation for all text manipulation in Python.

### Key Mental Models
- **Immutability Principle**: Operations on strings don't change the original; they return new strings. This makes strings predictable and safe to use across your program.
- **Zero-Based Indexing**: First character is always at position 0, enabling precise character access using square bracket notation and negative indices for reverse counting.
- **String as Sequence**: Strings are ordered collections where `len()` counts total characters and individual characters are accessed by position.

### Critical Patterns
- **Input → Type Validation**: Always validate that variables are strings using `isinstance()` before operating on them—prevents silent errors downstream.
- **Concatenation with `+`**: Combining strings requires explicit `+` operator; addition and repetition (`*`) are the foundational operations for building larger strings.
- **Preservation Through Operations**: Every operation (concatenation, repetition, slicing) leaves the original string unchanged—crucial for predictable code behavior.

### AI Collaboration Keys
- Ask AI why Python chose immutability for strings instead of allowing modification, connecting the design decision to practical benefits (preventing bugs, thread safety, performance).
- Explore the distinction between built-in functions (`len()`) and methods (`str.method()`) through AI explanation—understanding this distinction enables better pattern recognition in future lessons.

### Common Mistakes
- Trying to modify a string directly with index assignment (`text[0] = "H"` fails) because strings are immutable; must create new string instead.
- Forgetting that `len()` returns an integer, not a string, causing type confusion in concatenation or display.
- Assuming string indexing works like natural language (starting at 1) rather than programmer indexing (starting at 0).

### Connections
- **Builds on**: Basic Python types (int, float, bool) from Chapter 19; understanding data types as a concept.
- **Leads to**: String methods (Lesson 2) for transformation; f-strings (Lesson 3) for formatted output; type casting (Lesson 4) for converting between types.
