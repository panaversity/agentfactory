### Core Concept
Type inspection (type(), isinstance(), id()), type casting (converting between types), and understanding when Python converts automatically enable robust data validation and conversion—essential for handling user input and ensuring data integrity.

### Key Mental Models
- **Identity vs. Equality**: `is` checks object identity (same memory location); `==` checks value equality (both refer to same singleton for None and cached small integers)
- **Explicit vs. Implicit Casting**: Explicit casting (you call int(), str(), float()) is safer and clearer; implicit casting (Python auto-converts in mixed arithmetic) can hide precision loss
- **Lossy Conversion**: Converting float to int truncates decimals; converting str to int requires valid number format
- **Type Objects**: `type()` returns type objects, not strings; compare with `==` or use `isinstance()` for flexibility

### Critical Patterns
- Use `isinstance(value, int)` for type checking (respects inheritance, more Pythonic than `type(value) == int`)
- Check for None with `value is None` (identity check for singleton) rather than `value == None`
- Explicit casting shows intent: `age_int = int(age_str)` clearly signals conversion and documents where type changes
- Integer interning: Python caches -5 to 256 for memory efficiency; beyond that range, objects are separate
- Use `bool()` to check truthiness: `bool(value)` converts any value to True/False following Python's truthy/falsy rules

### AI Collaboration Keys
For type validation, ask AI: "Create a validator that checks if input is the right type and converts safely." For edge cases like currency or precision requirements, ask: "Show me examples where simple int/float casting fails and how to handle them."

### Common Mistakes
- Using `==` to compare with None instead of `is None` (semantically incorrect, though sometimes works)
- Ignoring data loss when converting float to int (19.99 becomes 19, not 20)
- Assuming type hints are enforced (Python runtime doesn't validate them; use mypy for type checking)
- Forgetting that `int("25")` works but `int("25.5")` fails (must convert float first)
- Confusing type objects with type names (type(42) returns class object, not string "int")

### Connections
- **Builds on**: All previous types and truthiness concepts (Lessons 1-4)
- **Leads to**: Exception handling for invalid conversions (Chapter 25) and file I/O with type conversions (Chapter 27)
