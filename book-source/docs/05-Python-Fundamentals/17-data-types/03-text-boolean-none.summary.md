### Core Concept
Strings store text data with three quote options, booleans represent True/False decisions, None represents the absence of a value, and Python evaluates all values as truthy or falsy—enabling powerful pattern checking without explicit comparisons.

### Key Mental Models
- **Quote Flexibility**: Single, double, and triple quotes all create strings; choose based on content (single quotes for strings containing double quotes, triple quotes for multi-line text)
- **String Immutability**: Once created, strings can't be modified in place; changes require creating new strings
- **Truthy/Falsy Semantics**: Every value can be evaluated as True or False (0, "", [], {}, None are falsy; everything else is truthy)
- **None ≠ Empty**: None means "no value exists"; it's distinct from zero or empty string which are actual values

### Critical Patterns
- Use type hints with union for optional values: `name: str | None = None` indicates a value that could be text or nothing
- Check for None with `is None`, not `== None`: `if value is None:` (uses singleton identity, not equality)
- Use `bool()` to convert any value to its truthiness: `bool("hello")` returns True, `bool("")` returns False
- String immutability means operations like slicing and joining create new strings: `name[1:]` doesn't modify original

### AI Collaboration Keys
For None handling, ask AI: "When might this value be None? How should I handle missing data?" For truthy/falsy checks, AI can help design Pythonic conditional logic that leverages truthiness instead of explicit comparisons.

### Common Mistakes
- Assuming True/False capitalization is optional (Python requires uppercase; `true` and `false` cause NameError)
- Treating None like zero or empty string (semantically different—use `is None` for checking)
- Attempting to modify strings directly with `string[0] = "x"` (immutability prevents this; create new string instead)
- Confusing quote types (mismatched or wrong choice causes syntax errors or unintended content)
- Misunderstanding truthy/falsy (non-empty string "0" is truthy, but integer 0 is falsy)

### Connections
- **Builds on**: Basic types and type decision framework (Lesson 1)
- **Leads to**: Collections (which have truthy/falsy semantics in Lesson 4) and conditional statements in Chapter 21
