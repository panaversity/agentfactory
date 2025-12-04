### Core Concept
Variables are named containers for data, and type hints describe what kind of data goes inside. Type hints transform variables from unnamed storage into explicit specifications: `age: int = 25` declares intent that `age = 25` cannot express. This bridges daily programming and AI collaboration.

### Key Mental Models
- **Type Hint as Specification**: `age: int = 25` is a one-line specification saying "I intend age to be a whole number"; AI interprets this intent when generating related code
- **Four Primitive Types Cover Foundation**: int (whole numbers), str (text), float (decimals), bool (true/false) are sufficient for A2-level programs; collections preview their existence without requiring mastery
- **PEP 8 as Professional Standard**: Lowercase_with_underscores, descriptive names, no spaces—conventions matter because AI systems and humans both expect consistency; violating conventions triggers misinterpretation

### Critical Patterns
- **Intent Through Naming**: `user_name` vs `userName` vs `x` differ in how clearly they communicate purpose; unclear names confuse both humans and AI interpretation
- **Type Validation Pattern**: `isinstance(age, int)` checks whether data matches declared intent; this pattern becomes validation backbone in production code
- **Collections Awareness**: Lists, dicts, tuples, sets are previewed but not taught; recognizing them later requires knowing they exist as distinct concepts

### AI Collaboration Keys
- Type hints make intent explicit to AI; without hints, `age = 25` is ambiguous (year? count? score?); with hints, AI knows exact semantic meaning
- Variable naming clarity directly impacts code quality; descriptive names like `customer_email` help AI generate better related code than `data1`
- Asking AI to explain WHY float beats int for decimal values teaches semantic thinking, not just syntax; forces design thinking about representation

### Common Mistakes
- Forgetting colon in type hints (syntax error: `age int = 25` vs correct `age: int = 25`)
- Using quotes around numbers (`age: int = "25"` makes it a string, violating the type hint intent)
- Thinking type hints are enforced (Python allows `age: int = "twenty-five"`; runtime behavior doesn't validate; you must use `isinstance()` for validation)
- Non-descriptive names (`x`, `a`, `data`) violate PEP 8 and confuse intent for both humans and AI

### Connections
- **Builds on**: Understanding Python's purpose and types, installation success
- **Leads to**: Collections (lists, dicts), functions with typed parameters, validation patterns, specification-first design in production
