### Core Concept
Python provides three fundamental collection types—lists (ordered, mutable sequences), tuples (ordered, immutable sequences), and dictionaries (key-value mappings)—each solving different data organization problems. The choice between them depends on mutability needs, ordering requirements, and lookup patterns.

### Key Mental Models
- **Sequences vs Mappings**: Lists and tuples access items by position; dictionaries access by meaningful key
- **Mutability Spectrum**: Mutable structures (lists, dicts) change after creation; immutable structures (tuples) provide safety guarantees
- **Type Hints as Intent**: `list[str]`, `tuple[int, int]`, `dict[str, int]` communicate data structure purpose to tools and teammates
- **Architectural Thinking**: Collection choice isn't arbitrary—it signals whether data changes, whether order matters, and how you'll retrieve values

### Critical Patterns
- **Sequence Selection**: Use lists when order matters and data changes; use tuples when data is fixed
- **Dict Advantages**: O(1) lookup by key vs O(n) list search, making dicts superior for frequent lookups by identifier
- **Immutability Benefits**: Tuples prevent accidental modification, enable use as dict keys, communicate "fixed data" intent

### AI Collaboration Keys
- AI can help explain design tradeoffs: "Which structure should I use for student enrollment?"
- AI can analyze lookup requirements and recommend dict for high-frequency key access
- AI excels at type hint design: generating complex union types for mixed-value collections

### Common Mistakes
- Treating all three structures as interchangeable (ignoring mutability/ordering tradeoffs)
- Using lists for frequent lookups by identifier (scaling problem: O(n) vs O(1))
- Missing that tuples are immutable ("Why can't I change this tuple?")

### Connections
- **Builds on**: Variables, basic data types (strings, integers, booleans)
- **Leads to**: List/dict methods (Lessons 2-9), nested collections, comprehensions, data structure selection patterns
