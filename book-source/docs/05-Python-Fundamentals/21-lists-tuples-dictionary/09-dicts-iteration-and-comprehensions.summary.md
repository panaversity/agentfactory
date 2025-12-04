### Core Concept
Three iteration methods: `.keys()` iterates key names, `.values()` iterates values only, `.items()` returns (key, value) tuples supporting unpacking. Dict comprehensions use `{key: value for item in iterable}` syntax, optionally with `if` conditions for filtering.

### Key Mental Models
- **Three Iteration Patterns**: Match intent—use `.keys()` when you need keys, `.values()` for value analysis, `.items()` for both (most common)
- **Unpacking Elegance**: `for name, grade in dict.items():` unpacks tuples automatically
- **Comprehension Syntax**: `{k: v for k, v in dict.items() if condition}` filters and transforms key-value pairs simultaneously
- **Key/Value Transformation**: Transform keys `{k.upper(): v for k, v in dict.items()}`, values `{k: v*2 for k, v in dict.items()}`, or both

### Critical Patterns
- **Keys Only**: `for name in grades.keys():` to iterate key names
- **Values Only**: `sum(prices.values())` to calculate with values
- **Both with Unpacking**: `for city, temp in temps.items():` reads naturally
- **Filtered Comprehension**: `{k: v for k, v in dict.items() if v > 20}` keeps pairs matching condition

### AI Collaboration Keys
- AI generates iteration patterns: "How do I sum all values in a dict?"
- AI transforms dict comprehensions: "Convert this loop to a comprehension"
- AI clarifies filtering: "How do I filter dict items with multiple conditions?"

### Common Mistakes
- Iterating `for k in dict:` (implicit keys) vs explicit `.keys()` (clearer intent)
- Forgetting `.items()` exists, manually indexing instead
- Complex nested comprehensions that should become readable loops

### Connections
- **Builds on**: Dict creation and modification (Lessons 7-8), comprehensions concept (Lesson 5)
- **Leads to**: Data processing pipelines (Lesson 11), real-world applications
