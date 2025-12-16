### Core Concept
List comprehensions transform data in a single line: `[expression for item in iterable]` for simple transformation, or `[expression for item in iterable if condition]` for filtered transformation. They're functionally equivalent to loops but more concise and often faster.

### Key Mental Models
- **Comprehension Structure**: `[expr for var in source if cond]` maps to "for each var in source, if condition passes, add expr to new list"
- **Filter Then Transform**: The `if` condition acts as a gate; items failing the condition are skipped entirely, not transformed to empty values
- **Clarity Over Cleverness**: Simple comprehensions are readable; nested or multi-condition comprehensions should become traditional loops
- **Readability Threshold**: One-line transformations remain clear; anything requiring multiple reads needs refactoring to loops

### Critical Patterns
- **Transform Pattern**: `[x*2 for x in numbers]` doubles each; `[word.upper() for word in words]` uppercases each
- **Filter Pattern**: `[n for n in numbers if n > 5]` keeps only items > 5; `[w for w in words if len(w) > 3]` keeps long strings
- **Combine Pattern**: `[x*2 for x in numbers if x % 2 == 0]` filters evens, then doubles (filter first, then transform)

### AI Collaboration Keys
- AI converts loops to comprehensions: "Turn this for-loop into a comprehension"
- AI clarifies filter conditions: "Show me how to filter items matching multiple criteria"
- AI judges readability: "Is this comprehension clear, or should it be a loop?"

### Common Mistakes
- Nested comprehensions appearing clever but becoming unreadable: use loops instead
- Forgetting that `if` filters before transform: `[x*2 if x > 5 else 0 for x in list]` vs `[x*2 for x in list if x > 5]` are different
- Complex expressions inside comprehension that belong in separate steps

### Connections
- **Builds on**: List fundamentals (Lessons 1-3), understanding loops and conditionals
- **Leads to**: Dict comprehensions (Lesson 9), data processing patterns (Lesson 11)
