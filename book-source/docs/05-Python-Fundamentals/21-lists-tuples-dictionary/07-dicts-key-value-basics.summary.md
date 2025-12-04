### Core Concept
Dictionaries map keys to values using curly braces `{"key": value}`. Unlike sequences that access by position, dicts access by meaningful keys. Safe access uses `.get(key, default)` to avoid `KeyError` when keys are uncertain; bracket notation `dict[key]` is for certain keys.

### Key Mental Models
- **Unordered Mapping**: Order is implementation detail; access is by key, never by position
- **Unique Keys Constraint**: Duplicate keys overwrite previous values silently—useful for updates but requires careful design
- **Union Types**: `dict[str, int | str | bool]` documents mixed-value types; improves clarity for AI tools and teammates
- **Safe vs Direct Access**: `.get(key, default)` prevents errors; bracket notation `dict[key]` is appropriate when key is certain

### Critical Patterns
- **Creation with Type Hints**: `student: dict[str, str | int] = {"name": "Alice", "age": 20}`
- **Safe Access Pattern**: `gpa = student.get("gpa", "N/A")` returns "N/A" if missing
- **Adding/Updating**: Same syntax: `dict[key] = value` adds new or overwrites existing

### AI Collaboration Keys
- AI generates dict structures for data: "What dict schema should I use for student records?"
- AI clarifies when to use dict vs list: "Would a dict be better for this lookup?"
- AI designs union types: "What type hints express mixed-value dicts?"

### Common Mistakes
- Using bracket notation without checking key exists: `student["gpa"]` crashes if missing
- Confusing dict with list: accessing by meaningful key, not position
- Forgetting keys must be unique: duplicate keys silently overwrite

### Connections
- **Builds on**: Tuples (Lesson 6), understanding key-value mappings
- **Leads to**: Dict modification (Lesson 8), iteration (Lesson 9), structure selection (Lesson 10)
