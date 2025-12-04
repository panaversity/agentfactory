### Core Concept
Frozensets are immutable set variants that can be used as dictionary keys or nested in sets. The immutability constraint unlocks hashability—enabling frozensets to participate in contexts where regular mutable sets fail.

### Key Mental Models
- **Immutability Tradeoff**: Can't modify after creation; gains hashability and use as dict keys
- **Hashability Superpower**: Frozensets are hashable (unlike sets); can be dictionary keys or set members
- **No Modification Methods**: `frozenset` has no `.add()`, `.remove()`; operations return new frozensets
- **Read-Only Operations**: All set operations (union, intersection) work; return new frozensets

### Critical Patterns
- **Dict Keys**: Use frozensets as keys when data shouldn't change (permission levels, coordinate groups)
- **Nesting**: Create sets of frozensets (`set[frozenset[str]]`) for complex grouped structures
- **Type-Safety**: Frozensets communicate "this collection is immutable" to future maintainers
- **Permission Systems**: Map frozenset roles to access levels with O(1) lookup

### AI Collaboration Keys
- AI explains when to use frozenset vs set by asking "will this be a dict key?"
- AI demonstrates "unhashable type: set" errors and how frozensets solve them

### Common Mistakes
- Trying to modify frozenset and getting AttributeError instead of error that suggests frozenset
- Using regular sets as dict keys and getting "unhashable" error
- Not seeing frozensets as the only way to nest collections

### Connections
- **Builds on**: Set creation and hashing fundamentals (Lessons 1-3)
- **Leads to**: Garbage collection patterns where frozensets categorize objects (Lesson 6 capstone)
