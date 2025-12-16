### Core Concept
Collections group multiple items together in different structures (list, tuple, dict, set, range) based on ordering, mutability, and lookup needs. Type hints make collection structure explicit, enabling AI partners to suggest appropriate operations for each type.

### Key Mental Models
- **Ordered vs. Keyed Lookup**: Lists/tuples access items by position (`items[0]`); dicts access by name/key (`person["name"]`)
- **Mutable vs. Immutable**: Lists/dicts change; tuples/sets/ranges don't (immutability protects data integrity)
- **Uniqueness**: Sets automatically remove duplicates; lists/tuples allow duplicates; dicts allow duplicate values but not keys
- **Structure Intent**: Type hints communicate data structure (list[int] means "ordered integers", dict[str, float] means "name-to-price lookup")

### Critical Patterns
- Use `list[str]` for ordered, changeable sequences: shopping lists, scores, items that grow/shrink
- Use `tuple[float, float]` for fixed coordinate pairs or structured returns where order and immutability matter
- Use `dict[str, int]` for lookup tables: phone book (name→number), student records (ID→grade)
- Use `set[str]` to deduplicate automatically: unique user IDs, tags, email addresses
- Use `range(0, 10)` for efficient number sequences without storing every value in memory

### AI Collaboration Keys
When uncertain about collection type, ask AI: "I need to store [data description]. Should I use list, dict, tuple, or set? Which access pattern (by position or by name) fits best?" AI helps reason through mutability and ordering constraints.

### Common Mistakes
- Choosing list when dict would be better (by-position access is awkward when you really want by-name lookup)
- Using list for fixed data that shouldn't change (tuple is safer—immutability prevents accidental modification)
- Forgetting that sets are unordered (can't rely on position, no duplicates—different use case than lists)
- Not including type hints for collections (AI partners can't infer what's inside without them)

### Connections
- **Builds on**: Core types and type decision framework (Lessons 1-3)
- **Leads to**: Collection methods and deep operations (Chapter 23) and binary types with files (Chapter 27)
