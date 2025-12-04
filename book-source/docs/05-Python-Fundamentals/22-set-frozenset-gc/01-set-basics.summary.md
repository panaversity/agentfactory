### Core Concept
Sets store unique, unordered items efficiently by using hashing. This enables O(1) lookup while automatically eliminating duplicates—a foundational data structure for membership testing and deduplication patterns.

### Key Mental Models
- **Uniqueness Property**: Sets automatically discard duplicate values; adding "apple" twice stores it only once
- **Hash-Based Storage**: Each element's hash determines its storage location, enabling instant lookups without scanning
- **Immutability Requirement**: Only hashable (immutable) types work in sets; mutable objects like lists can't be added
- **Unordered Nature**: Sets don't preserve insertion order—an acceptable tradeoff for speed

### Critical Patterns
- **Deduplication**: Convert lists to sets to remove duplicates instantly (`set([1, 1, 2, 2])` becomes `{1, 2}`)
- **Membership Testing**: Use sets when checking existence matters more than order
- **add() vs remove() vs discard()**: Choose based on whether missing elements should error or fail silently

### AI Collaboration Keys
- AI helps explain when uniqueness matters in your problem ("should I use a set here?")
- AI demonstrates hashability rules through error exploration

### Common Mistakes
- Using `{}` for empty set instead of `set()` (creates empty dict)
- Trying to add mutable types (lists, dicts) and getting "unhashable type" error
- Expecting set membership to preserve order

### Connections
- **Builds on**: Basic collections (lists, tuples from earlier chapters)
- **Leads to**: Set operations (Lesson 2), hashing internals (Lesson 3), frozensets (Lesson 4)
