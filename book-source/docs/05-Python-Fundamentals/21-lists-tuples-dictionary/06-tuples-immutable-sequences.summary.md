### Core Concept
Tuples are immutable sequences created with parentheses `(1, 2, 3)` that cannot be modified after creation. Immutability enables two critical features: using tuples as dictionary keys (only hashable types allowed) and unpacking values into multiple variables simultaneously.

### Key Mental Models
- **Immutability Guarantee**: Attempting modification raises `TypeError`; this is a feature signaling "this data won't change"
- **Hashability Requirement**: Tuples can be dict keys because their hash never changes; lists cannot because they're mutable
- **Unpacking Pattern**: `x, y = (10, 20)` assigns tuple values to separate variables; this works seamlessly with function returns
- **Type Hint Variants**: Fixed-size `tuple[int, int]` for pairs; variable-length `tuple[int, ...]` for unknown-size sequences

### Critical Patterns
- **Single-Element Syntax**: `(1,)` is tuple; `(1)` is just integer—trailing comma required
- **Dict Key Usage**: `locations: dict[tuple[int, int], str] = {(0,0): "Start", (10,20): "Forest"}`
- **Unpacking with Ignore**: `x, y, _ = coordinates` ignores third value using underscore placeholder

### AI Collaboration Keys
- AI explains when immutability matters: "Why use tuple instead of list for coordinates?"
- AI designs unpacking patterns for function returns
- AI clarifies hashability: "Why can I use tuples as dict keys but not lists?"

### Common Mistakes
- Forgetting trailing comma for single-element tuple: `(42)` is int, not tuple
- Trying to modify tuple: `t[0] = 5` raises TypeError
- Using list as dict key: Python prevents this with `TypeError: unhashable type`

### Connections
- **Builds on**: Lists (Lessons 2-5), understanding immutability concept
- **Leads to**: Dictionaries (Lessons 7-9), function design with multiple returns
