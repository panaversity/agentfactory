### Core Concept
Lists are modified through methods (`append()`, `extend()`, `insert()`, `remove()`, `pop()`, `clear()`) that change the original list in-place, returning `None`. The critical distinction is that append adds one item (possibly a list), while extend unpacks and adds individual items.

### Key Mental Models
- **Mutation Pattern**: All modification methods modify list in-place and return `None` (except `pop()` which returns deleted item)
- **append() vs extend()**: `append([1,2])` creates `[[1,2]]` (nested); `extend([1,2])` creates `[1, 2]` (unpacked)
- **Deletion by Value vs Index**: `remove()` deletes by value (first match only); `pop()` deletes by index and returns item
- **Error Handling**: `remove()` raises ValueError if value missing; `pop()` raises IndexError if index out of range; use defensive checks

### Critical Patterns
- **Safe Deletion**: Check existence before `remove()` with `if value in list`; use `pop()` with default: `list.pop(idx, default)`
- **List Building**: Start with `[]`, then `append()` or `extend()` items as needed during processing
- **In-Place Modification**: Understand that method calls modify original—no return value to capture

### AI Collaboration Keys
- AI clarifies `append()` vs `extend()` semantics with nested examples
- AI provides defensive code patterns: "How do I safely remove an item that might not exist?"
- AI helps debug the "but where's my new list?" problem: ".append() returns None, not the list"

### Common Mistakes
- `new_list = cart.append("item")` assigns `None` to new_list, not the updated list
- `append(["butter", "jam"])` creates nested list; should use `extend()` for unpacking
- `remove()` crashes silently when value doesn't exist; needs `if value in list` check first

### Connections
- **Builds on**: List creation (Lesson 2), indexing, slicing
- **Leads to**: Sorting/searching (Lesson 4), comprehensions (Lesson 5), iteration patterns
