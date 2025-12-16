### Core Concept
CRUD operations on dicts: Create (literal or assignment), Read (bracket or `.get()`), Update (assignment to existing key), Delete (using `del` for certain keys or `.pop(key, default)` for safe removal). The `in` operator checks key existence before accessing.

### Key Mental Models
- **Add/Update Duality**: Same syntax `dict[key] = value` adds new or updates existing—Python doesn't distinguish
- **Deletion Strategies**: `del dict[key]` for certain keys (crashes if missing); `.pop(key, default)` for uncertain keys (safe fallback)
- **Key Existence Pattern**: `if key in dict:` before access prevents KeyError when uncertain
- **Clear vs Delete**: `.clear()` empties entire dict; `del dict[key]` removes specific key; `pop()` removes and returns value

### Critical Patterns
- **Safe Addition**: Assignment works but consider `if key not in dict:` for conditional additions
- **Safe Deletion**: Check before `del`: `if "bananas" in inventory: del inventory["bananas"]`
- **Pop for Handlers**: `quantity = inventory.pop("bananas", 0)` handles missing gracefully with default

### AI Collaboration Keys
- AI provides defensive code patterns: "How do I safely delete a key that might not exist?"
- AI clarifies del vs pop semantics with practical scenarios
- AI helps debug "key not found" errors

### Common Mistakes
- `del dict[key]` crashes if key missing; should use `.pop(key, default)` or check first
- Forgetting `.pop()` returns the deleted value; capturing important
- Overwriting values unintentionally with duplicate key assignment

### Connections
- **Builds on**: Dict basics (Lesson 7), understanding mutations
- **Leads to**: Dict iteration (Lesson 9), data processing (Lesson 11)
