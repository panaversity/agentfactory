### Core Concept
Sorting uses `.sort()` (modifies in-place, returns `None`) or `sorted()` function (returns new list, preserves original). Reversing uses `.reverse()` (modifies in-place) or `[::-1]` slice (creates new). Querying uses `.count()` and `.index()` to find occurrence count and first position.

### Key Mental Models
- **Method vs Function Pattern**: `.sort()` modifies and returns `None`; `sorted()` preserves original and returns new list
- **Search Semantics**: `.index()` returns first position or raises `ValueError`; `.count()` returns 0 if not found (safe)
- **Aliasing Hazard**: Simple assignment creates alias; `.copy()` required for independent copy to prevent unintended modifications
- **Reversal Options**: `.reverse()` is direct but destructive; `[::-1]` is functional but less obvious at first glance

### Critical Patterns
- **Sort Pattern Selection**: Use `.sort()` when okay to modify; use `sorted()` for pipeline operations needing original
- **Safe Index Finding**: Check `if value in list` before calling `.index()` to avoid ValueError
- **Copy for Safety**: Always `.copy()` before passing list to untrusted code that might modify it

### AI Collaboration Keys
- AI clarifies method/function distinctions with side-effect examples
- AI explains when O(n) list search vs O(1) dict lookup matters for performance
- AI designs defensive code: "How do I safely find and remove an item?"

### Common Mistakes
- `new_list = scores.sort()` assigns `None` to new_list, not sorted scores
- Using `.index()` without checking if value exists first
- Assignment creates alias: `backup = scores` doesn't protect original from mutations

### Connections
- **Builds on**: List creation (Lesson 2), modification methods (Lesson 3)
- **Leads to**: Comprehensions (Lesson 5), performance awareness (Lesson 10)
