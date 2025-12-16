### Core Concept
Lists are created using literals (`[1, 2, 3]`) or the `list()` constructor, support zero-based indexing (positive and negative), and can be sliced to extract subsequences. Type hints like `list[int]` communicate data intent and enable tool support.

### Key Mental Models
- **Zero-Based Indexing**: First item is index 0; negative indices count backward from -1 (last item)
- **Slicing Syntax**: `list[start:stop:step]` where stop is exclusive; `[::-1]` reverses elegantly
- **Aliasing vs Copying**: Assignment (`list2 = list1`) creates an alias (same object); `.copy()` creates independent copy
- **Homogeneous Type Hints**: `list[int]` signals "all items are integers"; mixed types use `list[int | str]`

### Critical Patterns
- **Index Selection**: Use `list[0]` for first, `list[-1]` for last, avoiding the need to know length
- **Slice Patterns**: `[:3]` gets first three, `[1:4:2]` gets every other from 1-4, `[-3:]` gets last three
- **Copy Pattern**: Always use `.copy()` to create independent copy when needed for preservation

### AI Collaboration Keys
- AI explains indexing behavior: "Why does index 10 crash on a 4-item list?"
- AI helps debug aliasing: "Show me why both variables changed when I modified one"
- AI generates slice patterns for data extraction: "Get every 10th item from a 1000-item list"

### Common Mistakes
- Confusing negative indices: `-1` is last, not "item before first"
- Forgetting that `[:3]` returns 3 items (indices 0, 1, 2), not up to index 3
- Creating aliases by mistake: `copy = original` doesn't create independent copy without `.copy()`

### Connections
- **Builds on**: Basic data types, variables, type hints
- **Leads to**: List modification methods (Lesson 3), sorting/searching (Lesson 4), comprehensions (Lesson 5)
