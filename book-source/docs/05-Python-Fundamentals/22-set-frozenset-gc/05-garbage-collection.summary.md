### Core Concept
Python's automatic memory management uses reference counting (primary) plus cycle detection (backup). When an object's refcount reaches zero, Python immediately frees it; circular references are eventually freed by garbage collection.

### Key Mental Models
- **Reference Counting**: Each object tracks "how many things reference me"; deleted at refcount = 0
- **Immediate Freeing**: No waiting—reference counting frees objects instantly when references vanish
- **Circular References**: A → B → A creates orphaned objects reference counting can't detect
- **Cycle Detector**: `gc` module periodically detects and breaks circular references

### Critical Patterns
- **Two-System Design**: Reference counting handles 95% of deletions; gc handles the 5% of circular refs
- **Refcount Mechanics**: Each new reference increases count; each deletion decreases it
- **Generational GC**: Young objects collected frequently; old objects less often
- **Memory Profiling**: Use `gc.get_objects()` to count tracked objects before/after operations

### AI Collaboration Keys
- AI walks through refcount tracking with `sys.getrefcount()` to show counts changing
- AI demonstrates circular references with `__del__` printing to show freeing occurs after `gc.collect()`

### Common Mistakes
- Forgetting that `sys.getrefcount()` returns one extra (the function arg itself)
- Assuming circular references never happen or are always freed immediately
- Not understanding that `gc.collect()` is only needed for cycles, not normal deletions

### Connections
- **Builds on**: Object creation and assignment (earlier chapters)
- **Leads to**: Memory profiler capstone applying gc to track real applications (Lesson 6)
