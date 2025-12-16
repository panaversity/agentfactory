### Core Concept
A memory profiler capstone integrates all chapter concepts: using sets to track object IDs, frozensets for immutable categorization, and the gc module for memory analysis. This synthesizes individual lessons into a professional-grade diagnostic tool.

### Key Mental Models
- **Specification-First Design**: Plan tool requirements before writing code; design with AI collaboration
- **Set-Based Tracking**: Use `set[int]` for O(1) object ID membership testing and deduplication
- **Frozenset Categorization**: Use `dict[frozenset[str], set[int]]` to group objects by immutable type
- **GC Integration**: Apply `gc.collect()` and `gc.get_objects()` to analyze real memory state

### Critical Patterns
- **Two-Tier Profiling**: Reference counting tracks most deletions; cycle detector handles circular refs
- **Edge Case Testing**: Validate circular references (A→B, B→A), large graphs (1000+ objects), mixed types
- **Memory Measurement**: Track bytes before/after using `sys.getsizeof()` aggregated across object graphs
- **Professional Patterns**: Docstrings, type hints, assertions, and measurable test outcomes

### AI Collaboration Keys
- AI helps refine specification by asking clarifying questions about requirements
- AI generates core profiler class; student validates through testing
- AI suggests edge cases and design patterns for robustness

### Common Mistakes
- Skipping specification and diving directly into implementation
- Not testing circular references (appear to work until gc limitations surface)
- Forgetting that frozensets as dict keys solve "we need to group by multiple attributes" patterns

### Connections
- **Builds on**: All Lessons 1-5 (sets, operations, hashing, frozensets, garbage collection)
- **Integration Point**: Demonstrates spec-driven development with AI at Layer 4 (capstone orchestration)
