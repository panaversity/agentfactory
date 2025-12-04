### Core Concept
Structure selection answers four questions: (1) Does data change? (2) Does order matter? (3) How do I find items? (4) What intent does structure communicate? Lists serve ordered mutable sequences; tuples preserve fixed data and enable dict keys; dicts provide O(1) lookup by meaningful identifier.

### Key Mental Models
- **Mutability Question**: Changing data → list/dict; fixed data → tuple
- **Ordering Question**: Order matters → list/tuple; order irrelevant → dict
- **Lookup Performance**: Position-based searching O(n) with lists; key-based O(1) with dicts (critical for 10K+ items)
- **Semantic Communication**: Structure choice signals intent—tuple means "unchangeable", list means "evolving", dict means "keyed lookup"

### Critical Patterns
- **Decision Tree**: Need mutability? → list/dict. Unchangeable? → tuple. Fast lookups? → dict. Ordering matters? → list/tuple.
- **Nested Structures**: `dict[int, list[str]]` combines keys→lists, `dict[str, dict[str, int]]` nests dicts for hierarchical data
- **Anti-Pattern Recognition**: Lists for frequent ID lookups (slow); dicts for ordered sequences (loses order intent)

### AI Collaboration Keys
- AI analyzes requirements: "Which structure fits: user by ID lookup, task queue, or game coordinates?"
- AI demonstrates O(1) vs O(n) performance differences with timing tests
- AI validates architecture: "Is my nested dict structure appropriate for this data?"

### Common Mistakes
- Using list for 10K-item ID lookups (O(n) search crushing performance)
- Dict for data requiring positional access or sorting
- Over-complicating with nested structures when simpler alternative exists

### Connections
- **Builds on**: All prior lessons (synthesis focus, no new concepts)
- **Leads to**: Capstone data processing (Lesson 11), production code design
