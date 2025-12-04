### Core Concept

Advanced dataclass features enable production-grade data models: `field()` provides fine control (mutable defaults via `default_factory`, metadata for validation hints, `init`/`repr`/`compare` flags), `__post_init__()` runs after construction for validation and computed fields, `InitVar` passes temporary data to `__post_init__()` without storing it, and serialization patterns (`.asdict()`, JSON conversion) integrate dataclasses with APIs. These tools prevent invalid states at creation time and provide hooks for custom behavior while maintaining dataclass cleanliness.

### Key Mental Models

**Mutable Default Factory**: Every instance needs its own list/dict copy. `default_factory=list` creates a new list per instance (vs shared `[]`). **Post-Init Validation**: Checks happen after construction, preventing invalid instances from existing. **Metadata-Driven Validation**: Attach validation rules to fields; custom validators read metadata at runtime. **Immutable Initialization**: `InitVar` parameters appear in `__init__()` signature but aren't stored (useful for password hashing: receive plaintext, store hash only).

### Critical Patterns

1. **Mutable defaults**: `field(default_factory=list)` for lists/dicts
2. **Field customization**: `field(repr=False)` for secrets, `field(init=False)` for computed fields
3. **Validation in `__post_init__()`**: Raise `ValueError` for invalid states
4. **InitVar for temporary data**: Pass data to post-init without storing
5. **Serialization**: `asdict(instance)` → dict, then `json.dumps()`

### AI Collaboration Keys

Challenge AI: "Design a User dataclass with password. User receives plain password in `__init__()`, but only the hash is stored. What pattern enables this?" Answer: InitVar. Then work on: "Add email validation to `__post_init__()`. What error should invalid emails raise?" Explore tradeoffs between validation strictness and usability. Ask: "How would you serialize this dataclass to JSON without exposing password_hash?"

### Common Mistakes

1. **Forgetting `default_factory` for mutables**: Causes all instances to share same list/dict
2. **Field ordering**: Required fields still must precede defaults (even with `field()`)
3. **Nested frozen dataclasses with mutable fields**: Frozen prevents reassignment but not mutation of nested objects
4. **Over-validating in `__post_init__()`**: Too strict validation causes legitimate data to be rejected

### Connections

Builds on Lesson 3's basics. Shows how dataclasses handle complexity rivaling traditional classes. Lesson 1-2 (metaclasses) revealed *how* Python creates classes; Lesson 4 shows what you can do *with* that mechanism via dataclass decorators. Lesson 5 uses these patterns to decide: traditional class vs dataclass vs metaclass. Essential for AI-native development: dataclasses declare data structure intent; AI validates and serializes; you focus on business logic.
