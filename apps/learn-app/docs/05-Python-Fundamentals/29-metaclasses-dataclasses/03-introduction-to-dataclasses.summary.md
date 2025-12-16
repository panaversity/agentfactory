### Core Concept

Dataclasses are decorators that auto-generate `__init__()`, `__repr__()`, and `__eq__()` based on type-hinted fields, eliminating 50% of boilerplate in data-heavy classes. Write field names with type hints; `@dataclass` generates constructors, string representations, and equality comparisons automatically. Use `frozen=True` for immutability (prevents attribute modification, enables dict keys), `order=True` for comparison operators (`<`, `>`, `<=`, `>=`), and default values for optional fields. Type hints are mandatory—they tell the decorator which variables are fields versus class variables.

### Key Mental Models

**Boilerplate Elimination**: Instead of manual `__init__(self, x, y): self.x = x; self.y = y`, type hints declare intent and the decorator generates mechanics. **Declaration Over Implementation**: You describe what data you need; Python handles the infrastructure. **Immutability for Safety**: `frozen=True` prevents accidental mutation of configuration or shared objects. **Ordering for Sorting**: `order=True` uses lexicographic order (compare fields left-to-right).

### Critical Patterns

1. **Simple data container**: Fields with type hints, auto-generated `__init__()` and `__repr__()`
2. **Optional fields**: Default values for fields (required fields must come first)
3. **Immutable config**: `frozen=True` prevents post-creation modification
4. **Comparable objects**: `order=True` enables sorting and comparisons
5. **Key mistake**: Never use mutable defaults like `[]` or `{}` (lesson 4 covers `field(default_factory=...)`)

### AI Collaboration Keys

Ask AI: "Compare the manual Person class with @dataclass Person. What code disappeared?" This reveals boilerplate cost. Then: "What happens if I use `frozen=True`? Why would I want immutability?" Explore tradeoffs. Work together on: "Design a Product dataclass with name, price, tags. Why does order of fields matter?" Practice recognizing when dataclass vs traditional class fits better.

### Common Mistakes

1. **Mutable defaults** (`items: list = []`): All instances share same list (lesson 4 shows fix with `default_factory`)
2. **Field ordering**: Required fields must come before fields with defaults (or `__init__()` signature breaks)
3. **Type hints mandatory**: Without them, decorator doesn't know which variables are fields
4. **Frozen with mutable fields**: `frozen=True` prevents reassignment but not mutation of nested objects

### Connections

Foundation for Lesson 4 (advanced features like validation, serialization, InitVar). Contrasts with Lesson 1-2's metaclasses: dataclasses USE metaclasses internally but hide complexity, providing developer convenience. Lesson 5 shows when to choose dataclass over metaclass or traditional class. Key insight: dataclasses are about data representation; metaclasses are about class creation mechanics. Both solve different problems elegantly.
