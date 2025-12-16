### Core Concept

Metaclasses solve five real-world problems: **validation** (enforce required class attributes at definition time), **registration** (auto-populate central registry when subclasses are defined), **singleton** (ensure one instance globally), **abstract enforcement** (validate required methods exist), and **field collection** (Django pattern: discover field definitions and generate functionality). Each pattern customizes class creation to reduce boilerplate or enforce constraints impossible with regular code. Choice between metaclass, `__init_subclass__()`, and decorators depends on whether you need class-creation-time customization or simpler post-creation modification.

### Key Mental Models

**Timing is everything**: Metaclass `__new__()` executes at class definition time (when you write `class MyClass:`), not at instantiation time. This enables registration and validation before any code uses the class. **Registry Pattern**: A class-level dictionary stores all subclasses automatically. **Validation Before Use**: Invalid classes never exist; errors appear at definition time, not deep in runtime. **Plugin Architecture**: Each plugin automatically registers itself—no manual bookkeeping.

### Critical Patterns

1. **Attribute validation**: Check `namespace` has required fields, raise error if missing
2. **Class registration**: Store `cls` in dictionary keyed by name or category
3. **Singleton enforcement**: Cache instances, return same object for repeated instantiations
4. **Framework field discovery**: Iterate namespace finding Field instances, collect metadata
5. **Alternative (`__init_subclass__()`)**: Simpler than metaclass for validation-only scenarios

### AI Collaboration Keys

Challenge AI with: "Show me plugin registration both ways: manual decorator vs metaclass. Which would you use for a production framework with 100+ plugins?" This forces AI to weigh complexity vs automation benefit. Then ask about `__init_subclass__()` as alternative: "When is it enough?" Converge on: "Metaclass for frameworks, `__init_subclass__()` for app code."

### Common Mistakes

1. **Over-engineering**: Using metaclass when `@decorator` or `__init_subclass__()` suffices
2. **Inheritance gotchas**: Forgetting subclasses inherit parent's metaclass (can complicate MRO)
3. **Performance overhead**: Metaclass adds cost to every class definition
4. **Testing difficulty**: Metaclass behavior is hard to mock; testing becomes complex

### Connections

Builds on Lesson 1's class creation flow. Shows why Lesson 3's dataclasses work: they use metaclasses internally but hide the complexity. Real-world equivalent: Django ORM uses ModelBase metaclass to turn field definitions into database schema. SQLAlchemy uses DeclarativeMeta to build SQL relationships. Leads to Lesson 5: deciding when metaclasses are justified vs when simpler approaches (dataclasses, decorators) are better.
