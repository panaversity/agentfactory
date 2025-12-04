### Core Concept

Design patterns are reusable solutions to architectural problems. This capstone integrates all chapter concepts (inheritance, polymorphism, composition, special methods) into four patterns. Mastery: knowing which pattern solves which problem and implementing them correctly in context.

### Key Mental Models

**Singleton**: One instance, global access. `__new__` controls creation. Problem: too many instances create inconsistent state. Solution: guarantee exactly one.

**Factory**: Decouple creation from usage. Registry maps type names to classes. Problem: hardcoded type checks everywhere. Solution: single place creates objects.

**Observer**: Event-driven communication. Problem: tight coupling (A calls B directly). Solution: A doesn't know B exists; notifies observers.

**Strategy**: Runtime algorithm selection. Problem: if-else chains for behaviors. Solution: encapsulate behaviors, switch at runtime.

### Critical Patterns

- **Singleton**: `__new__` returns cached `_instance`, `__init__` checks `_initialized`
- **Factory**: `_registry` dict maps names to classes, `create_agent()` instantiates
- **Observer**: `attach()`, `detach()`, `notify()` calls observers' `update()`
- **Strategy**: Context holds strategy, delegates to `strategy.execute()`

### AI Collaboration Keys

Implement all 4 patterns in agent system. Manager (Singleton) creates ChatBot, CodeBot, DataBot (Factory), notifies them (Observer), each uses reasoning or creative strategy (Strategy). How do they integrate?

### Common Mistakes

- Singleton with too much state (debugging nightmare)
- Factory without ABC base class (hard to enforce interface)
- Observer without notification mechanism
- Strategy without clear interface
- Not recognizing when patterns apply

### Connections

**Prerequisite**: All Lessons 1-4 (inheritance/ABC, polymorphism, composition, special methods). **Integration**: Synthesizes chapter concepts into production architecture. **Enables**: Parts 6-13 multi-agent systems using all 4 patterns.
