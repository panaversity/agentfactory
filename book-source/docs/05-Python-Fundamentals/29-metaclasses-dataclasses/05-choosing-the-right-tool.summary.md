### Core Concept

Three tools solve different problems: **Metaclasses** customize how classes are created (appropriate for plugin registration, class validation, framework design—problems requiring class-creation-time intervention). **Dataclasses** represent data cleanly with type hints and auto-generated methods (appropriate for API models, configuration objects, domain models—problems requiring clean data representation). **Traditional classes** implement complex behavior and stateful operations (appropriate for business logic, rich inheritance hierarchies, objects with many methods). Success depends on recognizing problem domain and applying the right tool. Most production code uses all three.

### Key Mental Models

**Problem Domain Recognition**: Metaclasses excel at "how is this class created?" Dataclasses excel at "what data does this represent?" Traditional classes excel at "what behavior does this object perform?" **Timing Matters**: Metaclass decisions happen at class-definition time; dataclass decisions at instantiation time; traditional class methods operate at runtime. **Complexity Tradeoff**: Metaclasses are hardest to understand; dataclasses moderate; traditional classes most intuitive. Choose simplest that solves your problem.

### Critical Patterns

1. **Metaclass domain**: Plugin systems, class registries, framework field discovery, validation at definition-time
2. **Dataclass domain**: API requests/responses, configuration objects, simple domain models, data transfer objects
3. **Traditional class domain**: Business logic with rich methods, stateful objects, complex inheritance
4. **Decision matrix**: Problem → domain → tool selection with tradeoff analysis

### AI Collaboration Keys

Give AI a scenario: "I'm building a web framework where developers define handlers as classes. I want auto-registration and method validation." Ask: "Metaclass or dataclass? Explain the tradeoff." Push back: "What if I used `__init_subclass__()` instead?" Work toward architectural synthesis: "Can one project use all three?" Yes—frameworks often do. Ask AI to justify real-world choices: "Why did Django choose metaclasses for ORM models?"

### Common Mistakes

1. **Over-engineering** with metaclasses when `@dataclass` or decorator suffices
2. **Using dataclass** for behavior-heavy objects (should be traditional class)
3. **Underestimating complexity** of metaclass code (hard to debug, test, maintain)
4. **Not recognizing hybrid patterns**: Production systems often combine all three in different layers

### Connections

Synthesizes Lessons 1-4: metaclass mechanics (1), practical patterns (2), dataclass basics (3), advanced features (4). Teaches architectural decision-making essential for production code. Real-world connection: Django combines metaclasses (ORM model discovery), dataclasses (or named tuples for serialization), and traditional classes (querysets with rich behavior). Lesson 5 is about systems thinking: knowing all three tools deeply, recognizing when each applies, and designing coherent systems combining them appropriately.
