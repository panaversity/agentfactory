### Core Concept

**Composition** (has-a) is often better than inheritance (is-a) for flexible designs. Instead of Penguin *is-a* Bird with unwanted flying, Penguin *has* a Swimmer capability. Professional OOP prioritizes composition because it adapts to requirement changes better than rigid inheritance hierarchies.

### Key Mental Models

**Has-A vs Is-A**: Inheritance creates rigid classification. Composition creates flexible capability stacking (Agent *has* memory, decision-making, logging as separate objects).

**Liskov Substitution Fix**: Penguin *is-a* Bird violates LSP if Bird flies but Penguin can't. Composition fixes this: Penguin has optional Flyer (None if can't fly).

**Module Organization**: Split code across files (modules) and organize into packages (directories with __init__.py) for reuse and namespace management.

**Aggregation vs Composition**: Composition creates strong ownership (Car owns Engine). Aggregation is weak (Library references Books—books survive deletion).

### Critical Patterns

- `self.engine = Engine(200)` — Composition: Car owns Engine
- `self.departments: list[Department] = []` — Aggregation: weak references
- `class Car: def __init__(self, engine: Engine):` — Inject dependencies
- `from animals import Dog` — Module imports
- `from .submodule import function` — Package-relative imports

### AI Collaboration Keys

Refactor Bird hierarchy (Penguin, Eagle, Duck) to composition using Flyer/Swimmer capabilities. How does adding SwimmerWithDiving not break existing code?

### Common Mistakes

- Choosing inheritance when composition is more flexible
- Single-file chaos at scale (need modules)
- Circular imports (prevent with __init__.py)
- Aggregation when composition needed

### Connections

**Prerequisite**: Lesson 1-2 inheritance and polymorphism. **Alternative to**: Inheritance patterns. **Enables**: Lesson 5 agent systems (orchestrator has sub-agents).
