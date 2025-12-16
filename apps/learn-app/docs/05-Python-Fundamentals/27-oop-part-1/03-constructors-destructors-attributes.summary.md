### Core Concept

Constructors with defaults and the class vs instance attribute distinction determine data scope. Instance attributes (unique per object) vs class attributes (shared by all). Mutable default parameters create a classic gotcha—all instances accidentally share the same list.

### Key Mental Models

**Default Strategy**: Required parameters first (name, email), optional last (role, status). **Class vs Instance**: "Each dog has its own name" (instance) vs "all dogs are canines" (class/shared). **Shadowing Trap**: Creating instance attribute with class attribute name hides the class version temporarily. Use `ClassName.attribute = value` to modify shared data, not `obj.attribute = value`.

### Critical Patterns

Type hints with defaults. Protected `_attribute` naming. Access class attributes via `ClassName.attribute`. Fix mutable defaults with `None` → empty container pattern. Use `hasattr()` and `__dict__` to inspect attribute ownership.

### AI Collaboration Keys

Ask AI to visualize memory layout differences. Challenge it with mutable default scenarios. When designing configuration-heavy classes, ask whether settings should be class or instance attributes.

### Common Mistakes

Mutable defaults (`items: list = []`). Shadowing class attributes by accident. Putting all data in class attributes when they should be instance. Forgetting validation in `__init__`.

### Connections

Extends Lesson 2. Enables Lesson 4 (encapsulation). Required for Capstone (proper defaults and scope). Prerequisite for Part 2 (inheritance and hierarchies).
