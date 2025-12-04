### Core Concept

Metaclasses are classes whose instances are classes themselves. Every Python class is an instance of a metaclass (default: `type`). When you write `class Dog: pass`, Python calls the `type` metaclass to create the Dog class object. This enables class creation customization: intercept class definition, validate structure, auto-register classes, or inject behavior at the framework level. Metaclasses sit at the top of Python's object hierarchy: instances → classes → metaclasses.

### Key Mental Models

**The Type Hierarchy**: Objects are instances of classes; classes are instances of metaclasses. `type(42) = int`, `type(int) = type` (type is self-referential). This reveals Python's bootstrap: everything inherits from `object`, all classes inherit from `type`. **Class Creation as Code**: When Python encounters `class MyClass: ...`, it doesn't use magic—it calls `type.__new__()` and `type.__init__()` just like any other function. You can customize this by creating custom metaclasses.

### Critical Patterns

1. **Dynamic class creation**: `type('Name', (bases,), {attributes})` creates classes at runtime (used by Django, SQLAlchemy)
2. **Custom metaclass syntax**: `class MyMeta(type): def __new__(mcs, name, bases, namespace): ...`
3. **Validation at class definition time**: Check namespace before class creation, raising errors immediately (prevents invalid classes from existing)
4. **Class-level APIs**: Methods in metaclasses become available on the class itself, not instances

### AI Collaboration Keys

Ask AI: "Why does `type(type)` return `type`?" This reveals the bootstrap paradox. Then: "Can I create classes without the `class` keyword?" Guides you to metaclass-as-factory insight. Work with AI to trace class creation flow: definition → `__new__()` → `__init__()` → class object returned. This timing distinction (definition-time vs instantiation-time) is where metaclass power lives.

### Common Mistakes

1. **Using metaclasses for instance behavior**: Metaclasses modify class creation, not instance behavior. Use `__init__()` for that
2. **Confusing `mcs` and `cls`**: In metaclass `__new__()`, first param is `mcs` (the metaclass), not `cls` (the class being created)
3. **Forgetting `super().__new__()`**: Always call the parent `__new__()` to actually create the class object
4. **Complexity trap**: Metaclasses are powerful but make code hard to understand. Always ask: "Is there a simpler approach?" first

### Connections

Prerequisite: Understand class creation, inheritance, special methods from OOP chapters. Enables: Lesson 2's practical patterns (registration, validation, singleton). Foundational for: Understanding Django ORM, SQLAlchemy declarative base, and any framework that customizes class creation. Contrasts with Lesson 3's dataclasses, which hide complexity; metaclasses expose creation mechanics for framework designers.
