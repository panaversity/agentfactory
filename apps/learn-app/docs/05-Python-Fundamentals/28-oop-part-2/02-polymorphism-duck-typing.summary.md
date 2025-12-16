### Core Concept

**Polymorphism** lets the same method call produce different behavior by object type. Design question: enforce this through inheritance contracts (ABC) or trust shared behavior (duck typing)? ABC catches errors at instantiation; duck typing at runtime.

### Key Mental Models

**Polymorphism via Inheritance**: Shape.area() means Circle.area() and Rectangle.area() override the parent. Caller doesn't know which—Python resolves by actual type.

**Duck Typing**: "If it walks like a duck and quacks like a duck, it's a duck." Objects don't inherit from Shape to work in a shapes list—they just need `area()` method.

**ABC as Contract**: `@abstractmethod` forces subclasses to implement methods. Python prevents instantiation if forgotten—catching errors early.

**Protocol vs ABC**: typing.Protocol defines structural typing (required methods) without inheritance hierarchy. More flexible than ABC.

### Critical Patterns

- `class Shape(ABC):` with `@abstractmethod def area(self): pass`
- `class Agent(ABC):` with `@abstractmethod def process()`
- Duck typing: No inheritance, implement required methods, works polymorphically
- `from abc import ABC, abstractmethod`

### AI Collaboration Keys

Compare ABC vs duck typing. Create a Payment system using ABC, then refactor with duck typing (no inheritance). When would each fail? Which feels Pythonic?

### Common Mistakes

- Using ABC when duck typing is sufficient
- Forgetting `@abstractmethod` implementation (TypeError at instantiation)
- Treating duck typing as "no contract"—interface still matters
- Confusing method name equality with true polymorphism

### Connections

**Prerequisite**: Lesson 1 inheritance and method override. **Enables**: Lesson 5 agent polymorphism with ChatAgent/CodeAgent/DataAgent inheriting from Agent ABC. **Alternative**: Lesson 3 composition as inheritance alternative.
