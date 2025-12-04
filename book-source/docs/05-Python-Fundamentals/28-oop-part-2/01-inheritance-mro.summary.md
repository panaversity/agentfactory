### Core Concept

Python's **Method Resolution Order (MRO)** determines which method executes in inheritance trees. Using `super()` correctly ensures parent initialization completes before child setup—miss this and subtle bugs cascade. Mastery: understanding that `super()` respects MRO while direct parent calls bypass it.

### Key Mental Models

**Is-a Relationship**: Dog *is-a* Animal (inheritance means specialization). Contrast with composition: Car *has-an* Engine.

**C3 Linearization**: Python resolves method lookups left-to-right, depth-first, avoiding duplicates. The diamond problem (multiple paths to same parent) breaks simple algorithms—C3 exists to solve this.

**Method Override**: Child replaces parent's method. Same name (`area()`) produces different results for Circle vs Rectangle.

### Critical Patterns

- `class Dog(Animal):` — Single inheritance syntax
- `super().__init__(args)` — Call parent constructor FIRST
- `super().method_name()` — Call parent method and extend
- `__mro__` — Inspect method resolution order when debugging
- Multiple inheritance: `class Duck(Flyer, Swimmer):` — Order matters

### AI Collaboration Keys

Ask AI to trace what happens when you forget `super().__init__()`. What attributes stay uninitialized? Why does direct parent calling (`Parent.__init__(self)`) bypass MRO?

### Common Mistakes

- Forgetting `super().__init__()` leaves parent attributes uninitialized
- Direct parent calls bypass MRO
- Diamond inheritance without understanding MRO
- Using inheritance when composition is more flexible

### Connections

**Prerequisite**: Chapter 28 OOP Part 1. **Enables**: Lesson 2 polymorphism, Lesson 4 special methods in inherited classes. **Foundation for**: Multi-agent architectures with agent type hierarchies.
