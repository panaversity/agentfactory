### Core Concept

Generic classes package type-safe behavior into reusable containers: `Stack[T]` or `Cache[K, V]` adapt to any data type without code duplication. Protocols define structural contracts ("acts-like" relationships) without inheritance hierarchies, enabling flexible bounds on generic type parameters. Together, they scale from simple containers to sophisticated type-safe APIs.

### Key Mental Models

**Generic Classes Encapsulate Polymorphism**: `Stack[T]` is one class that behaves differently based on T, preserving type information. Without generics, you'd write `StackInt`, `StackStr`, `StackUser`—identical implementations. With generics, you write once; the system specializes for each T.

**Multiple Type Parameters as Independent Contracts**: `Cache[K, V]` says K and V are completely independent—K could be str while V is User. Each parameter can be constrained separately: `Cache[str, T: Comparable]` means keys are strings, values must support comparison. This flexibility scales to complex data structures.

**Protocols as Structural Typing**: Instead of "B must inherit from A," Protocols say "B acts-like A if B implements these methods." This breaks the coupling inheritance creates. A type satisfies a Protocol if it has the right methods, not because it declared the inheritance. This enables retroactive typing—even external types work if they implement the protocol.

**Bounds Transform Flexibility into Safety**: Unbounded `Stack[T]` accepts anything. Bounded `Stack[T: Comparable]` restricts T to comparable types, enabling your code to use `>` safely. The bound documents what operations your code performs on T.

### Critical Patterns

**PEP 695 Generic Classes**: `class Stack[T]: ...` declares T right in the class. Old style `class Stack(Generic[T]): ...` with `T = TypeVar('T')` separates declaration. Prefer modern syntax—it's clearer and the future direction.

**Protocol as Bound**: When your function needs T to support specific methods, create a Protocol defining those methods, then bound T to the Protocol: `def find_max[T: Comparable](items: list[T])`. This is more flexible than inheritance-based constraints.

**Never Overengineering**: Not everything deserves generics. Ask: "Does this work identically for different types?" If not, don't genericize. Good candidates: containers (Stack, Queue, Cache), repositories. Bad candidates: business logic, type-specific utilities.

### AI Collaboration Keys

Generic classes serve as specifications that AI can implement correctly. `Stack[T]` spec tells AI: "Implement one Stack that preserves type T through push/pop operations." Type checkers validate AI-generated code before execution, catching type violations that would otherwise require testing.

### Common Mistakes

**Confusing Generic[T] with T**: Generic is a base class (old style). T is the type parameter. Modern syntax `class Stack[T]` handles both elegantly—don't mix styles.

**Not constraining when operations require specific methods** leads to code that looks right but fails. If you call `.upper()` on T, bound T to a Protocol with .upper().

**Overusing generics for one-type code**: If you'll only ever use `Stack[int]`, just write `IntStack`. Generics add complexity—use only when actual reuse justifies it.

**Overthinking variance** (covariance/contravariance) is premature optimization. It rarely matters in practice—learn it when you hit a specific type error that requires understanding variance.

### Connections

Builds on: Lesson 3 (generics overview), Lesson 1 (Pydantic validation), classes (Chapter 28-30), type hints (Chapter 20)

Enables: Lesson 6 (generic ConfigManager), production-grade type-safe containers, advanced architectural patterns, frameworks (FastAPI, SQLAlchemy)
