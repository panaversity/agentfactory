### Core Concept

Generics enable writing one type-safe function that works with ANY type while preserving type information throughout—eliminating code duplication without sacrificing type safety. `get_first[T]` works for `list[int]`, `list[str]`, `list[User]` identically, yet your IDE knows the exact return type for each call, enabling autocomplete and error detection before runtime.

### Key Mental Models

**Type Variables as Placeholders**: T isn't a real type; it's a placeholder representing "whatever type the caller provides." When you call `get_first([1, 2, 3])`, Python infers T=int. When you call `get_first(["a", "b"])`, Python infers T=str. Same function, different T for each call.

**Type Information Flows**: Generics aren't about runtime behavior—they're about static analysis. Your IDE uses T to track types through function calls: if T=int for a call, then the return type is `int | None`, and the IDE knows what methods int supports.

**Generics vs Any: Preservation vs Erasure**: `Any` says "I don't care about types" (erases type information). Generics say "I preserve whatever type you give me" (maintains type information). Your IDE can help with generics but not with Any.

**Modern Syntax Simplifies Intent**: PEP 695 syntax `def func[T](...): ` declares type parameters right where they're used. Legacy `T = TypeVar('T')` with `def func(...)` separates declaration from usage. Modern syntax is clearer.

### Critical Patterns

**Type Parameter Binding**: When you declare `def get_first[T](items: list[T]) -> T | None`, the T in `list[T]` and `-> T | None` refer to the SAME type. Whatever type is in the list is what gets returned. This binding is the power—it preserves type information.

**Inference vs Explicit Specification**: Python infers T from arguments in 99% of cases. Rarely you might need explicit specification: `get_first[int]([1, 2, 3])`. Start with inference; use explicit only when type inference ambiguity exists.

**No Runtime Enforcement**: Generics don't check types at runtime. Python still executes dynamically. Generics are for static analysis tools (IDE, mypy, pyright). A function accepting `list[int]` will run with `list[str]` without crashing—generics just guide tooling.

### AI Collaboration Keys

Generics transform AI code generation from fragile to safe. When you ask AI to implement `Stack[User]`, you're specifying both the algorithm AND the type contract. AI generates code; type checkers validate that all type flows are correct before execution—reducing bugs from AI-generated code.

### Common Mistakes

**Using Any instead of Generics** throws away type information the IDE could use. Generics take one extra line but provide IDE support that saves debugging time.

**Overconstraining with unnecessary bounds** limits flexibility. If your function doesn't require special methods, don't add bounds—let T be anything.

**Thinking Generics enforce runtime types** is fundamental misunderstanding. Generics are development-time tooling, not runtime enforcement. Python remains dynamically typed.

**Mixing legacy and modern syntax** confuses readers. Pick PEP 695 exclusively—it's the future.

### Connections

Builds on: Type hints (Chapter 20), Python functions (Chapter 24), protocol concepts (Lesson 4)

Enables: Lesson 4 (generic classes), production type-safe containers (Stack, Queue, Cache), Lesson 6 (ConfigManager with generics), API frameworks with type safety
