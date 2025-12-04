### Core Concept
Type hints are optional syntax annotations declaring what types functions expect, enabling Pyright (static type checker) to catch 30–50% of bugs without running code—shifting error detection from "runtime crash" to "development time," before code reaches users or production.

### Key Mental Models
- **Static vs. Dynamic Analysis**: Python executes dynamically (types change at runtime), but type hints enable static pre-execution checking—catching mismatches before running, not after crashes
- **Python 3.13+ Modern Syntax**: `str | None` (union type) replaces older `Union[str, None]`; cleaner, more readable, same semantics
- **Type Hint Optionality**: Hints don't enforce at runtime (Python ignores them); they're metadata for tools like Pyright, making them safe to add gradually
- **Type Hierarchy Matching**: Functions with typed parameters validate callers at call site; return types validate function implementation—bidirectional correctness checking

### Critical Patterns
- **Essential Type Annotations**: Function parameters and return types are minimum useful coverage; enable rapid bug detection for most common errors
- **Optional Type Checking**: `str | None` requires null-check before use (prevents crashes); Pyright enforces this discipline, catching defensive coding violations

### AI Collaboration Keys
- AI generates type hints and interprets Pyright error messages without requiring deep type system knowledge
- AI explains why specific types are required, building understanding through examples

### Common Mistakes
- Assuming type errors mean code is broken (Pyright is conservative; many are safe with runtime checks)
- Using older syntax (`Union`, `Optional`) instead of Python 3.13+ (`|`)
- Ignoring type errors because code "works" (catches bugs before production)

### Connections
- **Builds on**: Ruff linting, understanding Python functions (taught later in chapter series), project structure
- **Leads to**: Production-quality code, advanced types (generics, protocols), test-driven development with confidence
