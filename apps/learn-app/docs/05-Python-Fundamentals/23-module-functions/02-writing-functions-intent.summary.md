### Core Concept
Functions are reusable blocks of code with clear intent communicated through type hints and docstrings. Rather than writing the same calculation repeatedly, write it once as a function with type hints that specify what data it accepts and returns. Type hints are your specification—they encode intent before implementation.

### Key Mental Models
- **Intent Before Implementation**: Type hints and docstrings tell you WHAT a function does before looking at HOW. This separates semantics (meaning) from syntax (code).
- **Type Hints as Contract**: `def add(a: int, b: int) -> int:` makes a promise: "Give me two integers, I return one integer." This contract enables AI validation and human understanding.
- **Docstring as Documentation**: Docstrings explain purpose, parameters, and return value for the USER of the function, not the implementer.
- **Parameters as Data**: Function parameters are named slots for incoming data. Return values are outgoing data.

### Critical Patterns
- **Function Signature Primacy**: Design the signature (`def`, parameters, type hints, docstring) before writing the body.
- **Type Hints on Complex Types**: `list[float]`, `dict[str, int]`, `tuple[int, int]` make intent explicit about data structures.
- **Default Parameters**: `uppercase: bool = False` makes parameters optional with sensible defaults.
- **Multiple Returns as Tuple**: Return structured data via tuples with unpacking: `x, y = get_coordinates()`.
- **Predicates (Boolean Returns)**: Functions answering yes/no return `bool`: `is_even()`, `is_within_range()`.

### AI Collaboration Keys
AI can generate function bodies from clear specifications (signatures + docstrings). The workflow: you write the specification, AI implements it, you validate. Better specifications produce better AI output.

### Common Mistakes
- Mixing responsibilities (function does multiple unrelated things)
- Vague type hints or missing docstrings (AI can't help if intent is unclear)
- Side effects (printing, modifying global state) mixed with computation logic

### Connections
- **Builds on**: Lesson 1 (module imports), Chapter 19 (type hints), Chapter 23 (collections)
- **Leads to**: Lesson 3 (parameters and returns), understanding function composition
