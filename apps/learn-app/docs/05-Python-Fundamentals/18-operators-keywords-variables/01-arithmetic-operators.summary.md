### Core Concept
Arithmetic operators are symbols that perform mathematical operations on numbers. Python has seven operators (+, -, *, /, //, %, **) that follow familiar math rules but with critical type implications: division always returns float even with integer operands, while floor division returns int.

### Key Mental Models
- **Type Coercion Rule**: When mixing int and float, the result is always float (float "wins" in type resolution)
- **Division Design**: Python chose two division operators (/ for float results, // for integer results) to handle different use cases explicitly rather than implicitly converting
- **Operator Precedence**: PEMDAS applies, but explicit parentheses clarify intent and are considered best practice even when not required
- **Operand → Operation → Type**: Each operator takes two operands, performs calculation, and returns specific type determined by both the operator and operand types

### Critical Patterns
- Division gotcha: `10 / 5` returns `2.0` (float), not `2` (int). Use `//` for integer results
- Modulo for cycling: `7 % 3 = 1`, `8 % 3 = 2`, `9 % 3 = 0` (repeats) — useful for checking even/odd or cycling through ranges
- Type verification: Always use `type()` to validate arithmetic operation results, especially division

### AI Collaboration Keys
- Ask AI about design choices ("Why did Python create both / and //?") rather than just operator mechanics
- Collaborate on edge cases: ask AI what happens with 0.1 + 0.2 (floating-point precision) and why
- Validate operator precedence confusion by having AI trace complex expressions step-by-step

### Common Mistakes
- Assuming `10 / 5` gives integer 2 when it actually gives float 2.0
- Forgetting that `int + float` automatically converts result to float
- Using ^ for exponentiation (it's XOR); must use ** for powers

### Connections
- **Builds on**: Basic number concepts from Chapter 18 (int, float types)
- **Leads to**: Comparison operators (Lesson 2) which evaluate arithmetic expressions, assignment operators (Lesson 4) which modify variables using arithmetic
