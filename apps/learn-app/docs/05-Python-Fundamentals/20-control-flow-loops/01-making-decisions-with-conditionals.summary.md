### Core Concept
Conditionals use `if`, `elif`, and `else` statements to make decisions based on conditions that evaluate to `True` or `False`. They control which code runs based on real-world circumstances your program needs to respond to.

### Key Mental Models
- **Boolean Evaluation**: Comparison operators (`==`, `!=`, `>`, `<`, etc.) return `True` or `False`, forming the foundation of decisions
- **Sequential Decision Tree**: Python evaluates conditions top-to-bottom and stops at the first match; later branches never execute once a condition is true
- **Nested Criteria**: Complex decisions often require checking one condition, then only if true, checking another condition (multi-level validation)
- **Logical Operators**: `and`, `or`, and `not` combine multiple conditions to express complex real-world rules

### Critical Patterns
- **If-Else Binary Decisions**: Choose between exactly two paths based on one condition
- **If-Elif-Else Chains**: Handle multiple possibilities by checking conditions in order until one matches
- **Nested If Statements**: Validate prerequisites at each level before proceeding deeper into decision logic
- **Complex Conditions with and/or**: Express eligibility rules combining multiple independent checks

### AI Collaboration Keys
- AI can generate multi-condition decision logic from problem descriptions; your role is validating that the conditions match your intent
- Ask AI to trace execution through your conditions with test inputs to verify logic flow
- When conditions feel convoluted, ask AI: "Can you simplify this logic?" (combining conditions often clarifies intent)

### Common Mistakes
- **Off-By-One Condition Errors**: Using `>=` when you meant `>` or vice versa (test edge cases like boundary values)
- **Unreachable Code**: Placing more general conditions before specific ones causes later branches to never execute
- **Type Mismatches**: Comparing string `"25"` with integer `18` instead of converting types first
- **Missing Indentation**: Python uses indentation to define which code belongs to conditionals; forgetting it causes `IndentationError`

### Connections
- **Builds on**: Chapter 20 (comparison operators, boolean values, logical operators)
- **Leads to**: Lesson 2 (pattern matching with `match-case`), Lesson 5 (combining conditionals with loops in nested structures)
