### Core Concept
Logical operators combine multiple True/False conditions into single boolean results. The three operators (and, or, not) create complex decision logic: `and` requires both true, `or` requires at least one true, `not` reverses the value. These form the foundation for sophisticated conditional reasoning.

### Key Mental Models
- **AND = Both Required**: Returns True only when both conditions are True; False if either is False
- **OR = At Least One**: Returns True if any condition is True; False only when both are False
- **NOT = Reversal**: Flips True to False and False to True; simplest operator
- **Short-Circuit Evaluation**: Python stops evaluating once result is determined (False for `and`, True for `or`) — an optimization that affects code behavior
- **Operator Precedence**: `not` > `and` > `or`; parentheses clarify and override default order

### Critical Patterns
- Range checking: `(x > 5) and (x < 10)` checks if x is between 5 and 10
- Permission logic: `(is_admin or is_moderator) and account_active` — user must be admin/moderator AND account must be active
- De Morgan's Laws: `not (a and b)` equals `(not a) or (not b)` — logically equivalent expressions that help simplify complex conditions

### AI Collaboration Keys
- Ask AI about short-circuit evaluation: "Why does Python stop after first False in `and`?" (performance + side effects)
- Collaborate on complex precedence: trace `True or False and False` step-by-step with AI
- Validate operator choice: "Should I use `and` or `or` here?" with your actual business logic

### Common Mistakes
- Confusing AND/OR: writing `and` when `or` needed (breaks permission logic)
- Forgetting parentheses with mixed operators: `True or False and False` needs explicit grouping to be clear
- Using `=` instead of `==` inside logical expressions: `if x > 5 and y = 3:` fails

### Connections
- **Builds on**: Comparison operators (Lesson 2) provide boolean values to combine
- **Leads to**: Control flow (Chapter 22) uses logical operators in complex if/elif conditions
