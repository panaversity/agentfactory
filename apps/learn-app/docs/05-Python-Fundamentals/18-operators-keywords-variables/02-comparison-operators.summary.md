### Core Concept
Comparison operators evaluate whether a condition is true or false by asking yes/no questions about values. There are six operators (==, !=, >, <, >=, <=) that all return boolean values, enabling decision-making and validation throughout Python programs.

### Key Mental Models
- **True/False as Values**: Comparison results are boolean values (not strings), usable in further logic and assignments
- **Value vs. Type Equality**: `==` compares values regardless of type in most cases (5 == 5.0 is True), but type mismatches between different kinds (int vs. string) make values not equal
- **Assignment vs. Comparison**: Single `=` assigns; double `==` compares. Confusing them causes SyntaxError in conditionals
- **Magnitude Questions**: `>`, `<`, `>=`, `<=` all ask about ordering, with `=` in `>=` and `<=` meaning "or equal to"

### Critical Patterns
- Comparison always returns bool type: `5 > 3` returns `True` (bool), not the number 1
- Cross-type value equality: `5 == 5.0` is True (values match despite type difference), but `5 == "5"` is False (string is different type)
- Real-world validation: age checks, password length, score ranges—all use comparisons before decisions

### AI Collaboration Keys
- Ask AI about design rationale: "Why = for assignment and == for equality?" (design consistency/clarity tradeoffs)
- Collaborate on edge cases: True == 1, False == 0 (special comparison rules with booleans)
- Validate understanding: have AI explain why `if x = 5:` fails but `if x == 5:` works

### Common Mistakes
- Using `=` in conditionals instead of `==` (syntax error)
- Expecting `10 >= 10` to be False (it's True — "or equal to" is included)
- Thinking `5 == "5"` is True (type mismatch makes it False)

### Connections
- **Builds on**: Arithmetic operators (Lesson 1) provide values to compare
- **Leads to**: Logical operators (Lesson 3) combine multiple comparisons; control flow (Chapter 22) uses comparisons in if statements
