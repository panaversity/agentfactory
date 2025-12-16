### Core Concept

Three essential skills: protecting data (encapsulation), choosing the right method type (instance/class/static), and using properties for validation. Instance methods operate on object state; class methods on shared data; static methods are utility functions. Properties look like attributes but run validation code.

### Key Mental Models

**Access Control**: Public (`.name`), Protected (`._balance`), Private (`.__pin`). Properties mediate—look like attributes but execute validation. **Method Type Decision Tree**: Operates on THIS object? → Instance. On the class or creates instances? → Class method. Utility function? → Static method.

### Critical Patterns

`@property` for validated getters. `@setter` for constraints. Factory patterns with `@classmethod`. Naming: public, `_protected`, `__private`. Validation in properties prevents invalid states.

### AI Collaboration Keys

Ask why `@property` beats `get_balance()`—understand Python perspective. Design access control for sensitive data (passwords, API keys). Ask method type questions and listen to scope reasoning.

### Common Mistakes

Over-using private attributes. Getters/setters without validation. Wrong method type selection. Not realizing properties execute code—`account.balance = 5000` runs validation, it's not instant assignment.

### Connections

Builds on Lessons 2-3. Enables Capstone design. Required for Part 2 (inheritance depends on proper encapsulation). Foundational for professional Python.
