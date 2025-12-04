### Core Concept
Multi-module projects organize code through separation of concerns: each module has one clear responsibility. A calculator with operations.py (pure math), utils.py (I/O), main.py (orchestration), and test_calculator.py (validation) demonstrates how professional Python scales. This pattern makes code testable, reusable, and maintainable.

### Key Mental Models
- **Separation of Concerns**: Each module owns one responsibility. Operations does math. Utils handles user interaction. Main coordinates them.
- **Pure Functions**: operations.py contains functions with no side effects—no printing, no global state, just input → computation → output.
- **Module Imports**: main.py imports custom modules like built-in modules: `import operations`, then calls `operations.add()`.
- **Testing Isolation**: test_calculator.py tests operations independently using `assert`, validating that functions work before integration.
- **If __name__ == "__main__"**: Pattern distinguishing direct execution from importing. Allows module to be imported without running its code.

### Critical Patterns
- **Operations Module**: Pure functions with type hints. Return `Type | None` for operations that might fail (division by zero, sqrt of negative).
- **Utils Module**: I/O functions (input, validation, display). Returns `Type | None` for invalid input.
- **Main Orchestration**: `import` modules, call functions conditionally (if/elif for menu choices), handle None returns.
- **Test Functions**: Each test function checks one operation with `assert`, covering normal, edge, and error cases.
- **Module Extensibility**: Adding new operation requires changes to three files: operations.py (implementation), main.py (menu), test_calculator.py (tests).

### AI Collaboration Keys
AI can generate operations.py, utils.py, and main.py from specifications of what each module should do. AI especially helps with test_calculator.py—generating test cases for edge cases you might miss.

### Common Mistakes
- Mixing I/O and math in one module (hard to test math separately)
- Not testing functions individually (bugs found late)
- Returning raw values without checking for error cases (crashes on edge cases)
- Importing with wrong paths (all modules must be in same directory or proper package structure)

### Connections
- **Builds on**: Lessons 1-4 (all module and function concepts), Chapters 18-23 (data types, operations)
- **Leads to**: Chapter 25+ (exception handling for better error management, OOP for larger systems)
