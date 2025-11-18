---
sidebar_position: 5
title: "Lesson 5: Building a Calculator Utility (Capstone Project)"
description: "Integrate modules, functions, parameters, returns, and scope into a real multi-module project"
keywords: ["capstone", "modules", "multi-file", "separation of concerns", "testing", "project organization"]
cefr_level: "B1-B2"
estimated_time: "70 minutes"
proficiency_by_end: "Students complete a working multi-module calculator project with clear separation of concerns"
prerequisites: ["Lessons 1-4: All module and function concepts", "Chapter 14-18: Data types and operations"]
learning_objectives:
  - "Organize code into multiple modules with clear responsibilities"
  - "Import and use custom functions from self-written modules"
  - "Design functions that work together in a larger system"
  - "Test functions to validate they work correctly"
  - "Understand separation of concerns principle in code organization"
ai_native_learning: true
try_with_ai_prompts: 4
colearning_elements: ["conceptual_prompt", "teaching_commentary", "specification_challenge", "ai_tip"]
---

## Lesson 5: Building a Calculator Utility (Capstone Project)

**CEFR Level**: B1-B2 (Integrated Application and Synthesis)
**Time Estimate**: 70 minutes
**What You'll Learn**: You'll build a complete Python project using modules and functions. This capstone integrates everything from Lessons 1-4: module organization, function design, parameter patterns, returns, scope, and testing. You'll see professional code organization patterns.

---

## Capstone Project Overview

**The Task**: Build a multi-module calculator application that demonstrates:
- **Separation of concerns**: Different files handle different responsibilities
- **Module imports**: Main program imports and uses custom modules
- **Function specifications**: Clear type hints and docstrings
- **Parameter patterns**: Functions with required and optional parameters
- **Return values**: Functions returning single values, tuples, and optional results
- **Testing**: Validating that functions work correctly

**Project Structure**:
```
calculator/
├── operations.py      # Mathematical operations
├── utils.py          # Utility functions (I/O, validation, display)
├── main.py           # Main program orchestrating the calculator
└── test_calculator.py # Tests for the operations
```

### 💬 AI Colearning Prompt

> Ask your AI: "I'm building a calculator project with multiple modules. What's the best way to organize this? Should operations be in one file, utilities in another, and main orchestrate them?"

**Expected Understanding**: You see that module organization is a design choice with clear benefits: testability, reusability, clarity.

### 🎓 Instructor Commentary

You just built a professional Python project. Notice: each file has a clear purpose. Operations do math. Utils handle I/O. Main orchestrates. This is the separation of concerns principle. It makes code easier to test, modify, and reuse. When your projects grow, this organization scales. A 100-line script can be in one file. A 10,000-line application needs modular organization. You're learning that pattern now, at scale you can handle.

---

## Step 1: Create `operations.py` — Mathematical Operations

This module contains pure functions that perform calculations. No side effects (no printing, no modifying global state).

### 💻 Code Idea: Operations Module

```python
# File: operations.py
"""
Calculator operations module.

Contains pure mathematical functions for basic calculations.
All functions have clear type hints and docstrings.
"""

def add(a: float, b: float) -> float:
    """Return the sum of two numbers."""
    return a + b


def subtract(a: float, b: float) -> float:
    """Return the difference of two numbers (a - b)."""
    return a - b


def multiply(a: float, b: float) -> float:
    """Return the product of two numbers."""
    return a * b


def divide(a: float, b: float) -> float | None:
    """
    Divide a by b.

    Parameters:
        a (float): Dividend
        b (float): Divisor

    Returns:
        float | None: The quotient if b != 0, None if division by zero
    """
    if b == 0:
        return None
    return a / b


def power(base: float, exponent: float) -> float:
    """Return base raised to the exponent."""
    return base ** exponent


def square_root(number: float) -> float | None:
    """
    Calculate square root.

    Parameters:
        number (float): The number to take square root of

    Returns:
        float | None: The square root if number >= 0, None otherwise
    """
    if number < 0:
        return None
    return number ** 0.5
```

**Design Choices**:
- All functions are pure (no side effects)
- Type hints are explicit
- Functions that can fail return `Type | None`
- Docstrings are minimal but complete

---

## Step 2: Create `utils.py` — Utility Functions

This module handles user interaction, input validation, and output formatting.

### 💻 Code Idea: Utilities Module

```python
# File: utils.py
"""
Utility functions for calculator.

Contains display, input handling, and validation functions.
These functions handle I/O while operations.py handles math.
"""

def display_menu() -> None:
    """Display the calculator menu."""
    print("\n" + "=" * 25)
    print("    Simple Calculator")
    print("=" * 25)
    print("1. Add")
    print("2. Subtract")
    print("3. Multiply")
    print("4. Divide")
    print("5. Power")
    print("6. Square Root")
    print("7. Exit")
    print("=" * 25)


def get_operation_choice() -> str:
    """Get user's operation choice."""
    choice: str = input("Choose operation (1-7): ").strip()
    return choice


def get_numbers() -> tuple[float, float] | None:
    """
    Get two numbers from user.

    Returns:
        tuple[float, float] | None: (num1, num2) if valid, None if invalid
    """
    try:
        num1: float = float(input("Enter first number: "))
        num2: float = float(input("Enter second number: "))
        return num1, num2
    except ValueError:
        print("Invalid input. Please enter numbers.")
        return None


def get_single_number() -> float | None:
    """
    Get a single number from user (for square root).

    Returns:
        float | None: The number if valid, None if invalid
    """
    try:
        number: float = float(input("Enter a number: "))
        return number
    except ValueError:
        print("Invalid input. Please enter a number.")
        return None


def display_result(operation: str, result: float | None) -> None:
    """
    Display the calculation result.

    Parameters:
        operation (str): Name of the operation performed
        result (float | None): Result, or None if operation failed
    """
    if result is None:
        print("Calculation not possible (e.g., division by zero, sqrt of negative)")
    else:
        print(f"Result of {operation}: {result:.2f}")
```

**Design Choices**:
- Input functions handle validation
- Return `Type | None` for operations that might fail
- Display functions have no return value (`-> None`)
- Clear separation: utils handles I/O, operations does math

---

## Step 3: Create `main.py` — Main Program

This file orchestrates the calculator by importing and using the other modules.

### 💻 Code Idea: Main Program

```python
# File: main.py
"""
Main calculator program.

Imports operations and utils modules, orchestrates the calculator flow.
This file demonstrates module imports and program structure.
"""

import operations
import utils


def run_calculator() -> None:
    """Run the calculator in a loop until user exits."""
    while True:
        utils.display_menu()
        choice: str = utils.get_operation_choice()

        if choice == "1":
            numbers = utils.get_numbers()
            if numbers:
                num1, num2 = numbers
                result = operations.add(num1, num2)
                utils.display_result("Addition", result)

        elif choice == "2":
            numbers = utils.get_numbers()
            if numbers:
                num1, num2 = numbers
                result = operations.subtract(num1, num2)
                utils.display_result("Subtraction", result)

        elif choice == "3":
            numbers = utils.get_numbers()
            if numbers:
                num1, num2 = numbers
                result = operations.multiply(num1, num2)
                utils.display_result("Multiplication", result)

        elif choice == "4":
            numbers = utils.get_numbers()
            if numbers:
                num1, num2 = numbers
                result = operations.divide(num1, num2)
                utils.display_result("Division", result)

        elif choice == "5":
            numbers = utils.get_numbers()
            if numbers:
                num1, num2 = numbers
                result = operations.power(num1, num2)
                utils.display_result("Power", result)

        elif choice == "6":
            number = utils.get_single_number()
            if number is not None:
                result = operations.square_root(number)
                utils.display_result("Square Root", result)

        elif choice == "7":
            print("Thank you for using Calculator. Goodbye!")
            break

        else:
            print("Invalid choice. Please try again.")


if __name__ == "__main__":
    run_calculator()
```

**Key Patterns**:
- `if __name__ == "__main__"`: Program only runs if executed directly (not imported)
- `import operations` and `import utils`: Imports custom modules
- Function calls like `operations.add()` show module.function pattern
- Logic is clear: display menu, get input, call operation, display result

---

## Step 4: Create `test_calculator.py` — Testing

This file validates that functions in `operations.py` work correctly.

### 💻 Code Idea: Test Module

```python
# File: test_calculator.py
"""
Test the calculator operations.

Run with: python test_calculator.py
Or: python -m pytest test_calculator.py
"""

import operations


def test_add():
    """Test addition operation."""
    assert operations.add(2, 3) == 5
    assert operations.add(-1, 1) == 0
    assert operations.add(0, 0) == 0
    print("✓ test_add PASSED")


def test_subtract():
    """Test subtraction operation."""
    assert operations.subtract(5, 3) == 2
    assert operations.subtract(0, 5) == -5
    assert operations.subtract(10, 10) == 0
    print("✓ test_subtract PASSED")


def test_multiply():
    """Test multiplication operation."""
    assert operations.multiply(3, 4) == 12
    assert operations.multiply(-2, 3) == -6
    assert operations.multiply(0, 100) == 0
    print("✓ test_multiply PASSED")


def test_divide():
    """Test division operation."""
    assert operations.divide(10, 2) == 5.0
    assert operations.divide(10, 0) is None  # Division by zero
    assert operations.divide(-8, 2) == -4.0
    print("✓ test_divide PASSED")


def test_power():
    """Test power operation."""
    assert operations.power(2, 3) == 8
    assert operations.power(10, 0) == 1
    assert operations.power(2, -1) == 0.5
    print("✓ test_power PASSED")


def test_square_root():
    """Test square root operation."""
    assert operations.square_root(9) == 3.0
    assert operations.square_root(0) == 0.0
    assert operations.square_root(-1) is None  # Can't sqrt negative
    print("✓ test_square_root PASSED")


if __name__ == "__main__":
    test_add()
    test_subtract()
    test_multiply()
    test_divide()
    test_power()
    test_square_root()
    print("\n✓ All tests passed!")
```

**Testing Pattern**:
- Each test function checks one operation
- Use `assert` to verify expected behavior
- Test normal cases, edge cases, and error cases
- Run all tests to validate the project

---

## How to Run the Project

### Run the Calculator (Interactive)

```bash
# Navigate to project directory
cd calculator/

# Run the main program
python main.py
```

The calculator will display a menu. Choose operation (1-7) and enter numbers.

### Run the Tests (Validation)

```bash
# Run tests to verify operations work
python test_calculator.py

# Output should show:
# ✓ test_add PASSED
# ✓ test_subtract PASSED
# ... (all tests)
# ✓ All tests passed!
```

---

## Understanding the Project Structure

### Why Separate Files?

**`operations.py`** — Pure Functions
- Does: Mathematical calculations
- Doesn't: Print, read input, modify global state
- Benefit: Easy to test, easy to reuse in other projects

**`utils.py`** — I/O and Validation
- Does: User interaction, input validation, display formatting
- Doesn't: Do math (that's operations.py)
- Benefit: Can be reused for other programs that need input/output

**`main.py`** — Orchestration
- Does: Import and coordinate other modules
- Doesn't: Do math or handle I/O directly
- Benefit: Clear, readable flow of program logic

**`test_calculator.py`** — Verification
- Does: Test operations.py functions
- Benefit: Confidence that functions work before using them

---

## Module Imports — How It Works

```python
# In main.py:
import operations  # Imports operations.py
import utils       # Imports utils.py

# Now you can use:
operations.add(5, 3)       # Calls add() from operations.py
utils.display_menu()       # Calls display_menu() from utils.py
```

Python searches for `operations.py` and `utils.py` in:
1. Same directory as main.py ✓
2. Standard library
3. Installed packages

Since all files are in the same directory, imports work automatically.

---

## 🚀 Specification Challenge

Your calculator is working! Now extend it:

1. **Add a new operation** to `operations.py` — for example, `modulo()` (remainder) or `absolute_value()`
2. **Add the operation to** `main.py` — add it to the menu and create a branch for it
3. **Add a test** for your new operation to `test_calculator.py`

This teaches you how good module design enables extensibility without breaking existing code.

---

## ✨ AI Tool Tip

Your calculator imports operations like `import operations`. This works because Python looks for `operations.py` in the same directory. When sharing code:
- **Same directory**: Use `import operations`
- **Different directories**: Use packages with `__init__.py` (more advanced, Chapter 24+)
- **Not sure**: Ask your AI: "How do I organize Python modules for sharing?"

---

## Try With AI: Calculator Utility Capstone

### Part 1: Design Complete Module Architecture (Your Turn First)

**Before asking AI**, design a production-ready calculator from scratch:

```
calculator/
├── operations.py      # Your design?
├── utils.py          # Your design?
├── main.py           # Your design?
└── test_calculator.py # Your design?
```

**Your architecture tasks**:
- **For operations.py**: List 6+ mathematical operations you'll implement
  - Which functions can fail? (Design `Type | None` returns)
  - Which operations need input validation?
  - How will you handle edge cases (sqrt negative, divide by zero)?

- **For utils.py**: List all I/O functions needed
  - Menu display, input capture, error handling, result formatting
  - Design function signatures with type hints

- **For main.py**: Sketch the program flow
  - How will you map user choices to operations?
  - Where does validation happen?
  - How do you gracefully exit?

- **Design decision**: If you add calculation history, which module owns it?

Document your complete design before Part 2.

---

### Part 2: AI Implements Core Operations (Discovery)

Share your architecture with AI:

> "Here's my calculator architecture design: [paste your complete design]
>
> Implement the foundation:
>
> 1. **operations.py**: Create all mathematical functions following this pattern:
>    - Type hints: `def divide(a: float, b: float) -> float | None:`
>    - Docstrings with Parameters and Returns sections
>    - Edge case handling (return None for invalid operations)
>    - Pure functions (no printing, no side effects)
>
> 2. **test_calculator.py**: Write comprehensive tests:
>    - At least 3 test cases per operation
>    - Test normal cases, edge cases, and error cases
>    - Use assert statements to validate behavior
>    - Run all tests and show me the output
>
> 3. **Code review**: Compare your design to the implementation
>    - Did AI follow your type hints?
>    - Are edge cases handled correctly?
>    - What would you improve?"

**Your evaluation**:
- Run the tests yourself - add one test that FAILS on purpose to see error messages
- Call operations directly in Python REPL - verify behavior matches expectations
- Review docstrings - are they clear enough for another developer?

---

### Part 3: Student Teaches AI (Challenge Module Responsibility)

Challenge AI with separation of concerns:

> "I want to extend the calculator with these features. For EACH feature, analyze which module should own it:
>
> **Feature A: Calculation History**
> - Store last 10 calculations as list of tuples: (operation, num1, num2, result)
> - Show history on demand
>
> **Feature B: Result Formatting**
> - Format results as currency, percentages, scientific notation
> - User chooses format style
>
> **Feature C: Operation Validation**
> - Check if inputs are valid before calling operations
> - Prevent invalid type combinations
>
> For EACH feature, evaluate:
> 1. Should it go in operations.py, utils.py, or a NEW module? Why?
> 2. What would break if you put it in the WRONG module?
> 3. How does this choice affect testability and reusability?
> 4. Implement your recommended architecture for ONE feature"

**Your debugging**:
- Deliberately implement a feature in the wrong module (e.g., put UI code in operations.py)
- Run the calculator - it might work, but what's wrong architecturally?
- Refactor to the correct module - measure the difference in code clarity

---

### Part 4: Build Complete Working Calculator (Convergence)

Integrate everything into a working application:

> "Complete the full calculator implementation:
>
> 1. **utils.py**: Implement all I/O functions
>    ```python
>    def display_menu() -> None: ...
>    def get_operation_choice() -> str: ...
>    def get_numbers() -> tuple[float, float] | None: ...
>    def get_single_number() -> float | None: ...
>    def display_result(operation: str, result: float | None) -> None: ...
>    ```
>    - Each function handles its own validation
>    - Return None for invalid input
>    - Clear error messages for users
>
> 2. **main.py**: Orchestrate the calculator
>    - Import using `import operations` and `import utils`
>    - Menu loop with user choice handling (1-7)
>    - Call the correct operation based on choice
>    - Handle None returns from both utils and operations
>    - Use `if __name__ == '__main__':` pattern
>
> 3. **Extensions**: Add these operations
>    - Modulo (remainder): 17 % 5 = 2
>    - Absolute value: |-15| = 15
>    - Update menu, add to operations.py, add tests
>
> 4. **Full integration test**: Run the calculator
>    - Test every menu option including new operations
>    - Trigger error cases (invalid input, divide by zero)
>    - Verify graceful exit"

**Refinement**:
> "Refactor to a plugin architecture:
> ```python
> OPERATIONS = {
>     '1': ('Addition', operations.add, 2),
>     '2': ('Subtraction', operations.subtract, 2),
>     # ... map choice to (name, function, num_inputs)
> }
> ```
> - Replace if/elif chain with dictionary lookup
> - Show how this makes adding operations trivial (just add dict entry)
> - Demonstrate extensibility: add a new operation without modifying main loop logic"

---

### Part 5: Reflect and Integrate Chapter 20 Concepts (Capstone Reflection)

Synthesize everything you learned across all 5 lessons:

> "Review your complete calculator project and reflect on Chapter 20 concepts:
>
> **Module Organization (Lesson 1)**:
> - How did you decide what goes in each module?
> - What import patterns did you use? Why those specific patterns?
> - What would break if you used `from operations import *`?
>
> **Function Design (Lesson 2)**:
> - Review your function names - do they communicate intent clearly?
> - Are your docstrings useful? Would another developer understand them?
> - Which functions follow single responsibility principle? Which don't?
>
> **Parameters & Returns (Lesson 3)**:
> - Which functions use optional parameters? Are the defaults sensible?
> - Where did you use `Type | None` returns? Why there specifically?
> - Show me a function call using keyword arguments for clarity
>
> **Scope & Closures (Lesson 4)**:
> - Did you use any global variables? Could you eliminate them?
> - Are all variables in appropriate scopes (local vs module-level)?
> - Where could closures improve your design? (e.g., creating operation factories)
>
> **Integration Analysis**:
> - What was the HARDEST design decision? Why?
> - If you started over, what would you do differently?
> - How would you extend this to support:
>   - Persistent history (save to file)
>   - Custom user-defined operations
>   - Unit conversion (temperature, distance, etc.)
>
> Create a '**lessons_learned.md**' documenting:
> - Top 3 design principles you applied
> - Top 3 mistakes you made and how you fixed them
> - Top 3 concepts that 'clicked' during this capstone"

---

**Time**: 45-60 minutes
**Outcome**: You've built a production-quality multi-module calculator demonstrating module organization, function design, parameter patterns, scope management, and separation of concerns. You can articulate design decisions, evaluate trade-offs, and apply all Chapter 20 concepts in an integrated system.
