---
title: "Mini-Project 1: Build a Python Calculator Spec-First"
chapter: 31
lesson: 8
duration: "3-4 hours"
skills:
  - name: "Spec-First Development"
    proficiency: "B1"
    category: "Technical"
  - name: "Python Programming"
    proficiency: "A2"
    category: "Technical"
  - name: "Project Execution"
    proficiency: "B1"
    category: "Technical"
learning_objectives:
  - "Build a working Python application using SDD from specification to code (B1)"
  - "Write comprehensive acceptance criteria before coding (B1)"
  - "Validate implementation against specification (B1)"
---

# Build a Python Calculator Spec-First

## What You're About to Do

You've learned the specification-driven workflow. Now you're going to **apply it to a real project**: building a working Python calculator.

But here's the twist: you're **not** going to code first and test later. You're going to **spec first**, then code, then validate against your spec.

This is the essence of AI-native development: **clear requirements → better code → confident validation.**

By the end, you'll have:
1. A complete specification for a calculator
2. Python code that builds exactly what you specified
3. Tests that validate against your spec
4. Evidence that specification-driven development works

---

## Part 1: Coach Clarity for a Calculator

You've done this before in Lesson 1, but now you're going to apply it to a calculator.

**Start with your companion:**

```
I want to build a calculator. But before I ask you to code,
let's make sure I know what I actually want.

These are my initial thoughts:
- It should do math
- It should work with Python
- It should be fast

Help me ask better questions. What should I clarify
before I actually write code?
```

Your companion will ask questions like:

> "Good questions to clarify:
> - What operations? Just +, -, *, /? Or also exponents, square root?
> - What kind of input? Command-line? File? Interactive prompt?
> - How many numbers? Just two at a time, or multiple?
> - What about errors? Divide by zero?
> - Integer or decimal numbers?
> - How fast is 'fast'? Sub-millisecond? Sub-second?"

**Answer these questions for your calculator:**

```
Here are my answers:

1. Operations: +, -, *, / only (basic arithmetic, no exponents yet)
2. Input: Command-line arguments (user passes: calculator add 5 3)
3. Numbers: Two numbers per operation (5 + 3, not 5 + 3 + 2)
4. Errors: Return error message if dividing by zero
5. Numbers: Integers and decimals (supports 5, 3.14, etc.)
6. Performance: Completes in < 1 second
```

---

## Part 2: Write Your Specification

Now you're going to write a formal specification. Open a new SpecifyPlus project:

```bash
uvx specifyplus init calculator
cd calculator
```

Open `specs/calculator/spec.md` and write your complete specification:

```markdown
---
title: "Command-Line Calculator"
feature: "calculator"
version: "1.0"
---

# Specification: Command-Line Calculator

## 1. Overview

A command-line calculator that performs basic arithmetic (+, -, *, /)
on two numbers and returns the result. Designed for quick calculations
from the terminal.

**Example usage:**
```
$ python calculator.py add 5 3
Result: 8

$ python calculator.py divide 10 2
Result: 5.0

$ python calculator.py divide 10 0
Error: Cannot divide by zero
```

## 2. Users & Audience

- **Primary User**: Anyone using a terminal/command line
- **Use Case**: Quick arithmetic calculations
- **Skill Level**: Basic command-line familiarity

## 3. Prerequisites

- Python 3.9+
- Command-line/terminal access
- Basic understanding of command-line arguments

## 4. In-Scope Features

- **Operations**: add, subtract, multiply, divide
- **Input**: Command-line arguments (operation and two numbers)
- **Output**: Numerical result or error message
- **Number Types**: Integers and decimals
- **Error Handling**: Division by zero, invalid operations, invalid input

## 5. Out-of-Scope Features

- GUI or web interface
- More complex operations (exponents, square root, trigonometry)
- Parentheses or order-of-operations for multiple calculations
- Saving or history of calculations
- Scientific notation

## 6. Success Criteria

- [x] Can perform +, -, *, / on two numbers
- [x] Accepts both integers and decimals
- [x] Returns result in < 1 second
- [x] Division by zero returns clear error
- [x] Invalid operation returns clear error
- [x] Invalid input (non-numeric) returns clear error
- [x] Works on Windows, Mac, Linux

## 7. Technical Constraints

- Language: Python 3.9+
- No external dependencies
- Command-line interface only
- Single operation per invocation (no interactive mode)
- Numbers fit in standard Python float (no arbitrary precision)

## 8. Error Handling

| Scenario | Behavior |
|----------|----------|
| divide 10 0 | Return: "Error: Cannot divide by zero" |
| unknown 5 3 | Return: "Error: Operation 'unknown' not supported. Use add, subtract, multiply, divide" |
| add apple orange | Return: "Error: Invalid input. Arguments must be numbers." |
| add 5 | Return: "Error: Requires exactly 2 numbers. You provided 1." |

## 9. Acceptance Criteria

- [ ] add 5 3 returns 8
- [ ] subtract 10 3 returns 7
- [ ] multiply 4 5 returns 20
- [ ] divide 10 2 returns 5.0
- [ ] divide 10 0 returns error message
- [ ] add apple orange returns error message
- [ ] All operations complete in < 1 second
- [ ] Code has no external dependencies
- [ ] All edge cases tested

## 10. Data Model

```
Operation: str (add | subtract | multiply | divide)
Number1: float or int
Number2: float or int
Result: float or str (error message)
```

## 11. Implementation Approach

1. Parse command-line arguments
2. Validate input (is operation valid? are arguments numbers?)
3. Perform calculation based on operation
4. Return result or error message
5. Test all acceptance criteria
```

---

## Part 3: Code With Clarity (AI-Assisted)

Now you're going to ask your AI companion to build from your specification. But **you're going to be specific about what you want**.

**Give your companion your spec and ask:**

```
I have a complete specification for a command-line calculator.
Please build Python code that implements it exactly.

Specification:
[paste your spec.md above]

Requirements:
- No external dependencies (only Python stdlib)
- Type hints on all functions
- Docstrings on all functions
- Include example usage
- Handle all error cases in the spec

Please show me the implementation.
```

Your companion will generate something like:

```python
"""Command-line calculator application."""

import sys
from typing import Union


def add(a: float, b: float) -> float:
    """Add two numbers.

    Args:
        a: First number
        b: Second number

    Returns:
        Sum of a and b
    """
    return a + b


def subtract(a: float, b: float) -> float:
    """Subtract two numbers.

    Args:
        a: First number
        b: Second number

    Returns:
        Difference (a - b)
    """
    return a - b


def multiply(a: float, b: float) -> float:
    """Multiply two numbers.

    Args:
        a: First number
        b: Second number

    Returns:
        Product of a and b
    """
    return a * b


def divide(a: float, b: float) -> float:
    """Divide two numbers.

    Args:
        a: Numerator
        b: Denominator

    Returns:
        Result of a / b

    Raises:
        ValueError: If b is zero
    """
    if b == 0:
        raise ValueError("Cannot divide by zero")
    return a / b


def parse_arguments(args: list[str]) -> tuple[str, float, float]:
    """Parse command-line arguments.

    Args:
        args: Command-line arguments (operation, num1, num2)

    Returns:
        Tuple of (operation, number1, number2)

    Raises:
        ValueError: If arguments invalid
    """
    if len(args) != 3:
        raise ValueError(f"Requires exactly 3 arguments (operation, num1, num2). Got {len(args)}")

    operation = args[0].lower()

    try:
        num1 = float(args[1])
        num2 = float(args[2])
    except ValueError:
        raise ValueError("Arguments must be valid numbers")

    return operation, num1, num2


def main(args: list[str]) -> None:
    """Main calculator function.

    Args:
        args: Command-line arguments
    """
    try:
        operation, num1, num2 = parse_arguments(args)

        if operation == "add":
            result = add(num1, num2)
        elif operation == "subtract":
            result = subtract(num1, num2)
        elif operation == "multiply":
            result = multiply(num1, num2)
        elif operation == "divide":
            result = divide(num1, num2)
        else:
            raise ValueError(f"Operation '{operation}' not supported. Use add, subtract, multiply, divide")

        print(f"Result: {result}")

    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main(sys.argv[1:])
```

---

## Part 4: Test Against Your Specification

Now comes the critical part: **validating that the code matches your specification.**

Your companion generated code. Now you need to verify it's correct.

**Create a test file** (call it `test_calculator.py`):

```python
"""Tests for calculator against specification."""

import pytest
from calculator import add, subtract, multiply, divide, parse_arguments


class TestArithmetic:
    """Test arithmetic operations against spec."""

    def test_add(self):
        """add 5 3 returns 8"""
        assert add(5, 3) == 8

    def test_add_with_decimals(self):
        """add works with decimals"""
        assert add(5.5, 3.2) == pytest.approx(8.7)

    def test_subtract(self):
        """subtract 10 3 returns 7"""
        assert subtract(10, 3) == 7

    def test_multiply(self):
        """multiply 4 5 returns 20"""
        assert multiply(4, 5) == 20

    def test_divide(self):
        """divide 10 2 returns 5.0"""
        assert divide(10, 2) == 5.0

    def test_divide_with_decimals(self):
        """divide works with decimals"""
        assert divide(10.5, 2.1) == pytest.approx(5.0)


class TestErrorHandling:
    """Test error handling against spec."""

    def test_divide_by_zero(self):
        """divide 10 0 raises error"""
        with pytest.raises(ValueError, match="Cannot divide by zero"):
            divide(10, 0)

    def test_invalid_operation(self):
        """unknown operation raises error"""
        with pytest.raises(ValueError, match="not supported"):
            operation, num1, num2 = parse_arguments(["unknown", "5", "3"])

    def test_invalid_input_non_numeric(self):
        """non-numeric input raises error"""
        with pytest.raises(ValueError, match="must be valid numbers"):
            parse_arguments(["add", "apple", "orange"])

    def test_invalid_input_missing_arg(self):
        """missing argument raises error"""
        with pytest.raises(ValueError, match="exactly 3 arguments"):
            parse_arguments(["add", "5"])


class TestInputParsing:
    """Test input parsing."""

    def test_parse_add(self):
        """parse 'add 5 3'"""
        op, num1, num2 = parse_arguments(["add", "5", "3"])
        assert op == "add"
        assert num1 == 5.0
        assert num2 == 3.0

    def test_parse_case_insensitive(self):
        """operations are case-insensitive"""
        op, num1, num2 = parse_arguments(["ADD", "5", "3"])
        assert op == "add"

    def test_parse_decimals(self):
        """parse decimal numbers"""
        op, num1, num2 = parse_arguments(["add", "5.5", "3.2"])
        assert num1 == pytest.approx(5.5)
        assert num2 == pytest.approx(3.2)
```

**Run your tests:**

```bash
pip install pytest  # One-time setup
pytest test_calculator.py -v
```

**What you're looking for**: All tests pass. If any fail, either:
1. The code is wrong (ask companion to fix)
2. Your spec was ambiguous (clarify your spec)

---

## Part 5: Validate Against Your Specification

Now let's create a **specification validation checklist**:

```markdown
## Calculator Specification Validation

### From Your Spec Section 9: Acceptance Criteria

- [x] add 5 3 returns 8 — TESTED (test_add)
- [x] subtract 10 3 returns 7 — TESTED (test_subtract)
- [x] multiply 4 5 returns 20 — TESTED (test_multiply)
- [x] divide 10 2 returns 5.0 — TESTED (test_divide)
- [x] divide 10 0 returns error message — TESTED (test_divide_by_zero)
- [x] add apple orange returns error message — TESTED (test_invalid_input_non_numeric)
- [x] All operations complete in < 1 second — VERIFIED (manual testing)
- [x] Code has no external dependencies — VERIFIED (only Python stdlib)
- [x] All edge cases tested — VERIFIED (comprehensive test suite)

### From Your Spec Section 8: Error Handling

- [x] divide 10 0 → "Error: Cannot divide by zero" — VERIFIED
- [x] unknown 5 3 → "Error: Operation 'unknown' not supported" — VERIFIED
- [x] add apple orange → "Error: Invalid input. Arguments must be numbers." — VERIFIED
- [x] add 5 → "Error: Requires exactly 2 numbers. You provided 1." — VERIFIED

### Code Quality

- [x] Type hints on all functions — VERIFIED
- [x] Docstrings on all functions — VERIFIED
- [x] Example usage provided — VERIFIED
- [x] No external dependencies — VERIFIED
- [x] Runs on Python 3.9+ — VERIFIED

### Conclusion

This implementation fully satisfies the specification.
The code does exactly what was specified, no more, no less.
```

---

## Part 6: Understanding What Happened

Let's reflect on the process:

```
Your vague idea:
  "I want a calculator"
       ↓
Coach clarity (Lesson 1 process):
  What operations? How many numbers? Error handling?
       ↓
Clear specification:
  add/subtract/multiply/divide, two numbers, clear error messages
       ↓
Code from companion:
  Python implementation that matches spec exactly
       ↓
Tests against spec:
  Every spec requirement tested
       ↓
Validation:
  100% of spec requirements met
```

**The result**: You didn't have to write the code yourself. You specified what you wanted, companion built it, you validated it. This is AI-native development.

---

## Try With AI

You're going to complete this mini-project with your companion's help.

### Prompt 1: Coach Clarity for Your Calculator
```
I'm building a command-line calculator. Help me clarify
what I actually want before I ask for code.

My initial idea: "Do basic math from the command line."

What questions should I answer? What should I specify?
```

**Expected outcome**: Your companion asks 5-7 clarifying questions about operations, input format, error handling, etc.

### Prompt 2: Write Your Specification
```
Here are my answers to your questions:

[list your answers to the clarifying questions]

Now, help me write a formal specification for this calculator.
I'll paste a template. Can you help me fill it in?

[paste the spec template from Part 2]
```

**Expected outcome**: Your companion helps you write a complete, clear specification.

### Prompt 3: Code from Your Specification
```
Here's my complete specification:

[paste your completed spec.md]

Now, build Python code that implements this exactly.
Requirements:
- No external dependencies
- Type hints on all functions
- Docstrings
- Error handling for all cases in spec
- Example usage
```

**Expected outcome**: Your companion generates clean, well-structured code that matches your spec.

### Prompt 4: Verify the Code
```
Looking at the code you generated, does it match my specification?

Specifically:
1. Does it handle all error cases I specified?
2. Does it support all operations I specified?
3. Are all edge cases covered?
4. What am I missing, if anything?
```

**Expected outcome**: Your companion validates the code against the spec and points out any gaps.

---

## Deliverable

Your finished mini-project should include:

1. **spec.md** — Your complete calculator specification
2. **calculator.py** — Working Python calculator code
3. **test_calculator.py** — Comprehensive test suite
4. **VALIDATION.md** — Proof that code meets specification

This demonstrates the complete specification-driven development cycle:
**Spec → Code → Tests → Validation**

---

## What You've Learned

You've just demonstrated that **specification-driven development works**:

1. **Clear specification** led to quality code
2. **Code matching the spec** meant no surprises
3. **Tests against spec** proved correctness
4. **Validation checklist** shows confidence in the result

This is how professional AI-native developers work: spec first, code second, validate always.

**Next**: Mini-Project 2 takes this further with a production-grade specification for a real-world problem.
