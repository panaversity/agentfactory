---
title: "Raising and Custom Exceptions"
chapter: 21
lesson: 3
duration_minutes: 45

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Raising Exceptions Strategically"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can write functions that check preconditions and raise exceptions when violated, with appropriate error messages"

  - name: "Custom Exception Design"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can create custom exception classes inheriting from Exception that represent domain-specific errors"

  - name: "Error Message Clarity"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Analyze"
    digcomp_area: "Communication"
    measurable_at_this_level: "Student can write error messages that explain what went wrong, why it matters, and what the user should do"

learning_objectives:
  - objective: "Write functions that check preconditions and raise exceptions when violated"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Function with validation logic that raises appropriate exception"

  - objective: "Create custom exception classes that inherit from Exception"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Custom exception class used to represent domain-specific error"

  - objective: "Write error messages that explain what went wrong and why"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Error messages in custom exceptions demonstrate understanding of context and user needs"

cognitive_load:
  new_concepts: 3
  assessment: "3 new concepts (raise statement, custom exception classes, meaningful error messages) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Design exception hierarchies for complex domain (e.g., banking, e-commerce); consider exception inheritance patterns"
  remedial_for_struggling: "Start with simple validation (single check, simple message); build to multiple validations"

# Generation metadata
generated_by: "lesson-writer v3.0.0"
source_spec: "specs/015-part-4-chapter-21/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Raising and Custom Exceptions

So far, you've written code that *catches* exceptions—code that anticipates errors and handles them gracefully. But what about the functions *you* write? How do they signal errors to their callers?

This lesson teaches you to think defensively: Write functions that check their inputs, then raise exceptions when something goes wrong. You'll also create custom exception classes that tell the world exactly what problem occurred. This is how professional Python code communicates errors—not through error codes or return values, but through well-designed exceptions that carry meaning.

## Why Raise Exceptions?

Consider this simple function:

```python
def set_age(age: int) -> None:
    """Set the user's age (must be 0-150)."""
    if age < 0 or age > 150:
        print("Invalid age!")  # Problem: vague, hard to handle
        return

    print(f"Age set to {age}")
```

This function catches the error, but it just prints a message. The caller has no idea an error occurred—the function returned normally. Compare it to a defensive version:

```python
class InvalidAgeError(Exception):
    """Raised when age is outside valid range."""
    pass

def set_age(age: int) -> None:
    """Set the user's age (must be 0-150)."""
    if age < 0 or age > 150:
        raise InvalidAgeError(f"Age must be 0-150, got {age}")

    print(f"Age set to {age}")
```

Now the error is *explicit*. The caller knows something failed and can respond accordingly:

```python
try:
    set_age(-5)
except InvalidAgeError as e:
    print(f"Error: {e}")
    # Maybe retry, use a default, or ask the user again
```

#### 💬 AI Colearning Prompt

> "Show me an example where raising an exception is better than returning an error code. What makes exceptions easier to handle than if/else checks?"

Raising exceptions solves a fundamental problem: **Functions can signal errors clearly, and callers can handle them strategically.** Without exceptions, you'd need to check every return value for errors—tedious and error-prone.

---

## The `raise` Statement

To signal an error from your function, use the `raise` keyword:

```python
def divide(a: float, b: float) -> float:
    """Divide a by b. Raise ValueError if b is 0."""
    if b == 0:
        raise ValueError("Cannot divide by zero")
    return a / b

# Using it:
try:
    result = divide(10, 0)
except ValueError as e:
    print(f"Math error: {e}")
```

When `raise` executes, the function stops immediately and passes the exception up the call stack. The nearest `try/except` catches it (or the program crashes if no handler exists).

#### 🎓 Instructor Commentary

> In AI-native development, raising exceptions is how functions *communicate intent*. You're not just saying "error"—you're saying *what* error and *why* it matters. That's professional defensive programming.

### Key Points About `raise`

- **Stops execution immediately**: Code after `raise` doesn't run
- **Passes control to nearest try/except**: Callers can handle the error
- **Includes a message**: Always explain what went wrong
- **Works with any exception type**: ValueError, TypeError, custom exceptions, etc.

---

## Code Example 1: Basic Validation and Raising

Here's a complete example showing when to raise and how it's handled:

```python
def validate_positive_integer(value: int) -> None:
    """Ensure value is positive. Raise ValueError if not."""
    if not isinstance(value, int):
        raise TypeError(f"Expected int, got {type(value).__name__}")

    if value <= 0:
        raise ValueError(f"Must be positive, got {value}")

    print(f"Valid: {value}")

# Caller's perspective:
try:
    validate_positive_integer(5)    # Success: prints "Valid: 5"
    validate_positive_integer(-3)   # Raises ValueError
except ValueError as e:
    print(f"Validation error: {e}")
except TypeError as e:
    print(f"Type error: {e}")
```

**Specification → AI Prompt → Validation Steps**:

- **Spec Reference**: Function validates precondition (positive integer) and raises exception when violated
- **AI Prompt Used**: "Create a function that validates input and raises appropriate exceptions for type and value errors"
- **Validation Steps**:
  1. Call with valid input (5) → prints success message
  2. Call with negative input (-3) → catches ValueError
  3. Call with wrong type ("5") → catches TypeError
- **Result**: All three paths work correctly; exceptions communicate specific errors

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:
> "I need a function that validates an email address (must contain @). Generate the function with custom exceptions. Then explain: when should I raise ValueError vs creating a custom exception?"

**Expected Outcome**: You'll understand that custom exceptions are worth creating when you want to communicate domain-specific errors clearly.

---

## Custom Exception Classes

When Python's built-in exceptions don't quite fit, create your own. A custom exception class inherits from `Exception`:

```python
class InvalidAgeError(Exception):
    """Raised when age is outside valid range (0-150)."""
    pass

class InvalidEmailError(Exception):
    """Raised when email format is invalid."""
    pass

class InsufficientFundsError(Exception):
    """Raised when account balance is too low."""
    pass
```

Custom exceptions are **minimal**—just a class inheriting from Exception with a docstring. The docstring explains what this exception means and when it should be raised.

### Using Custom Exceptions

Once defined, custom exceptions work exactly like built-ins:

```python
def set_age(age: int) -> None:
    """Set user's age. Raise InvalidAgeError if out of range."""
    if age < 0 or age > 150:
        raise InvalidAgeError(f"Age must be 0-150, got {age}")

    print(f"Age set to {age}")

# Caller handles the custom exception:
try:
    set_age(200)
except InvalidAgeError as e:
    print(f"Please enter a valid age: {e}")
```

#### Code Example 2: Custom Exception Classes

Here's a complete banking example showing multiple custom exceptions:

```python
class BankAccount:
    """Simple bank account with error handling."""

    class InsufficientFundsError(Exception):
        """Raised when withdrawal exceeds balance."""
        pass

    class InvalidAmountError(Exception):
        """Raised when amount is negative or zero."""
        pass

    def __init__(self, balance: float):
        if balance < 0:
            raise ValueError("Initial balance cannot be negative")
        self.balance = balance

    def withdraw(self, amount: float) -> None:
        """Withdraw money. Raise exceptions if preconditions fail."""
        if amount <= 0:
            raise self.InvalidAmountError(f"Amount must be positive, got {amount}")

        if amount > self.balance:
            raise self.InsufficientFundsError(
                f"Cannot withdraw {amount}. Balance: {self.balance}"
            )

        self.balance -= amount
        print(f"Withdrew {amount}. New balance: {self.balance}")

# Using it:
try:
    account = BankAccount(100)
    account.withdraw(50)        # Success
    account.withdraw(200)       # Raises InsufficientFundsError
except BankAccount.InsufficientFundsError as e:
    print(f"Transaction failed: {e}")
```

---

## Meaningful Error Messages

A good error message does three things:

1. **Explains what went wrong**: Not just "Error!" but "Age must be 0-150, got 200"
2. **Shows the actual value**: Include the problematic value so users understand the context
3. **Hints at the solution**: "Email must contain @ symbol" helps users fix it

Compare:

```python
# Poor: Vague message
raise ValueError("Invalid age")

# Better: Shows value
raise ValueError(f"Invalid age: {age}")

# Best: Explains range and shows value
raise ValueError(f"Age must be 0-150, got {age}")
```

---

## Code Example 3: Complete Validation Function

Here's a realistic example—validating form data with multiple checks:

```python
def validate_user_signup(name: str, age: int, email: str) -> None:
    """Validate user signup data. Raise exceptions for invalid inputs."""

    # Check name
    if not isinstance(name, str):
        raise TypeError(f"Name must be string, got {type(name).__name__}")

    if not name or name.isspace():
        raise ValueError("Name cannot be empty")

    # Check age
    if not isinstance(age, int):
        raise TypeError(f"Age must be int, got {type(age).__name__}")

    if age < 0:
        raise ValueError(f"Age cannot be negative, got {age}")

    if age > 150:
        raise ValueError(f"Age seems unrealistic, got {age}")

    # Check email
    if not isinstance(email, str):
        raise TypeError(f"Email must be string, got {type(email).__name__}")

    if "@" not in email:
        raise ValueError(f"Email missing @ symbol: {email}")

    if "." not in email.split("@")[1]:
        raise ValueError(f"Email domain missing dot: {email}")

    print(f"Valid signup: {name}, {age}, {email}")

# Caller handles specific errors:
try:
    validate_user_signup("Alice", 30, "alice@example.com")  # Success
except (TypeError, ValueError) as e:
    print(f"Validation failed: {e}")
```

---

## Code Example 4: Raising from Lessons 1-2

Combining what you've learned about try/except with raising exceptions:

```python
def process_data_file(filename: str) -> list[dict]:
    """Read and validate data from file. Handle and raise exceptions."""

    validated_records = []

    try:
        with open(filename, 'r') as f:
            for line_num, line in enumerate(f, 1):
                try:
                    # Try to parse each line
                    parts = line.strip().split(",")
                    if len(parts) != 3:
                        raise ValueError(f"Expected 3 fields, got {len(parts)}")

                    name, age_str, email = parts
                    age = int(age_str)  # ValueError if not numeric

                    # Validate
                    if age < 0 or age > 150:
                        raise ValueError(f"Age {age} out of range")

                    if "@" not in email:
                        raise ValueError(f"Invalid email: {email}")

                    # Success
                    validated_records.append({
                        "name": name,
                        "age": age,
                        "email": email
                    })

                except ValueError as e:
                    print(f"Line {line_num}: Skipping - {e}")
                    continue

    except FileNotFoundError:
        print(f"File not found: {filename}")
        return []

    finally:
        print(f"Processed {len(validated_records)} valid records")

    return validated_records

# Usage:
records = process_data_file("users.txt")
```

#### ✨ Teaching Tip

> Use Claude Code to design error messages. Ask: "What should my error message tell the user so they can fix the problem? Show me good vs bad examples." Test with real scenarios where you intentionally trigger errors.

---

## Practice Exercise 1: Age Validator with Custom Exception

Write a function `set_user_age()` that:
- Takes an integer age as parameter
- Raises `InvalidAgeError` if age < 0 or age > 150
- Error message: `"Age must be 0-150, got {age}"`
- Otherwise prints success: `"Age set to {age}"`

Then write code that calls it with an invalid age and catches the exception.

**Hint**: Start with the custom exception class definition, then the function.

---

## Practice Exercise 2: Email Validator

Write a function `validate_email()` that:
- Takes an email string
- Checks: contains exactly one "@" symbol
- Checks: has text before and after the "@"
- Checks: domain (after @) contains a dot
- Raises `InvalidEmailError` with helpful message if any check fails
- Otherwise returns True

Test it with:
- Valid: "alice@example.com"
- Invalid: "alice" (no @)
- Invalid: "@example.com" (no local part)
- Invalid: "alice@example" (no dot in domain)

---

## Practice Exercise 3: Multiple Validations and Custom Exceptions

Create two custom exceptions:
- `NegativeNumberError` - raised when number < 0
- `TooLargeError` - raised when number > 1000

Write a function `process_number()` that:
- Takes an integer
- Raises `NegativeNumberError` if < 0
- Raises `TooLargeError` if > 1000
- Otherwise prints `"Processing: {number}"`

Write code that catches each exception separately and handles appropriately.

---

## Common Mistakes to Avoid

**❌ Mistake 1: Forgetting to pass a message**
```python
raise ValueError()  # Vague! User sees no context
```

**✅ Better:**
```python
raise ValueError(f"Expected positive int, got {value}")
```

---

**❌ Mistake 2: Raising but not catching in caller**
```python
def dangerous_function():
    raise ValueError("Something went wrong")

dangerous_function()  # Crashes! No try/except
```

**✅ Better:**
```python
try:
    dangerous_function()
except ValueError as e:
    print(f"Error handled: {e}")
```

---

**❌ Mistake 3: Using generic Exception instead of specific types**
```python
raise Exception("Age is invalid")  # Too vague
```

**✅ Better:**
```python
raise ValueError(f"Age must be positive, got {age}")
# Or custom:
raise InvalidAgeError(f"Age must be 0-150, got {age}")
```

---

## Try With AI

Use your preferred AI companion tool (Claude Code, Gemini CLI, or ChatGPT web) to explore these prompts. Each one builds on your understanding of raising exceptions and designing custom exceptions.

### Prompt 1: Apply — Choosing Exception Types

**Ask your AI**:
> "I'm writing a function that validates age (0-150). Should I use ValueError or create a custom InvalidAgeError? What are the pros and cons of each approach?"

**Expected Outcome**: You'll understand that built-in exceptions work for general validation, but custom exceptions communicate intent more clearly in domain code.

---

### Prompt 2: Analyze — When to Raise vs Return Errors

**Ask your AI**:
> "Compare two approaches to error handling: (1) raising exceptions, (2) returning error codes (like returning -1 on failure). When should you use each? What makes exceptions better for Python?"

**Expected Outcome**: You'll recognize that exceptions separate error handling from normal control flow, making code clearer and easier to debug.

---

### Prompt 3: Evaluate — Custom Exception Design

**Ask your AI**:
> "Review this custom exception design and tell me if it's too specific or too generic: I have separate exceptions for InvalidAgeError, InvalidNameError, InvalidEmailError in my user validator. Should these be separate exceptions or one ValidationError? Explain your reasoning."

**Expected Outcome**: You'll think about exception granularity and understand that domain-specific exceptions are better than generic ones.

---

### Prompt 4: Create — Design Exception Hierarchy

**Ask your AI**:
> "I'm building an email validator service. Design an exception hierarchy (parent and child exceptions) for these scenarios: (1) email format invalid, (2) domain doesn't exist, (3) email already registered, (4) user not authorized. Then ask me: what structure did you choose and why? Is that better than having one ValidationError?"

**Expected Outcome**: You'll understand that exception hierarchies allow callers to handle categories of errors (ValidationError) or specific errors (FormatInvalidError) as needed.

---

**Safety Note**: When testing code that raises exceptions, intentionally trigger errors to verify your exception handling works correctly. Always test both the happy path (valid data) and the error path (invalid data).

