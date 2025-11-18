---
sidebar_position: 2
title: "Lesson 2: Writing Functions with Intent"
description: "Learn how to write functions with clear parameters, return values, type hints, and docstrings"
keywords: ["functions", "def", "type hints", "docstrings", "parameters", "return values", "intent"]
cefr_level: "A2-B1"
estimated_time: "60 minutes"
proficiency_by_end: "Students can write functions with type hints and docstrings that clearly communicate intent"
prerequisites: ["Lesson 1: Module imports concept", "Chapter 14: Type hints syntax", "Chapter 18: Collections (lists, dicts)"]
learning_objectives:
  - "Write function definitions with parameters, return values, and type hints"
  - "Write docstrings that explain function purpose, parameters, and return value"
  - "Understand type hints as a communication protocol between developers and AI"
  - "Call functions correctly and use their return values"
ai_native_learning: true
try_with_ai_prompts: 4
colearning_elements: ["conceptual_prompt", "teaching_commentary", "specification_challenge", "ai_tip"]
---

## Lesson 2: Writing Functions with Intent

**CEFR Level**: A2-B1 (Foundational with Application)
**Time Estimate**: 60 minutes
**What You'll Learn**: Functions are reusable blocks of code. You'll learn to write functions with clear intent—using type hints to tell Python (and AI) what data you need, and docstrings to explain what your function does.

---

## Why Write Functions? — Code Reuse and Clear Intent

**The Problem**: You find yourself writing the same code over and over. A calculation you need in three different places. A data validation check used throughout your script.

**The Solution**: Write a function once, call it many times. Better yet, write it with such clear intent (type hints + docstring) that other developers (or AI) understand exactly what it does without reading the implementation.

### 💻 Code Idea: Function as Organized Intent

```python
# Instead of this (repeated calculation):
result1: int = 5 + 3
result2: int = 10 + 7
result3: int = 2 + 8

# Do this (function, reused):
def add(a: int, b: int) -> int:
    """Return the sum of two integers."""
    return a + b

result1: int = add(5, 3)
result2: int = add(10, 7)
result3: int = add(2, 8)
```

**Why this matters**: The function signature (`def add(a: int, b: int) -> int:`) tells you everything you need to know: it takes two integers and returns an integer. That's the intent. The docstring explains the purpose. The body is just the implementation.

### 💬 AI Colearning Prompt

> Ask your AI: "I need a function that takes a list of prices and calculates total cost with 10% tax. What should the type hints be? What should the docstring say? Write just the function signature (def line through docstring, no body yet)."

**Expected Understanding**: You see that **intent comes before implementation**. A clear function signature (especially with type hints) lets you think about WHAT you need before HOW to code it.

### 🎓 Instructor Commentary

Type hints are not just syntax—they're your contract with other developers and AI. When you write `def calculate(items: list[float]) -> float:`, you're making a promise: "I accept a list of floats and return a single float." This is incredibly powerful. AI can validate your code against this contract. Other developers know exactly what to pass and what to expect back. Syntax is cheap and forgettable. Semantics—the meaning behind the code—is what matters. Type hints encode semantics directly into your function signature.

---

## Function Definition — The Structure

**Components of a Function**:

```python
def function_name(parameter1: Type1, parameter2: Type2) -> ReturnType:
    """Docstring: explain what the function does."""
    # Function body: code that does the work
    result = parameter1 + parameter2
    return result
```

- **`def`** — Python keyword that starts a function definition
- **`function_name`** — What you call the function (use descriptive names)
- **`parameters`** — Variables the function receives when called
- **`Type hints`** — Tell what type each parameter should be, and what type is returned
- **`Docstring`** — Multi-line string explaining the function's purpose
- **`return`** — Send a value back to the caller

### 💻 Code Idea: Simple Function with Type Hints

```python
# Function definition
def add(a: int, b: int) -> int:
    """Add two integers and return the sum."""
    return a + b

# Calling the function
result: int = add(5, 3)
print(f"5 + 3 = {result}")  # Output: 8

# Another example
def greet(name: str) -> str:
    """Return a greeting message for the given name."""
    return f"Hello, {name}!"

greeting: str = greet("Alice")
print(greeting)  # Output: Hello, Alice!
```

**Key Observation**: The type hints (`int`, `str`, `->`) are hints for humans and type checkers. Python executes the function the same way without them, but they make intent clear.

---

## Type Hints as Intent Specification

**What Type Hints Say**: Type hints are your specification. They answer: "What data types should this function receive? What type will it return?"

### 💻 Code Idea: Type Hints on Complex Parameters

```python
def calculate_average(numbers: list[float]) -> float:
    """
    Calculate the average of a list of numbers.

    Parameters:
        numbers (list[float]): List of numeric values

    Returns:
        float: The average (mean) of all numbers
    """
    if len(numbers) == 0:
        return 0.0

    total: float = sum(numbers)
    average: float = total / len(numbers)
    return average

# Call the function with a list of floats
scores: list[float] = [85.5, 90.0, 78.5, 92.0]
avg: float = calculate_average(scores)
print(f"Average score: {avg:.2f}")  # 86.5
```

**Reading the Type Hint**: `numbers: list[float]` says "numbers must be a list of floats." Return type `-> float` says "this function returns a single float."

---

## Docstrings — Communication for Humans and AI

**Purpose**: A docstring explains what the function does. It's not a comment about implementation details—it's documentation for the USER of the function.

### 💻 Code Idea: Well-Documented Function

```python
def divide_with_remainder(dividend: int, divisor: int) -> tuple[int, int]:
    """
    Divide two integers and return quotient and remainder.

    Parameters:
        dividend (int): Number to divide
        divisor (int): Number to divide by

    Returns:
        tuple[int, int]: (quotient, remainder)
    """
    quotient: int = dividend // divisor
    remainder: int = dividend % divisor
    return quotient, remainder

# Use the function
q, r = divide_with_remainder(17, 5)
print(f"17 ÷ 5 = {q} remainder {r}")  # 3 remainder 2
```

**Docstring Structure**:
1. **One-line summary** — What does this function do?
2. **Parameters section** — What inputs does it take?
3. **Returns section** — What does it return?

---

## Functions with Multiple Parameters and Default Values

### 💻 Code Idea: Function with Optional Parameter

```python
def format_name(first: str, last: str, uppercase: bool = False) -> str:
    """
    Format a person's name.

    Parameters:
        first (str): First name
        last (str): Last name
        uppercase (bool): Whether to convert to uppercase (default: False)

    Returns:
        str: Formatted full name
    """
    full_name: str = f"{first} {last}"
    return full_name.upper() if uppercase else full_name

# Call with default parameter
print(format_name("alice", "smith"))          # alice smith

# Call with optional parameter specified
print(format_name("alice", "smith", True))    # ALICE SMITH
```

**Key Point**: The parameter `uppercase: bool = False` means it's optional. If you don't provide it, Python uses `False` by default.

---

## Functions Returning Multiple Values

**Pattern**: When a function needs to return multiple pieces of information, return a tuple and unpack it.

### 💻 Code Idea: Return and Unpack Multiple Values

```python
def get_user_info(username: str) -> tuple[str, str, int]:
    """
    Get user information.

    Returns:
        tuple[str, str, int]: (name, email, age)
    """
    # In a real scenario, this would query a database
    if username == "alice":
        return "Alice Smith", "alice@example.com", 30
    else:
        return "Unknown", "unknown@example.com", 0

# Unpack the returned tuple
name, email, age = get_user_info("alice")
print(f"Name: {name}, Email: {email}, Age: {age}")
# Output: Name: Alice Smith, Email: alice@example.com, Age: 30
```

**Why this pattern?** Type hints make it clear you're returning three values. The unpacking line shows exactly what each return value represents.

---

## Functions Checking Conditions — Predicates

**Pattern**: Functions that return `True` or `False` are called predicates. They answer yes/no questions.

### 💻 Code Idea: Predicate Functions

```python
def is_even(number: int) -> bool:
    """Check if a number is even."""
    return number % 2 == 0

def is_within_range(value: int, min_val: int, max_val: int) -> bool:
    """Check if a value is within a specified range."""
    return min_val <= value <= max_val

# Use predicates in conditions
print(is_even(4))                    # True
print(is_even(7))                    # False
print(is_within_range(50, 0, 100))   # True
print(is_within_range(150, 0, 100))  # False
```

**Key Point**: Boolean return types (`-> bool`) make it clear the function answers a yes/no question.

---

## Functions Processing Collections

### 💻 Code Idea: Function Working with Lists

```python
def filter_scores(scores: list[float], minimum: float = 0.0) -> list[float]:
    """
    Filter scores, keeping only those above minimum.

    Parameters:
        scores (list[float]): List of score values
        minimum (float): Minimum acceptable score (default: 0.0)

    Returns:
        list[float]: Filtered list of scores
    """
    result: list[float] = []
    for score in scores:
        if score >= minimum:
            result.append(score)
    return result

# Use the function
all_scores: list[float] = [65.0, 85.5, 72.0, 95.0, 58.5]
passing: list[float] = filter_scores(all_scores, 70.0)
print(f"Passing scores: {passing}")  # [85.5, 72.0, 95.0]
```

**Reading the Type Hint**: `scores: list[float]` → input list of floats, `-> list[float]` → output list of floats.

---

## Calling Functions — Using Return Values

### 💻 Code Idea: Function Return Values Flow Through Code

```python
def double(x: int) -> int:
    """Return the value doubled."""
    return x * 2

def add_ten(x: int) -> int:
    """Add ten to the value."""
    return x + 10

# Chain function calls: the output of one becomes input to next
starting_value: int = 5
doubled: int = double(starting_value)      # 10
final: int = add_ten(doubled)              # 20

print(f"Start: {starting_value} → Double: {doubled} → Add 10: {final}")
# Output: Start: 5 → Double: 10 → Add 10: 20
```

**Key Insight**: Function return values are just data. You can pass them to another function, store them, or print them.

---

## 🚀 Specification Challenge

Choose a real calculation you do regularly (convert temperature, calculate tip, check if someone qualifies for a discount). Write the function **signature only** (just `def` line through docstring, no body):

1. What parameters do you need?
2. What type should each parameter be?
3. What type will you return?
4. What will your docstring say?

Then tell your AI: *"Here's my function signature. Generate the implementation."* Compare what you expected vs. what AI generated. This teaches you how clarity of intent affects code generation.

---

## ✨ AI Tool Tip

Your AI can generate function bodies from docstrings. The process:

1. **You write the specification** (function signature + docstring)
2. **Your AI implements it** ("Implement this function")
3. **You validate it works** (Test with examples)

Better specifications → better AI output. This is how AI-native developers work.

---

## Try With AI: Function Design Workshop

### Part 1: Refactor Repetitive Code (Your Turn First)

**Before asking AI**, you have this repetitive script:

```python
# User registration system with duplicate code
email1 = "alice@test.com"
username1 = "alice"
if "@" not in email1 or len(username1) < 3:
    print("Invalid user data")
else:
    print(f"User {username1} registered: {email1}")

email2 = "bob@test.com"
username2 = "bob"
if "@" not in email2 or len(username2) < 3:
    print("Invalid user data")
else:
    print(f"User {username2} registered: {email2}")

email3 = "charlie@test.com"
username3 = "charlie"
if "@" not in email3 or len(username3) < 3:
    print("Invalid user data")
else:
    print(f"User {username3} registered: {email3}")

# ... imagine 50 more users like this
```

**Your task**: Design function signatures (NO implementation yet)
- What function(s) would eliminate this repetition?
- What parameters do you need? What types?
- What should each function return? What type?
- Write ONLY the `def` line + docstring for each function

Document your function signatures before Part 2.

---

### Part 2: AI Explains Single Responsibility (Discovery)

Share your signatures with AI:

> "Here are my function signatures to refactor repetitive code: [paste your signatures]
>
> Teach me about function design:
> 1. Review my signatures - are they following the 'single responsibility principle'? (one function, one job)
> 2. Show me good vs bad function naming: `process_data()` vs `validate_email()` - which is clearer?
> 3. Explain when to split one function into two - how do you know when a function is doing too much?
> 4. Implement my functions and show how they eliminate the repetition"

**Your evaluation**:
- Compare AI's implementation to what you imagined - same logic?
- Try calling the functions with edge cases: empty string, None, very long input
- Does the function name clearly describe what it does? Could you improve it?

---

### Part 3: Student Teaches AI (When NOT to Create Functions)

Challenge AI with over-abstraction:

> "I wrote this function to avoid repetition:
> ```python
> def add_numbers(a: int, b: int) -> int:
>     '''Add two numbers.'''
>     return a + b
>
> # Used like this:
> total = add_numbers(5, 3)
> ```
>
> Analyze this decision:
> 1. Is wrapping `a + b` in a function actually helpful? Why or why not?
> 2. When does extracting code into a function make things WORSE (over-abstraction)?
> 3. Give me three examples where creating a function is wrong: too simple, too specific, too unclear
> 4. Create a decision checklist: 'Should I extract this into a function?' with yes/no criteria"

**Your debugging**:
- Find a place in your code where you created an unnecessary function
- What's the cost? (More code, more indirection, harder to read?)
- Refactor it back to inline code - is it actually simpler?

---

### Part 4: Build Function Design Checklist (Convergence)

Create a practical function design guide:

> "Build a comprehensive function design toolkit for me:
>
> 1. **Naming Convention Guide**: Show me 10 function names (5 good, 5 bad) with explanations
>    - Good: `validate_email()`, `calculate_tax()`, `format_date()`
>    - Bad: `do_stuff()`, `process()`, `handle()`
>
> 2. **Parameter Design Patterns**: Create functions demonstrating:
>    - Required parameters only
>    - Mix of required + optional with defaults
>    - When to use `bool` flags vs separate functions
>
> 3. **Return Value Patterns**: Show me these scenarios:
>    - Function that returns a single value
>    - Function that returns multiple values as tuple
>    - Function that might fail: `Type | None` pattern
>
> 4. **Docstring Template**: Give me a reusable template with Parameters, Returns, Raises sections"

**Refinement**:
> "Apply this toolkit to a real scenario: design a user authentication system with these functions:
> - `validate_password()` - check password strength
> - `hash_password()` - secure password storage
> - `authenticate_user()` - login verification
> - `create_session()` - session management
>
> For EACH function: write signature + docstring ONLY (no implementation). Show me how good design makes the implementation obvious."

---

**Time**: 30 minutes
**Outcome**: You can identify when to extract functions vs keep code inline, apply single responsibility principle, design clear function signatures that make implementation obvious, and create professional naming conventions and documentation patterns.
