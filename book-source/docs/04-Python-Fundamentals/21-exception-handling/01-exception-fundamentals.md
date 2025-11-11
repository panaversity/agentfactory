---
title: "Exception Fundamentals"
chapter: 21
lesson: 1
duration_minutes: 45

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Recognizing Exception Types"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Remember"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student identifies ValueError, TypeError, ZeroDivisionError by name from error messages and code context"

  - name: "Understanding Try/Except Flow"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student explains what code runs in try block vs except block and predicts behavior for different inputs"

  - name: "Reading Tracebacks"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student identifies error location, error type, and error message from Python traceback output"

  - name: "Writing Basic Exception Handlers"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student writes try/except block that catches specific exception types with appropriate recovery code"

learning_objectives:
  - objective: "Understand what exceptions are and why programs crash without error handling"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Explain the difference between exception occurring and exception being caught"

  - objective: "Write basic try/except blocks that prevent crashes"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Write try/except code catching at least 2 exception types with appropriate handlers"

  - objective: "Recognize common exception types by name (ValueError, TypeError, ZeroDivisionError)"
    proficiency_level: "A2"
    bloom_level: "Remember"
    assessment_method: "Match error messages to exception types and predict which exceptions could occur in given code"

  - objective: "Read Python tracebacks to find where errors occur"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Analyze traceback output and identify error type, location, and line number"

cognitive_load:
  new_concepts: 4
  assessment: "4 new concepts (exceptions as objects, try/except structure, common exception types, tracebacks) within A2 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Challenge: Write function that raises different exceptions for different invalid inputs; have peer predict which exception will occur"
  remedial_for_struggling: "Focus on ValueError from user input first (int(input())); practice this pattern before introducing other exception types"

# Generation metadata
generated_by: "lesson-writer v3.0.0"
source_spec: "specs/015-part-4-chapter-21/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Exception Fundamentals

Every Python program you write will encounter situations that don't go as planned. A user might enter text when you expect a number. A calculation might divide by zero. A file you're trying to open might not exist. Without proper error handling, your program crashes and leaves the user confused. In this lesson, you'll learn how to anticipate errors and handle them gracefully using Python's **exception handling** system.

## Why Errors Matter

Let's imagine a simple program that converts user input to an integer:

```python
def convert_to_integer(user_input: str) -> int:
    return int(user_input)

result = convert_to_integer("hello")
print(f"Number is: {result}")
```

What happens when you run this? Python crashes with an error message. The user sees:

```
ValueError: invalid literal for int() with base 10: 'hello'
Traceback (most recent call last):
  File "example.py", line 5, in <module>
    result = convert_to_integer("hello")
  File "example.py", line 2, in convert_to_integer
    return int(user_input)
ValueError: invalid literal for int() with base 10: 'hello'
```

Your program stops. Nothing else runs. The error is unhelpful to the user (what should they do?). This is where exception handling comes in.

## What Are Exceptions?

An **exception** is an event that disrupts normal program flow. When something unexpected happens, Python creates an **exception object** that contains three pieces of information:

1. **Exception Type** (the name): What kind of error occurred? (`ValueError`, `TypeError`, `ZeroDivisionError`)
2. **Error Message** (the description): What went wrong? (`"invalid literal for int() with base 10: 'hello'"`)
3. **Traceback** (the location): Where did it happen? (which file, which function, which line)

Instead of crashing immediately, you can **catch** these exceptions and decide how to respond.

#### 💬 AI Colearning Prompt

> "How does Python determine what type of exception to create? Why does string-to-int conversion raise ValueError specifically?"

This is Python's way of telling you what went wrong so you can handle it intelligently.

## Anatomy of a Traceback

Before we fix errors, you need to read them. Let's break down the error message from above:

```
Traceback (most recent call last):
  File "example.py", line 5, in <module>
    result = convert_to_integer("hello")
  File "example.py", line 2, in convert_to_integer
    return int(user_input)
ValueError: invalid literal for int() with base 10: 'hello'
```

Reading from bottom to top:

- **Last line**: `ValueError: invalid literal for int() with base 10: 'hello'` — This is the exception type and message. It tells you: Python tried to convert `'hello'` to an integer and failed.
- **Middle lines**: These show the call chain. The error happened at line 2 in `convert_to_integer()`, which was called from line 5 in the main code.
- **Top line**: `Traceback (most recent call last):` — This header tells you "here's where the program stopped."

The traceback shows you the **path** the error took through your code. This makes debugging much easier.

#### 🎓 Instructor Commentary

> In AI-native development, you don't memorize every exception type—you understand how to READ error messages and ask AI clarifying questions. The traceback is your roadmap. Semantics (understanding the error) matters more than syntax (remembering 60 exception types).

## Basic Try/Except Structure

Here's how to handle the error gracefully:

```python
def convert_to_integer(user_input: str) -> int | None:
    try:
        return int(user_input)
    except ValueError:
        print(f"'{user_input}' is not a valid integer. Please try again.")
        return None

result = convert_to_integer("hello")
if result is not None:
    print(f"Success! Number is: {result}")
else:
    print("Conversion failed.")
```

Output:
```
'hello' is not a valid integer. Please try again.
Conversion failed.
```

The program doesn't crash anymore. Instead, it handles the error and continues.

Here's how it works:

- **`try:` block** — Code that might cause an exception goes here
- **`except:` block** — Code that runs if that exception occurs

If an exception happens in the `try` block, Python stops executing that block, checks if you have an `except` block for that exception type, and runs the exception handler instead.

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:

> "I have a function that divides two numbers. Describe to me what exceptions could happen (like dividing by zero). Then show me a try/except block that handles them."

**Expected Outcome:** You'll see that describing your function's potential errors is the first step—then AI helps you write handlers for each one.

## Common Exception Types

You'll see these three exception types constantly. Learn to recognize them by name:

### ValueError: Wrong Value, Right Type

This happens when you pass a value of the correct **type** but wrong **value**:

```python
def parse_age(age_str: str) -> int:
    try:
        age = int(age_str)
        if age < 0:
            raise ValueError(f"Age cannot be negative, got {age}")
        return age
    except ValueError as e:
        print(f"Age error: {e}")
        return 0

result = parse_age("abc")
# Output: Age error: invalid literal for int() with base 10: 'abc'

result = parse_age("-5")
# Output: Age error: Age cannot be negative, got -5
```

The issue: the input is a string (correct type), but it's not a valid number (wrong value).

### TypeError: Wrong Type Altogether

This happens when you use the wrong **type** of data with an operation:

```python
def add_values(a: int, b: int) -> int:
    try:
        return a + b
    except TypeError as e:
        print(f"Type error: Cannot add these types. {e}")
        return 0

result = add_values(5, "10")
# Output: Type error: Cannot add these types. unsupported operand type(s) for +: 'int' and 'str'

result = add_values(5, [1, 2, 3])
# Output: Type error: Cannot add these types. unsupported operand type(s) for +: 'int' and 'list'
```

The issue: you tried to add a number to a string or list. Python can't do that.

### ZeroDivisionError: Dividing by Zero

This happens when math goes wrong:

```python
def divide_numbers(numerator: float, denominator: float) -> float | None:
    try:
        result = numerator / denominator
        return result
    except ZeroDivisionError:
        print("Error: Cannot divide by zero!")
        return None

print(divide_numbers(10, 2))      # 5.0
print(divide_numbers(10, 0))      # None (error handled)
```

The issue: mathematically, you can't divide by zero. Python prevents this with an exception.

#### ✨ Teaching Tip

> Use Claude Code to test your exception handling with various inputs. Try intentionally providing bad data (non-numeric strings, zero values, negative numbers) to see which exceptions your code triggers. This builds intuition for what could go wrong.

## Three Simple Exercises

Now you'll practice writing exception handlers. Each exercise gets slightly harder.

### Exercise 1: Catch ValueError from Input

Write a function that repeatedly asks the user for a number until they provide a valid one:

```python
def get_positive_integer() -> int:
    while True:
        try:
            value = int(input("Enter a positive integer: "))
            if value > 0:
                return value
            else:
                print("Please enter a number greater than zero.")
        except ValueError:
            print("That's not a valid integer. Try again.")

result = get_positive_integer()
print(f"You entered: {result}")
```

**Your task**: Run this code. What happens when you enter "abc"? What happens when you enter "-5"? What happens when you enter "42"?

### Exercise 2: Catch ZeroDivisionError with Fallback

Write a function that divides two numbers. If the denominator is zero, return a fallback value:

```python
def safe_divide(numerator: float, denominator: float, fallback: float = 0.0) -> float:
    try:
        return numerator / denominator
    except ZeroDivisionError:
        print(f"Cannot divide {numerator} by zero. Returning fallback: {fallback}")
        return fallback

print(safe_divide(10, 2))       # 5.0
print(safe_divide(10, 0))       # 0.0 (fallback)
print(safe_divide(7, 0, -1))    # -1 (custom fallback)
```

**Your task**: Run this code. Then modify it to return the absolute value of the numerator as fallback instead of 0.0.

### Exercise 3: Trace an Error and Fix It

Here's broken code. Run it, read the traceback, and explain what went wrong:

```python
def process_data(data: list[str]) -> int:
    try:
        total = 0
        for item in data:
            total = total + int(item)
        return total
    except ValueError:
        print("Item in list is not a valid integer.")
        return -1

result = process_data(["5", "10", "fifteen", "20"])
print(f"Total: {result}")
```

**Your task**:
1. Run this code and read the error message.
2. In the traceback, identify: What line caused the error? What is the error type? What was the problematic input?
3. Why did the exception handler still print its message?

---

## Try With AI

Use your preferred AI companion (Claude Code, Gemini CLI, or ChatGPT web) to deepen your understanding of exception handling.

### Prompt 1: Understand Exception Objects

Ask your AI:

> "Explain what happens when Python encounters an error: How does it create an exception object? What information does the exception object contain?"

**Expected Outcome**: You understand that exceptions are objects with type, message, and traceback—not magic, but structured information Python uses to communicate errors.

### Prompt 2: Apply to Your Code

Ask your AI:

> "What exceptions could occur in this code? [paste your code]. For each exception, write a try/except block that handles it."

(Replace with actual code from one of your programs from Chapters 12-20.)

**Expected Outcome**: You can identify potential errors in your own code and sketch handlers for them.

### Prompt 3: Analyze Exception Design

Ask your AI:

> "Why does Python have separate ValueError and TypeError instead of just one 'error' exception? When would you catch each one differently?"

**Expected Outcome**: You understand that different exception types communicate different problems—ValueError says 'the value is wrong', TypeError says 'the type is wrong'. This distinction lets you write targeted recovery strategies.

### Prompt 4: Synthesize Your Learning

Ask your AI:

> "I've learned about try/except, ValueError, TypeError, and ZeroDivisionError. Show me a realistic function that could raise all three exception types. For each, write a separate except block explaining what that exception means."

**Expected Outcome**: You've seen a complex example integrating multiple exception types, reinforcing that real code often needs multiple error handlers.

---

**Note on Safety**: When testing code with AI, always review the generated exception handlers. Ask "Is this error message helpful to users?" and "Would this recovery strategy actually solve the problem?" Exception handling is about user experience—writing messages and recovery paths that help users fix their mistakes.
