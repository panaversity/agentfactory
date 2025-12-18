---
title: "Python Keywords"
sidebar_position: 5
---

# Python Keywords

## What You'll Learn

- What Keywords Are
- Reserved Words You Can't Use as Variables
- Categories of Keywords
- Common Keyword Uses

## The Problem

You try to name a variable `class` or `for`, but Python gives you a syntax error. Why can't you use these perfectly good words?

## Why Keywords Matter

Keywords are reserved words that Python uses for its own syntax. You cannot use them as variable names, function names, or any other identifier. Understanding keywords helps you:

- Avoid naming conflicts
- Understand Python's structure
- Read code more effectively
- Recognize syntax patterns

Let's discover what words Python reserves for itself.

## What Are Keywords?

Keywords are special words that Python has reserved for specific purposes. They define the structure and logic of Python programs.

```python
# You CANNOT do this - these are keywords
# class = "Math"  # SyntaxError!
# for = 10        # SyntaxError!
# if = True       # SyntaxError!

# You CAN use descriptive alternatives
course_class = "Math"
loop_count = 10
is_valid = True
```

**Key Point**: Keywords are case-sensitive. `True` is a keyword, but `true` is not (though using `true` would be confusing).

## How to See All Keywords

Python provides a way to list all keywords:

```python
import keyword

# Get the list of all keywords
all_keywords = keyword.kwlist
print(f"Python has {len(all_keywords)} keywords")
print(all_keywords)
```

Output:
```
Python has 35 keywords
['False', 'None', 'True', 'and', 'as', 'assert', 'async', 'await',
 'break', 'class', 'continue', 'def', 'del', 'elif', 'else', 'except',
 'finally', 'for', 'from', 'global', 'if', 'import', 'in', 'is',
 'lambda', 'nonlocal', 'not', 'or', 'pass', 'raise', 'return',
 'try', 'while', 'with', 'yield']
```

## Keywords by Category

### Value Keywords

These represent specific values:

```python
# Boolean values
is_active = True
is_deleted = False

# Absence of value
result = None  # No value assigned yet

# Check types
print(type(True))   # <class 'bool'>
print(type(False))  # <class 'bool'>
print(type(None))   # <class 'NoneType'>
```

### Logical Operator Keywords

You already learned these in Lesson 3:

```python
# and - both must be True
can_enter = is_member and has_ticket

# or - at least one must be True
gets_discount = is_student or is_senior

# not - inverts the value
is_invalid = not is_valid
```

### Membership and Identity Keywords

```python
# in - check if item exists in collection
numbers = [1, 2, 3, 4, 5]
has_three = 3 in numbers
print(f"Contains 3: {has_three}")  # True

# not in - check if item doesn't exist
has_ten = 10 in numbers
print(f"Contains 10: {has_ten}")  # False

# is - check if same object (identity)
a = None
is_none = a is None
print(f"Is None: {is_none}")  # True

# is not - check if different object
b = 5
is_not_none = b is not None
print(f"Is not None: {is_not_none}")  # True
```

### Control Flow Keywords (Preview)

You'll learn these in detail in Chapter 18:

```python
# if, elif, else - conditional branching
# for, while - loops
# break, continue - loop control
# pass - placeholder that does nothing
```

### Function and Class Keywords (Preview)

You'll learn these in later chapters:

```python
# def - define a function (Chapter 21)
# return - return a value from function (Chapter 21)
# class - define a class (Chapter 25)
# lambda - create anonymous function (Chapter 21)
```

### Import Keywords

```python
# import - bring in a module
import math

# from - import specific items from a module
from math import pi, sqrt

# as - give an alias
import math as m
```

### Exception Keywords (Preview)

You'll learn these in Chapter 22:

```python
# try, except, finally - handle errors
# raise - create an error
# assert - debugging check
```

## Keywords You'll Use Most Often

### True, False, None

The most fundamental keywords:

```python
# Booleans for conditions
is_logged_in = True
has_permission = False

# None for "no value"
user_input = None  # Will be filled later
search_result = None  # Nothing found
```

### and, or, not

For combining conditions (Lesson 3):

```python
# Complex conditions
can_vote = age >= 18 and is_citizen
gets_bonus = is_vip or score > 1000
is_active = not is_deleted
```

### in, not in

For checking membership:

```python
# Check if value exists
valid_grades = ["A", "B", "C", "D", "F"]
grade = "B"
is_valid = grade in valid_grades
print(f"Valid grade: {is_valid}")  # True

# Check if value is missing
missing = "Z" not in valid_grades
print(f"Z is not valid: {missing}")  # True
```

### is, is not

For identity checking (especially with None):

```python
# The correct way to check for None
value = None

# Good - use 'is' for None
is_empty = value is None
print(f"Is None: {is_empty}")  # True

# Also good
has_value = value is not None
print(f"Has value: {has_value}")  # False
```

**Important**: Use `is` for `None`, `True`, `False`. Use `==` for other value comparisons.

```python
# Correct usage
x = None
check1 = x is None      # Good for None

y = 5
check2 = y == 5         # Good for values
```

## Checking if a Word is a Keyword

```python
import keyword

# Check specific words
print(keyword.iskeyword("for"))      # True
print(keyword.iskeyword("print"))    # False (it's a built-in function, not keyword)
print(keyword.iskeyword("score"))    # False (valid variable name)
print(keyword.iskeyword("class"))    # True
```

## Common Naming Mistakes

### Mistake 1: Using Keywords as Variables

```python
# WRONG - these cause SyntaxError
# class = "Biology"
# for = 10
# import = "data.csv"

# RIGHT - use descriptive alternatives
course_class = "Biology"
iteration_count = 10
import_file = "data.csv"
```

### Mistake 2: Shadowing Built-in Functions

While not keywords, avoid using these names too:

```python
# AVOID - shadows built-in functions
# list = [1, 2, 3]     # Now you can't use list()
# str = "hello"        # Now you can't use str()
# print = "output"     # Now you can't use print()

# BETTER - use descriptive names
numbers_list = [1, 2, 3]
greeting_str = "hello"
output_text = "output"
```

### Mistake 3: Case Sensitivity

```python
# These are different!
true = "this is a string"    # Valid variable (but confusing!)
# True = "this is a string"  # SyntaxError! True is a keyword

# Don't do this - it's confusing
none = "something"  # Valid but terrible practice
# None = "something"  # SyntaxError! None is a keyword
```

## Practice: Identify the Keywords

Look at this code and identify all keywords:

```python
import math

is_positive = True
value = None

has_value = value is not None
in_range = 0 in [0, 1, 2] and is_positive

pi_value = math.pi
```

**Keywords used**: `import`, `True`, `None`, `is`, `not`, `in`, `and`

## Reference: All 35 Python Keywords

| Category | Keywords |
|----------|----------|
| Values | `True`, `False`, `None` |
| Logical | `and`, `or`, `not` |
| Membership/Identity | `in`, `is` |
| Control Flow | `if`, `elif`, `else`, `for`, `while`, `break`, `continue`, `pass` |
| Functions | `def`, `return`, `lambda`, `yield` |
| Classes | `class` |
| Imports | `import`, `from`, `as` |
| Exceptions | `try`, `except`, `finally`, `raise`, `assert` |
| Context | `with` |
| Async | `async`, `await` |
| Scope | `global`, `nonlocal` |
| Other | `del` |

## Debugging With AI

**"I can't use this variable name"**

```python
class = "Math 101"  # SyntaxError!
```

Tell AI: *"I tried to name my variable 'class' but got a SyntaxError."*

AI will explain: `class` is a reserved keyword. Use `course_class`, `class_name`, or another descriptive name instead.

**"What's the difference between is and =="**

Tell AI: *"When should I use 'is' vs '==' in Python?"*

AI will explain: Use `is` for identity (same object) and checking `None`. Use `==` for value equality.

## Try With AI

**Explore keywords:**
> "List all Python keywords and group them by what they're used for. Which ones are most important for beginners? Which ones will I learn later?"

**Understand identity vs equality:**
> "Explain the difference between `x is None` and `x == None`. Why is `is` preferred for None? Show examples where `is` and `==` give different results."

**Check your names:**
> "I want to name my variables: list, type, input, for, score, class_name. Which of these are bad choices and why?"
