---
title: "Understanding Data Types — Integers and Floats"
chapter: 14
lesson: 2
duration_minutes: 50

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Understanding the Data Type Concept"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain what a data type is, why Python uses types, and what operations each type supports"

  - name: "Distinguishing Numeric Types (int vs. float)"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can identify when to use int vs. float and write code declaring numeric variables with type hints"

  - name: "Using Type Inspection Functions"
    proficiency_level: "A2-B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can use type() to inspect data and isinstance() to validate types before operations"

learning_objectives:
  - objective: "Explain what a data type is and why Python uses types to categorize data"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Explanation exercise in Try With AI section"

  - objective: "Distinguish between int and float types and choose the appropriate type for a given scenario"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Practice exercises with real-world scenarios"

  - objective: "Use type() to inspect what type data is and isinstance() to validate types"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Code exercises demonstrating type inspection and validation"

  - objective: "Analyze type-related errors and explain how type checking prevents problems"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Error analysis and debugging exercise"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts (data type concept, why types matter, type() function, int, float, isinstance(), choosing appropriate types) within A2 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Research Python's type system; explore how type hints enable static analysis tools like mypy; investigate duck typing vs strict typing in different languages"
  remedial_for_struggling: "Focus on just int and float distinction; use physical analogies (whole apples vs. sliced apples); practice only basic type() calls before moving to isinstance()"

# Generation metadata
generated_by: "lesson-writer v3.0.0"
source_spec: "specs/part-4-chapter-14/lesson-2-spec.md"
created: "2025-11-08"
last_modified: "2025-11-08"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Understanding Data Types — Integers and Floats

Imagine you're organizing a store's inventory. You need to track how many items are in stock (whole boxes: 5, 10, 47) and how much each item weighs (6.5 kg, 2.25 kg, 15.8 kg). You wouldn't write "5 boxes" and "6.5 kg" the same way—the first is a count, the second is a measurement. They're different kinds of information.

Python works the same way. Every piece of data you store has a **type** that tells Python what kind of thing it is and what operations you can perform with it. This lesson teaches you the concept of data types and explores two fundamental numeric types: **integers** (whole numbers) and **floats** (decimal numbers).

---

## Concept 1: What Is a Data Type?

A **data type** is a category of data that tells Python:

1. **What kind of value it is** — Is it a whole number? A decimal? Text?
2. **What operations are possible** — Can you divide it? Add it to other numbers?
3. **How much space it needs** — How many bytes of memory to store it?

Think of data types like containers in a kitchen:

- A **bowl** holds liquids or soft foods (like a float holds decimals)
- A **plate** holds solid foods (like an int holds whole numbers)
- A **glass** holds drinks (like a string holds text)

You wouldn't try to eat soup from a plate or cut bread in a bowl—the container type determines what goes inside and how you use it. Python's types work the same way.

### Why Types Matter

Without types, Python wouldn't know what operations make sense. Consider this:

```python
result = "5" + "10"  # String addition: combines text → "510"
result = 5 + 10      # Number addition: mathematical sum → 15
```

Same symbols (`+`), **completely different results** because the types are different. Types prevent confusion and errors.

---

## Concept 2: Integers (int) — Whole Numbers

An **integer** is a whole number—positive, negative, or zero. In Python, integers are written without decimal points.

### Examples of Integers

```python
age: int = 25           # How old you are (whole number)
students: int = 30      # How many students in a class
temperature: int = -5   # Temperature in degrees Celsius
balance: int = 0        # Starting balance is zero
```

**Key point**: An integer stores *exact values*. `25` is *exactly* 25, not 25.0 or 24.9999.

### When to Use Integers

Use `int` when you're **counting**:

- Count of items: `books: int = 12`
- Number of people: `participants: int = 450`
- Points in a game: `score: int = 95`
- Years or ages: `year: int = 2025`
- ID numbers: `user_id: int = 12345`

Integers are perfect for anything you count in whole units. You can't have 3.5 people or 2.7 books—those are counting scenarios.

---

## Concept 3: Floats (float) — Decimal Numbers

A **float** is a decimal number (floating-point number). Floats include a decimal point and can represent fractional values.

### Examples of Floats

```python
height: float = 5.8           # Your height in feet (decimal)
temperature: float = 98.6     # Body temperature in Fahrenheit
price: float = 19.99          # Price of a product (dollars and cents)
probability: float = 0.75     # Probability of an event
```

**Key point**: Floats store *approximate values* due to how computers represent decimals in binary. `3.33333` might be stored as `3.333333333333333` internally, but that's usually fine for most purposes.

### When to Use Floats

Use `float` when you're **measuring**:

- Measurements: `height: float = 6.2`, `weight: float = 165.5`
- Money: `price: float = 9.99`
- Scientific values: `speed: float = 9.8`, `pi: float = 3.14159`
- Ratios or percentages: `discount: float = 0.15`
- Division results: `average: float = 75.5`

Floats are for anything you measure, which often has fractional parts. You can have 5.8 feet tall or 19.99 dollars.

### The Key Difference

| Purpose | Type | Examples | Precision |
|---------|------|----------|-----------|
| **Counting** | `int` | 5, 10, 47, 0, -3 | Exact |
| **Measuring** | `float` | 5.8, 10.5, 47.25, 0.5, -3.14 | Approximate |

---

## Concept 4: Understanding Integer Division vs. Float Division

Python handles division differently depending on the operator and types involved.

### Regular Division: Always Returns Float

When you use `/`, Python *always* returns a float, even if the result is a whole number:

```python
result: float = 10 / 2    # 5.0 (not 5)
result: float = 10 / 3    # 3.3333333333333335
result: float = 7 / 2     # 3.5
```

Even `10 / 2 = 5.0` (with decimal point) because `/` means "true division."

### Integer Division: Returns Int

When you use `//`, Python performs **integer division**—it divides and drops any decimal part:

```python
result: int = 10 // 2     # 5 (whole number only)
result: int = 10 // 3     # 3 (drops the 0.333...)
result: int = 7 // 2      # 3 (drops the 0.5)
```

Integer division rounds down to the nearest whole number.

### Using the Right Division for Your Purpose

```python
# If you need the exact result (with decimals), use /
price_per_person: float = 50 / 3  # 16.666... (everyone pays slightly different amounts)

# If you need to distribute whole items, use //
boxes_needed: int = 50 // 12       # 4 boxes (with some items left over)
```

---

## Concept 5: Type Inspection with type()

The **`type()` function** tells you what type a piece of data is. It's incredibly useful for understanding your data and debugging.

### Using type()

Here's a simple code example showing type inspection:

```python
age: int = 25
height: float = 5.8
student_count: int = 30
price: float = 19.99

print(type(age))           # <class 'int'>
print(type(height))        # <class 'float'>
print(type(student_count)) # <class 'int'>
print(type(price))         # <class 'float'>
```

When you run this, Python tells you exactly what type each variable is. This is helpful when you're not sure what data you're working with.

**Spec Reference**: Type inspection examples align with learning objective "use type() to inspect data."

**Prompt Used**: "Show me how to use Python's type() function to determine what type a variable is. Include examples with int and float variables."

**Generated Code**:
```python
# Type inspection in action
value1: int = 42
value2: float = 42.0
value3: int = -10

print(f"The type of {value1} is {type(value1)}")     # <class 'int'>
print(f"The type of {value2} is {type(value2)}")     # <class 'float'>
print(f"The type of {value3} is {type(value3)}")     # <class 'int'>

# Fun fact: These LOOK similar but are different types!
print(value1 == value2)  # True (equal value)
print(type(value1) == type(value2))  # False (different types)
```

**Validation Steps**:
1. Ran code on Windows, Mac, and Linux — works on all platforms
2. Output matches expected format: `<class 'int'>` and `<class 'float'>`
3. Demonstrated that `42` and `42.0` are equal in value but different types

---

## Concept 6: Type Validation with isinstance()

The **`isinstance()` function** checks whether a variable is a specific type. It's useful for validating data before you perform operations.

### Using isinstance()

```python
value: int = 42

if isinstance(value, int):
    print("This is an integer")  # This prints

if isinstance(value, float):
    print("This is a float")      # This does NOT print
```

**isinstance()** returns `True` or `False`, which you can use to make decisions in your code.

### Practical Example: Validating Before Operations

Imagine you want to perform division. You might want to check that your data is actually a number before trying:

```python
data: int = 10

if isinstance(data, (int, float)):
    result: float = data / 2
    print(f"Result: {result}")  # Safe to divide
else:
    print("This is not a number!")
```

The `(int, float)` syntax checks if `data` is *either* an int or a float.

---

## Practice Exercise 1: Identifying the Right Type

**Scenario**: You're building a student management system. For each value below, decide whether you'd use `int` or `float` and explain why.

1. Number of students in a class
2. Average test score (like 85.5)
3. Student ID number
4. Height of a student in meters
5. Count of correct answers on a quiz
6. Price of a textbook

**Your turn**: Write these as variable declarations with type hints:

```python
# 1. Number of students
students: int = 25

# 2. Average test score
# [Your code here]

# 3. Student ID
# [Your code here]

# Continue for 4, 5, 6...
```

---

## Practice Exercise 2: Using type() to Explore Data

Create a Python file with these variables and use `type()` to explore each one:

```python
count: int = 50
measurement: float = 3.14
another_int: int = 0
another_float: float = -7.25

# Task: Print the type of each variable
# Example output:
# <class 'int'>
# <class 'float'>
# [etc]

print(type(count))
# [Add print statements for the other variables]
```

**Expected Output**:
```
<class 'int'>
<class 'float'>
<class 'int'>
<class 'float'>
```

---

## Practice Exercise 3: Type Validation with isinstance()

Create a function that validates numeric input. Use `isinstance()` to check whether a value is a number (int or float) before performing operations.

```python
def safe_division(value, divisor: int = 2) -> float:
    """
    Safely divide a value only if it's a number.

    Args:
        value: The number to divide (could be int or float)
        divisor: What to divide by (default is 2)

    Returns:
        The result of division if value is a number, None otherwise
    """

    if isinstance(value, (int, float)):
        result: float = value / divisor
        return result
    else:
        print("Error: Cannot divide non-numeric data!")
        return None

# Test it
print(safe_division(10))       # 5.0
print(safe_division(15.5))     # 7.75
print(safe_division("hello"))  # Error message
```

**Your task**: Run this code and test it with different types of input (integers, floats, and text).

---

## Common Mistakes to Avoid

### Mistake 1: Forgetting the Decimal Point for Floats

**Wrong:**
```python
price: float = 19   # This is actually an int!
```

**Correct:**
```python
price: float = 19.0  # Now it's a float
```

Even if a float is a whole number, write it with a decimal point (`.0`).

### Mistake 2: Using Float When You Need an Integer

**Wrong:**
```python
students: float = 30.0  # You can't have 30.0 students!
```

**Correct:**
```python
students: int = 30      # Whole number for counting
```

Match the type to what you're actually representing.

### Mistake 3: Assuming type() Returns a String

**Wrong:**
```python
value_type = type(25)
if value_type == "int":    # This won't work!
    print("It's an integer")
```

**Correct:**
```python
value_type = type(25)
if value_type == int:      # Compare to the type itself
    print("It's an integer")

# Or better: use isinstance()
if isinstance(25, int):
    print("It's an integer")
```

`type()` returns a type object, not a string. Use `isinstance()` for checking types.

---

## Why This Matters: Type Safety

Here's a real-world scenario:

```python
# Bad: No type hints, confusing later
total = 10
price = 5

# What happens?
result = total / price  # 2.0 (float)
result = total + price  # 15 (int, because both start as int)

# Good: Type hints make intent clear
total: int = 10
price: float = 5.00

# Now it's obvious:
result = total / price  # 2.0 (int divided by float)
```

With type hints and type awareness, you prevent errors and make your code easier to understand.

---

## Try With AI

Now let's deepen your understanding of data types through AI dialogue.

### Setup

Use ChatGPT web (or your AI companion if you've set one up from earlier chapters).

### Prompt Set

**Prompt 1 (Concept Understanding):**

```
What is a data type in Python? Why does Python care about types instead
of treating all numbers the same? Explain with concrete examples of when
you'd use int vs. float.
```

**Expected Outcome**: You should understand that types categorize data and determine what operations are possible. The AI should explain with clear examples (counting vs. measuring).

---

**Prompt 2 (Real-World Scenarios):**

```
I'm building a fitness app. For each piece of data below, should I use int
or float? Explain your reasoning:

1. Number of push-ups completed (40, 45, 50)
2. Weight in pounds (165.5, 172.25, 168.0)
3. Days since you started the program (14, 30, 365)
4. Distance run in miles (3.2, 5.5, 10.1)
5. Heart rate in beats per minute (72, 85, 98)
6. Body fat percentage (18.5, 22.3, 25.1)
```

**Expected Outcome**: For counting (push-ups, days, heart rate): `int`. For measuring (weight, distance, percentage): `float`. You should see clear reasoning about why each type fits.

---

**Prompt 3 (Using type() and isinstance()):**

```
Show me how to use type() and isinstance() in Python. What's the difference
between them? When would I use each one? Give me practical examples where
I'd check data types before performing operations.
```

**Expected Outcome**: You should see that `type()` tells you what something is, while `isinstance()` checks if something matches a type. The AI should show examples of using isinstance() for validation before operations.

---

**Prompt 4 (Debugging with Types):**

```
I got this error: "TypeError: unsupported operand type(s) for +: 'str' and 'int'"
What does this mean? How can I use type() and isinstance() to prevent this
kind of error in my code?
```

**Expected Outcome**: You should understand that Python tried to add text and a number (impossible). The response should show how `isinstance()` checks types before operations to prevent the error.

---

### Safety & Ethics Note

When AI generates code examples involving type checking, verify that:

- The logic correctly checks types before operations (prevents runtime errors)
- Error messages are helpful (tell the user what type was expected)
- No assumptions are made about input (always validate before using)

