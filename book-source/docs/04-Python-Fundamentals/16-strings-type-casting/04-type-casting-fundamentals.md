---
title: "Type Casting Fundamentals: Converting Between Types Safely"
chapter: 16
lesson: 4
duration_minutes: 52

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "String-to-Number Conversions"
    proficiency_level: "A2-B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can convert string to int/float using int() and float(); predict failures; validate success with isinstance()"

  - name: "Number-to-String Conversions"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can convert numbers to strings using str(); understand why (for concatenation or output)"

  - name: "Boolean Conversions with bool()"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Literacy"
    measurable_at_this_level: "Student can convert values to bool using bool(); predict True/False conversion results for strings and numbers"

  - name: "Error Handling for Type Conversions"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply + Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can predict ValueError; validate input before conversion; ask AI for solutions when errors occur"

  - name: "Type Validation and Validation-First Thinking"
    proficiency_level: "B1"
    category: "Technical/Metacognitive"
    bloom_level: "Apply + Analyze"
    digcomp_area: "Safety"
    measurable_at_this_level: "Student uses isinstance() and type() to validate conversions; describes intent using type hints"

learning_objectives:
  - objective: "Convert between core scalar types (int, float, str, bool) safely and correctly predict conversion outcomes"
    proficiency_level: "A2-B1"
    bloom_level: "Apply"
    assessment_method: "Conversion exercises with validation"

  - objective: "Identify when type conversion will fail and apply validation-first approach before conversions"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Error prediction and handling exercises"

  - objective: "Use isinstance() and type() to validate conversions and demonstrate type safety understanding"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Type validation in code examples"

cognitive_load:
  new_concepts: 5
  assessment: "5 new concepts (string-to-number, number-to-string, boolean conversions, error handling, validation-first) at A2-B1 limit"

differentiation:
  extension_for_advanced: "Explore conversion edge cases (whitespace, decimal strings with int(), type coercion in expressions); ask AI about type hierarchies"
  remedial_for_struggling: "Focus on simple conversions first (int and str); practice validation patterns before exploring error cases; use isinstance() as safety net"

# Generation metadata
generated_by: "content-implementer v1.0.0"
source_spec: "specs/part-4-chapter-16/spec.md"
created: "2025-11-08"
last_modified: "2025-11-08"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Type Casting Fundamentals: Converting Between Types Safely

Every program needs to transform data. User input arrives as strings—but you might need numbers for calculations. Numbers need to become strings for display. This lesson teaches you how to safely convert between Python's core types (int, float, str, bool) and validate that conversions work correctly.

In AI-Native development, you describe what transformation you need ("I need to convert user input to a number"), explore how to do it safely with your AI companion, validate the result worked as expected, and learn from errors when conversions fail. This lesson builds that pattern.

## What Type Casting Is and Why It Matters

**Type casting** is transforming data from one type to another. It happens constantly in real programs:

- User types "42" into a text box → you convert to integer for math
- You calculate a total price → convert to string for display
- User enters text → check if it's "true" or "false" value for a decision

Python provides built-in functions to cast between types: `int()`, `float()`, `str()`, and `bool()`. Understanding when and how to use these functions—and when they fail—is essential for robust programs.

There are two ways types can change:

1. **Implicit conversion**: Python changes types automatically (rare, mostly in expressions)
2. **Explicit conversion**: You explicitly call a conversion function like `int()`

This lesson focuses on explicit conversions you control.

### Built-in Functions You'll Use in This Lesson

Throughout this lesson, you'll use Python's **built-in functions**—utility tools that Python provides automatically. These work with many types of data and don't require importing:

**Type Conversion Functions** (new in this lesson):
- **`int(value)`** - converts to integer
- **`float(value)`** - converts to decimal number
- **`str(value)`** - converts to string
- **`bool(value)`** - converts to boolean (True/False)

**Type Validation Functions** (from Lesson 1):
- **`isinstance(value, type)`** - checks if a value is a specific type (returns True/False)
- **`type(value)`** - shows exactly what type a value is
- **`len(value)`** - counts characters in a string

Think of these as Python's toolbox. You don't need to understand how they work internally—just when to use them and what they return.

## String to Numbers: int() and float()

The most common conversion is transforming user input (always strings) into numbers for calculations.

#### Example 4.1: Converting Strings to Numbers

```python
# Converting string "42" to integer 42
num_str: str = "42"
num_int: int = int(num_str)  # "42" becomes 42
print(f"String: {num_str}, Integer: {num_int}, Type: {type(num_int)}")
# String: 42, Integer: 42, Type: <class 'int'>

# Converting string "3.14" to float 3.14
price_str: str = "19.99"
price_float: float = float(price_str)  # "19.99" becomes 19.99
print(f"String: {price_str}, Float: {price_float}, Type: {type(price_float)}")
# String: 19.99, Float: 19.99, Type: <class 'float'>

# Real-world pattern: Convert user input for calculations
user_age_str: str = input("Enter your age: ")  # User types "25"
user_age: int = int(user_age_str)  # Convert to integer
next_year: int = user_age + 1  # Now we can do math
print(f"You are {user_age} now, {next_year} next year")

# Validate conversion succeeded using isinstance()
input_value: str = "100"
converted: int = int(input_value)
if isinstance(converted, int):
    print(f"Successfully converted '{input_value}' to integer {converted}")

# Practical: Math with converted values
length_str: str = "5.5"
width_str: str = "3.2"
length: float = float(length_str)
width: float = float(width_str)
area: float = length * width
print(f"Area: {area:.2f}")  # Formatted with f-string
```

**Key Insight**: The `int()` and `float()` functions transform strings to numbers. Notice how we use `isinstance()` to verify the conversion succeeded.

#### 💬 AI Colearning Prompt

> "Explain why `int('3.14')` fails but `float('3.14')` works. What's the difference between how Python parses integer strings vs float strings?"

---

## When Conversions Fail: Understanding Errors

Not every string can convert to a number. This is where validation-first thinking matters.

#### Example 4.2: Invalid Conversions and Validation Patterns

```python
# This works: "42" is a valid integer string
num1: int = int("42")  # ✓ Success

# This FAILS: Can't convert "3.14" directly to int (has decimal point)
# num2: int = int("3.14")  # ✗ ValueError: invalid literal for int()
# If you run this, Python stops with an error!

# Workaround: Convert via float first
num2: int = int(float("3.14"))  # ✓ "3.14" → 3.14 → 3 (decimal discarded)

# This FAILS: "abc" is not a number
# num3: int = int("abc")  # ✗ ValueError: invalid literal for int()
# Python can't convert letters to numbers!

# Best practice: Validate BEFORE converting
user_input: str = "25 apples"
if user_input.isdigit():  # Check if string contains only digits
    count: int = int(user_input)
    print(f"Count: {count}")
else:
    print(f"'{user_input}' is not a valid number - skipping conversion")

# Another example: Check for valid format
age_input: str = "twenty-five"
if age_input.isdigit():
    age: int = int(age_input)
    print(f"Age: {age}")
else:
    print(f"Cannot convert '{age_input}' to integer - please enter digits only")

# Whitespace handling (very common with user input)
user_age: str = " 42 "  # User accidentally added spaces
cleaned_age: str = user_age.strip()  # Remove whitespace FIRST
# Now check if it's valid
if cleaned_age.isdigit():
    age: int = int(cleaned_age)  # Now conversion works safely
    print(f"Age: {age}")
```

#### 🎓 Expert Insight

> Errors are information. When a conversion fails with `ValueError`, Python is protecting you from bad data. Instead of fixing errors after they happen, validate FIRST with `.isdigit()`, `.strip()`, or other checks. Ask your AI: "How can I check if a string is valid before converting?" This validation-first approach prevents errors entirely.

---

## Numbers to Strings: str()

Converting numbers to strings is simpler—it always succeeds. You do this when you need to concatenate or display numbers.

#### Example 4.3: Converting Numbers to Strings

```python
# Integer to string
count: int = 42
count_str: str = str(count)  # 42 becomes "42"
print(f"Count: {count_str}, Type: {type(count_str)}")
# Count: 42, Type: <class 'str'>

# Float to string
price: float = 19.99
price_str: str = str(price)  # 19.99 becomes "19.99"
print(f"Price: {price_str}")

# Common use: Concatenation requires strings
x: int = 5
y: int = 3
result: int = x + y  # 8 (math)
math_result: str = f"{x} + {y} = {result}"  # "5 + 3 = 8" (string)
print(math_result)

# Note: F-strings handle conversion automatically
message: str = f"You have {count} items"  # No str() needed!

# Compare: Old way (without f-strings):
# message_old = "You have " + str(count) + " items"

# Validation: str() always succeeds
num: float = 3.14159
converted: str = str(num)
print(f"Original: {num} ({type(num).__name__}), Converted: {converted} ({type(converted).__name__})")
# Original: 3.14159 (<class 'float'>), Converted: 3.14159 (<class 'str'>)
```

**Why This Matters**: Numbers can't be concatenated directly with strings (you'd get a TypeError). Converting to strings (or using f-strings, which do it automatically) solves this.

#### 🤝 Practice Exercise

> **Ask your AI**: "Show me 5 real-world examples where type conversion matters (user input, database queries, calculations). For each, what validation would you do BEFORE conversion? Then explain the validation-first pattern and why it prevents errors."

**Expected Outcome**: You understand real-world type conversion scenarios and learn to validate before converting.

---

## Boolean Conversions: The bool() Function

Just like you can convert values to `int()`, `float()`, or `str()`, you can also convert any value to a boolean (`True` or `False`) using the `bool()` function. Every value in Python can be represented as either True or False.

#### Example 4.4: Converting Values to Booleans

```python
# String to boolean: Non-empty strings become True, empty becomes False
bool_empty: bool = bool("")       # False (empty string)
bool_text: bool = bool("hello")   # True (non-empty string)
bool_space: bool = bool(" ")      # True (space is non-empty!)
bool_zero_str: bool = bool("0")   # True (string "0" is non-empty!)

print(f"bool(''): {bool_empty}")          # False
print(f"bool('hello'): {bool_text}")      # True
print(f"bool(' '): {bool_space}")         # True
print(f"bool('0'): {bool_zero_str}")      # True (surprising!)

# Number to boolean: 0 is False, any other number is True
bool_zero_int: bool = bool(0)      # False
bool_one: bool = bool(1)           # True
bool_neg: bool = bool(-5)          # True
bool_float: bool = bool(0.0)       # False
bool_pi: bool = bool(3.14)         # True

print(f"bool(0): {bool_zero_int}")       # False
print(f"bool(1): {bool_one}")            # True
print(f"bool(-5): {bool_neg}")           # True
print(f"bool(0.0): {bool_float}")        # False
print(f"bool(3.14): {bool_pi}")          # True

# Validation: Verify conversion type
result: bool = bool(42)
print(f"Type: {type(result)}, Value: {result}")  # Type: <class 'bool'>, Value: True

# Another validation example
converted: bool = bool("hello")
print(f"isinstance check: {isinstance(converted, bool)}")  # True
```

**Key Pattern**: The `bool()` function follows these conversion rules:
- **Strings**: Empty string `""` → False; any other string → True
- **Numbers**: `0` and `0.0` → False; any other number → True
- **Use this for**: Understanding what Python considers "empty" vs "has a value"

#### 🔮 Coming in Chapter 17

> You'll use boolean conversions in **conditionals** (Chapter 17: Control Flow and Loops). There, you'll learn why understanding True/False matters for `if` statements and decision-making in your programs. For now, focus on how `bool()` converts different types.

---

## Validation-First Type Safety

The most important pattern: **Validate before converting**. This prevents errors and makes your code robust.

#### Example 4.5: Validation-First Approach

```python
# Pattern 1: Simple validation before conversion
user_input: str = "42"

if user_input.isdigit():
    num: int = int(user_input)
    result: int = num * 2
    print(f"Doubled: {result}")
else:
    print(f"'{user_input}' is not a valid integer")

# Pattern 2: Multi-step validation with cleaning
age_input: str = "  25  "
cleaned: str = age_input.strip()  # Step 1: Remove whitespace
if cleaned.isdigit():               # Step 2: Check if valid digits
    age: int = int(cleaned)         # Step 3: Convert (safe now)
    print(f"Age: {age}")
else:
    print(f"Invalid age: '{age_input}' - please enter digits only")

# Pattern 3: Validate after conversion
raw_price: str = "19.99"
price: float = float(raw_price)

# Verify the conversion succeeded as expected
if isinstance(price, float):
    total: float = price * 1.08  # Add 8% tax
    print(f"Total with tax: ${total:.2f}")
else:
    print("Conversion failed")

# Pattern 4: Check type before using
value: str = "hello"
if isinstance(value, str):
    length: int = len(value)
    print(f"String length: {length}")

# Another example: Checking multiple values
data: str = "42"
if isinstance(data, str) and data.isdigit():
    number: int = int(data)
    print(f"Converted: {number}")

# Validation-first thinking: Always know what type you expect and verify you got it
```

**Core Principle**: Validation-first means you check that your input makes sense BEFORE you try to convert it. This prevents ValueError and makes errors easier to understand.

---

## Connecting the Dots: When to Use What

This table helps you choose the right conversion:

| Need | Use | Example | Result Type |
|------|-----|---------|-------------|
| String input → math | `int()` or `float()` | `int("42")` | int or float |
| Number → display | `str()` (or f-string) | `str(42)` | str |
| Check truthiness | `bool()` | `bool("")` | bool |
| Validate type | `isinstance()` | `isinstance(x, int)` | bool |

---

## Try With AI: Type Conversion Debug Challenge

You've learned `int()`, `float()`, `str()`, and `bool()` conversions. Now debug real conversion errors to understand when and why conversions fail—with AI as your debugging partner.

### Part 1: Break Conversions Deliberately (Your Turn First)

**Before asking AI**, run these conversions and observe what happens:

```python
# Which of these will crash? Predict first, then test:
test_1 = int("42")        # Your prediction: ___
test_2 = int("3.14")      # Your prediction: ___
test_3 = int("  100  ")   # Your prediction: ___
test_4 = float("hello")   # Your prediction: ___
test_5 = bool("")         # Your prediction: ___
test_6 = bool("0")        # Your prediction: ___
```

**Your task**:
1. Predict which will work, which will crash
2. Run the code and collect the actual errors
3. Write down: WHY did each succeed or fail?

---

### Part 2: AI Explains Conversion Rules (Discovery)

Share your error results with AI:

> "I tested type conversions and got these results: [paste predictions vs. actual]. Explain:
> 1. Why does `int('3.14')` fail but `float('3.14')` works?
> 2. Why does `int(' 100 ')` work (with spaces)?
> 3. Why does `bool('')` return False but `bool('0')` return True?
> 4. What's the rule for when conversions succeed vs. fail?"

**Your task**: Evaluate AI's explanation.
- Does it explain that `int()` requires EXACTLY an integer format?
- Does it show that `int()` ignores leading/trailing whitespace?
- Can you explain the bool truthiness rules now?

---

### Part 3: Student Teaches AI (Safe Conversion Patterns)

AI explained conversion rules. But does it know the SAFEST way to convert?

Challenge AI with validation patterns:

> "Show me 3 ways to safely convert user input `'42'` to int:
> 1. Using try/except to catch ValueError
> 2. Using `.isdigit()` to check before converting
> 3. Using a default value if conversion fails
>
> For each, explain: When would you use this pattern? Which is most 'pythonic'? What are the tradeoffs?"

**Your task**: Compare the patterns.
- Which pattern is most readable?
- Which handles edge cases like negative numbers (`'-42'`)?
- Which pattern would YOU use in production code?

This teaches AI about defensive programming styles.

---

### Part 4: Build Input Validator Together (Convergence)

Now build a robust input validator with AI:

> "Build a safe calculator that:
> 1. Takes two string inputs (number1, number2)
> 2. Validates both can convert to float
> 3. Performs addition if valid
> 4. Shows clear error message if invalid
> 5. Uses type hints throughout
>
> Make it crash-proof—handle all edge cases (empty strings, text, spaces, negative numbers)."

**Your task**: Review AI's validator.
- Does it check validity BEFORE converting?
- Does it handle all edge cases you tested in Part 1?
- Does it provide helpful error messages?

Iterate if needed:
> "The error messages aren't helpful. Make them explain WHAT was wrong and HOW to fix it."

---

**Time**: 25-30 minutes total
**Outcome**: You've discovered when conversions fail through experimentation, learned safe validation patterns, and built a crash-proof input validator that handles all edge cases.
