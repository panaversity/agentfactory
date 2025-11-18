---
title: "Type Utilities and Capstone Project"
chapter: 14
lesson: 5
duration_minutes: 65

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Using Type Inspection Functions"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can use type(), id(), and isinstance() to inspect and validate object types in real code"

  - name: "Understanding Type Casting"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can convert between types using explicit casting and recognize when Python performs implicit casting"

  - name: "Distinguishing Explicit vs Implicit Conversion"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can identify whether type conversion is explicit (programmer-requested) or implicit (Python automatic)"

  - name: "Awareness of Advanced Type Topics"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Remember"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student understands why integer interning and number systems matter, even if not using them immediately"

  - name: "Integrating Chapter Concepts in Projects"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can build small programs that combine type inspection, casting, and decision-making to solve real problems"

learning_objectives:
  - objective: "Use type(), id(), and isinstance() to inspect and validate object types in code"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Writing code that uses these functions to solve type-checking problems; identifying best function for different situations"

  - objective: "Perform explicit type casting between compatible types and explain what happens to data"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Converting between int/float/str/bool with code; explaining data loss when converting float to int"

  - objective: "Distinguish implicit casting (Python automatic) from explicit casting (programmer-controlled)"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Identifying automatic conversions in arithmetic expressions; explaining why explicit casting is sometimes safer"

  - objective: "Demonstrate awareness of advanced topics (interning, number systems) without needing mastery"
    proficiency_level: "B1"
    bloom_level: "Remember"
    assessment_method: "Explaining why interning exists and how to detect it with 'is' operator; recognizing binary/hex/octal notation"

  - objective: "Complete capstone project integrating all chapter knowledge: type inspection, casting, decision-making"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "Functional Type Explorer program that handles user input, detects types, performs casting, and displays results"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts (isinstance, id, type casting, implicit/explicit, interning, number systems, capstone) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Explore CPython internals: Why does Python cache small integers? Research object allocation strategies. Extend capstone with custom type detection for lists containing mixed types."
  remedial_for_struggling: "Focus on three core functions (type, isinstance, int/float/str casting). Skip advanced topics initially. Build simplified version of capstone with guided steps."

# Generation metadata
generated_by: "lesson-writer v3.0.0"
source_spec: "specs/013-chapter-14-redesign/spec.md"
created: "2025-01-15"
last_modified: "2025-01-15"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Type Utilities and Capstone Project

## What You'll Learn

- Inspecting types with type(), id(), isinstance()
- Converting between types (casting)
- When Python converts automatically
- How Python stores integers 
- Binary, hexadecimal, octal numbers
- Building a User Profile Validator

You've learned about Python's 13 data types across four lessons. Now it's time to master the tools that inspect, validate, and convert between types—and build a real project that brings everything together.

## The type() Function: Understanding Type Objects

You've used `type()` throughout this chapter to inspect data types. But what does `type()` actually return?

**Key insight**: `type()` doesn't return a string like `"int"`. It returns a **type object** — Python's internal representation of the type class.

```python
result = type(42)
print(result)              # <class 'int'>
print(type(result))        # <class 'type'> — the type of a type!
```

This matters when comparing types programmatically:

```python
age: int = 25
is_integer: bool = type(age) == int  # Comparing type objects
print(is_integer)  # True
```

**When to use type() vs isinstance()** (coming next):
- `type(x) == int`: Checks for exact type match only
- `isinstance(x, int)`: More flexible, accepts subclasses (preferred in most cases)

---

## The id() Function: Object Identity

Every object in Python has a unique identifier—its **memory address**. The `id()` function returns this.

```python
x: int = 42
print(id(x))  # Something like 140234567890 (exact number varies by system)
```

**Why does this matter?**

1. **Understanding object identity** — Two variables might hold the same value but be different objects
2. **Debugging memory issues** — Professional developers use `id()` to track object allocation
3. **Understanding Python's optimization** — Some values are cached (shown next with interning)

### A Critical Insight: The None Singleton

Python has a special rule: **there is only ONE `None` object in your entire program**. Every variable set to `None` points to the same object.

```python
x: None = None
y: None = None

print(id(x))           # Same ID
print(id(y))           # Same ID
print(x is y)          # True (same object!)
```

This is why the `is` operator is important: **`is` checks object identity, while `==` checks value equality**.

```python
a: int = 42
b: int = 42

print(a == b)          # True (values are equal)
print(id(a) == id(b))  # True! (Python caches small integers)
print(a is b)          # True! (same cached object)

x: int = 1000
y: int = 1000
print(x == y)          # True (values equal)
print(x is y)          # False! (different objects, no caching for large integers)
```

**Key insight**: For numbers -5 to 256, Python reuses the same object. For larger numbers, it creates new objects. This is **integer interning** (covered in advanced section below).

### Using id() in Real Code

When might you use `id()` in practice? Mostly for debugging or learning how Python works. But here's a practical example:

```python
# Creating two greetings to compare their identity
greeting1: str = "hello"
greeting2: str = "hello"

print(greeting1 == greeting2)  # True (same value)
print(id(greeting1) == id(greeting2))  # Might be True! (Python optimizes string literals)
```

---

## The isinstance() Function: Type Checking

`isinstance()` is your primary tool for checking types in real programs.

**Syntax**: `isinstance(object, type)`

**Returns**: `True` if the object is an instance of that type, `False` otherwise

```python
age: int = 25
price: float = 19.99
name: str = "Alice"

print(isinstance(age, int))      # True
print(isinstance(price, float))  # True
print(isinstance(name, str))     # False (not a float)
```

### isinstance() vs type() == ...

Why is `isinstance()` better? Because it respects **inheritance** (a more advanced topic you'll hit in Object-Oriented Programming). For now, just remember:

- **`type(x) == int`** — Checks for exactly `int`, nothing else
- **`isinstance(x, int)`** — Checks if `x` is `int` or any subclass of `int`

In beginner code, they're equivalent. But `isinstance()` is the Pythonic way.

### Checking Multiple Types

`isinstance()` can check if an object is **one of several types**:

```python
value: float = 3.14

# Is value a number (int OR float)?
is_number: bool = isinstance(value, (int, float))
print(f"Is number: {is_number}")  # True

# Is value text OR a number?
is_text_or_number: bool = isinstance(value, (str, int, float))
print(f"Is text or number: {is_text_or_number}")  # True
```

### isinstance() in Real Code

Here's a practical example showing how to check types:

```python
# Checking different types and showing results
val1: str = "hello"
val2: int = 42
val3: float = 3.14

# Check type and prepare appropriate processing
is_text: bool = isinstance(val1, str)
is_integer: bool = isinstance(val2, int)
is_decimal: bool = isinstance(val3, float)

print(f"'{val1}' is text: {is_text}")      # True
print(f"{val2} is integer: {is_integer}")  # True
print(f"{val3} is decimal: {is_decimal}")  # True

# You'll learn to use these checks in conditional logic in Chapter 17
```

**Exercise**: Write code that checks whether a value is a number (int or float), text, or something else using isinstance().

---

## Type Casting: Converting Between Types

**Type casting** (also called **type conversion**) means converting data from one type to another.

### Explicit Casting: You Control It

You've been doing this without realizing it. When you write `int("25")`, you're explicitly converting a string to an integer.

**int() — Convert to integer:**

```python
# String to int
age_str: str = "25"
age_int: int = int(age_str)
print(age_int + 5)  # 30 (works because it's now a number)

# Float to int (LOSES THE DECIMAL!)
price: float = 19.99
whole: int = int(price)  # Becomes 19 (truncated, not rounded)
print(whole)  # 19

# Bool to int
flag: bool = True
as_int: int = int(flag)  # Becomes 1
print(as_int)  # 1
```

**float() — Convert to float:**

```python
# String to float
price_str: str = "19.99"
price: float = float(price_str)

# Int to float (no data loss)
whole: int = 42
decimal: float = float(whole)  # Becomes 42.0
print(decimal)  # 42.0
```

**str() — Convert to string:**

```python
# Number to string
count: int = 100
count_str: str = str(count)
print(count_str + " items")  # 100 items

# Any type to string
result: str = str(3.14)  # "3.14"
result2: str = str(True)  # "True"
result3: str = str([1, 2, 3])  # "[1, 2, 3]"
```

**bool() — Convert to boolean:**

```python
# Any value to bool (uses Python's truthy/falsy rules from Lesson 3)
print(bool(0))        # False (zero is falsy)
print(bool(42))       # True (non-zero is truthy)
print(bool(""))       # False (empty string is falsy)
print(bool("hello"))  # True (non-empty string is truthy)
print(bool([]))       # False (empty list is falsy)
print(bool([1, 2]))   # True (non-empty list is truthy)
print(bool(None))     # False (None is always falsy)
```

### Lossy Conversions: Data Loss

When you convert a **float to an int**, the decimal part is **truncated** (chopped off, not rounded):

```python
price: float = 19.99
whole: int = int(price)  # 19 (not 20!)

precise: float = 3.9
rounded_down: int = int(precise)  # 3 (loses .9)
```

If you need true rounding, use the built-in `round()` function:

```python
price: float = 19.99
rounded: int = round(price)  # 20 (proper rounding)
```

**Always be aware of data loss when casting**. Document your intention clearly:

```python
# Bad (unclear intent)
user_age = int(input("Age: "))  # User might enter 25.5, we silently truncate to 25

# Better (clear intent)
user_age_input: str = input("Age (whole number): ")
user_age: int = int(user_age_input)  # We explicitly expect a whole number
```

### Implicit Casting: Python Does It Automatically

Sometimes Python converts types **without you asking**. This happens in **mixed arithmetic** (numbers of different types in the same expression).

**Rule**: When you mix `int` and `float`, Python automatically converts the `int` to `float` and performs the operation as float arithmetic.

```python
result: float = 5 + 3.14  # int + float
print(result)              # 8.14 (float)
print(type(result))        # <class 'float'>

# Python automatically converted the int 5 to 5.0 (float)
# Then did: 5.0 + 3.14 = 8.14
```

This is convenient, but **be aware of precision loss with large numbers**:

```python
big_int: int = 999999999999999999
big_float: float = 0.1
result: float = big_int + big_float  # Int becomes float (might lose precision)
print(result)  # Due to float's limited precision, small errors creep in
```

### Why Explicit is Better Than Implicit

**Python's philosophy** (from "The Zen of Python"): "Explicit is better than implicit."

Compare these two:

```python
# Implicit (hard to debug)
user_input: str = "25"
new_age: int = user_input + 5  # TypeError! Can't add string and int

# Explicit (clear what's happening)
user_input: str = "25"
age_int: int = int(user_input)  # Conversion is visible in code
new_age: int = age_int + 5  # Now it works: 30
```

The explicit version is clearer. Future readers (or you six months later) immediately understand that conversion is happening.

### Type Casting Exercise

Create a program that:
1. Asks the user for their age as text (using `input()`)
2. Converts it to an integer
3. Calculates their age in 10 years
4. Converts the result back to a string
5. Prints a complete sentence

**Note**: What happens if the user enters "twenty five" instead of "25"? Your program will crash. We'll learn how to handle such errors gracefully in Chapter 21 (Exception Handling).

---

## Advanced Topic: Integer Interning (For Curious Learners)

This section is optional. It explains something advanced but doesn't affect your ability to write Python code.

### What Is Interning?

Python **caches integers from -5 to 256**. This means if you create two variables with the value 42, they're actually the same object in memory.

```python
a: int = 42
b: int = 42

print(id(a) == id(b))  # True! Same object
print(a is b)          # True! 'is' confirms they're identical
```

But larger numbers don't get cached:

```python
x: int = 1000
y: int = 1000

print(id(x) == id(y))  # Might be False (separate objects)
print(x is y)          # Might be False
```

### Why Does Python Do This?

**Memory efficiency**. Small integers (-5 to 256) are used constantly in programs (loop counters, indices, common values). By reusing the same object, Python saves memory. Since integers are immutable (can't be changed), it's safe to share them.

### Why Should You Care?

In practical code: **rarely**. You use `==` to compare values, not `is`.

```python
# This is what you do (correct)
age: int = 25
result: bool = age == 25
print(f"Age is 25: {result}")  # True - this is reliable!

# Don't do this (unreliable for numbers)
result2: bool = age is 25  # Works sometimes, fails sometimes (depends on caching!)
print(f"Age is 25 (using 'is'): {result2}")  # Might be True or False!
```

But understanding interning helps you grasp how Python manages memory behind the scenes.

**Note**: You'll learn conditional statements (`if`/`else`) in Chapter 17. For now, focus on understanding comparisons return `True` or `False`.

### Detecting Interning

Here's how to explore integer interning:

```python
# Small integers (cached)
a: int = 100
b: int = 100
print(f"100: a is b = {a is b}")  # True (cached)

# Boundary case
c: int = 256
d: int = 256
print(f"256: c is d = {c is d}")  # True (still cached at boundary)

# Just beyond cache
e: int = 257
f: int = 257
print(f"257: e is f = {e is f}")  # False (not cached)

# But equality still works
print(e == f)  # True (same value, different objects)
```

---

## Advanced Topic: Number Systems (For Curious Learners)

This section is optional. It explores how to write numbers in different bases.

### Beyond Decimal (Base 10)

You normally write numbers in **decimal** (base 10): 0, 1, 2, ..., 9, 10, 11, ...

But Python supports other number systems:

**Binary (Base 2)** — Uses digits 0 and 1

```python
binary: int = 0b1010  # 0b prefix means "binary"
print(binary)         # 10 (decimal equivalent)

# Binary is how computers think
# 0b1010 = 1×8 + 0×4 + 1×2 + 0×1 = 8 + 2 = 10
```

**Hexadecimal (Base 16)** — Uses 0-9 and A-F (A=10, B=11, ..., F=15)

```python
hex_val: int = 0x1A  # 0x prefix means "hexadecimal"
print(hex_val)       # 26 (decimal equivalent)

# 0x1A = 1×16 + 10×1 = 16 + 10 = 26
```

**Octal (Base 8)** — Uses digits 0-7

```python
octal: int = 0o12  # 0o prefix means "octal"
print(octal)       # 10 (decimal equivalent)

# 0o12 = 1×8 + 2×1 = 8 + 2 = 10
```

### Why Different Number Systems?

- **Binary**: Computers store everything as 0s and 1s. Useful for bit-level operations
- **Hexadecimal**: Compact way to write binary (one hex digit = 4 binary digits). Common in colors (#FF5733), memory addresses
- **Octal**: Historically used for file permissions in Unix/Linux

### Converting Between Systems

Python provides functions:

```python
# Convert to binary string
print(bin(10))      # '0b1010'

# Convert to hex string
print(hex(26))      # '0x1a'

# Convert to octal string
print(oct(10))      # '0o12'

# Convert back to decimal (using int() with base argument)
print(int('0b1010', 2))  # 10 (binary to decimal)
print(int('0x1A', 16))   # 26 (hex to decimal)
print(int('0o12', 8))    # 10 (octal to decimal)
```

### ASCII: Text as Numbers

Each character has a number. **ASCII** (American Standard Code for Information Interchange) maps characters to numbers:

```python
print(ord('A'))      # 65 (character to ASCII number)
print(ord('a'))      # 97
print(ord('0'))      # 48 (digit character zero, not the number zero)

print(chr(65))       # 'A' (ASCII number to character)
print(chr(97))       # 'a'
print(chr(48))       # '0'

# This is why 'A' and 'a' are different!
# 'A' = 65, 'a' = 97
```

**Practical use**: You'll use `ord()` and `chr()` rarely in beginner code, but they're essential for advanced text processing.

### Number Systems Exercise

Explore these on your own:
1. Write the numbers 0-15 in binary, hexadecimal, and decimal
2. Find the ASCII code for your name's first letter
3. Use `hex()` on some large numbers to see hexadecimal representation

---

## Capstone Project: User Profile Validator

Now let's build a real-world program that validates user registration data.

### Project Requirements

Create a **User Profile Validator** that:

1. **Accepts user input** for name, age, and email
2. **Validates data types** (name should be text, age should be a number)
3. **Performs type checking** using isinstance()
4. **Converts types as needed** (age from string to int)
5. **Reports validation results** clearly

### Why This Project Matters

In real applications, you can't trust user input. Someone might enter "twenty-five" for age instead of 25. Your code needs to:
- Check that data is the right type
- Convert when possible (type casting)
- Report errors when data is invalid
- Ensure data integrity before saving

### Starter Code

Here's the structure. Your job is to complete the validation logic:

```python
# User Profile Validator
# A real-world data validation program

print("=== User Profile Validator ===\n")

# Step 1: Collect user input
print("Enter your profile information:")
name: str = input("Name: ")
age_input: str = input("Age: ")
email: str = input("Email: ")

print("\n--- Validation Results ---\n")

# Step 2: Validate name (check type and content)
# Using truthy/falsy from Lesson 3: empty strings are falsy
name_is_string: bool = isinstance(name, str)
name_has_content: bool = bool(name)  # True if not empty
print(f"Name '{name}':")
print(f"  - Is string: {name_is_string}")
print(f"  - Has content: {name_has_content}")

# Step 3: Validate and convert age
# Convert age_input (string) to int
# Note: Checking if age is valid requires Chapter 15 (operators)
age: int = int(age_input)  # Assumes valid number for now
age_is_int: bool = isinstance(age, int)
print(f"Age {age}:")
print(f"  - Is integer: {age_is_int}")
print(f"  - Successfully converted from string")

# Step 4: Validate email (check type and content)
# Using truthy/falsy: empty strings are falsy
email_is_string: bool = isinstance(email, str)
email_has_content: bool = bool(email)  # True if not empty
print(f"Email '{email}':")
print(f"  - Is string: {email_is_string}")
print(f"  - Has content: {email_has_content}")
# Note: Checking email format (@) requires operators from Chapter 15

# Step 5: Summary
print("\n--- Summary ---")
print("All validations complete! Review results above.")

# Step 6: Show type information (educational)
print("\n--- Type Information ---")
print(f"Name type: {type(name)}")
print(f"Age type: {type(age)} (converted from {type(age_input)})")
print(f"Email type: {type(email)}")
```

### Example Output

```
=== User Profile Validator ===

Enter your profile information:
Name: Alice Smith
Age: 25
Email: alice@example.com

--- Validation Results ---

Name 'Alice Smith':
  - Is string: True
  - Has content: True
Age 25:
  - Is integer: True
  - Successfully converted from string
Email 'alice@example.com':
  - Is string: True
  - Has content: True

--- Summary ---
All validations complete! Review results above.

--- Type Information ---
Name type: <class 'str'>
Age type: <class 'int'> (converted from <class 'str'>)
Email type: <class 'str'>
```

### What You're Learning

This project teaches you:
- **Type validation** — Checking data is the right type before using it
- **Type casting** — Converting user input (always strings) to appropriate types
- **isinstance()** — Verifying types programmatically
- **Real-world patterns** — This is how production systems validate user data
- **Note**: Full error handling with try/except is covered in Chapter 21

### Capstone Extensions (Stretch Goals)

Once your validator works, enhance it:

1. **Phone validation** — Add phone number field with format checking
2. **Multiple users** — Validate multiple profiles and store results
3. **Advanced email validation** — Check for proper email format (username@domain.com)
4. **Save to file** — Write validated profiles to a JSON file
5. **Type statistics** — Track validation success rates

---

## Practice Exercises (Complete All 4)

### Exercise 1: Type Inspection Challenge

Write a program that analyzes different values and shows their type properties:
- Type name
- Whether it's numeric (int or float)
- Whether it's truthy (would be True in an if statement)
- Whether it can be converted to string

```python
# Exercise 1: Type Inspection Challenge
# Analyze different values and show their type information

# Test different values
value1 = 42
value2 = "hello"
value3 = 3.14
value4 = True
value5 = None

print("=== Type Inspection Results ===\n")

# Analyze value1
print(f"Value: {value1}")
print(f"  Type: {type(value1)}")
print(f"  Is numeric? {isinstance(value1, (int, float))}")
print(f"  Is truthy? {bool(value1)}")
print(f"  Can be string? {isinstance(str(value1), str)}")
print()

# Analyze value2
print(f"Value: {value2}")
print(f"  Type: {type(value2)}")
print(f"  Is numeric? {isinstance(value2, (int, float))}")
print(f"  Is truthy? {bool(value2)}")
print(f"  Can be string? {isinstance(str(value2), str)}")
print()

# TODO: Analyze value3, value4, and value5 following the same pattern
```

### Exercise 2: Casting Chain Challenge

Write a program that:
1. Starts with a string value
2. Converts it to int
3. Converts that int to float
4. Converts that float back to string
5. Prints each step and its type

```python
# Exercise 2: Type Casting Chain
# Demonstrate converting through multiple types

original: str = "42"
print(f"Original: '{original}' (type: {type(original).__name__})")

# Convert to int
as_int: int = int(original)
print(f"As int: {as_int} (type: {type(as_int).__name__})")

# Convert to float
as_float: float = float(as_int)
print(f"As float: {as_float} (type: {type(as_float).__name__})")

# Convert back to string
back_to_str: str = str(as_float)
print(f"Back to string: '{back_to_str}' (type: {type(back_to_str).__name__})")

# Compare original vs final
print(f"\nOriginal: '{original}'")
print(f"Final: '{back_to_str}'")
print(f"Same value? {original == back_to_str}")

# TODO: Try with different starting values like "0", "100", "-5"
# What happens when you start with "3.14"?
```

### Exercise 3: Implicit vs Explicit Conversion

Identify which lines use **implicit** and which use **explicit** casting:

```python
# Example lines (classify each as implicit or explicit)
result1: float = 5 + 3.14          # ___________
result2: str = str(100)            # ___________
result3: int = int("50")           # ___________
result4: float = int(3.5)          # ___________
result5: bool = 1 > 0              # ___________

# Write your answers, then test them in Python
# Implicit means Python does it automatically
# Explicit means you called a function like int(), str(), float()
```

**Answer Key** (scroll down or try it yourself first!):
- result1: Implicit (int automatically becomes float in mixed arithmetic)
- result2: Explicit (you called str())
- result3: Explicit (you called int())
- result4: Explicit (you called int())
- result5: Neither (this is comparison creating a bool, not casting)

### Exercise 4: Complete the Capstone

Build the User Profile Validator program following the scaffolded code above. Get it working with basic functionality first, then add one of the extensions.

**Your validator should**:
- Accept user input for name, age, and email
- Use `isinstance()` to check types
- Attempt type casting and handle errors gracefully
- Display clear validation results

---


---

## Try With AI: Type Casting Debugging Challenge

You've learned `type()`, `isinstance()`, and type casting. Now apply these tools to debug real problems—with AI as your debugging partner.

### Part 1: Find the Type Errors (Your Turn First)

**Before asking AI**, analyze this broken program and identify ALL type-related errors:

```python
# User Profile System (BROKEN VERSION)
user_age = input("Enter your age: ")  # Returns string!
user_score = "95.5"
is_premium = "True"

# Calculations (these will break!)
age_next_year = user_age + 1
average_score = user_score / 2
premium_check = is_premium and True

print(f"Age next year: {age_next_year}")
print(f"Average score: {average_score}")
print(f"Premium status: {premium_check}")
```

**Your diagnostic task**:
1. What type is `user_age`? What type does `+ 1` expect?
2. What type is `user_score`? Can you divide a string?
3. What type is `is_premium`? Will `and` work with strings?
4. How would you fix EACH error using type casting?

Write down your diagnosis before moving to Part 2.

---

### Part 2: Validate Diagnosis with AI

Now share the broken code with AI:

> "Here's a broken Python program with type errors: [paste code]. Find ALL the type-related bugs. For each bug: (1) Explain what's wrong, (2) Show the error Python would give, (3) Show the fix using type casting. Use `type()` to verify the fix works."

**Your task**: Compare AI's diagnosis to yours.
- Did you find all the errors?
- Did AI explain the root cause correctly? (input() returns str, not int!)
- Do the fixes use the right casting functions? (`int()`, `float()`, `bool()`)

---

### Part 3: Student Teaches AI (Edge Cases)

AI fixed the obvious errors. But does it know about type casting edge cases?

Challenge AI with tricky inputs:

> "What happens if the user enters 'twenty-five' instead of '25' for age? What happens if they enter '3.14' (a decimal) when we expect an integer? Show me the errors and explain how to handle invalid input gracefully using try/except (preview of Chapter 21)."

**Your task**: Study AI's error handling.
- What error does `int('twenty-five')` raise? (ValueError!)
- What about `int('3.14')`? (ValueError! Use float() first, then int())
- Can you explain WHY these fail?

This teaches AI about validation complexity—and prepares you for error handling.

---

### Part 4: Build Robust Type Validator Together

Now iterate to create production-quality validation.

Ask AI:

> "Rewrite the User Profile System with robust type validation. For each input: (1) Use `isinstance()` to check type, (2) Use type casting with error handling, (3) Provide helpful error messages if casting fails. Make it impossible for bad input to crash the program."

**Your task**: Review AI's robust version.
- Does it validate types before using them?
- Does it handle ValueError gracefully?
- Can you explain WHY validation prevents crashes?

Iterate if needed:
> "Add type hints to all variables and show me how `type()` confirms the casts worked."

---

### Part 5: Extend the Capstone (Optional Advanced)

Final challenge—enhance the Type Explorer capstone from the lesson:

> "Extend my Type Explorer to: (1) Handle lists like '1,2,3' by splitting and casting each element, (2) Show implicit vs explicit casting with examples, (3) Test integer interning for numbers -10 to 300 using `is` operator. Keep code modular with type hints."

**Outcome**: If you complete this, you've mastered type utilities at a professional level.

---

**Time**: 30-40 minutes total
**Outcome**: You've practiced systematic type debugging, learned to handle edge cases, built robust validation with AI, and optionally extended the capstone project to production quality.
