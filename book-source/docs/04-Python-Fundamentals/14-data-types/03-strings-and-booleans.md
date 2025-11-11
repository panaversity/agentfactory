---
title: "Strings and Booleans — Text and Truth"
chapter: 14
lesson: 3
duration_minutes: 50

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment, accreditation alignment, and differentiation
skills:
  - name: "String Basics: Creating and Understanding Text Data"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can create string variables with type hints, recognize single/double/triple-quote syntax, and understand that strings represent text"

  - name: "Boolean Logic: True, False, and Truthy/Falsy Evaluation"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can create bool variables, understand True/False values, and predict how values evaluate as truthy or falsy in Python contexts"

  - name: "None Type: Representing No Value"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can identify None as a placeholder for 'no value' and understand its behavior in boolean contexts"

  - name: "Type Conversion: Converting Between String, Int, Float, and Bool"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can convert between data types using str(), int(), float(), bool() and identify when conversions are valid or will fail"

learning_objectives:
  - objective: "Create string variables with type hints and recognize different quotation styles (single, double, triple quotes)"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Writing string variables with type hints in code examples"

  - objective: "Distinguish between True/False boolean values and understand truthy/falsy evaluation rules"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Predicting boolean evaluation of numbers, strings, and None in conditional contexts"

  - objective: "Use type conversion functions (str(), int(), float(), bool()) appropriately and recognize conversion failures"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Converting between types and handling scenarios where conversions might fail"

  - objective: "Understand None as a placeholder for missing values and its role in Python programs"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Identifying appropriate uses of None in code examples"

cognitive_load:
  new_concepts: 6
  assessment: "6 new concepts (str type, quote styles, bool type, truthy/falsy, None type, type conversion) within A2 limit of 7 ✓. Concepts build progressively with concrete examples before abstract rules."

differentiation:
  extension_for_advanced: "Challenge: Write a function that safely converts user input (string) to the correct type. What edge cases exist? Can you handle them gracefully?"
  remedial_for_struggling: "Focus on Concepts 1-2 first (strings and booleans). Use the concrete examples repeatedly. Type conversion (Concept 6) is a B1 skill—it's okay to spend extra time here."

# Generation metadata
generated_by: "lesson-writer v1.0"
source_spec: "specs/part-4-chapter-14/spec.md"
source_plan: "specs/part-4-chapter-14/plan.md"
source_tasks: "specs/part-4-chapter-14/tasks.md"
created: "2025-11-08"
last_modified: "2025-11-08"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Strings and Booleans — Text and Truth

In Lessons 1 and 2, you learned what variables are and explored numbers. Now it's time to explore two more essential data types: **strings** (text) and **booleans** (True/False). You'll also meet **None** (the "no value" type) and learn how to convert between different types.

These three types appear in almost every Python program. Strings let you work with text, booleans let you make decisions, and understanding type conversion lets you handle data safely when it arrives as text (like from user input or a web API).

## Concept 1: Strings — Storing and Recognizing Text

A **string** is a data type that represents text. It can contain letters, numbers, punctuation, spaces—any text you can type.

You create a string by putting text between quotes:

```python
# Creating strings with type hints
name: str = "Alex"
message: str = "Hello, Python!"
greeting: str = "Hi"

# Strings can contain spaces and punctuation
sentence: str = "Welcome to Python learning!"
emoji_message: str = "Hello 👋 Python 🐍"
```

Notice the type hint `: str` which tells Python and anyone reading your code that this variable holds text.

### Understanding Quote Styles

Python lets you use single quotes, double quotes, or triple quotes for strings:

```python
# Single quotes
single: str = 'Hello'

# Double quotes
double: str = "Hello"

# Both work identically—choose one style and stick with it for consistency
```

**When to use triple quotes** (which can span multiple lines):

```python
# Triple quotes for multi-line text
poem: str = """Roses are red,
Violets are blue,
Python is here,
And it's all new!"""

print(poem)
```

The triple quotes preserve line breaks, making them useful for longer blocks of text.

### Strings vs. Numbers: A Critical Distinction

This is crucial: `"123"` (a string) is NOT the same as `123` (an integer). Look carefully:

```python
string_number: str = "123"      # Text that looks like a number
actual_number: int = 123        # An actual number

# They look similar when printed, but they're different types
print(string_number)            # Prints: 123
print(actual_number)            # Prints: 123
```

**Why does this matter?** Because later in this lesson, you'll learn type conversion—how to transform "123" (text) into 123 (actual number).

### Strings and Simple Concatenation

You can combine strings using the `+` operator:

```python
greeting: str = "Hi"
name: str = "Alex"

# Concatenating strings with +
full_message: str = greeting + ", " + name
print(full_message)             # Prints: Hi, Alex
```

**Note**: String methods (like `.upper()`, `.lower()`, `.replace()`, etc.) are covered in **Chapter 16**. For now, just know that `+` joins strings together.

### Empty Strings: When Strings Have No Content

A string can be empty (contain nothing):

```python
empty: str = ""
print(len(empty))               # Prints: 0
```

Empty strings are useful placeholders, but we'll return to this when exploring functions and conditions.

## Concept 2: Boolean Values — True and False

A **boolean** (`bool`) is a data type with only two possible values: `True` or `False`. These represent the two states of a yes/no question.

```python
# Creating booleans with type hints
is_student: bool = True
has_graduated: bool = False

# Use them to answer questions
is_learning: bool = True
is_done: bool = False
```

Notice the capitals: **`True`** and **`False`** (not `true` or `false`). Python is strict about this.

### Why Booleans Matter

Booleans are the foundation of decision-making in code. Later chapters will teach you `if` statements that make decisions based on boolean values. Right now, focus on **recognizing and creating** them.

### Truthy and Falsy: Beyond True and False

Here's where booleans get interesting. In Python, values aren't just True or False—they have a **truthiness** (whether Python treats them as True or False in a boolean context).

Let's see which values are **truthy** (treated as True) and which are **falsy** (treated as False):

```python
# FALSY VALUES (treated as False in boolean context)
zero: int = 0
empty_string: str = ""
nothing: None = None

# TRUTHY VALUES (treated as True in boolean context)
positive_number: int = 5
negative_number: int = -1
text_with_content: str = "hello"
```

**Why this matters**: When you later write `if` statements, Python automatically evaluates values as truthy or falsy:

```python
age: int = 0
if age:
    print("Age is truthy")      # This will NOT execute (0 is falsy)

count: int = 5
if count:
    print("Count is truthy")    # This WILL execute (5 is truthy)

message: str = ""
if message:
    print("Message is truthy")  # This will NOT execute (empty string is falsy)
```

**Remember this key principle**: In Python, `0`, empty strings, and `None` are always falsy. Any other number, non-empty string is truthy.

## Concept 3: None — The "No Value" Type

**`None`** is a special value that means "no value" or "nothing". It's useful when a variable exists but has no meaningful content yet.

```python
# None represents "no value"
result: None = None
data: None = None
```

When will you use `None`? In real programs, functions often return `None` when they succeed but have nothing specific to return. For example:

```python
# A function that might return None
response: None = print("Hello")  # print() returns None (it just displays text)
```

### None in Boolean Contexts

`None` is **falsy**—it evaluates as False in boolean contexts:

```python
data: None = None
if data:
    print("Data exists")        # This will NOT execute (None is falsy)
```

**Important**: `None` is not the same as `False`. They're different types. `None` means "no value", while `False` means "the boolean value False". The distinction will matter in advanced code, but for now: **`None` is falsy**.

## Concept 4: Understanding Type Hints for str, bool, and None

By now, you've seen type hints for `int`, `float`, `str`, `bool`, and `None`. Here's a summary:

```python
name: str = "Alex"              # Variable stores text
is_ready: bool = True           # Variable stores True or False
age: int = 25                   # Variable stores an integer
height: float = 5.9             # Variable stores a decimal number
no_value: None = None           # Variable represents "no value"
```

Type hints are **specifications**. They tell Python (and anyone reading your code) what kind of data each variable should hold. This helps catch errors early and makes code clearer.

## Concept 5: Type Conversion — Converting Between Types

Sometimes you have data in one type but need it in another. **Type conversion** is the process of changing one data type into another.

### Converting Strings to Numbers

The most common scenario: user input arrives as text (a string), but you need to do math with it. Use `int()` to convert a string to an integer:

```python
# Converting string to integer
number_str: str = "123"
number_int: int = int(number_str)

print(number_int)               # Prints: 123 (as an integer, not text)
print(type(number_int))         # Prints: <class 'int'>
```

Similarly, use `float()` to convert a string to a decimal number:

```python
# Converting string to float
price_str: str = "19.99"
price_float: float = float(price_str)

print(price_float)              # Prints: 19.99 (as a float)
```

### Converting to Strings

Use `str()` to convert any type to a string:

```python
# Converting number to string
count: int = 42
count_str: str = str(count)

print(count_str)                # Prints: 42 (as text, not a number)
print(type(count_str))          # Prints: <class 'str'>

# Converting float to string
price: float = 9.99
price_str: str = str(price)
print(price_str)                # Prints: 9.99 (as text)
```

### Converting to Booleans

Use `bool()` to convert values to boolean:

```python
# Any non-zero number converts to True
bool(5)                         # True
bool(-1)                        # True
bool(0)                         # False

# Any non-empty string converts to True
bool("hello")                   # True
bool("")                        # False

# None converts to False
bool(None)                      # False
```

### When Conversion Fails: Understanding Errors

Not all conversions are valid. Trying to convert a string that doesn't look like a number will cause an error:

```python
# This will work
number: int = int("123")        # Success

# This will FAIL with a ValueError
# bad_number: int = int("abc")   # Error! "abc" isn't a number
```

**Key insight**: When working with type conversion in real programs, you'll need error handling (covered in **Chapter 17** on control flow). For now, understand that conversions can fail if the data doesn't match the target type.

## Concept 6: Putting It Together — A Complete Example

Let's combine everything you've learned in this lesson:

```python
# Specification: Get user age (as text) and determine if they're a teenager

# String: Accept user input (always arrives as text)
age_str: str = input("How old are you? ")

# Type conversion: Convert text to a number
age_int: int = int(age_str)

# Boolean: Check if they're a teenager (13-19)
is_teenager: bool = (13 <= age_int <= 19)

# Using the boolean result
if is_teenager:
    message: str = "You're a teenager!"
else:
    message: str = "You're not a teenager."

print(message)
```

**What happened**:
1. You got text from the user (a string)
2. You converted it to a number (type conversion)
3. You created a boolean by comparing the number
4. You used that boolean to decide what message to print

This pattern—getting text, converting it, and using booleans for decisions—appears constantly in real Python programs.

## Practice Exercise 1: Creating and Using Strings

Create variables for the following scenarios:

```python
# 1. Store your name as a string
# Type your code here

# 2. Store a greeting message that includes your name
# Type your code here

# 3. Create an empty string (you'll fill it later)
# Type your code here

# Run your code and print each variable to verify they work
```

**Expected output**:
```
Your name: [your name]
Greeting: Hello, [your name]!
Empty string: (nothing appears)
```

## Practice Exercise 2: Working with Booleans

Predict whether each of these values is truthy or falsy. Write `True` or `False` next to each:

```python
# Predict the truthiness of each value
value1: int = 0                 # Truthy or Falsy? _____
value2: str = "0"              # Truthy or Falsy? _____
value3: str = ""               # Truthy or Falsy? _____
value4: int = -5               # Truthy or Falsy? _____
value5: None = None            # Truthy or Falsy? _____

# After predicting, check your answers:
if value1:
    print("value1 is truthy")
else:
    print("value1 is falsy")

# Repeat for value2, value3, value4, value5
```

**Expected learning**: Understanding that `0`, empty strings, and `None` are falsy; everything else is truthy.

## Practice Exercise 3: Type Conversion

Convert the following values between types. Run your code to verify each conversion works:

```python
# 1. Convert string to integer
text: str = "42"
number: int = int(text)
print(f"Converted {text} to {number} (type: {type(number)})")

# 2. Convert integer to string
count: int = 100
count_text: str = str(count)
print(f"Converted {count} to {count_text} (type: {type(count_text)})")

# 3. Convert string to float
price_text: str = "29.99"
price: float = float(price_text)
print(f"Converted {price_text} to {price} (type: {type(price)})")

# 4. Convert values to boolean
print(bool(0))                  # Should be: False
print(bool(5))                  # Should be: True
print(bool(""))                 # Should be: False
print(bool("hello"))            # Should be: True
```

**Expected learning**: Type conversion works when the source data matches the target type. Numbers in strings convert cleanly; random text won't.

## Checkpoint: What You Should Understand Now

After this lesson, you should be able to:

- Create string variables with type hints and recognize different quote styles
- Distinguish between True/False boolean values and understand truthy/falsy evaluation
- Recognize None as a placeholder for "no value"
- Convert between strings, numbers, and booleans using `str()`, `int()`, `float()`, `bool()`
- Understand when conversions work and when they might fail

## Code Generation Record

This lesson includes code examples created through AI collaboration. Here's the specification and validation record:

**Specification**: Create clear, runnable code examples demonstrating:
1. String creation with type hints (single, double, triple quotes)
2. Boolean values and truthy/falsy evaluation
3. Type conversion between str, int, float, bool
4. None as a placeholder
5. Real-world scenarios (user input conversion)

**AI Prompts Used**:
- "Show Python 3.14+ examples of string creation with type hints"
- "Demonstrate truthy and falsy values in Python with clear examples"
- "Create examples of safe type conversion between strings, ints, floats, and booleans"
- "Show a real-world example of converting user input (string) to a number"

**Validation Steps**:
- All code examples are syntactically correct Python 3.14+
- All code includes appropriate type hints
- Code follows PEP 8 naming conventions
- Examples progress from simple (string creation) to complex (type conversion)
- No hardcoded secrets or sensitive data
- Examples are platform-independent (work on Windows, Mac, Linux)

---

## Try With AI

Now let's apply what you've learned with your AI partner. Use your preferred AI tool (ChatGPT web, Claude CLI, or another AI assistant you've set up from previous chapters).

### Prompt 1: Understanding Truthy and Falsy

**Copy this prompt into your AI tool**:

```
I'm learning Python data types. Explain truthy and falsy values:

1. Why is 0 falsy but -1 truthy?
2. Why is an empty string "" falsy but "0" truthy?
3. How does None behave as a boolean?
4. Give me a real-world scenario where I'd care about truthy/falsy values.
```

**Expected Response**: The AI should explain:
- Zero is a special case (represents nothing/absence)
- Non-zero numbers are always truthy (they represent presence/quantity)
- Empty strings are falsy (no content); non-empty strings are truthy
- None means "no value" so it's falsy
- Scenario: checking if a user entered something before processing it

### Prompt 2: When Does Type Conversion Fail?

**Copy this prompt into your AI tool**:

```
When I convert a string to an integer in Python using int("abc"), I get an error.
Explain:

1. Why does int("123") work but int("abc") fails?
2. What's the rule for successful string-to-integer conversion?
3. How would I safely convert user input to a number without crashing my program?
```

**Expected Response**: The AI should explain:
- `int()` only succeeds if the string contains digits (optionally with a leading + or -)
- Non-numeric strings like "abc" fail because they can't be interpreted as numbers
- Safe conversion requires error handling (try/except - covered in Chapter 17)
- Real programs validate input before conversion

### Prompt 3: Understanding None

**Copy this prompt into your AI tool**:

```
In Python, what is None?

1. Is None the same as False?
2. When should I use None vs other values?
3. How does None differ from an empty string "" or zero 0?
4. Show me a practical example where None makes sense to use.
```

**Expected Response**: The AI should explain:
- None is distinct from False (different types, though both are falsy)
- None means "no value exists" (vs False meaning "boolean False")
- None is for representing missing/undefined data
- Example: function returns None when it has nothing to return
- Practical scenario: A user's middle name might be None if they don't have one

### Prompt 4: Safe Type Conversion (Stretch Challenge)

**Copy this prompt into your AI tool**:

```
I need to write code that:
- Asks the user for a number
- Converts their input from text to an integer
- Handles the case where they type something that's not a number
- Shows them a friendly error message if conversion fails

Write Python code (with type hints) that does this. Don't worry about try/except yet—just show me the attempt and what would break if they enter invalid input. I'll add error handling later.
```

**Expected Response**: The AI should provide:
- Code structure: `input()` → `int()` conversion → type hints
- Explanation of where it breaks (int() on non-numeric text)
- Note that this requires error handling for production use
- Sneak preview that try/except (Chapter 17) solves this problem

### Safety & Ethics Note

When working with type conversion:
- Input data (from users, APIs, files) can be unexpected
- Always validate before conversion
- Never assume text is a number just because it looks like one
- In professional code, use error handling (Chapter 17) to gracefully handle conversion failures

**Next Step**: Explore what you can do with strings using methods (Chapter 16) and make decisions with booleans using control flow (Chapter 17).
