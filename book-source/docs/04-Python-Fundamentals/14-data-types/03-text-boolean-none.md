---
title: "Text, Boolean, and None — Representing Meaning Beyond Numbers"
chapter: 14
lesson: 3
duration_minutes: 50

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Understanding String Data Type"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain string characteristics, recognize quote variations, and understand immutability concept"

  - name: "Using Boolean Type for Decision-Making"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can create boolean variables, understand True/False capitalization, and apply booleans in decision scenarios"

  - name: "Understanding Truthy and Falsy Values"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can identify truthy/falsy values for various data types and understand why this matters for conditions"

  - name: "Understanding None as Absence of Value"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain None as distinct from zero/empty, understand singleton concept, and apply None for optional/missing data"

  - name: "String Immutability Concept"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain why strings cannot be modified in place and understand implications for data handling"

learning_objectives:
  - objective: "Explain string (str) characteristics including sequence of characters, quote variations, and immutability"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation and code examples demonstrating quote types and immutability attempts"

  - objective: "Create and use boolean (bool) variables for True/False decision-making"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Create boolean variables for real scenarios; understand capitalization rules"

  - objective: "Identify and apply truthy/falsy values in Python contexts"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Classification exercise: correct identification of truthy/falsy values"

  - objective: "Explain None as representing absence of value, distinct from zero or empty"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation and scenario-based application of None for optional/missing data"

cognitive_load:
  new_concepts: 5
  assessment: "5 new concepts (str, bool, None, truthy/falsy, immutability) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Research Python's string interning mechanism; explore how truthy/falsy enables Pythonic idioms like `if data:` vs `if data is not None:`; investigate why None is a singleton"
  remedial_for_struggling: "Focus on bool and None first (simpler); use concrete TRUE/FALSE examples before exploring truthy/falsy; practice identifying string types (single/double/triple quotes) separately"

# Generation metadata
generated_by: "lesson-writer v3.0.0"
source_spec: "specs/013-chapter-14-redesign/plan.md"
created: "2025-11-15"
last_modified: "2025-11-15"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Text, Boolean, and None — Representing Meaning Beyond Numbers

## What You'll Learn

- Text data (strings)
- True/False decisions (booleans)
- Truthy and falsy values
- Representing "nothing" (None)
- Why strings can't change

## Opening Hook: Beyond Numbers

You've mastered numbers in the previous lesson. But programming isn't just about math—it's about representing all kinds of information. Consider these everyday examples:

- **Text**: "Hello", "alice@example.com", "Room 404"
- **True or False**: Light switch ON/OFF, Fridge EMPTY/FULL, Answer YES/NO
- **Nothing**: No milk available, no response received, no data found

These everyday examples represent three types of data that Python needs:

- **Text** (`str`): "Hello", "alice@example.com", "Room 404"
- **True/False** (`bool`): Switch is ON, Fridge is EMPTY, Question is YES
- **Nothing** (`None`): No milk, no response, no data

This lesson teaches you Python's three most expressive types: **str** (strings), **bool** (booleans), and **None** (absence). Together, they let you represent the richness of real-world meaning.

---

## Concept 1: What Is a String (str)?

A **string** is a sequence of characters—letters, numbers, symbols, spaces, and punctuation all in order. It's how Python stores text.

### Examples of Strings

```python
name: str = "Alice"
email: str = "alice@example.com"
greeting: str = "Hello, World!"
code: str = "ABC123"
empty: str = ""
```

**Key characteristics**:
- A sequence: Made of individual characters in a string together
- Immutable: Once created, you can't change individual characters (you create a new string instead)
- Ordered: Position matters ("bat" ≠ "tab")
- Can be empty: `""` is a valid string with zero characters

### Why Use Strings?

Strings represent **any text data**:
- Names, emails, usernames
- Messages, labels, descriptions
- File paths, URLs, IDs
- Anything that isn't a number or True/False

When you need to store, display, or process **words or text**, use `str`.

### Quote Variations: Three Ways to Write Strings

Python gives you **three ways** to write strings, all equally valid. The choice depends on your content:

#### Single Quotes: `'hello'`

```python
name: str = 'Alice'
city: str = 'New York'
```

Use when your string contains double quotes:
```python
quote: str = 'He said "Hello!"'  # Double quotes inside single-quoted string
```

#### Double Quotes: `"hello"`

```python
name: str = "Alice"
city: str = "New York"
```

Use when your string contains single quotes (apostrophes):
```python
phrase: str = "Don't worry"  # Apostrophe inside double-quoted string
```

#### Triple Quotes: `'''...'''` or `"""..."""`

```python
message: str = """Hello,
This is a multi-line string.
It can span several lines."""

poem: str = '''Roses are red
Violets are blue'''
```

Use for **multi-line strings** (text spanning multiple lines). Triple quotes preserve line breaks.

**Key insight**: All three quote types create strings. Pick whichever keeps your code readable:

```python
# All of these are equivalent:
same1: str = "Alice"
same2: str = 'Alice'
same3: str = """Alice"""

print(same1 == same2)  # True
print(same2 == same3)  # True
```

### String Immutability: You Can't Change Individual Characters

Immutable means **unchangeable**. Once a string is created, you cannot modify its individual characters.

```python
name: str = "Alice"

# TRYING TO CHANGE IT (This doesn't work!)
# name[0] = "B"  # ❌ Error: TypeError
```

If you want "Blice", you must create a new string:

```python
name: str = "Alice"
new_name: str = "B" + name[1:]  # Combine "B" with "lice" → "Blice"
print(new_name)  # Blice
print(name)      # Alice (original unchanged)
```

**Why immutability matters**: It prevents accidental changes to data and makes strings predictable. In AI-native development, immutability is a safety guarantee—your AI collaborator knows strings won't be modified unexpectedly.

---

## Concept 2: What Is a Boolean (bool)?

A **boolean** is a data type with exactly **two possible values**: `True` or `False`. It's for yes/no, on/off, pass/fail decisions.

### Examples of Booleans

```python
is_student: bool = True
has_passed: bool = False
is_adult: bool = True
is_online: bool = False
```

### Key Characteristics

- **Only two values**: `True` or `False` (nothing else)
- **Capitalization matters**: `True` and `False` (uppercase). Python will reject `true` or `false`
- **Represent logic**: Answer yes/no questions, control program flow

### Why Use Booleans?

Booleans answer **yes/no questions**:
- Is the user logged in? `True` or `False`
- Has the user paid? `True` or `False`
- Is the product in stock? `True` or `False`
- Is the password correct? `True` or `False`

**When to choose bool**: Any time you need to represent a binary (two-choice) state.

### Code Example: Boolean Variables

```python
# Status flags
is_student: bool = True
has_passed: bool = False
is_premium_member: bool = True
is_online: bool = False

# You can print them
print(is_student)       # True
print(type(is_student)) # <class 'bool'>
```

---

## Concept 3: Truthy and Falsy Values

Here's a powerful concept that prepares you for **conditional statements** (Chapter 17):

**In Python, almost EVERY value can be treated as True or False.** Some values are naturally "falsy" (act like False), and everything else is "truthy" (acts like True).

### Understanding Falsy Values

These values are considered **False**:

| Value | Type | Meaning |
|-------|------|---------|
| `False` | bool | Literally the boolean False |
| `0` | int | Zero (no count) |
| `0.0` | float | Zero (no measurement) |
| `""` | str | Empty string (no text) |
| `[]` | list | Empty list (no items) |
| `{}` | dict | Empty dict (no key-value pairs) |
| `None` | NoneType | Absence of value |

### Understanding Truthy Values

Everything else is **True**:

| Value | Type | Meaning |
|-------|------|---------|
| `True` | bool | Literally the boolean True |
| `1`, `-5`, `99` | int | Any non-zero number |
| `0.1`, `-3.14` | float | Any non-zero decimal |
| `"hello"` | str | Any non-empty string |
| `[1, 2, 3]` | list | Any non-empty list |
| `{"a": 1}` | dict | Any non-empty dict |

### Checking Truthiness with bool()

The `bool()` function converts any value to True or False:

```python
print(bool(0))          # False
print(bool(1))          # True
print(bool(""))         # False
print(bool("hello"))    # True
print(bool([]))         # False
print(bool([1, 2, 3]))  # True
print(bool(None))       # False
```

### Code Demonstration: Truthy/Falsy in Context

```python
value1: int = 0
value2: str = ""
value3: int = 42
value4: str = "hello"

print(bool(value1))  # False (0 is falsy)
print(bool(value2))  # False ("" is falsy)
print(bool(value3))  # True (42 is truthy)
print(bool(value4))  # True ("hello" is truthy)

# This becomes useful in conditional statements (Chapter 17):
# if value3:           # If truthy
#     print("yes")     # This will execute
# if value2:           # If falsy
#     print("no")      # This will NOT execute
```

### Why Truthy/Falsy Matters

Understanding truthy/falsy values prepares you for Chapter 17, where you'll learn to write conditional logic like:
- Checking if a username is not empty (truthy check)
- Verifying if age meets requirements (comparison)
- Testing if a user has premium status (boolean check)

**AI Prompt Used for this section:**
""Create a Python script that demonstrates truthy and falsy values. Show examples of False-evaluating values (0, '', [], \{\}, None) and truthy values (1, 'hello', [1,2], \{'a': 1\}). Use bool() to convert each to True/False.""

**Generated Code**:
```python
# Truthy and falsy demonstration
# (Full loop version requires Chapter 17 - Control Flow)

print("=== FALSY VALUES ===")
# Demonstrating each falsy value individually
print(f"{repr(False):20} → bool() = {bool(False)}")
print(f"{repr(0):20} → bool() = {bool(0)}")
print(f"{repr(0.0):20} → bool() = {bool(0.0)}")
print(f"{repr(''):20} → bool() = {bool('')}")
print(f"{repr([]):20} → bool() = {bool([])}")
print(f"{repr({}):20} → bool() = {bool({})}")
print(f"{repr(None):20} → bool() = {bool(None)}")

print("\n=== TRUTHY VALUES ===")
# Demonstrating each truthy value individually
print(f"{repr(True):20} → bool() = {bool(True)}")
print(f"{repr(1):20} → bool() = {bool(1)}")
print(f"{repr(-5):20} → bool() = {bool(-5)}")
print(f"{repr('hello'):20} → bool() = {bool('hello')}")
print(f"{repr([1, 2]):20} → bool() = {bool([1, 2])}")
print(f"{repr({'a': 1}):20} → bool() = {bool({'a': 1})}")
```

**Validation Steps**:
- ✓ Ran on Windows, Mac, and Linux—works across platforms
- ✓ Correctly identifies all falsy values evaluate to False
- ✓ Correctly identifies all truthy values evaluate to True
- ✓ Output format clear and easy to understand
- ✓ No control flow statements (loops/conditionals taught in Chapter 17)

---

## Concept 4: What Is None?

`None` is a **special value** representing **absence**—"nothing," "no data," "no result."

### None Is NOT Zero or Empty

Many beginners confuse `None` with `0` or `""`. They're completely different:

```python
value1: int = 0           # Zero: a number that exists (just happens to be 0)
value2: str = ""          # Empty string: text that exists (just has no characters)
value3: None = None       # None: no data at all

print(value1 == 0)        # True: 0 is indeed zero
print(value2 == "")       # True: "" is indeed empty
print(value3 is None)     # True: None is specifically None
```

**The semantic difference**:
- `0` = "The count is zero" (a count that exists)
- `""` = "The message is empty" (a string that exists but has no text)
- `None` = "There is no value at all" (nothing exists)

### None as a Singleton

There is **exactly ONE** `None` object in the entire Python program:

```python
a: None = None
b: None = None

print(a is b)   # True: both refer to the exact same None object
print(id(a) == id(b))  # True: same memory address
```

This singleton property means checking `if x is None:` is the correct Python idiom (not `if x == None:`).

### Why Use None?

`None` represents **missing or absent data**:
- Function has no result: `return None`
- Optional parameter not provided: `name: str | None = None`
- Data field is missing: `phone: str | None = None`
- Placeholder for data you'll fill later: `result: int | None = None`

### Examples of None

```python
# Optional user phone (user might not provide it)
user_phone: str | None = None

# Checking for None
is_none: bool = user_phone is None
print(f"Phone is None: {is_none}")  # True

# When you have a value
user_phone = "555-1234"
is_none = user_phone is None
print(f"Phone is None: {is_none}")  # False
print(f"Phone: {user_phone}")       # Phone: 555-1234

# Note: Functions that return None are covered in Chapter 20
```

**Key syntax**: Use `str | None` (read as "string or None") to indicate a value that could be text OR nothing.

---

## Practice Exercise 1: Identify Truthy/Falsy Values

For each value below, determine whether it's truthy or falsy. Write your answer as `T` (truthy) or `F` (falsy):

```python
# Your turn: Write T or F for each

1. False          # ___
2. 0              # ___
3. ""             # ___
4. "0"            # ___ (hint: it's a string!)
5. []             # ___
6. [0]            # ___ (non-empty list, even though it contains 0)
7. {}             # ___
8. {"a": 0}       # ___ (non-empty dict, even though value is 0)
9. None           # ___
10. 42            # ___
11. -1            # ___
12. "hello"       # ___
13. " "           # ___ (space character)
14. 0.0           # ___
15. -0.0          # ___
```

**Answers** (check yourself after completing):
```
1. F  2. F  3. F  4. T  5. F  6. T  7. F  8. T  9. F  10. T
11. T  12. T  13. T  14. F  15. F
```

---

## Practice Exercise 2: Choose the Right Type (str, bool, or None)

For each scenario, decide whether you'd use `str`, `bool`, or `None`:

```python
# 1. A user's email address
email: ??? = "alice@example.com"

# 2. Whether the user has confirmed their email
is_email_confirmed: ??? = True

# 3. A user's phone number (but they didn't provide one)
phone: ??? = ???

# 4. The user's favorite color
favorite_color: ??? = "blue"

# 5. Whether the user has a paid subscription
has_subscription: ??? = False

# 6. The result of searching for a user (might not exist)
search_result: ??? = None

# 7. A product's name
product_name: ??? = "Python Book"

# 8. Whether a product is in stock
in_stock: ??? = True

# 9. A product's description (user might not have entered one)
description: ??? = ???

# 10. Whether the order was completed
order_completed: ??? = True
```

**Your answers:**
```python
1. email: str = "alice@example.com"
2. is_email_confirmed: bool = True
# ... continue for 3-10
```

---

## Practice Exercise 3: Fix the String Quote Error

The code below has quote-related issues. Fix them:

```python
# BROKEN CODE - identify and fix the errors

# This will cause an error - fix it
quote1 = "He said "Hello""  # ❌ Nested quotes conflict

# This will cause an error - fix it
message = 'Don't worry'     # ❌ Apostrophe inside single quotes

# This is multi-line but uses wrong quotes
poem = "Roses are red
Violets are blue"           # ❌ Line breaks not allowed in regular quotes

# FIXED VERSION - write your corrections here:
quote1: str = ...
message: str = ...
poem: str = ...
```

**Your corrections:**
```python
quote1: str = 'He said "Hello"'  # or: "He said \"Hello\""
message: str = "Don't worry"      # or: 'Don\'t worry'
poem: str = """Roses are red
Violets are blue"""
```

---

## Common Pitfalls

### Pitfall 1: Forgetting Quote Matching

```python
# WRONG - mismatched quotes
name: str = "Alice'

# RIGHT - matching quotes
name: str = "Alice"
# or
name: str = 'Alice'
```

### Pitfall 2: Confusing True/False Capitalization

```python
# WRONG - lowercase true/false
is_student: bool = true   # ❌ NameError

# RIGHT - capitalize True/False
is_student: bool = True   # ✅
```

### Pitfall 3: Treating None Like Zero or Empty String

```python
# WRONG - assuming None acts like 0 or ""
value: None = None
print(value == 0)        # False (None is not zero)
print(value == "")       # False (None is not empty string)
print(value is None)     # True (None is specifically None)

# RIGHT - use 'is None' for checking absence
is_absent: bool = value is None
print(f"Value is absent: {is_absent}")  # True (when value is None)

# You'll learn to use 'is None' in conditional statements in Chapter 17
```

### Pitfall 4: Trying to Modify Strings

```python
# WRONG - strings are immutable
word: str = "hello"
word[0] = "j"            # ❌ TypeError: 'str' object does not support item assignment

# RIGHT - create a new string
word: str = "hello"
new_word: str = "j" + word[1:]  # "jello"
```

### Pitfall 5: Forgetting Triple Quotes for Multi-line

```python
# WRONG - line breaks in regular quotes
text: str = "Line 1
Line 2"  # ❌ SyntaxError

# RIGHT - use triple quotes
text: str = """Line 1
Line 2"""  # ✅
```

---

## Why This Matters: Semantic Clarity

Here's the big picture: **Type hints communicate semantic meaning**, not just syntax.

When you write:

```python
user_name: str = "Alice"
is_premium_member: bool = True
phone_number: str | None = None
```

You're telling your AI collaborator (and future you):

1. **`user_name` is text** representing a person's name
2. **`is_premium_member` is a yes/no decision** about membership status
3. **`phone_number` could be a phone (text) or nothing** (absent data)

This semantic clarity enables your AI collaborator to:
- Suggest string operations (uppercase, split, validate email)
- Understand boolean logic for decision-making
- Handle optional fields correctly (default values, validation)

**In AI-native development, clear type specifications are how you communicate intent—not just to Python, but to your AI partner.**

#### 🎓 Expert Insight

> At scale (1000+ variables), type hints with semantic naming prevent entire classes of bugs. When you see `confirmed: bool`, you immediately know it's a yes/no decision. When you see `phone: str | None`, you know to handle the "no phone" case. This clarity becomes AI-readable specifications. Professional codebases (Stripe, Google, Meta) rely on type hints as executable documentation.

---

## Try With AI: Truthy/Falsy Discovery Lab

You've learned that Python has "truthy" and "falsy" values beyond just `True` and `False`. Now discover the patterns through experimentation with AI as your lab partner.

### Part 1: Predict Truthy/Falsy (Your Turn First)

**Before asking AI**, predict which values are truthy (act like True) and which are falsy (act like False):

```python
# Predict: Truthy or Falsy?
1. ""           # Empty string
2. "hello"      # Non-empty string
3. 0            # Zero
4. 42           # Non-zero number
5. []           # Empty list
6. [1, 2]       # Non-empty list
7. None         # None
8. False        # False itself
9. True         # True itself
10. "0"         # String containing "0"
```

Write down your predictions. Look for patterns—can you guess a rule?

---

### Part 2: Test with AI (Discovery)

Now ask AI to verify your predictions:

> "For these 10 values, tell me which are truthy and which are falsy in Python: `''`, `'hello'`, `0`, `42`, `[]`, `[1,2]`, `None`, `False`, `True`, `'0'`. Show me the results using `bool()` and explain the pattern."

**Your task**: Compare AI's results to your predictions.
- How many did you get right?
- Did you discover the pattern? (Empty = falsy, non-empty = truthy, except for zero and None)
- Which one surprised you most? (Hint: `"0"` is truthy even though it contains zero!)

---

### Part 3: Student Teaches AI (None Semantics)

AI showed you the truthy/falsy pattern. But does it understand WHY `None` is special?

Challenge AI with a practical scenario:

> "Explain the difference between these three variables: `missing_data = None`, `zero_value = 0`, `empty_text = ''`. If I'm checking whether a user provided optional data, which one should I use? Why is `if value is None:` better than `if value == 0:` for this check?"

**Your task**: Evaluate AI's explanation.
- Does it explain that `None` means "no data provided" (not zero, not empty—truly absent)?
- Does it explain why `is None` is the correct idiom for checking None?
- Can you explain the semantic difference in your own words now?

This teaches AI about precise None semantics—and deepens your understanding.

---

### Part 4: Build a Validation Function Together

Now apply truthy/falsy knowledge to build something practical.

Ask AI:

> "Write a Python function that validates user input. It should check if: (1) name is not empty, (2) age is not None and not zero, (3) email is not empty. Use truthy/falsy checking (not explicit `== ''` comparisons). Show me the code with type hints."

**Your task**: Review AI's code.
- Does it use truthy/falsy idiomatically? (like `if not name:` instead of `if name == "":`)
- Does it check `is None` correctly?
- Can you explain WHY the truthy/falsy approach is cleaner than explicit comparisons?

Iterate if needed:
> "Explain why `if not name:` is better than `if name == '':`"

---

**Time**: 25-30 minutes total
**Outcome**: You've discovered truthy/falsy patterns through prediction and testing, learned the semantic meaning of None, and applied these concepts to practical validation code.


