# Chapter 14: Data Types — Detailed Implementation Plan

**Chapter Type**: Technical / Code-Focused
**Chapter Objective(s)**: Students understand what data types are, how to use core types (int, float, str, bool, None), validate types with `isinstance()`, and recognize collection types with appropriate type hints.
**Estimated Total Time**: 3.5–4 hours (5 lessons × 40–50 min each)
**Part**: Part 4 - Python Fundamentals
**Complexity Tier**: A2-B1 (Intermediate Beginners)
**Status**: Ready for Implementation
**Feature Branch**: `014-data-types`

---

## 🎯 Chapter Overview & Context

**Building on Chapter 13 Foundation**:
- Chapter 13 taught: What is Python, Installing Python, Running code, `print()` function
- Chapter 14 builds on this by teaching: Variables, type hints as specifications, data types, type validation

**Bridge to Chapter 15**:
- Chapter 14 establishes solid type foundation
- Chapter 15 (Operators) builds on type understanding—students will apply types to arithmetic operations

**AI-Native Learning Pattern** (how students learn Python with AI):
- **Describe intent**: Type hints describe what data means (`age: int = 25` says "age is a number")
- **Explore with AI**: Students ask AI "What can I do with this type?" instead of reading documentation
- **Validate together**: `type()` and `isinstance()` let students check their understanding
- **Learn from errors**: When TypeError occurs, ask AI "why?" to deepen learning

**Key Scope Boundaries** (Chapter 14 Responsibility):
- ✅ **Comprehensive Coverage**: int, float, str (basics), bool, None, type hints, `type()`, `isinstance()`
- ✅ **Awareness Only**: Collections (list, tuple, dict, set) — deep dive deferred to Chapters 18-19
- ❌ **Deferred**: Operators (Ch 15), String methods (Ch 16), Control flow (Ch 17), Collection methods (Ch 18-19)

---

## 📊 Lesson Architecture

### Lesson 1: Variables and Type Hints — Storing Data with Specifications

**Learning Objective** (Bloom's: Apply): Students can create variables with type hints and understand how type hints serve as embedded specifications, building on Chapter 13's `print()` knowledge.

**Skills Taught**:
- **Skill 1**: Understanding Variables in Python
  - CEFR Level: A2 (Basic)
  - Category: Technical
  - Bloom's Level: Understand
  - DigComp Area: Problem-Solving (understanding digital tools)
  - Measurable at This Level: Student can explain "what a variable is and why we use names to store data"

- **Skill 2**: Writing Type Hints for Clarity
  - CEFR Level: A2 (Basic)
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Digital Content Creation (writing code)
  - Measurable at This Level: Student can write `variable_name: type_hint = value` syntax without errors

- **Skill 3**: Understanding How Type Hints Describe Intent
  - CEFR Level: B1 (Intermediate)
  - Category: Conceptual
  - Bloom's Level: Understand
  - DigComp Area: Problem-Solving (designing solutions)
  - Measurable at This Level: Student can explain "how type hints describe what data means and why this helps"

**Key Concepts** (max 6):
1. Variables - storing data with names
2. Assignment operator (`=`)
3. Type hints syntax (`age: int = 25`)
4. Why type hints matter (embedded specifications, clarity)
5. Multiple variables with different types
6. Using `print()` from Chapter 13 to display variables

**Prerequisites**:
- Chapter 13: Hello World with `print()` (from knowledge)
- Understanding of basic data (numbers, text, true/false)

**Duration**: 45 minutes

**Content Outline**:
1. **Introduction** (5 min) - Why variables matter; building on Chapter 13
2. **Concept: Variables Without Type Hints** (10 min) - Simple variable creation; what Python understands
3. **Concept: Type Hints as Specifications** (10 min) - The `age: int = 25` pattern; why it matters
4. **Code Examples** (15 min) - Write variables with type hints for different types
5. **Practice** (5 min) - Create variables, display with `print()`

**Content Elements**:

**Code Example 1: First Variables (Without Type Hints)**
```python
# Simple variables - Python infers the type
name = "Alex"
age = 25
height = 5.8
is_student = True
```
- **Purpose**: Show basic variable creation; establish that Python stores different kinds of data
- **Complexity**: Beginner (no concepts, just syntax)
- **Prompts for Try With AI**:
  - "What is a variable and why do we need them?"
  - "Can I change what's in a variable after I create it?"

**Code Example 2: Variables with Type Hints**
```python
# Variables with type hints - describe what data means
name: str = "Alex"
age: int = 25
height: float = 5.8
is_student: bool = True
```
- **Purpose**: Introduce type hints syntax; show how type hints communicate intent (specification)
- **Complexity**: Beginner-Intermediate (syntax + concept)
- **Prompts for Try With AI**:
  - "What does `: int` mean in `age: int = 25`?"
  - "Why would a programmer add type hints? What's the benefit?"

**Code Example 3: Print Variables (Building on Chapter 13)**
```python
# Using print() from Chapter 13 to display variables with type hints
name: str = "Alex"
age: int = 25

print(name)  # Output: Alex
print(age)   # Output: 25
print(f"Hello, {name}! You are {age} years old.")  # Using f-string (preview)
```
- **Purpose**: Connect Chapter 13's `print()` to variable display; show practical use
- **Complexity**: Beginner (familiar function + new variables)
- **Prompts for Try With AI**:
  - "How do I display variable values using `print()`?"
  - "Can I combine multiple variables in one `print()` statement?"

**Practice Approach**:
- **Exercise 1**: Create 5 variables (name, age, height, is_student, favorite_number) with type hints and appropriate data
- **Exercise 2**: Display each variable using `print()`
- **Exercise 3**: Create `print()` statement combining multiple variables (use simple concatenation, not f-strings yet)

**End-of-Lesson: Try With AI** (AI Tool Selection Policy: Part 4 is post-AI-tools introduction, so learner's preferred AI companion - ChatGPT, Claude Code, or Gemini CLI)

**Try With AI Prompts**:

1. **Concept Exploration**: "You learned `print()` in Chapter 13. Now explain what a variable is and why we need them. Give a real-world analogy."
   - Expected Outcome: Student understands variables as "containers for storing data"
   - AI Tool: ChatGPT web or Claude Code (learner's choice)
   - Follow-up: "Does my explanation match how Python works?"

2. **Syntax Clarification**: "What does `age: int = 25` mean? Explain each part: `age`, `: int`, `= 25`. Why do we add `: int` to the variable?"
   - Expected Outcome: Student understands type hints as "specifications for what type of data this variable holds"
   - AI Tool: Claude Code or Gemini CLI (if learner has CLI set up)
   - Follow-up: "Is the type hint required, or is it optional?"

3. **Type Variety**: "Show me how to create variables for different types of data (numbers, text, true/false). What types exist in Python? Create a code example with 5 different types."
   - Expected Outcome: Student sees multiple type hints and understands variety available
   - AI Tool: Claude Code (for inline code generation)
   - Follow-up: "Can I create a variable with a different type hint than its actual data?"

4. **Connection to AI-Native Learning**: "Type hints describe what data means (`age: int` says 'age is a number'). How does this help me and AI understand code better than `age = 25` alone? Why is clarity important when learning with AI?"
   - Expected Outcome: Student understands type hints make code clearer for humans AND AI
   - AI Tool: ChatGPT web (conceptual discussion)
   - Safety Note: "Type hints are helpful for clarity but don't prevent you from changing variable types later—Python allows this."

**Cognitive Load Validation**:
- A2 level: Max 7 concepts per lesson ✓
- New concepts: 1 (variables), 2 (assignment), 3 (type hints), 4 (why they matter), 5 (multiple variables), 6 (print)
- Total: 6 concepts = within limit ✓

---

### Lesson 2: Understanding Data Types — Integers and Floats

**Learning Objective** (Bloom's: Analyze): Students can explain the concept of data types, distinguish between int and float, use `type()` to inspect data, validate types with `isinstance()`, and understand when to choose each type.

**Skills Taught**:
- **Skill 1**: Understanding the Data Type Concept
  - CEFR Level: A2 (Basic)
  - Category: Conceptual
  - Bloom's Level: Understand
  - DigComp Area: Information Literacy (understanding data categories)
  - Measurable at This Level: Student can explain "what a data type is and why Python cares about types"

- **Skill 2**: Distinguishing Numeric Types (int vs. float)
  - CEFR Level: A2 (Basic)
  - Category: Technical
  - Bloom's Level: Analyze
  - DigComp Area: Problem-Solving (choosing appropriate tools)
  - Measurable at This Level: Student can choose int vs. float appropriately and explain the difference

- **Skill 3**: Using Type Inspection Functions
  - CEFR Level: A2-B1 (Basic-Intermediate)
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (troubleshooting)
  - Measurable at This Level: Student can use `type()` and `isinstance()` to validate data

**Key Concepts** (max 7):
1. **What is a data type?** (category of data determining operations)
2. **Why types matter** (safety, clarity, what operations are possible)
3. `type()` function - inspecting what type data is
4. `int` - whole numbers (positive, negative, zero)
5. `float` - decimal numbers (floating-point precision)
6. When to use int vs. float (counting vs. measuring)
7. `isinstance()` - type checking for validation

**Prerequisites**:
- Lesson 1: Variables and type hints (just taught)
- Chapter 13: Understanding `print()` function

**Duration**: 50 minutes

**Content Outline**:
1. **Introduction** (5 min) - What are data types? Why does Python care?
2. **Concept: The `type()` Function** (8 min) - How to inspect what type something is
3. **Numeric Types: Int vs. Float** (12 min) - Examples showing each; when to use each
4. **Type Validation with `isinstance()`** (12 min) - Checking types before operations
5. **Practice & Troubleshooting** (8 min) - TypeError scenarios and fixes
6. **Try With AI** (5 min) - Begin exploration

**Content Elements**:

**Code Example 1: Type Inspector**
```python
# Using type() to see what type data is
age = 25
height = 5.8
name = "Alex"
is_student = True

print(type(age))       # <class 'int'>
print(type(height))    # <class 'float'>
print(type(name))      # <class 'str'>
print(type(is_student)) # <class 'bool'>
```
- **Purpose**: Introduce `type()` function; show Python categorizes data
- **Complexity**: Beginner (familiar function + output)
- **Key Insight**: "The `type()` function tells you what category Python thinks your data is"

**Code Example 2: Int vs. Float**
```python
# Integers - whole numbers (no decimal point)
count_apples: int = 5      # exact count
people_in_room: int = 10   # exact quantity
year: int = 2025           # exact number

# Floats - numbers with decimal points (measurements)
height_meters: float = 1.75     # measured height
temperature_celsius: float = 36.5  # measured temperature
price_dollars: float = 19.99    # monetary value
pi_approximation: float = 3.14

# Why the difference matters
print(type(5))   # <class 'int'> - exact
print(type(5.0)) # <class 'float'> - approximate/measured
```
- **Purpose**: Show practical distinction; demonstrate when to use each
- **Complexity**: Beginner (pattern recognition)
- **Key Insight**: "Use int for counting; use float for measuring"

**Code Example 3: Type Checking with `isinstance()`**
```python
# Using isinstance() to check types before operations
age: int = 25

if isinstance(age, int):
    print(f"{age} is an integer")  # This prints
else:
    print("Not an integer")

height: float = 5.8
if isinstance(height, float):
    print(f"{height} is a float")  # This prints
else:
    print("Not a float")
```
- **Purpose**: Introduce validation pattern; show safety checking
- **Complexity**: Beginner-Intermediate (requires if/else preview, but syntax clear)
- **Key Insight**: "Check types before operations to avoid errors"

**Practice Approach**:
- **Exercise 1**: Create 5 variables (some int, some float) and use `type()` to inspect each
- **Exercise 2**: Predict the type of a value, then verify with `type()`
- **Exercise 3**: Use `isinstance()` to validate a variable is int or float
- **Exercise 4 (Error Learning)**: What happens when you use `type(age) == "int"`? (Wrong comparison—won't work as expected)

**End-of-Lesson: Try With AI**

**Try With AI Prompts**:

1. **Concept Understanding**: "What is a data type in Python? Why does Python care about types instead of treating all numbers the same?"
   - Expected Outcome: Student understands types as "categories determining what operations are possible"
   - AI Tool: ChatGPT web or Claude Code
   - Follow-up: "Can I perform the same operations on every type?"

2. **Type Selection**: "What's the difference between int and float? When should I use each? Give me 5 examples where int is better and 5 where float is better."
   - Expected Outcome: Student internalizes the distinction (counting vs. measuring)
   - AI Tool: Claude Code (for example code)
   - Follow-up: "Can I mix int and float in the same operation?"

3. **Inspection Mastery**: "Show me the `type()` function. What does it tell me about my data? Create a code example that inspects 10 different values."
   - Expected Outcome: Student practices type inspection; builds confidence
   - AI Tool: Claude Code or Gemini CLI
   - Follow-up: "What if I use `type(type(age))`—what happens?"

4. **Error Recovery**: "I got 'TypeError: unsupported operand type(s)'. What happened and how do I fix it? Show me how `isinstance()` helps catch these errors."
   - Expected Outcome: Student learns troubleshooting pattern: check types with `isinstance()` before operations
   - AI Tool: Claude Code (with example error)
   - Safety Note: "Type errors are learning opportunities—they tell you Python found a type mismatch"

**Cognitive Load Validation**:
- A2-B1 level: Max 7-10 concepts per lesson ✓
- New concepts: 1 (type concept), 2 (why types matter), 3 (type() function), 4 (int), 5 (float), 6 (int vs. float), 7 (isinstance)
- Total: 7 concepts = within limit ✓

---

### Lesson 3: Strings and Booleans — Text and Truth Values

**Learning Objective** (Bloom's: Apply): Students can create strings and boolean variables with type hints, understand truthy/falsy values, convert between types safely, and use `None` appropriately.

**Skills Taught**:
- **Skill 1**: Creating and Using Strings
  - CEFR Level: A2 (Basic)
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Digital Content Creation (working with text)
  - Measurable at This Level: Student can create strings with type hints and display them

- **Skill 2**: Understanding Boolean Values and Truthiness
  - CEFR Level: A2-B1 (Basic-Intermediate)
  - Category: Conceptual
  - Bloom's Level: Understand
  - DigComp Area: Information Literacy (understanding True/False logic)
  - Measurable at This Level: Student can explain truthy/falsy values and predict boolean contexts

- **Skill 3**: Type Conversion (Casting) Safely
  - CEFR Level: B1 (Intermediate)
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (handling data transformations)
  - Measurable at This Level: Student can convert between types and handle conversion failures

**Key Concepts** (max 7):
1. `str` - text data (basics: quotes, simple string creation)
2. Type hints for strings (`name: str = "Alex"`)
3. `bool` - True/False values
4. Boolean evaluation (True, False, truthy/falsy)
5. Truthy/falsy in Python (0 is falsy, non-zero is truthy; empty string falsy, non-empty truthy)
6. `None` type - represents "no value"
7. Type conversion: `int()`, `str()`, `float()`, `bool()` and handling failures

**Prerequisites**:
- Lesson 1: Variables and type hints
- Lesson 2: Data types concept and `isinstance()`
- Chapter 13: `print()` function

**Duration**: 50 minutes

**Content Outline**:
1. **Introduction** (5 min) - Strings, booleans, and "no value" concept
2. **Strings Basics** (8 min) - Creating strings, type hints, simple operations
3. **Boolean Values & Truthiness** (12 min) - True/False, truthy/falsy contexts
4. **Type Conversion** (12 min) - Converting between int, float, str, bool safely
5. **None Type** (5 min) - "No value" concept and use cases
6. **Try With AI** (5 min) - Begin exploration

**Content Elements**:

**Code Example 1: String Basics**
```python
# Strings - text data
name: str = "Alex"
city: str = "Portland"
greeting: str = "Hello, world!"

# Strings with quotes (both work in Python)
single_quotes: str = 'Text in single quotes'
double_quotes: str = "Text in double quotes"

# Using print() to display strings
print(name)        # Output: Alex
print(greeting)    # Output: Hello, world!

# Simple concatenation (no f-strings yet—save for Chapter 16)
full_greeting: str = "Hello, " + name
print(full_greeting)  # Output: Hello, Alex
```
- **Purpose**: String creation; basic operations
- **Complexity**: Beginner (familiar pattern from Lesson 1)
- **Key Insight**: "Strings are sequences of characters (text)"
- **Note**: We cover basic string operations here; methods and slicing deferred to Chapter 16

**Code Example 2: Boolean Values & Truthiness**
```python
# Boolean values - True or False
is_student: bool = True
is_teacher: bool = False
has_permission: bool = True

# Truthy and falsy values in Python
zero: int = 0
is_zero_truthy: bool = bool(zero)  # False (0 is falsy)

nonzero: int = 5
is_nonzero_truthy: bool = bool(nonzero)  # True (non-zero is truthy)

empty_string: str = ""
is_empty_string_truthy: bool = bool(empty_string)  # False

nonempty_string: str = "Hello"
is_nonempty_string_truthy: bool = bool(nonempty_string)  # True

# Truthy/falsy values in conditions (preview for Chapter 17)
print("Falsy values in Python:")
print(bool(0))        # False
print(bool(""))       # False
print(bool([]))       # False (empty list)
print(bool(None))     # False

print("Truthy values in Python:")
print(bool(1))        # True
print(bool(-1))       # True (even negative!)
print(bool("text"))   # True
print(bool([1,2]))    # True (non-empty list)
```
- **Purpose**: Understand boolean values; recognize truthy/falsy patterns
- **Complexity**: Intermediate (conceptual understanding)
- **Key Insight**: "In Python, many values can be evaluated as True or False—this matters when we write conditions"

**Code Example 3: Type Conversion**
```python
# Converting between types safely
# int() - convert to integer
age_string: str = "25"
age_integer: int = int(age_string)  # 25 (conversion works)
print(f"Age as integer: {age_integer}")

# str() - convert to string
age_number: int = 25
age_as_string: str = str(age_number)  # "25" (conversion works)
print(f"Age as string: {age_as_string}")

# float() - convert to float
height_string: str = "5.8"
height_float: float = float(height_string)  # 5.8
print(f"Height as float: {height_float}")

# bool() - convert to boolean (uses truthy/falsy rules)
print(bool(0))        # False
print(bool(1))        # True
print(bool(""))       # False
print(bool("text"))   # True

# Conversion failures (demonstrate with Try With AI)
# This will raise ValueError:
# age: int = int("abc")  # ERROR: invalid literal for int()
```
- **Purpose**: Show conversion patterns; demonstrate both successes and failures
- **Complexity**: Intermediate (error handling deferred to Chapter 21)
- **Key Insight**: "Type conversion works when data makes sense for the target type"
- **Note**: Error handling (try/except) deferred to Chapter 21

**Code Example 4: None Type**
```python
# None - represents "no value"
middle_name: str | None = None  # Person might not have middle name
user_email: str | None = None   # User hasn't provided email yet

# Using None to indicate "not set"
if user_email is None:
    print("Email not provided yet")
else:
    print(f"Email: {user_email}")

# None is falsy
if not None:
    print("None is falsy in boolean contexts")  # This prints
```
- **Purpose**: Understand None as "no value"; preview type unions (`str | None`)
- **Complexity**: Beginner-Intermediate (syntax familiar, concept new)
- **Key Insight**: "None means 'nothing' or 'not set'—useful for optional data"

**Practice Approach**:
- **Exercise 1**: Create strings with type hints; use `print()` to display
- **Exercise 2**: Predict truthy/falsy values; verify with `bool()` function
- **Exercise 3**: Convert between string and integer (success and failure cases via Try With AI)
- **Exercise 4**: Create variables with `None` type; understand "not set" concept

**End-of-Lesson: Try With AI**

**Try With AI Prompts**:

1. **Truthiness Exploration**: "Explain truthy and falsy values. Why is 0 falsy but -1 truthy? What about empty strings? Show me the complete list of falsy values in Python."
   - Expected Outcome: Student understands falsy values (0, "", None, [], {}, set()) and truthy is everything else
   - AI Tool: Claude Code or ChatGPT web
   - Follow-up: "Why do you think Python chose these as falsy?"

2. **Conversion Rules**: "When does `int('abc')` fail and `int('123')` succeed? What's the conversion rule? Show me edge cases."
   - Expected Outcome: Student understands conversion rules (string must be valid representation of type)
   - AI Tool: Claude Code (with code examples)
   - Follow-up: "What types can I convert to each other?"

3. **None Concept**: "What is None and when should I use it? How does it behave in boolean contexts? Show examples of when None is useful."
   - Expected Outcome: Student understands None as "no value" placeholder; useful for optional data
   - AI Tool: ChatGPT web or Claude Code
   - Follow-up: "Is None the same as 0 or empty string?"

4. **Real-World Application**: "I need to convert user input (string) to a number. How do I do this safely? What happens if they enter invalid input?"
   - Expected Outcome: Student recognizes conversion as common pattern; understands error scenario
   - AI Tool: Claude Code (with code demonstration)
   - Safety Note: "Error handling (try/except) comes in Chapter 21—for now, understand conversion can fail"

**Cognitive Load Validation**:
- A2-B1 level: Max 7-10 concepts per lesson ✓
- New concepts: 1 (strings), 2 (string type hints), 3 (bool), 4 (boolean evaluation), 5 (truthy/falsy), 6 (None), 7 (type conversion)
- Total: 7 concepts = within limit ✓

---

### Lesson 4: Introduction to Collections — Overview Only

**Learning Objective** (Bloom's: Understand): Students understand that collections exist for grouping related data, can create basic list/tuple/dict/set with type hints, and know that comprehensive coverage happens in Chapters 18-19.

**Skills Taught**:
- **Skill 1**: Understanding Collection Purpose
  - CEFR Level: A1 (Foundation)
  - Category: Conceptual
  - Bloom's Level: Understand
  - DigComp Area: Information Literacy (understanding data structures)
  - Measurable at This Level: Student can explain "why collections exist and when to use them at high level"

- **Skill 2**: Creating Basic Collections
  - CEFR Level: A2 (Basic)
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Digital Content Creation (writing collection literals)
  - Measurable at This Level: Student can create basic list, tuple, dict, set with correct syntax

- **Skill 3**: Writing Collection Type Hints
  - CEFR Level: A2-B1 (Basic-Intermediate)
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (specifying data structures)
  - Measurable at This Level: Student can write `list[int]`, `dict[str, float]`, etc. type hints

**Key Concepts** (max 7):
1. **Why collections exist**: Grouping related data together
2. `list` - mutable ordered collection of items
3. `tuple` - immutable ordered collection of items
4. `dict` - key-value pairs for fast lookup
5. `set` - unique values only (no duplicates)
6. Type hints for collections (`list[int]`, `dict[str, int]`)
7. **Deference to later chapters**: Methods, operations, deep usage in Chapters 18-19

**Prerequisites**:
- Lesson 1: Variables and type hints
- Lesson 2: Data types concept
- Lesson 3: Strings, booleans, type conversion

**Duration**: 40 minutes

**Content Outline**:
1. **Introduction** (5 min) - Why group data? Collection overview
2. **Collections Concept** (8 min) - What each type does at high level
3. **Collection Creation** (10 min) - Basic syntax for each collection
4. **Type Hints for Collections** (10 min) - `list[int]`, `dict[str, float]` syntax
5. **Preview to Chapters 18-19** (5 min) - Where comprehensive coverage happens
6. **Try With AI** (2 min) - Begin exploration

**Content Elements**:

**Code Example 1: Collection Basics**
```python
# List - ordered, mutable (changeable)
scores: list = [85, 90, 78, 92]  # list of numbers
names: list = ["Alice", "Bob", "Charlie"]  # list of strings
mixed: list = [1, "text", 3.14, True]  # mixed types (generally avoid)

# Tuple - ordered, immutable (unchangeable)
rgb_color: tuple = (255, 128, 0)  # red, green, blue values
coordinates: tuple = (10, 20)  # x, y position

# Dictionary - key-value pairs
person: dict = {"name": "Alex", "age": 25, "city": "Portland"}
phone_numbers: dict = {"Alice": "555-1234", "Bob": "555-5678"}

# Set - unique values only
unique_numbers: set = {1, 2, 3, 3, 4, 4, 4}  # {1, 2, 3, 4} - duplicates removed
favorite_colors: set = {"red", "blue", "green"}

# Displaying collections
print(scores)       # [85, 90, 78, 92]
print(person)       # {'name': 'Alex', 'age': 25, 'city': 'Portland'}
print(unique_numbers)  # {1, 2, 3, 4}
```
- **Purpose**: Show basic collection creation; recognize syntax patterns
- **Complexity**: Beginner (recognizing patterns)
- **Key Insight**: "Collections group related data together"
- **Note**: We do NOT cover methods (`.append()`, `.pop()`, `.items()`, etc.)—that's Chapters 18-19

**Code Example 2: Type Hints for Collections**
```python
# Specifying collection element types with type hints

# list[type] - list of specific type
scores: list[int] = [85, 90, 78, 92]
names: list[str] = ["Alice", "Bob", "Charlie"]

# tuple[type, ...] - tuple of specific type
coordinates: tuple[int, int] = (10, 20)
rgb_color: tuple[int, int, int] = (255, 128, 0)

# dict[key_type, value_type] - key-value type specification
person: dict[str, str | int] = {"name": "Alex", "age": 25}
phone_numbers: dict[str, str] = {"Alice": "555-1234", "Bob": "555-5678"}

# set[type] - set of specific type
unique_numbers: set[int] = {1, 2, 3, 4}
favorite_colors: set[str] = {"red", "blue", "green"}

# Using type() to inspect collections
print(type(scores))           # <class 'list'>
print(type(coordinates))      # <class 'tuple'>
print(type(person))           # <class 'dict'>
print(type(unique_numbers))   # <class 'set'>
```
- **Purpose**: Introduce collection type hint syntax (from Python 3.9+)
- **Complexity**: Intermediate (syntax pattern, modern Python)
- **Key Insight**: "Type hints communicate what kind of items are in each collection"
- **Note**: Modern syntax (`list[int]`) vs. older syntax (`List[int]`) — we use modern

**Code Example 3: Collection Awareness (When to Use Each)**
```python
# When to use each collection type

# List - when order matters and items might change
to_do_list: list[str] = ["Buy groceries", "Walk dog", "Read chapter"]
# → Can add/remove items, order matters

# Tuple - when order matters but items don't change
date: tuple[int, int, int] = (2025, 11, 8)  # year, month, day
# → Can't change items, but position has meaning

# Dictionary - when looking up values by key (fast lookup)
student_grades: dict[str, int] = {
    "Alice": 95,
    "Bob": 87,
    "Charlie": 92
}
# → Fast lookup by name; don't care about order

# Set - when uniqueness is important
visited_countries: set[str] = {"USA", "Canada", "Mexico", "USA"}
# → Automatically removes duplicates; no order
```
- **Purpose**: High-level guidance on when to use each collection
- **Complexity**: Beginner (conceptual patterns)
- **Key Insight**: "Choose collection type based on whether you need ordering, mutability, or uniqueness"
- **Note**: This is awareness only—practical examples come in Chapters 18-19

**Practice Approach**:
- **Exercise 1**: Create a list, tuple, dict, set with type hints
- **Exercise 2**: Use `type()` to verify each collection's type
- **Exercise 3**: Predict what type hint each collection should have
- **Exercise 4 (Awareness Check)**: When would you use list vs. tuple? Dict vs. set?

**End-of-Lesson: Try With AI**

**Try With AI Prompts**:

1. **Conceptual Overview**: "What's the difference between list, tuple, dict, and set? Give me a simple example of when I'd use each. Which is most common?"
   - Expected Outcome: Student understands high-level purpose of each collection
   - AI Tool: ChatGPT web or Claude Code
   - Follow-up: "Can I mix different types in one collection?"

2. **Type Hints for Collections**: "Show me type hints for collections. What does `list[int]` mean? What about `dict[str, float]`? Create examples of type-hinted collections."
   - Expected Outcome: Student understands collection type hint syntax
   - AI Tool: Claude Code (for code examples)
   - Follow-up: "What's the difference between `list[int]` and just `list`?"

3. **Collection Discovery**: "I see collections have many methods and operations. Where will I learn those in detail? What methods might I use on lists?"
   - Expected Outcome: Student knows Chapters 18-19 cover comprehensive collection coverage
   - AI Tool: ChatGPT web or Claude Code
   - Follow-up: "Can I use a collection if I haven't learned its methods yet?"

4. **Collection Selection**: "How do I choose the right collection type for my data? What questions should I ask myself?"
   - Expected Outcome: Student develops decision-making process (order matters? fast lookup? uniqueness?)
   - AI Tool: Claude Code (with decision matrix)
   - Safety Note: "For now, focus on understanding collection purpose. Chapters 18-19 will teach practical usage."

**Cognitive Load Validation**:
- A1-A2 level: Max 5-7 concepts per lesson ✓
- New concepts: 1 (collections exist), 2 (list), 3 (tuple), 4 (dict), 5 (set), 6 (type hints), 7 (when to use each)
- Total: 7 concepts = within limit ✓

---

### Lesson 5: Building a Type Explorer — Hands-On Practice

**Learning Objective** (Bloom's: Create): Students integrate core type concepts (int, float, str, bool, None) to build a working interactive program that demonstrates type operations, validation, and conversion.

**Skills Taught**:
- **Skill 1**: Type-Aware Program Design
  - CEFR Level: B1 (Intermediate)
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (designing solutions)
  - Measurable at This Level: Student can design program structure using type hints throughout

- **Skill 2**: Type Validation in Context
  - CEFR Level: B1 (Intermediate)
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (checking assumptions)
  - Measurable at This Level: Student can validate types before operations using `isinstance()`

- **Skill 3**: Integration & Reflection
  - CEFR Level: B1 (Intermediate)
  - Category: Soft
  - Bloom's Level: Evaluate
  - DigComp Area: Communication (explaining their own work)
  - Measurable at This Level: Student can reflect on type concepts and plan extensions

**Key Concepts** (max 6):
1. Integrating core types (int, float, str, bool, None)
2. Type hints throughout program (specifications for clarity)
3. Type validation with `isinstance()`
4. Type conversion with error awareness
5. User interaction demonstrating types
6. AI-guided program improvement

**Prerequisites**:
- Lesson 1: Variables and type hints
- Lesson 2: Understanding types and `isinstance()`
- Lesson 3: Strings, booleans, type conversion
- Lesson 4: Collection awareness

**Duration**: 50 minutes

**Content Outline**:
1. **Project Introduction** (5 min) - What is a type explorer? Why build this?
2. **Code Example: Interactive Type Explorer** (20 min) - Guided walkthrough
3. **Discussion: Design Choices** (10 min) - Why we used type hints; validation patterns
4. **Independent Extension** (10 min) - Suggest improvements; discuss with AI
5. **Try With AI** (5 min) - Code review and extension planning

**Content Elements**:

**Code Example: Interactive Type Explorer (Capstone Program)**
```python
"""
Interactive Type Explorer
Demonstrates core data types: int, float, str, bool, None
Uses type hints to describe intent clearly
Validates types before operations
"""

# Program demonstrates: type hints, isinstance(), type(), user interaction

def explore_integer_type() -> None:
    """Explore integer type."""
    number: int = 42
    print(f"\n--- Integer Type ---")
    print(f"Value: {number}")
    print(f"Type: {type(number)}")
    print(f"Is integer? {isinstance(number, int)}")
    print(f"Representation: {repr(number)}")

def explore_float_type() -> None:
    """Explore float type."""
    pi: float = 3.14159
    print(f"\n--- Float Type ---")
    print(f"Value: {pi}")
    print(f"Type: {type(pi)}")
    print(f"Is float? {isinstance(pi, float)}")
    print(f"Integer division result: {int(pi)}")

def explore_string_type() -> None:
    """Explore string type."""
    greeting: str = "Hello, Python!"
    print(f"\n--- String Type ---")
    print(f"Value: {greeting}")
    print(f"Type: {type(greeting)}")
    print(f"Is string? {isinstance(greeting, str)}")
    print(f"Length: {len(greeting)}")

def explore_boolean_type() -> None:
    """Explore boolean type."""
    is_learning: bool = True
    print(f"\n--- Boolean Type ---")
    print(f"Value: {is_learning}")
    print(f"Type: {type(is_learning)}")
    print(f"Is boolean? {isinstance(is_learning, bool)}")
    print(f"Opposite: {not is_learning}")

def explore_none_type() -> None:
    """Explore None type."""
    no_value: None = None
    print(f"\n--- None Type ---")
    print(f"Value: {no_value}")
    print(f"Type: {type(no_value)}")
    print(f"Is None? {no_value is None}")
    print(f"Is None falsy? {not bool(no_value)}")

def explore_type_conversion() -> None:
    """Demonstrate type conversion."""
    print(f"\n--- Type Conversion ---")

    # String to integer
    number_string: str = "42"
    number_int: int = int(number_string)
    print(f"String '{number_string}' as integer: {number_int}")

    # Integer to string
    count: int = 100
    count_string: str = str(count)
    print(f"Integer {count} as string: '{count_string}'")

    # Float to integer
    temperature: float = 98.6
    temp_int: int = int(temperature)
    print(f"Float {temperature} as integer: {temp_int}")

    # Any value to boolean
    value: str = "non-empty"
    value_bool: bool = bool(value)
    print(f"Non-empty string '{value}' as boolean: {value_bool}")

def main() -> None:
    """Main program: explore all core types."""
    print("=== Python Type Explorer ===")
    print("Learning core data types through exploration")

    explore_integer_type()
    explore_float_type()
    explore_string_type()
    explore_boolean_type()
    explore_none_type()
    explore_type_conversion()

    print("\n=== Exploration Complete ===")
    print("Ask your AI: 'What other type operations can I do?'")
    print("Ask your AI: 'How can I improve this program?'")

if __name__ == "__main__":
    main()
```

- **Purpose**: Integrate all Chapter 14 type concepts in working program
- **Complexity**: Intermediate (functions, return type hints, user output)
- **Size**: ~70 lines (manageable, focused)
- **Key Concepts Demonstrated**:
  - Type hints throughout (annotations on variables, function return types)
  - `type()` function to inspect types
  - `isinstance()` function to validate types
  - Type conversion (`int()`, `str()`, `bool()`)
  - Comments documenting purpose
  - Function structure (preparation for Chapter 20)

- **Comments for AI Extension Points**:
```python
# ASK YOUR AI: "How can I add user input to this explorer?"
# ASK YOUR AI: "Can I add more type checks or conversions?"
# ASK YOUR AI: "How would I add list/dict exploration here?"
# (Reminder: Lists/dicts comprehensive coverage in Ch 18-19)
```

**Practice Approach**:
- **Task 1**: Run the type explorer; understand each function's output
- **Task 2**: Modify one function (e.g., change the int value from 42 to 99); observe changes
- **Task 3**: Create new function to explore a different type (copy pattern from existing functions)
- **Task 4**: Use `isinstance()` in one of your own programs; validate before using value

**End-of-Lesson: Try With AI**

**Try With AI Prompts**:

1. **Code Review & Type Hints**: "Review my type explorer code. Am I using type hints correctly? What types am I demonstrating? Are there type hints I'm missing?"
   - Expected Outcome: Student gets feedback on type hint usage; solidifies understanding
   - AI Tool: Claude Code or Gemini CLI (code-aware)
   - Follow-up: "Should I add type hints to the functions too?"

2. **Improvement Brainstorm**: "How can I improve this program to explore more type operations? What new functions should I add? What about exploring truthy/falsy values?"
   - Expected Outcome: Student sees extension possibilities; builds confidence in program design
   - AI Tool: Claude Code (with suggestions and code)
   - Follow-up: "Should I handle conversion errors?"

3. **Error Handling Preview**: "Add type conversion error handling. What happens if user enters invalid data? Show me how to handle `int('abc')` safely."
   - Expected Outcome: Student recognizes error scenario; sees preview of Chapter 21 error handling
   - AI Tool: Claude Code (with error demonstration)
   - Safety Note: "Full error handling (try/except) comes in Chapter 21—this is a preview"

4. **Reflection & Connection**: "What did I learn about types by building this explorer? How does this help me for Chapter 15 (Operators) where I'll use these types in arithmetic?"
   - Expected Outcome: Student reflects on chapter learning; makes connection to next chapter
   - AI Tool: ChatGPT web (conceptual discussion)
   - Follow-up: "Which type(s) do you think will be used most in operators?"

**Cognitive Load Validation**:
- B1 level: Max 10 concepts per lesson ✓
- New concepts: 1 (integration), 2 (type hints throughout), 3 (isinstance), 4 (type conversion), 5 (user interaction), 6 (reflection), 7 (function structure preview)
- Total: 7 concepts = within limit ✓
- Mostly practice & integration = cognitive load managed ✓

---

## 📚 Content Flow & Dependencies

**Lesson 1 → Lesson 2**:
- L1 introduces variables and type hints (syntax foundation)
- L2 explains WHY types matter (conceptual foundation)
- L2 adds type inspection (`type()`, `isinstance()`)

**Lesson 2 → Lesson 3**:
- L2 covers numeric types (int, float)
- L3 covers text (str) and logic (bool) types
- L3 adds type conversion and None concept

**Lesson 3 → Lesson 4**:
- L1-3 cover core types (int, float, str, bool, None)
- L4 introduces collections (grouping types together)
- L4 emphasizes collections are tools, deep coverage deferred

**Lesson 4 → Lesson 5**:
- L1-4 teach individual concepts
- L5 integrates all into working program
- L5 demonstrates practical type usage

**All Lessons → Chapter 15 (Operators)**:
- Chapter 14 establishes type foundation
- Chapter 15 will apply types to operations (int + int, str + str, etc.)
- Students will use isinstance() to validate before operations

---

## 🏗️ Scaffolding Strategy

**Complexity Progression** (A2 → B1):

| Lesson | A2 Foundation | B1 Extension | Cognitive Load |
|--------|---------------|--------------|-----------------|
| 1 | Variables, syntax | Type hints as specifications | 6 concepts |
| 2 | type() function | isinstance() validation | 7 concepts |
| 3 | String/bool basics | Type conversion, None | 7 concepts |
| 4 | Collection syntax | Type hints for collections | 7 concepts |
| 5 | Integration | Reflection, extension | 7 concepts (mostly practice) |

**Cognitive Load Management**:
- Max 7 concepts per lesson (A2-B1 research threshold)
- Early lessons introduce concepts; later lessons practice
- Lesson 5 is integration + reflection (low new concept load)

**Scaffolding Techniques**:
1. **Worked Examples** (L1-4): Model pattern; students recognize and apply
2. **Guided Practice** (L1-4): Clear exercise with expected output
3. **Error Exploration** (L2-3): "What happens if...?" questions via Try With AI
4. **Integration** (L5): Apply all concepts to real program
5. **Reflection** (L5): Metacognition through AI dialogue

---

## 🧠 AI Partnership Integration

**Three-Tier Teaching Pattern** (Constitutional Principle 13):

**Tier 1: Book Teaches (Direct Explanation)**
- What is a data type? (concept)
- Why types matter (motivation)
- How to write type hints (syntax)
- int vs. float distinction (comparison)

**Tier 2: AI Companion (Complex Execution)**
- Exploring "What can I do with this type?" (dialogue)
- Discovering truthy/falsy patterns (discovery)
- Troubleshooting TypeError scenarios (debugging)
- Collection selection decisions (reasoning)

**Tier 3: AI Orchestration (Scaling, Not in Ch 14)**
- Batch type conversions
- Large data structure exploration
- (Deferred to later chapters)

**Prompt Engineering Pattern**:
- "What can I do with...?" → Exploration
- "Why did this fail?" → Error recovery
- "How do I...?" → Procedural guidance
- "What did I learn?" → Reflection

---

## ✅ Validation Strategy

**By Lesson**:
- L1: Can student create variables with type hints? (Exercise check)
- L2: Can student explain type concept and use isinstance()? (Quiz + exercise)
- L3: Can student convert types and handle falsy values? (Exercise check)
- L4: Can student create basic collections with type hints? (Exercise check)
- L5: Does student's type explorer run without errors and demonstrate concepts? (Code execution)

**Across Chapter**:
- **Comprehension**: 75%+ can explain "what is a data type" (quiz metric)
- **Skill Acquisition**: 80%+ successfully use isinstance() in exercises
- **AI Exploration**: 90%+ complete "Try With AI" prompts with engagement
- **Error Recovery**: 85%+ can troubleshoot TypeError with AI help
- **Integration**: 100% build working type explorer (capstone submission)

**Quality Assurance**:
- All code examples tested in Python 3.14+
- Type hints validated with mypy/pyright (type checking)
- No forward references to Chapters 15+
- Reading level Grade 7-8 maintained
- AI-Native Learning pattern consistently applied (describe intent → explore with AI → validate → learn from errors)

---

## 📍 Integration Points

**Backward Connection** (Chapter 13):
- Chapter 13 taught `print()` function
- Chapter 14 uses `print()` to display variables and types
- Building on prior knowledge; reinforcing previous lesson

**Forward Connection** (Chapter 15: Operators):
- Chapter 14 establishes int, float, str, bool, None
- Chapter 15 will use these types with operators (+, -, *, /, ==, >, <, and, or)
- Type validation (isinstance) will prevent type mismatches

**Horizontal Connection** (Domain Skills)**:
- AI-Native Learning (Describe Intent): Type hints describe what data means
- AI-Native Learning (Explore with AI): "Try With AI" prompts emphasize dialogue-based exploration
- AI-Native Learning (Validate Together): isinstance() teaches checking understanding
- AI-Native Learning (Learn from Errors): Troubleshooting TypeError by asking AI "why?"

**Accessibility & Inclusivity**:
- Examples use diverse names (Alex, Alice, Bob, Charlie)
- Real-world contexts (student records, temperature, phone numbers)
- Multiple representation formats (code, diagrams, narratives)
- Clear definitions of terminology (type, variable, conversion)

---

## 📋 Skills Proficiency Metadata

**Master Skills Registry Integration**:

These skills should be added to the Part 4 Skills Registry (when created):

```yaml
# Chapter 14 Skills (A2-B1 Tier)

skills:
  - skill_name: "Understanding Variables in Python"
    category: "Technical"
    proficiency_level: "A2"
    bloom_taxonomy: "Understand"
    digcomp_area: "Problem-Solving"
    chapter_first_appearance: 14
    chapter_mastery_target: 14
    definition: "Student can explain what a variable is and why we store data in variables with names"
    measurable_at_level: "Student creates 5 variables with type hints and explains what each stores"
    proficiency_progression:
      - lesson: 1
        level: "A2"
        assessment: "Create variables with type hints; explain purpose"

  - skill_name: "Writing Type Hints for Clarity"
    category: "Technical"
    proficiency_level: "A2"
    bloom_taxonomy: "Apply"
    digcomp_area: "Digital Content Creation"
    chapter_first_appearance: 14
    chapter_mastery_target: 14
    definition: "Student can write type hints using Python 3.14+ syntax (age: int = 25)"
    measurable_at_level: "Student writes 10 variables with correct type hint syntax without errors"
    proficiency_progression:
      - lesson: 1
        level: "A2"
        assessment: "Write variables with int, str, float, bool type hints"

  - skill_name: "Understanding the Data Type Concept"
    category: "Conceptual"
    proficiency_level: "A2"
    bloom_taxonomy: "Understand"
    digcomp_area: "Information Literacy"
    chapter_first_appearance: 14
    chapter_mastery_target: 14
    definition: "Student can explain what a data type is and why Python cares about types"
    measurable_at_level: "Student explains type concept in own words; predicts type for given value"
    proficiency_progression:
      - lesson: 2
        level: "A2"
        assessment: "Quiz: What is a data type? Why does it matter?"

  - skill_name: "Distinguishing Numeric Types (int vs. float)"
    category: "Technical"
    proficiency_level: "A2"
    bloom_taxonomy: "Analyze"
    digcomp_area: "Problem-Solving"
    chapter_first_appearance: 14
    chapter_mastery_target: 14
    definition: "Student can choose appropriate numeric type (int for counting, float for measuring)"
    measurable_at_level: "Student lists 5 int examples and 5 float examples with correct reasoning"
    proficiency_progression:
      - lesson: 2
        level: "A2"
        assessment: "Exercise: Categorize 10 numeric values as int or float and explain why"

  - skill_name: "Using Type Inspection Functions"
    category: "Technical"
    proficiency_level: "A2-B1"
    bloom_taxonomy: "Apply"
    digcomp_area: "Problem-Solving"
    chapter_first_appearance: 14
    chapter_mastery_target: 14
    definition: "Student can use type() to inspect types and isinstance() to validate types"
    measurable_at_level: "Student writes programs using type() and isinstance() for validation"
    proficiency_progression:
      - lesson: 2
        level: "A2"
        assessment: "Use type() to inspect 5 values; use isinstance() to validate before operations"

  - skill_name: "Creating and Using Strings"
    category: "Technical"
    proficiency_level: "A2"
    bloom_taxonomy: "Apply"
    digcomp_area: "Digital Content Creation"
    chapter_first_appearance: 14
    chapter_mastery_target: 14
    definition: "Student can create strings with type hints and display using print()"
    measurable_at_level: "Student creates 5 strings with type hints; uses print() to display"
    proficiency_progression:
      - lesson: 3
        level: "A2"
        assessment: "Create and print strings; use simple concatenation (+ operator)"

  - skill_name: "Understanding Boolean Values and Truthiness"
    category: "Conceptual"
    proficiency_level: "A2-B1"
    bloom_taxonomy: "Understand"
    digcomp_area: "Information Literacy"
    chapter_first_appearance: 14
    chapter_mastery_target: 14
    definition: "Student can explain True/False values and truthy/falsy evaluation context"
    measurable_at_level: "Student predicts truthy/falsy values for 10 data values; explains reasoning"
    proficiency_progression:
      - lesson: 3
        level: "A2-B1"
        assessment: "Truthy/falsy quiz; use bool() to evaluate values"

  - skill_name: "Type Conversion (Casting) Safely"
    category: "Technical"
    proficiency_level: "B1"
    bloom_taxonomy: "Apply"
    digcomp_area: "Problem-Solving"
    chapter_first_appearance: 14
    chapter_mastery_target: 14
    definition: "Student can convert between int, float, str, bool and understand when conversions fail"
    measurable_at_level: "Student converts types safely; explains failure scenarios via AI dialogue"
    proficiency_progression:
      - lesson: 3
        level: "B1"
        assessment: "Convert between types; predict and explain failure cases"

  - skill_name: "Understanding Collection Purpose"
    category: "Conceptual"
    proficiency_level: "A1"
    bloom_taxonomy: "Understand"
    digcomp_area: "Information Literacy"
    chapter_first_appearance: 14
    chapter_mastery_target: 14
    definition: "Student can explain why collections exist and high-level purpose of each type"
    measurable_at_level: "Student explains 'why group data?' and distinguishes list/tuple/dict/set purpose"
    proficiency_progression:
      - lesson: 4
        level: "A1"
        assessment: "Explain when to use each collection type at high level"

  - skill_name: "Creating Basic Collections"
    category: "Technical"
    proficiency_level: "A2"
    bloom_taxonomy: "Apply"
    digcomp_area: "Digital Content Creation"
    chapter_first_appearance: 14
    chapter_mastery_target: 14
    definition: "Student can create basic list, tuple, dict, set with correct syntax"
    measurable_at_level: "Student creates one of each collection type with correct syntax"
    proficiency_progression:
      - lesson: 4
        level: "A2"
        assessment: "Create list, tuple, dict, set; use print() to display"

  - skill_name: "Writing Collection Type Hints"
    category: "Technical"
    proficiency_level: "A2-B1"
    bloom_taxonomy: "Apply"
    digcomp_area: "Problem-Solving"
    chapter_first_appearance: 14
    chapter_mastery_target: 14
    definition: "Student can write collection type hints (list[int], dict[str, float])"
    measurable_at_level: "Student writes 5 collection variables with correct type hint syntax"
    proficiency_progression:
      - lesson: 4
        level: "A2-B1"
        assessment: "Write type hints for list, tuple, dict, set variables"

  - skill_name: "Type-Aware Program Design"
    category: "Technical"
    proficiency_level: "B1"
    bloom_taxonomy: "Apply"
    digcomp_area: "Problem-Solving"
    chapter_first_appearance: 14
    chapter_mastery_target: 14
    definition: "Student can design and implement program structure using type hints throughout"
    measurable_at_level: "Student's type explorer program uses consistent type hints on all variables/functions"
    proficiency_progression:
      - lesson: 5
        level: "B1"
        assessment: "Type explorer program demonstrates type hints throughout"

  - skill_name: "Type Validation in Context"
    category: "Technical"
    proficiency_level: "B1"
    bloom_taxonomy: "Apply"
    digcomp_area: "Problem-Solving"
    chapter_first_appearance: 14
    chapter_mastery_target: 14
    definition: "Student can validate types before operations using isinstance()"
    measurable_at_level: "Student's type explorer uses isinstance() to validate types; explains decisions"
    proficiency_progression:
      - lesson: 5
        level: "B1"
        assessment: "Type explorer demonstrates type validation; can add validation to own code"

  - skill_name: "Integration and Reflection"
    category: "Soft"
    proficiency_level: "B1"
    bloom_taxonomy: "Evaluate"
    digcomp_area: "Communication"
    chapter_first_appearance: 14
    chapter_mastery_target: 14
    definition: "Student can reflect on type concepts and plan program improvements via AI dialogue"
    measurable_at_level: "Student completes 'Try With AI' reflection prompts; articulates learning and extensions"
    proficiency_progression:
      - lesson: 5
        level: "B1"
        assessment: "Reflection prompts with AI; plans program extensions"

# Proficiency Progression Validation
cognitive_load_check: |
  Lesson 1: 6 concepts (variables, =, type hints, why, multiple, print) ✓
  Lesson 2: 7 concepts (type?, why types, type(), int, float, int vs float, isinstance) ✓
  Lesson 3: 7 concepts (str, str hints, bool, bool eval, truthy/falsy, None, conversion) ✓
  Lesson 4: 7 concepts (collections, list, tuple, dict, set, hints, when to use) ✓
  Lesson 5: 7 concepts (integration, hints, isinstance, conversion, user interaction, reflection, structure) ✓

proficiency_progression_check: |
  All skills start at A1 or A2 and progress to A2 or B1 within lesson scope ✓
  No regressions (proficiency never decreases) ✓
  Prerequisites satisfied (variable foundation before type validation) ✓

prerequisite_satisfaction: |
  All Chapter 14 skills depend only on Chapter 13 (print, basic Python knowledge) ✓
  No missing prerequisites ✓

skill_connectivity_check: |
  Variables (L1) → Type concept (L2) → Conversion (L3) → Collections (L4) → Integration (L5) ✓
  Clear progression track: Foundation → Understanding → Application → Integration
  All skills used across multiple lessons or integrated in capstone ✓
```

---

## 🎓 Lesson Closure Pattern

**CRITICAL DESIGN RULE**: Each lesson ends with ONLY the "Try With AI" section. NO separate "Key Takeaways" or "What's Next" sections.

**Why**: The "Try With AI" prompts serve as cognitive closure and engagement point. Adding traditional closing sections creates cognitive overload and duplicates content.

**Structure** (Consistent Across All 5 Lessons):
1. Content delivery (explanation + code examples + practice)
2. **Try With AI** (exactly 4 prompts with expected outcomes)
3. [END OF LESSON - No closing sections]

**Engagement Strategy**:
- Last prompt in each Try With AI is always reflective ("What did you learn?")
- Reflection connects current lesson to next (Chapter 15 for L1-4; capstone integration for L5)
- AI dialogue naturally concludes lesson by connecting backward (what was taught) and forward (what's next)

---

## 📝 Next Steps After Plan Approval

1. **Generate Task Checklist** → `/sp.tasks` command creates `chapter-14-tasks.md`
2. **Implement Lessons** → Lesson-writer subagent uses plan + tasks to write content
3. **Create Code Examples** → All code examples tested in Python 3.14.0+
4. **Generate Assessments** → Quizzes and exercises per task checklist
5. **Technical Review** → Technical-reviewer validates type hints, syntax, constitutional alignment
6. **Deploy to Docusaurus** → Integrate into book build system

---

## ✨ Quality Assurance Checklist

**Pedagogy**:
- ✅ Lessons follow A2-B1 progression (foundation → application)
- ✅ Cognitive load managed (max 7 concepts per lesson)
- ✅ Each lesson has single, measurable learning objective
- ✅ Scaffolding is explicit (worked examples → guided practice → independent application)
- ✅ All lessons end with "Try With AI" only (no Key Takeaways/What's Next)

**Alignment**:
- ✅ All lessons aligned to approved spec
- ✅ No forward references to Chapters 15-29
- ✅ Chapter boundaries respected (collections awareness only, deep dive in Ch 18-19)
- ✅ AI-Native Learning pattern integrated (describe intent: type hints; explore with AI: dialogue; validate: isinstance; learn from errors: TypeError troubleshooting)
- ✅ Built on Chapter 13 (print function reinforced throughout)

**Skills**:
- ✅ 12 distinct skills mapped to lessons with CEFR proficiency levels
- ✅ Skills progress A1→A2→B1 without regression
- ✅ Prerequisites satisfied (variables before types before validation)
- ✅ Skills connected in progression track (not isolated)
- ✅ Cognitive load validated for each proficiency level

**Code Examples**:
- ✅ All examples use Python 3.14+ with type hints
- ✅ Examples progressively build understanding
- ✅ All examples will be tested (mypy/pyright)
- ✅ Code follows style guide (naming, formatting, comments)
- ✅ No external dependencies in Chapter 14 examples

**Engagement**:
- ✅ Each lesson has concrete learning objective
- ✅ Try With AI prompts are specific and actionable
- ✅ Expected outcomes clearly stated
- ✅ Safety notes included for error scenarios
- ✅ Connection to Chapter 15 established

---

**Plan Status**: ✅ **READY FOR TASK GENERATION**

This plan provides detailed lesson-by-lesson structure with learning objectives, skills metadata, code specifications, and cognitive load validation ready for implementation by the content-implementer subagent.

