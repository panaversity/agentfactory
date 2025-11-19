---
title: "Type Hints: Organizing Your Data"
chapter: 13
lesson: 4
duration_minutes: 40
proficiency_level: "CEFR A2"
blooms_level: "Apply"

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Understanding Data Type Classification"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain why Python classifies data into types (int, str, float, bool) and give examples of each"

  - name: "Writing Type Hint Syntax"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can write variables with correct type hint syntax (name: str = value) and identify which type to use for different data"

  - name: "Recognizing Type Hints as Intent Documentation"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can explain that type hints express programmer intent (not strict enforcement) and why this helps code readability"

learning_objectives:
  - objective: "Explain what data types are and why classification matters"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Student describes types as categories of data and explains why 'age' should be int vs str with concrete examples"

  - objective: "Write variables with type hints using correct syntax"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student creates 10 variables with appropriate type hints (int, str, float, bool) based on data meaning"

  - objective: "Articulate why type hints are documentation, not enforcement"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Student explains: Python allows type violations but type hints communicate intent to humans and tools"

cognitive_load:
  new_concepts: 3
  assessment: "3 new concepts (data types as categories, type hint syntax, type hints as intent) at A2 limit of 5-7 ✓"

differentiation:
  extension_for_advanced: "Explore type checking tools (mypy), complex type hints (list[int], dict[str, int]), Optional types, type aliases"
  remedial_for_struggling: "Focus on int and str only; skip float and bool initially; use simple examples (age=int, name=str)"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/014-chapter-13-redesign/spec.md"
created: "2025-01-18"
last_modified: "2025-01-18"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Type Hints: Organizing Your Data

## What You'll Learn

- What data types are and why Python needs them
- The four basic Python types: int, str, float, bool
- How to write type hints in variable declarations
- Why type hints matter (intent, not enforcement)
- Common mistakes with type hints

---

## Opening Hook: Your Robot Organizes Its Knowledge

Our robot has come a long way. It can speak (`print()`), and it can remember (`variables`). But there's still a challenge: **the robot doesn't know how to organize information**.

Imagine the robot's memory looks like this:

```
Box 1: "25"
Box 2: 25
Box 3: "Hello"
Box 4: 3.14
Box 5: "True"
Box 6: True
```

You ask the robot: "Which of these are numbers you can do math with?"

The robot is confused:
- Box 1 says "25" — is that a number or text?
- Box 5 says "True" — is that text or a yes/no value?

Without labels describing what *kind* of data each box contains, the robot can't organize or use information effectively.

Now imagine you add labels:

```
Box 1: "25" [TEXT]
Box 2: 25 [WHOLE NUMBER]
Box 3: "Hello" [TEXT]
Box 4: 3.14 [DECIMAL NUMBER]
Box 5: "True" [TEXT]
Box 6: True [YES/NO VALUE]
```

Suddenly, everything is clear. The robot knows:
- Boxes 2 and 4 are numbers for math
- Boxes 1, 3, 5 are text for display
- Box 6 is a true/false decision value

**This is what type hints do.** They label your data so Python (and other programmers) know what kind of information each variable holds.

---

## What Are Data Types?

### The Concept (WHAT)

A **data type** is a category that classifies data based on what it represents and what operations you can perform on it.

Think of it like organizing a library:
- **Fiction** books → Read for entertainment
- **Reference** books → Look up facts
- **Textbooks** → Study and learn
- **Cookbooks** → Follow recipes

Each category serves a different purpose. You wouldn't use a cookbook to study history, and you wouldn't use a textbook to bake cookies.

Python does the same with data. It classifies information into types:
- **int** (integer) → Whole numbers like 25, -10, 0
- **str** (string) → Text like "Hello", "Alex", "Python"
- **float** (floating-point) → Decimal numbers like 3.14, -0.5, 99.99
- **bool** (boolean) → True or False values

### The Purpose (WHY)

Why does Python need types? Because **different data needs different operations**.

**Example: Adding Numbers vs. Joining Text**

With numbers:
```python
5 + 3  # Result: 8 (addition)
```

With text:
```python
"Hello" + " World"  # Result: "Hello World" (joining)
```

The `+` operator does *different things* depending on the type. For numbers, it adds. For text, it joins.

If Python didn't track types, it wouldn't know how to handle `+`:
```python
5 + "Hello"  # What does this mean? Python doesn't know!
```

**Error:**
```
TypeError: unsupported operand type(s) for +: 'int' and 'str'
```

Python says: "You can't add a number and text—they're different types!"

**Types prevent confusion.** They make sure data is used correctly.

---

## The Four Basic Types

Let's explore the four types you'll use most often in beginner programming.

### Type 1: int (Integer - Whole Numbers)

**What It Represents:**
Whole numbers with no decimal point: 0, 1, -5, 1000, -42

**When to Use:**
- Ages: `age = 25`
- Counts: `num_students = 30`
- Years: `year = 2024`
- Scores: `score = 100`
- Indices: `position = 3`

**What You Can Do:**
- Math: `10 + 5`, `20 - 3`, `4 * 7`, `15 // 3`
- Comparisons: `age >= 18`, `score == 100`

**Example:**
```python
age = 25
year = 2024
score = 100
```

---

### Type 2: str (String - Text)

**What It Represents:**
Text data enclosed in quotes: "Hello", 'Python', "user@email.com"

**When to Use:**
- Names: `name = "Alex"`
- Messages: `greeting = "Welcome!"`
- Email addresses: `email = "user@example.com"`
- Any text: `city = "Boston"`

**What You Can Do:**
- Join text: `"Hello" + " " + "World"` → `"Hello World"`
- Repeat: `"Ha" * 3` → `"HaHaHa"`
- Check length: `len("Python")` → `6`

**Example:**
```python
name = "Alex"
city = "New York"
message = "Hello, World!"
```

**Important:** Even numbers can be strings if they're in quotes:
```python
age = 25       # This is an int (number)
age = "25"     # This is a str (text)
```

---

### Type 3: float (Floating-Point - Decimal Numbers)

**What It Represents:**
Numbers with decimal points: 3.14, -0.5, 99.99, 0.001

**When to Use:**
- Prices: `price = 19.99`
- Measurements: `height = 5.9`
- Percentages: `tax_rate = 0.08`
- Scientific values: `pi = 3.14159`

**What You Can Do:**
- Math: `10.5 + 2.3`, `5.0 / 2.0`
- Comparisons: `price > 20.0`

**Example:**
```python
price = 19.99
height = 5.9
temperature = -3.5
```

**Difference from int:**
- `5` is an int
- `5.0` is a float

Both represent "five," but Python treats them as different types.

---

### Type 4: bool (Boolean - True/False)

**What It Represents:**
Only two values: `True` or `False` (yes/no, on/off, 1/0)

**When to Use:**
- Flags: `is_logged_in = True`
- Status: `has_permission = False`
- Conditions: `is_adult = age >= 18`

**What You Can Do:**
- Logic: `True and False` → `False`
- Conditions: `if is_logged_in:`
- Comparisons result in booleans: `10 > 5` → `True`

**Example:**
```python
is_student = True
has_license = False
is_raining = False
```

**Important:** Capitalization matters:
- ✅ `True` and `False` (correct)
- ❌ `true` and `false` (wrong - Python won't recognize these)

---

## Type Hints: Documenting Your Intent

Now that you know the four basic types, let's learn how to use **type hints**.

### What Are Type Hints? (WHAT)

**Type hints** are optional labels you add to variables to indicate what type of data they should hold.

Syntax:
```python
variable_name: type = value
```

Examples:
```python
age: int = 25
name: str = "Alex"
price: float = 19.99
is_student: bool = True
```

The `:` after the variable name introduces the type hint.

### Why Use Type Hints? (WHY)

Type hints serve **three important purposes**:

**Purpose 1: Document Intent**
Type hints tell other programmers (and your future self): "This variable is INTENDED to hold this type of data."

```python
age: int = 25
```

This says: "`age` should store a whole number." Anyone reading your code immediately understands.

**Purpose 2: Catch Mistakes Early**
Tools like **mypy** can check your code for type mismatches before you run it:

```python
age: int = "twenty-five"  # Mistake! age should be int, not str
```

A type checker would warn: "age is declared as int but assigned str."

**Purpose 3: Enable Better AI Collaboration**
When you use AI to help write code, type hints give AI more context:

```
"Create a function that calculates area.
Parameters: length (int), width (int)
Return: area (int)"
```

AI knows exactly what types to use because you specified them.

### Important: Type Hints Are NOT Enforcement

Here's the key insight: **Python doesn't enforce type hints**.

```python
age: int = 25
age = "twenty-five"  # Python allows this!
```

This code runs without errors. Python trusts you. Type hints are documentation, not rules.

**Think of type hints like road signs:**
- A speed limit sign says "55 MPH"
- You *can* drive 70 MPH (the car won't stop you)
- But you *shouldn't* (it's against the intended limit)

Similarly:
- Type hint says `age: int`
- You *can* assign `age = "text"` (Python won't stop you)
- But you *shouldn't* (it violates the documented intent)

---

## How to Write Type Hints

### Syntax Pattern

```python
variable_name: type = value
```

**Breakdown:**
1. `variable_name` — Your chosen variable name
2. `:` — Colon separator
3. `type` — The data type (`int`, `str`, `float`, `bool`)
4. `=` — Assignment operator
5. `value` — The actual data

### Examples with All Four Types

```python
# Integer (whole number)
age: int = 25
year: int = 2024
score: int = 100

# String (text)
name: str = "Alex"
city: str = "Boston"
email: str = "user@example.com"

# Float (decimal)
price: float = 19.99
height: float = 5.9
temperature: float = -3.5

# Boolean (True/False)
is_student: bool = True
has_license: bool = False
is_logged_in: bool = True
```

### Without Type Hints vs. With Type Hints

**Without type hints:**
```python
age = 25
name = "Alex"
price = 19.99
is_student = True
```

This works perfectly. Python infers the types automatically.

**With type hints:**
```python
age: int = 25
name: str = "Alex"
price: float = 19.99
is_student: bool = True
```

Same functionality, but now the code is **self-documenting**. Anyone reading it knows exactly what type each variable should hold.

---

## Practice Exercises

### Exercise 1: Add Type Hints

Take this code without type hints:

```python
name = "Sarah"
age = 30
city = "Chicago"
is_employed = True
salary = 75000.50
```

Rewrite it with type hints. Your code should look like:

```python
name: str = "Sarah"
age: int = 30
# ... (you complete the rest)
```

---

### Exercise 2: Choose the Right Type

For each variable, choose the correct type hint:

```python
student_count = 25          # int or str?
temperature = 98.6          # int or float?
user_email = "a@example.com"  # str or bool?
has_pet = False             # bool or str?
product_price = 29.99       # float or int?
zip_code = "02101"          # int or str?
```

**Tricky one:** `zip_code` looks like a number, but it's stored as text (`str`) because:
- You don't do math with zip codes
- Some zip codes start with 0 ("02101") — integers can't have leading zeros

---

### Exercise 3: Type Hint a Profile

Create `profile.py`:

```python
first_name: str = "Alex"
last_name: str = "Johnson"
age: int = 28
height: float = 5.9
is_student: bool = False
favorite_number: int = 7

print("Name:", first_name, last_name)
print("Age:", age)
print("Height:", height, "feet")
print("Student?", is_student)
print("Favorite number:", favorite_number)
```

Run it. Notice: type hints don't change how the program runs. They're documentation.

---

## Common Mistakes

### Mistake 1: Using Quotes for Numbers

**Wrong:**
```python
age: int = "25"  # int hint, but value is str
```

**Why:** The hint says `int`, but `"25"` is a string (text).

**Fix:**
```python
age: int = 25  # Remove quotes
```

---

### Mistake 2: Wrong Type Hint for Data

**Wrong:**
```python
price: int = 19.99  # int hint, but value has decimals
```

**Why:** `19.99` is a float, not an int. The hint is misleading.

**Fix:**
```python
price: float = 19.99
```

---

### Mistake 3: Lowercase True/False

**Wrong:**
```python
is_student: bool = true  # Should be True
```

**Error:**
```
NameError: name 'true' is not defined
```

**Fix:**
```python
is_student: bool = True  # Capital T
```

---

## For Curious Learners: Types as Classes (OOP Preview)

*Optional section. You can skip this and still master this lesson.*

In Python, everything is an **object**, and every type is actually a **class**.

When you write:
```python
age: int = 25
```

You're creating an instance of the `int` class. The value `25` is an object.

You can see an object's type using `type()`:

```python
age = 25
name = "Alex"
price = 19.99
is_student = True

print(type(age))        # <class 'int'>
print(type(name))       # <class 'str'>
print(type(price))      # <class 'float'>
print(type(is_student)) # <class 'bool'>
```

**Why This Matters:**
- `int`, `str`, `float`, `bool` are classes (blueprints for data)
- When you assign `age = 25`, Python creates an instance of the `int` class
- This "everything is an object" philosophy becomes important in Chapters 24-25 (Object-Oriented Programming)

For now, just know: types are more than labels—they're classes that define what data can do.

---

## Try With AI

### Prompt 1: Type Identification Practice

```
I'm learning Python data types.

Give me 10 values. For each one, tell me:
1. What type is it? (int, str, float, bool)
2. Why is that the correct type?
3. Show me the variable with a type hint

Examples:
- 42
- "Python"
- 3.14
- True
- "100"
- -5
- 0.5
- False
- "Hello, World!"
- 2024
```

**What to look for:** AI explains the difference between `100` (int) and `"100"` (str).

**Reflection:** Can you now identify types without help?

---

### Prompt 2: Fix Type Hint Errors

```
This code has type hint mistakes:

age: str = 25
name: int = "Alex"
price: float = "19.99"
is_student: bool = "True"

For each line:
1. What's wrong with the type hint?
2. What should the correct type hint be?
3. Show me the corrected code
```

**What to look for:** AI identifies mismatches between hints and values.

**Reflection:** Why do type hints matter if Python doesn't enforce them?

---

### Prompt 3: Build a Product Catalog

```
Help me create variables for a product catalog entry.

I need to store:
- Product name (text)
- Price (decimal number)
- Stock quantity (whole number)
- Is it on sale? (yes/no)
- Rating (decimal, out of 5.0)

Show me the code with proper type hints.
Then print a formatted product description.
```

**What to look for:** AI uses appropriate types for each piece of data.

**Reflection:** Run the code. Can you add two more product attributes?

---

### Prompt 4: Concept Check

```
Quiz me on type hints.

Ask me 3 questions:
1. What are the four basic Python types I learned?
2. What's the syntax for adding a type hint to a variable?
3. Why are type hints documentation rather than enforcement?

After each answer, tell me if I'm correct and explain.
```

**What to look for:** Can you answer confidently without looking back?

**Reflection:** Which type (int, str, float, bool) is clearest? Which is confusing?

---

**Safety Note:** When AI suggests type hints, verify they match the actual data. Type hints should accurately document intent.

**Next Steps:** You now understand data types and can label your variables! In [Lesson 5: User Input: Interactive Programs](./05-user-input-interactive-programs.md), you'll learn how to make programs that listen and respond to users.
