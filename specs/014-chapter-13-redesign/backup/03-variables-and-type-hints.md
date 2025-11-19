---
title: "Variables and Type Hints – Describing Intent"
chapter: 13
lesson: 3
duration_minutes: 75

skills:
  - name: "Variable Declaration and Assignment"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student can create variables with correct type hints (str, int, float, bool) without syntax errors"

  - name: "Type Hints as Intent Description"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can explain why type hints matter for AI collaboration and specification-first thinking"

  - name: "Python Naming Conventions (PEP 8)"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student can write variable names following PEP 8 (lowercase_with_underscores, descriptive, valid syntax)"

  - name: "Type Validation with isinstance()"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can use isinstance() to check variable types at runtime"

learning_objectives:
  - objective: "Create variables with type hints and understand their role in describing intent"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student writes 5 typed variables with correct syntax and explains why each type is appropriate"

cognitive_load:
  new_concepts: 6
  assessment: "6 concepts (variable, 4 primitive types, type hints, naming conventions, collection awareness, validation) at A2 limit with collection awareness as surface preview ✓"

differentiation:
  extension_for_advanced: "Explore type checking tools; research Python typing module for advanced type hints"
  remedial_for_struggling: "Focus on one type at a time; practice only int and str before float and bool"

generated_by: "content-implementer v3.0.0"
source_spec: "specs/016-part-4-chapter-13/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Lesson 3: Variables and Type Hints – Describing Intent

You've learned what Python is. You've got it installed. Now you're ready for your first real Python code.

In this lesson, you'll learn the most fundamental concept in programming: **variables**. A variable is simply a named container that holds data. Type hints are how you describe what kind of data goes in that container.

But here's where Chapter 13 differs from traditional Python courses: **Type hints aren't optional or advanced. They're core.** Type hints are how you describe intent—and describing intent is the core of AI-Driven Development.

## Variables: Names for Values

Imagine you have a filing cabinet with labeled drawers. Each drawer holds something specific.

In the drawer labeled "age," you store the number 25.

In the drawer labeled "name," you store the text "Alice."

In the drawer labeled "is_student," you store the fact that the person is (True) or isn't (False) a student.

In Python, these drawers are called **variables**.

Creating a variable looks like this:

```python
age = 25
name = "Alice"
```

The variable is the name (age, name). The value is what's inside (25, "Alice").

But here's the innovation: **You can tell Python what kind of data goes in each drawer before you use it.**

```python
age: int = 25
name: str = "Alice"
```

The part after the variable name (`: int`, `: str`) is called a **type hint**. It says "this variable holds an integer" or "this variable holds a string."

## The Four Core Primitive Types

Python has many data types, but the four most fundamental are:

### int – Whole Numbers

```python
age: int = 25
score: int = 100
count: int = 0
```

Use `int` when your data is a whole number (no decimals). Ages, counts, scores, quantities—all `int`.

### str – Text (Strings)

```python
name: str = "Alice"
city: str = "Portland"
greeting: str = "Hello, World!"
```

Use `str` when your data is text. Always put text in quotes. `str` stands for "string," which is programmer jargon for "text."

### float – Decimal Numbers

```python
price: float = 19.99
height: float = 5.7
temperature: float = 98.6
```

Use `float` when your data is a decimal number. "Float" is short for "floating-point number," which is the technical term for decimals in programming.

### bool – True or False

```python
is_student: bool = True
is_valid: bool = False
has_permission: bool = True
```

Use `bool` (short for boolean) when your data is a yes/no or true/false value. Booleans are surprisingly useful for checking conditions and making decisions—concepts you'll see in later chapters.

## Type Hints: Describing Intent

Here's why type hints matter so much:

Without type hints: `age = 25`

This works, but what is `age`? Is it a year? A count? A score? A price? The code doesn't say.

With type hints: `age: int = 25`

Now it's crystal clear. `age` is an integer. Anyone reading this code (including AI) knows exactly what to expect.

**Type hints are specifications.** When you write a type hint, you're saying "I intend for this variable to hold this kind of data."

This matters enormously for AI collaboration. When you ask your AI companion to help with your code, the type hints tell the AI exactly what you're building.

## Python Naming Conventions (PEP 8)

Python has a style guide called PEP 8 that professional developers follow. Here are the naming rules:

**Use lowercase with underscores:**
```python
user_name = "Alice"      # ✓ Good
userName = "Alice"       # ✗ Not Python style
USER_NAME = "Alice"      # ✗ Used for constants, not regular variables
```

**Be descriptive:**
```python
age = 25                 # ✓ Clear
a = 25                   # ✗ What does 'a' mean?
customer_name = "Bob"    # ✓ Clear
cn = "Bob"               # ✗ Confusing abbreviation
```

**Start with a letter or underscore:**
```python
user_age: int = 30       # ✓ Starts with letter
2user: int = 30          # ✗ Can't start with number
_temp: int = 0           # ✓ Underscore is valid
```

**No spaces:**
```python
favorite_color: str = "blue"    # ✓ Underscores, no spaces
favorite color: str = "blue"    # ✗ Spaces not allowed
```

**Avoid Python keywords:**
```python
user_class: str = "senior"      # ✓ Variable name is 'user_class'
class: str = "senior"           # ✗ 'class' is a Python keyword, can't use it
```

**Examples of well-named variables:**
- `user_name`, `total_price`, `is_valid`, `item_count`, `customer_email`

**Examples of poorly-named variables:**
- `x`, `a`, `data`, `stuff`, `value1` (all too vague)

When you follow these conventions, your code is easier to read, easier for AI to understand, and easier for other developers to work with.

## Collection Types Awareness (Preview)

Python has ways to store multiple values together. You'll learn these in detail in Chapters 18–19, but it's useful to know they exist:

- **list** — An ordered collection of items: `[1, 2, 3]` or `["apple", "banana"]`
- **dict** — Key-value pairs: `{"name": "Alice", "age": 25}`
- **tuple** — An immutable ordered collection: `(1, 2, 3)`
- **set** — An unordered collection of unique items: `{1, 2, 3}`

You don't need to understand these now. Just know they exist. When you see them in later chapters, you'll recognize them.

## Working With Variables: Checking Types

Now that you have variables, you might want to check what type they are. Python gives you functions for this.

**The `print()` function** — The `print()` function displays output to your screen. It's how you see what's inside variables.

```python
age: int = 25
print(age)  # Output: 25

name: str = "Alice"
print(name)  # Output: Alice
```

**The `type()` function** — The `type()` function tells you what kind of data a variable holds.

```python
age: int = 25
print(type(age))  # Output: <class 'int'>

name: str = "Alice"
print(type(name))  # Output: <class 'str'>
```

**The `isinstance()` function** — The `isinstance()` function checks if a variable is a specific type. It returns `True` or `False`.

```python
age: int = 25
print(isinstance(age, int))   # Output: True
print(isinstance(age, str))   # Output: False

name: str = "Alice"
print(isinstance(name, str))  # Output: True
```

This function is especially useful for validation—checking whether data is the right type before using it.

## Code Examples

Here's a complete program that demonstrates variables and type hints:

```python
# Demonstrating all four primitive types
name: str = "Alice"
age: int = 25
height: float = 5.7
is_student: bool = True

# Display the variables
print(name)
print(age)
print(height)
print(is_student)

# Check types
print(type(name))      # <class 'str'>
print(type(age))       # <class 'int'>

# Validate types
print(isinstance(age, int))      # True
print(isinstance(age, str))      # False
```

This example shows:
- Creating variables with type hints
- Using `print()` to display them
- Using `type()` to check what kind they are
- Using `isinstance()` to validate types

#### 💬 AI Colearning Prompt

Ask your AI companion: "Explain how type hints help an AI (like you) generate better code. Give a specific example: What's the difference between understanding `age = 25` vs. `age: int = 25`?"

This is a critical insight. Type hints are how you make your intent explicit. Your AI uses that explicitness to understand your specifications and generate appropriate code.

#### 🤝 Practice Exercise

> **Ask your AI**: "Create 5 typed variables representing a person: name, age, height in meters, student status (boolean), favorite programming language. Use realistic values. Then write print statements with f-strings displaying each. Explain why the type hint for height is `float` and not `int`."

**Expected Outcome**: You'll practice writing typed variables, understand the difference between int and float, see f-string formatting in action, and learn how type hints make code self-documenting.

#### 🎓 Expert Insight
> In AI-native development, type hints aren't "advanced Python"—they're foundational specifications. When you write `age: int = 25`, you're declaring intent that AI understands and can validate. Professional developers use type hints everywhere because they prevent bugs, enable better IDE support, and make code self-documenting. In this book, type hints are mandatory—not as busywork, but as practice for specification-first thinking.

## Common Mistakes

**Mistake 1**: Forgetting the colon in type hints

```python
age int = 25         # ✗ Wrong—missing colon
age: int = 25        # ✓ Correct
```

Python needs that colon to understand the type hint. Without it, you'll get a syntax error.

**Mistake 2**: Using quotes around numbers

```python
age: int = "25"      # ✗ Wrong—this is a string, not an integer
age: int = 25        # ✓ Correct—no quotes for numbers
```

The quotes tell Python "this is text." Numbers don't need quotes.

**Mistake 3**: Confusing type hints with enforcement

Many students think Python will refuse to run if you violate type hints. Actually, Python doesn't enforce type hints at runtime. You *can* write:

```python
age: int = "twenty-five"  # Python allows this (but shouldn't)
```

Python won't complain. But this violates the intent you declared with the type hint. Use `isinstance()` to check types yourself.

**Mistake 4**: Non-descriptive variable names

```python
x: int = 25              # ✗ What is x?
age: int = 25            # ✓ Clear purpose
```

"x" might work in math class, but in programming, variable names should describe purpose.

**Mistake 5**: Invalid variable names

```python
2age: int = 25           # ✗ Can't start with number
user age: int = 25       # ✗ Can't have spaces
class: str = "student"   # ✗ 'class' is a Python keyword
age2: int = 25           # ✓ Valid—can end with number
_age: int = 25           # ✓ Valid—can start with underscore
```

Remember: Start with letter/underscore, use lowercase_with_underscores, no spaces, avoid keywords.

---

## Try With AI

Use your AI companion (Claude Code or Gemini CLI) for these prompts.

**Prompt 1: Recall – Type Hint Syntax**

```
From Lesson 3, what is the correct syntax for a type hint?
Write the pattern: name: _______ = _______
Give three examples with different types (int, str, float).
```

**Expected Outcome**: You recall exact syntax. You can replicate the pattern. You demonstrate syntax memorization.

---

**Prompt 2: Understand – Why Type Hints Matter**

```
Explain: "Type hints describe intent."
What does this mean? How does `age: int = 25` describe intent differently than `age = 25`?
Ask your AI: "Why is `age: int = 25` better than `age = 25` from an AI collaboration perspective?"
```

**Expected Outcome**: You understand type hints as communication tools. You see connection to AI. You can explain the difference in clarity.

---

**Prompt 3: Apply – Creating Typed Variables**

```
Write 5 variables with correct type hints for:
1. A person's name (string)
2. How many years they've been coding (integer)
3. Their coding skill level as a percentage (float, 0-100)
4. Whether they prefer Python (boolean, True/False)
5. Something else important to you

Check your work: Ask your AI "Are these type hints correct? Can you explain why each one is right?"
```

**Expected Outcome**: You write correct syntax. You validate with AI. You demonstrate mastery of basic type hints.

---

**Prompt 4: Analyze – Type Validation and Intent (Cognitive Closure)**

```
Here's a puzzle: Python allows this code to run:

age: int = "twenty-five"  # Type hint says int, but we assigned a string!

Ask your AI:
1. Why does Python allow this (wrong data for the type hint)?
2. What's the difference between "type hints" and "type enforcement"?
3. How would you check if age is actually an integer? (Hint: isinstance())
4. Why would violating type hints be a bad idea in professional code?
```

**Expected Outcome**: You understand distinction between hints and enforcement. You learn isinstance() for validation. You grasp importance of type consistency. You close Lesson 3 with clear understanding of why this matters.

