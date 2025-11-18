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

## Try With AI: User Profile Design Challenge

You've learned variables and type hints as syntax. Now apply them to a real problem: designing a user profile system from scratch.

### Part 1: Describe Your Data Needs (Before AI)

**Scenario**: You're designing a user profile system. For each user, you need to store:

1. **Username** (like "alex_2024")
2. **Age** (like 25)
3. **Account balance** (like $47.50)
4. **Is verified?** (yes or no)

**Your task (before asking AI)**:
For EACH piece of data, answer in plain English:
- What kind of data is this? (whole number? text? decimal? yes/no?)
- What Python type would you use? (Guess: `int`, `str`, `float`, `bool`)

Write your thinking down. Don't write Python code yet. Just describe WHAT you need.

---

### Part 2: AI Introduces Type Hints (Discovery)

Now describe your needs to AI in plain English:

> "I'm building a user profile system. I need to store: username (text), age (whole number), account balance (dollars and cents), and verification status (yes/no). What Python variables with type hints should I use? Show me the code."

**Your task**: Compare AI's code to your Part 1 answers.
- Did you guess the right types?
- Does AI's code match your thinking?
- Ask AI: "Why did you choose `float` for balance instead of `int`?"

This teaches you: **Describe intent in English → AI suggests implementation.**

---

### Part 3: Student Teaches AI (Naming Conventions)

AI gave you working code, but the variable names might not follow PEP 8.

Check AI's code against these rules:
1. ✓ Uses lowercase_with_underscores (not camelCase)?
2. ✓ Descriptive names (not `x`, `a`, `data`)?
3. ✓ Starts with letter (not number)?

If AI violated a rule, tell it:

> "Your variable names don't follow PEP 8. The lesson says Python uses lowercase_with_underscores, not camelCase. Can you revise?"

**Your task**: Verify AI corrects the mistake.
- Did it fix the naming?
- Did it explain WHY PEP 8 matters?

This teaches AI about the professional standards you're learning.

---

### Part 4: Iterate on the Design (Convergence)

Now expand the requirements. Tell AI:

> "I need to add 3 more fields: email address, registration date (text like '2025-01-15'), and premium membership status (yes/no). Update the code with correct type hints and PEP 8 naming."

**Your task**: Review AI's updated code.
- Are all 7 variables present?
- Are type hints correct?
- Do names follow PEP 8?
- Can you explain WHY each type was chosen?

If something is unclear, iterate:
> "Why did you use `str` for registration date instead of a date type?"

Keep iterating until the design makes perfect sense.

---

### Part 5: Validate with isinstance() (Your Turn)

Final challenge—write validation code WITHOUT AI first.

**Task**: Add validation checks using `isinstance()` for all 7 variables. Example:
```python
print(isinstance(username, str))  # Should print True
```

Write the validation code yourself. Then ask AI:
> "Here's my validation code: [paste code]. Is this correct? What did I miss?"

**Outcome**: If you got it right, you understand type hints deeply. If not, you learned what you're still confused about.

---

**Time**: 25-30 minutes total
**Outcome**: You've practiced spec-first design (describe needs before coding), learned to critique AI code against standards (PEP 8), iterated on requirements, and validated your understanding independently.

