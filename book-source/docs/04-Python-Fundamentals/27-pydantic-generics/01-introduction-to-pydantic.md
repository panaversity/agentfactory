---
title: "Introduction to Pydantic and Data Validation"
chapter: 27
lesson: 1
duration_minutes: 35
sidebar_position: 1
description: "Learn Pydantic V2 for runtime data validation, distinguish between type hints and validation, create your first models, and handle validation errors with grace."

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Data Validation Concepts"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain why runtime validation is needed and give 2-3 examples where it prevents bugs"

  - name: "Pydantic BaseModel Usage"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can create a working model with 5 fields and validate input data against it"

  - name: "Exception Handling (ValidationError)"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can catch ValidationError and display helpful error messages to users"

  - name: "Type Hints Application"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can apply correct type hints (str, int, float, list[str], X | None) to model fields"

learning_objectives:
  - objective: "Explain the difference between type hints (static documentation) and Pydantic validation (runtime enforcement)"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation in own words after reading lesson content"

  - objective: "Create basic Pydantic models with 3-5 fields using built-in validators"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successful model creation and validation in Try With AI exercises"

  - objective: "Handle ValidationError exceptions and understand error messages"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Correct exception handling and error interpretation in practice exercises"

  - objective: "Apply Pydantic to validate simple data structures (user profiles, book records)"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Working code in Try With AI prompts demonstrating real-world application"

cognitive_load:
  new_concepts: 10
  assessment: "10 new concepts (RuntimeValidation, BaseModel, Automatic Validation, ValidationError, Field Types, Optional Fields, Default Values, Model Instantiation, Installing Pydantic, Error Messages) at B1 limit ✓"

differentiation:
  extension_for_advanced: "Explore nested model patterns; create custom validation logic using Field(); research how Pydantic handles type coercion"
  remedial_for_struggling: "Focus on Book example as primary case study; practice ValidationError handling with simplified model (3 fields); use AI to explain each error message"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/001-part-4-chapter-27/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Introduction to Pydantic and Data Validation

## The Validation Problem

Imagine you're building an AI agent that accepts user data. A user registers with their name, email, and age. But what happens if someone submits:

```
{
  "name": "Alice",
  "email": "not-an-email",
  "age": "twenty-five"
}
```

Your code might crash silently, store invalid data, or worse—send bad data to your AI, which then generates incorrect responses. The problem: **Python's type hints only document what SHOULD be there, but don't enforce what IS actually there at runtime.**

This is where Pydantic enters the game. **Pydantic is a library that validates data at runtime**—it checks that your data actually matches your requirements before your code uses it. Type hints say "this SHOULD be an int"; Pydantic makes it "this MUST be an int or validation fails."

### Why This Matters for AI-Native Development

When Claude Code generates JSON for you, you need to validate it's correct BEFORE using it. When you build APIs with FastAPI, Pydantic automatically validates every request. When you load configuration files, Pydantic ensures they're valid. In production systems, **validation is not optional—it's your safety net**.

---

## Section 1: Your First Pydantic Model

### Installing Pydantic

Like any Python library, Pydantic needs to be installed first. You've already learned this pattern in Chapter 12 with `uv`:

```bash
uv add pydantic
```

This installs Pydantic V2 (the modern version). Pydantic V1 is deprecated—always use V2.

### Creating Your First Model: A Book

Let's start simple. Imagine you're building a library application that stores books. Each book has:

- **title** (text, required)
- **author** (text, required)
- **year** (whole number, required, between 1000-2100)
- **price** (decimal number, required, must be >= 0)
- **isbn** (text, optional)

With Pydantic, you describe this structure in code:

```python
from pydantic import BaseModel

class Book(BaseModel):
    title: str
    author: str
    year: int
    price: float
    isbn: str | None = None  # Optional field with default value
```

That's it. You've created a Pydantic model. Now let's use it.

### Validation Happens Automatically

Creating a valid book works exactly as you'd expect:

```python
# Valid data - no errors
book = Book(
    title="Python Guide",
    author="Jane Doe",
    year=2024,
    price=29.99
)

print(book)
# Output: Book(title='Python Guide', author='Jane Doe', year=2024, price=29.99, isbn=None)

# Access fields like normal attributes
print(book.title)  # Output: Python Guide
print(book.price)  # Output: 29.99
```

But try passing invalid data:

```python
from pydantic import ValidationError

try:
    bad_book = Book(
        title="Test Book",
        author="Author",
        year="not a year",  # ERROR: should be int, got str
        price=-10  # ERROR: must be >= 0
    )
except ValidationError as e:
    print(e)
```

**Output** (showing what validation catches):

```
2 validation errors for Book
year
  Input should be a valid integer [type=int_type, input_value='not a year', input_type=str]
price
  Input should be greater than or equal to 0 [type=greater_than_equal, input_value=-10, input_type=float]
```

Pydantic caught BOTH errors at once. This is powerful—you don't have to debug one error, fix it, then discover another. You see everything that's wrong.

#### 💬 AI Colearning Prompt
> "What happens when you pass a string to an int field in Pydantic? Explain the validation error and what type coercion means."

---

## Section 2: Understanding Validation Errors

### Reading ValidationError Messages

Pydantic's error messages are designed to help you. Let's break down what you're seeing:

```python
from pydantic import BaseModel, ValidationError

class User(BaseModel):
    name: str
    age: int
    email: str

try:
    user = User(
        name="Bob",
        age="thirty",  # Error 1: not an int
        email="bob@example"  # Error 2: doesn't look like email
    )
except ValidationError as e:
    # Print full error details
    print(e)

    # Or access error details programmatically
    for error in e.errors():
        print(f"Field: {error['loc']}")     # Which field?
        print(f"Problem: {error['msg']}")   # What's wrong?
        print(f"Type: {error['type']}")     # What type of error?
```

This gives you:
- **loc** (location): Which field has the problem?
- **msg** (message): What's wrong in plain English?
- **type** (type of error): Was it a type mismatch? A constraint violation? A format issue?

#### 🎓 Expert Insight
> In AI-native development, type hints document intent but Pydantic enforces it. When AI agents generate JSON or APIs send data, runtime validation catches mismatches before they corrupt your system. This isn't defensive programming—it's professional practice.

### Multiple Errors at Once

One of Pydantic's superpowers is reporting ALL validation problems simultaneously. This saves debugging time:

```python
try:
    bad_user = User(
        name=123,           # Error: not a string
        age="not a number", # Error: not an int
        email="missing-at-sign"  # Error: invalid format
    )
except ValidationError as e:
    # Shows all 3 errors at once
    print(f"Found {len(e.errors())} validation errors")
    for error in e.errors():
        print(f"  - {error['loc'][0]}: {error['msg']}")
```


---

## Section 3: Nested Models

### Real Data Is Complex

So far we've created flat models with simple fields. But real data is hierarchical. A Book might have an Author, and an Author has multiple attributes:

```python
from pydantic import BaseModel

class Author(BaseModel):
    name: str
    bio: str

class Book(BaseModel):
    title: str
    authors: list[Author]  # List of Author objects!
    publication_date: str
```

Notice `authors: list[Author]`—this is a **list of Author models**. Pydantic validates each Author in the list.

### Using Nested Models

Creating a book with authors:

```python
# Method 1: Create Author objects first
author1 = Author(name="Alice Smith", bio="Python expert")
author2 = Author(name="Bob Johnson", bio="Data scientist")

book = Book(
    title="Advanced Python",
    authors=[author1, author2],
    publication_date="2024-01-15"
)

# Method 2: Pass dictionaries - Pydantic converts them
book2 = Book(
    title="Web Development",
    authors=[
        {"name": "Charlie Brown", "bio": "Full-stack developer"},
        {"name": "Diana Prince", "bio": "Frontend specialist"}
    ],
    publication_date="2024-03-20"
)

# Serialize back to dictionary for APIs or storage
print(book.model_dump())
# Output: {
#   'title': 'Advanced Python',
#   'authors': [
#     {'name': 'Alice Smith', 'bio': 'Python expert'},
#     {'name': 'Bob Johnson', 'bio': 'Data scientist'}
#   ],
#   'publication_date': '2024-01-15'
# }
```

Validation happens at all levels. If an Author's `name` is missing, Pydantic catches it:

```python
try:
    bad_book = Book(
        title="Test",
        authors=[
            {"name": "Valid Author", "bio": "Good"},
            {"bio": "Missing name!"}  # ERROR: name is required
        ],
        publication_date="2024-01-01"
    )
except ValidationError as e:
    print(e)
    # Shows error in nested structure:
    # authors.1.name: Field required
```

#### 🤝 Practice Exercise

> **Ask your AI**: "Create an Author model with name and bio fields. Then create a Book model that contains a single author field (not a list—just one Author). Generate code that creates a Book with a nested Author and demonstrates the validation error when author data is missing."

**Expected Outcome**: You'll see working nested model structure and understand how Pydantic validates nested fields, catching missing required fields at any level of nesting.

---

## Section 4: Common Mistakes

### Mistake 1: Forgetting BaseModel

Pydantic models must inherit from `BaseModel`:

```python
# WRONG - just a regular class, no validation
class Book:  # Missing: BaseModel
    title: str
    author: str

book = Book(title="Test", author="Author")
# This works but does NO validation!

# CORRECT - inherits from BaseModel
from pydantic import BaseModel

class Book(BaseModel):  # Inherits validation
    title: str
    author: str

book = Book(title="Test", author="Author")
# Now validation works
```

### Mistake 2: Not Handling ValidationError

If you don't catch `ValidationError`, your program crashes:

```python
# WRONG - will crash if data is invalid
book = Book(title="Test", author=123)  # Crash!

# CORRECT - handle the error gracefully
try:
    book = Book(title="Test", author=123)
except ValidationError as e:
    print(f"Invalid data: {e}")
    # Program continues, user sees helpful message
```

### Mistake 3: Mixing Up Type Hints

Type hints must be precise. `list` is different from `list[str]`:

```python
# Ambiguous - what's in the list?
tags: list  # Could contain anything

# Precise - list of strings
tags: list[str]  # Validates each item is a string

class Post(BaseModel):
    title: str
    tags: list[str]  # Pydantic validates each tag

# Valid
post = Post(title="AI", tags=["python", "pydantic"])

# Invalid - number in a list that should contain strings
try:
    post = Post(title="AI", tags=["python", 123])  # ERROR
except ValidationError as e:
    print(e)  # tags.1: Expected string, got int
```

---

## Try With AI: The Data Validation Discovery (4-Part Learning Challenge)

This challenge teaches you bidirectional learning: you discover problems, AI teaches concepts, you challenge AI's approach, then you build real artifacts.

---

### Part 1: You Discover (Student Discovers Problems)

**Your Turn** — Write code WITHOUT Pydantic to experience the validation problem:

```python
# Part 1: Manual validation (the hard way)
# Write a simple user registration system that validates data manually:

user_data = {
    "username": "alice",
    "email": "alice@example.com",
    "age": 25,
    "bio": "Software developer"
}

# Challenge: Write validation functions that ensure:
# 1. username: required, 3-20 characters, alphanumeric + underscore
# 2. email: required, contains @, domain has dot
# 3. age: required, integer between 13-120
# 4. bio: optional, max 200 characters

# Problem to discover:
# - You'll write lots of boilerplate validation code
# - Each field needs separate checks
# - Error messages aren't centralized
# - You can't reuse validation logic easily
# - Testing validation is tedious
```

**What You'll Realize**: Manual validation is repetitive, error-prone, and impossible to scale. Each field needs its own checks. This is where Pydantic saves you.

---

### Part 2: AI as Teacher (AI Explains Concepts)

**Ask your AI:**

> "I just wrote manual validation code for a user registration system (3-5 fields with different constraints). It's repetitive and error-prone. Show me how Pydantic solves this problem.
>
> First, explain the difference between type hints (static) and validation (runtime) with concrete examples from my validation code that would break. Then show me the same validation using Pydantic—how would you define a User model that validates the same rules automatically?"

**What AI Should Show You**:
- How Pydantic BaseModel replaces your manual validation functions
- How type hints alone (str, int) aren't enough—Pydantic enforces them at runtime
- How Field() constraints replace your custom validation logic
- Side-by-side comparison: your manual code vs. Pydantic

**Your Role**: Ask clarifying questions. "Why does Pydantic report ALL errors at once instead of stopping at the first one?" "Can I add custom validation beyond Field()?" "How does Pydantic know about email format?"

---

### Part 3: You as Teacher (You Challenge AI's Approach)

**Challenge AI** — Ask it to handle edge cases it might miss:

> "Now test your Pydantic model with edge cases I discovered in my manual validation:
> 1. What happens if I pass a string like '25' (quoted number) instead of integer 25?
> 2. What if email is 'alice@localhost' (no dot in domain)?
> 3. What if someone passes username='alice_' (underscore at end)?
> 4. What if age is 120.5 (float instead of int)?
>
> For each case, show me: a) What Pydantic does, b) Whether that's correct behavior, c) How to fix it if needed."

**Your Role**: Push back. "Your model accepts 'test@localhost'—but we need a proper domain. How do we add that validation?" or "Can we coerce '25' to integer 25 automatically?" This forces AI to improve its model.

**AI's Response Should Show**:
- How Pydantic handles type coercion (strings → ints when possible)
- How to add custom validators for complex rules
- How to reject invalid data with clear error messages

---

### Part 4: You Build (Production Artifact)

**Build a Complete User Validation System** — Synthesize everything into a working module:

```python
# deliverable: user_validation.py

from pydantic import BaseModel, Field, field_validator, ValidationError
from typing import Optional

class User(BaseModel):
    """Production User model with comprehensive validation."""

    # Define your fields with type hints + constraints
    username: str = Field(min_length=3, max_length=20, pattern=r"^[a-z0-9_]+$")
    email: str
    age: int = Field(ge=13, le=120)
    bio: Optional[str] = Field(default=None, max_length=200)

    # Custom validators for complex rules
    @field_validator('email')
    @classmethod
    def validate_email(cls, v: str) -> str:
        if '@' not in v or '.' not in v.split('@')[1]:
            raise ValueError('Invalid email format')
        return v.lower()

# Test your implementation
if __name__ == "__main__":
    # Valid user
    user = User(
        username="alice_123",
        email="alice@example.com",
        age=25,
        bio="Software developer"
    )
    print(f"✓ User created: {user.username}")

    # Invalid user—Pydantic catches all errors
    try:
        bad_user = User(
            username="ab",  # Too short
            email="alice@localhost",  # No dot in domain
            age=150,  # Too old
            bio="a" * 201  # Too long
        )
    except ValidationError as e:
        print(f"✗ Validation failed with {len(e.errors())} errors:")
        for error in e.errors():
            print(f"  {error['loc'][0]}: {error['msg']}")
```

**Success Criteria**:
- Your model validates all fields (no manual checks needed)
- Invalid data raises ValidationError with clear messages
- You understand why Field() worked vs. why you needed @field_validator
- You can explain the difference between Pydantic's automatic and custom validation

---

## Time Estimate
**25-30 minutes** (5 min discover, 8 min AI teaches, 7 min you challenge, 5 min build)
