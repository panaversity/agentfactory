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
generated_by: "lesson-writer v3.0.0"
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

> "Ask your AI: What happens if I pass a string to an int field in Pydantic? Show me the validation error and explain what type coercion means."

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

#### 🎓 Instructor Commentary

> Type hints are suggestions; Pydantic makes them laws. In AI-native development, you often receive data from external sources (APIs, AI agents, users). Pydantic ensures that data is valid BEFORE your code tries to use it. This defensive approach prevents subtle bugs that only show up later.

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

#### ✨ Teaching Tip

> In production code, always wrap Pydantic operations in try/except blocks. Use the error information to provide helpful feedback to users rather than crashing silently. Example: Instead of "Error: validation failed", tell users "Invalid email format. Please use name@domain.com".

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

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:

> "Create an Author model with name and bio fields. Then create a Book model that contains a single author field (not a list—just one Author). Generate code that creates a Book with a nested Author and shows the validation error when author data is missing."

**Expected Outcome**: Working nested model structure, validation demonstrating that missing nested fields are caught.

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

## Try With AI

Using your AI companion (Claude Code, Gemini CLI, or ChatGPT), practice Pydantic validation. These prompts progress from understanding concepts to creating production-ready validation.

### Prompt 1: Understand Type Hints vs Validation

**Ask your AI:**

> "Explain the difference between Python's built-in type hints and Pydantic's runtime validation. Give 2-3 concrete examples showing when type hints aren't enough and why Pydantic matters."

**Expected Outcome**: Your AI should explain that type hints are static (only visible to your IDE), while Pydantic enforces rules at runtime. Examples might include API requests with wrong data types, AI-generated JSON that doesn't match your schema, or configuration files with invalid values.

---

### Prompt 2: Apply - Create Your First Model

**Tell your AI:**

> "Create a Pydantic model for a Product with: name (required string), price (required float, must be >= 0), quantity (required integer), and description (optional string). Then write code that validates both valid and invalid product data, showing what validation errors look like."

**Expected Outcome**: A working Product model with proper field types, test cases showing successful validation, and examples of validation errors with explanations of what went wrong.

---

### Prompt 3: Analyze - When to Use Pydantic

**Ask your AI:**

> "When would you use Pydantic instead of just type hints? Give 3 real-world scenarios where runtime validation is critical. For each scenario, explain what could go wrong without validation."

**Expected Outcome**: Scenarios like API input validation (preventing bad data from reaching your code), LLM output validation (ensuring AI responses match your expected format), configuration file validation (catching typos in .env or YAML), or database schema enforcement. Each should explain the consequence of missing validation.

---

### Prompt 4: Create - Nested Models with Validation

**Tell your AI:**

> "Build a UserProfile model with a nested Address model. Include: UserProfile (name, email, age), Address (street, city, zip_code). Add validation that zip_code must be exactly 5 digits. Generate test data showing both valid and invalid zip codes, and demonstrate the validation errors."

**Expected Outcome**: Nested UserProfile and Address models with custom validation on zip_code, example data for valid cases (zip: "12345"), invalid cases (zip: "ABC"), and clear error messages showing which field failed validation and why.
