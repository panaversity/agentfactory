---
title: "Dictionaries Part 1 - Key-Value Mappings"
chapter: 18
lesson: 7
duration_minutes: 210

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Dictionary Creation with Type Hints"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can write dict[str, int] and dict[str, str | int] with proper syntax and understand type constraints"

  - name: "Key-Value Access and KeyError Handling"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can retrieve values using bracket notation, recognize KeyError, and use .get() with defaults"

  - name: "Dict Methods Introduction"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student understands purpose of .keys(), .values(), .items() and when to use each"

  - name: "Type-Safe Dictionary Design"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student uses union types str | int for heterogeneous values and reasons about type safety"

learning_objectives:
  - objective: "Create dictionaries using literals and type hints dict[K, V] to express intent"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student writes type-hinted dicts for real-world data (student records, configs)"

  - objective: "Access dictionary values using bracket notation and handle missing keys with .get()"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student writes code that retrieves values and explains when to use each access pattern"

  - objective: "Understand key-value mapping concept and when dicts are better than lists"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Student explains difference between list indexing and dict key lookup"

  - objective: "Apply union types to dictionaries with mixed-type values"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student creates dicts with dict[str, str | int] and validates type safety"

cognitive_load:
  new_concepts: 6
  assessment: "6 new concepts (dict literals, type hints dict[K,V], bracket notation, KeyError, .get(), unique keys) within A2 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Students reaching toward B1 can explore nested dicts (preview only), explore dict performance characteristics, and discuss hashability constraints for keys"
  remedial_for_struggling: "Provide worked examples with student records; focus on .get() method as safe access pattern; compare dict to list lookup time visually"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/001-part-4-chapter-18/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Dictionaries Part 1: Key-Value Mappings

Imagine you're tracking information about students in a class. You could use a list:

```python
students: list = ["Alice", 20, "Computer Science", 3.8]
```

But there's a problem: months later, do you remember which index holds the age? Is index 1 the age or the major? When you read that code again, it's confusing.

**Dictionaries solve this problem** by letting you use meaningful names (called **keys**) instead of mysterious numbers. Keys map to the values you care about. Instead of `students[1]`, you write `student["age"]`. The code reads like English.

In this lesson, you'll learn how to create dictionaries, access their values safely, and use type hints to make your intent crystal clear to Python tools and other developers.

## What Is a Dictionary?

A **dictionary** is a Python data structure that stores **key-value pairs**. Each key maps to one value, like a real dictionary where you look up a word (key) to find its definition (value).

Here's a simple example:

```python
student: dict[str, str | int] = {
    "name": "Alice",
    "age": 20,
    "major": "Computer Science"
}
```

In this dictionary:
- Keys are: `"name"`, `"age"`, `"major"`
- Values are: `"Alice"`, `20`, `"Computer Science"`
- The `dict[str, str | int]` type hint says: "Keys are strings, values can be either strings OR integers"

**Key insight**: Dictionaries are **unordered** mappings, not sequences like lists. You don't access values by position (like `student[0]`)—you access them by meaningful keys (like `student["age"]`).

#### 💬 AI Colearning Prompt

> "Why would you use a dictionary instead of a list to store student information? What problem does a key-value structure solve?"

This question helps you think about the **intent** behind choosing a dictionary. You're not memorizing syntax—you're understanding *when* to use each structure.

## Creating Dictionaries

There are two main ways to create dictionaries: **literals** (most common) and the `dict()` constructor (less common, useful for specific patterns).

### Dictionary Literals

The most Pythonic way is to use curly braces `{}` with `key: value` pairs:

```python
# Student record with multiple fields
student: dict[str, str | int] = {
    "name": "Alice",
    "age": 20,
    "major": "Computer Science"
}

# Configuration settings (all string values)
config: dict[str, str] = {
    "database_url": "postgres://localhost:5432",
    "debug_mode": "true",
    "api_key": "sk-..."
}

# Grade mapping (string keys, integer values)
grades: dict[str, int] = {
    "Alice": 95,
    "Bob": 87,
    "Charlie": 92
}

# Empty dictionary (to fill later)
empty_dict: dict[str, int] = {}
```

**Notice the type hints**: Each one tells you what types keys and values expect.

#### 🎓 Expert Insight

> In AI-native development, type hints like `dict[str, int]` aren't just syntax—they're **communication**. You're telling Python tools, your teammates, and AI collaborators: "Keys are strings, values are integers." This clarity unlocks better suggestions from AI and catches mismatches early.

### Union Types for Mixed Values

Sometimes dictionary values have different types. Use the **union operator** `|` to express this:

```python
# Student record with mixed types
student: dict[str, str | int] = {
    "name": "Alice",           # string value
    "age": 20,                 # integer value
    "major": "Computer Science" # string value
}

# Page configuration (strings and booleans mixed)
page_config: dict[str, str | bool] = {
    "title": "Home",          # string
    "dark_mode": True,        # boolean
    "subtitle": "Welcome"     # string
}
```

**Why use union types?** They document intent: "This dict can have multiple value types, and that's intentional."

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:

> "Create a user profile dictionary with the following fields: username (string), age (integer), is_verified (boolean), email (string). Use proper type hints including union types. Then explain why we use dict[str, str | int | bool] instead of just dict."

**Expected Outcome**: You'll understand how union types document mixed-type data and why that clarity matters in real code.

### The `dict()` Constructor

Less common, but useful when converting from other data:

```python
# Create from list of tuples
pairs: list[tuple[str, int]] = [("Alice", 95), ("Bob", 87)]
grades: dict[str, int] = dict(pairs)

# Create empty dict
empty: dict[str, str] = dict()
```

For now, stick with **literals** (`{}`). The `dict()` constructor is useful in advanced scenarios.

## Accessing Dictionary Values

Now you have a dictionary. How do you get values out?

### Bracket Notation: Direct Access

The simplest way—use square brackets with the key:

```python
student: dict[str, str | int] = {
    "name": "Alice",
    "age": 20,
    "major": "Computer Science"
}

print(student["name"])    # Output: Alice
print(student["age"])     # Output: 20
print(student["major"])   # Output: Computer Science
```

**This works perfectly** when you know the key exists. But what if you try to access a key that doesn't exist?

### KeyError: The Error You'll See

Access a non-existent key, and you get a **KeyError**:

```python
student: dict[str, str | int] = {
    "name": "Alice",
    "age": 20,
    "major": "Computer Science"
}

print(student["gpa"])  # KeyError: 'gpa'
```

The error message is clear: the key `"gpa"` doesn't exist. This is **by design**—Python tells you immediately when you ask for something that isn't there.

#### ✨ Teaching Tip

> Use Claude Code to experiment with KeyError. Ask: "What does the KeyError traceback tell me? Why is Python strict about missing keys?"

Understanding errors is part of learning. Python's strictness prevents silent bugs.

### Safe Access with `.get()` Method

Often, you **don't know if a key exists**, and you want a **default value** if it's missing. Use the `.get()` method:

```python
student: dict[str, str | int] = {
    "name": "Alice",
    "age": 20,
    "major": "Computer Science"
}

# Safe access: returns "N/A" if "gpa" doesn't exist
gpa = student.get("gpa", "N/A")
print(gpa)  # Output: N/A

# Safe access: returns "Unknown" if "hometown" doesn't exist
hometown = student.get("hometown", "Unknown")
print(hometown)  # Output: Unknown

# Safe access: key exists, returns actual value
name = student.get("name", "Unknown")
print(name)  # Output: Alice
```

**Syntax**: `dict.get(key, default_value)`

**When to use `.get()`**:
- You're not sure if a key exists
- You want a sensible default if it's missing
- You want to avoid KeyError

**When to use bracket notation** (`dict[key]`):
- You're certain the key exists
- You want an error if the key is missing (catching bugs)

#### 💬 AI Colearning Prompt

> "I'm building a feature that retrieves user settings. Sometimes a setting doesn't exist. Should I use bracket notation or .get()? What are the tradeoffs?"

This question teaches you to **think about edge cases**—what happens when data is incomplete. The answer shapes how you design your code.

## Important: Unique Keys

Every key in a dictionary **must be unique**. If you add a key that already exists, it **overwrites** the previous value:

```python
config: dict[str, str] = {
    "host": "localhost",
    "port": "5432"
}

# Add a new setting
config["debug"] = "true"
print(config)
# Output: {'host': 'localhost', 'port': '5432', 'debug': 'true'}

# Overwrite an existing key
config["port"] = "3306"  # Changed from 5432 to 3306
print(config)
# Output: {'host': 'localhost', 'port': '3306', 'debug': 'true'}
```

**This is intentional**. If you try to add a duplicate key, Python silently overwrites. This is often what you want (updating a setting), but it's worth knowing.

## Practical Example: Student Record System

Let's build a real-world example—a system to track student records:

```python
# Student record with type hints
student: dict[str, str | int] = {
    "name": "Alice",
    "age": 20,
    "student_id": 12345,
    "major": "Computer Science"
}

print(f"Name: {student['name']}")
print(f"ID: {student['student_id']}")

# Safely access a field that might not exist
gpa = student.get("gpa", "Not recorded")
print(f"GPA: {gpa}")

# Add a new field
student["gpa"] = 3.8
print(f"GPA (updated): {student['gpa']}")

# Safely update a field
student["age"] = student.get("age", 0) + 1
print(f"Age (updated): {student['age']}")
```

Output:
```
Name: Alice
ID: 12345
GPA: Not recorded
GPA (updated): 3.8
Age (updated): 21
```

**What's happening**:
1. We create a student record with mixed types (strings and integers)
2. We access values safely, knowing which keys exist
3. We use `.get()` with a default for optional fields
4. We update values by adding new keys or reassigning existing ones

This is how real programs track data—with dictionaries, not mysterious lists.

#### 🎓 Expert Insight

> Notice we used f-strings for output: `f"Name: {student['name']}"`. This is readable Python. The dictionary structure (meaningful keys) makes the code self-documenting. When you read this code in 6 months, you'll instantly understand what's stored and why.

## Preview: Dictionary Methods

Dictionaries have several useful methods. You'll explore these deeply in Lesson 8, but here's a quick preview:

- `.keys()` — List all keys: `student.keys()`
- `.values()` — List all values: `student.values()`
- `.items()` — List all key-value pairs: `student.items()`

```python
student: dict[str, str | int] = {
    "name": "Alice",
    "age": 20,
    "major": "Computer Science"
}

print(student.keys())    # dict_keys(['name', 'age', 'major'])
print(student.values())  # dict_values(['Alice', 20, 'Computer Science'])
print(student.items())   # dict_items([('name', 'Alice'), ('age', 20), ('major', 'Computer Science')])
```

These methods are useful when iterating (Lesson 9) or examining dictionary structure. For now, know they exist.

## Quick Reference: Bracket vs `.get()`

| Scenario | Use This | Why |
|----------|----------|-----|
| Key definitely exists | `dict[key]` | Direct access, fails loudly if wrong |
| Key might not exist | `.get(key, default)` | Safe fallback, no error |
| You want an error if missing | `dict[key]` | Catches bugs early |
| You want a sensible default | `.get(key, default)` | Handles missing data gracefully |

## Practice Exercises

### Exercise 1: Create a Typed Dictionary

Create a dictionary representing a book with the following fields:
- `title` (string): "The Pragmatic Programmer"
- `author` (string): "David Thomas"
- `year` (integer): 1999
- `pages` (integer): 352

Write the dictionary with proper type hints. Then print the title and author.

```python
# Write your solution here
book: dict[str, str | int] = {
    # ... fill this in
}

print(f"Title: {book['title']}")
print(f"Author: {book['author']}")
```

**Check your understanding**: Can you explain what `dict[str, str | int]` means?

### Exercise 2: Safe Access with `.get()`

Using the book dictionary from Exercise 1, safely access:
- `publisher` (doesn't exist) with default "Unknown Publisher"
- `pages` (exists) to get the actual value
- `rating` (doesn't exist) with default 0.0

Print all three values.

```python
# Extend your solution from Exercise 1
book: dict[str, str | int] = {
    "title": "Python Mastery",
    "author": "Jane Smith",
    "pages": 320
}

publisher = book.get("publisher", "Unknown Publisher")
rating = book.get("rating", 0.0)
pages_count = book.get("pages")

print(f"Publisher: {publisher}")
print(f"Rating: {rating}")
print(f"Pages: {pages_count}")
```

**Check your understanding**: Why does `.get()` not raise a KeyError?

### Exercise 3: Union Types with Mixed Values

Create a dictionary representing a website configuration:
- `domain` (string): "example.com"
- `port` (integer): 8080
- `ssl_enabled` (boolean): True
- `cache_ttl` (integer): 3600

Use the correct type hint for mixed types. Then access and print each value.

```python
# Write your solution here (remember: use dict[str, str | int | bool])
config: dict[str, str | int | bool] = {
    # ... fill this in
}

# Print each value
```

**Check your understanding**: Why do we use `str | int | bool` instead of just `str`?

### Exercise 4: Real-World Application

You're building a contact management system. Create a dictionary for a contact with:
- `name`: "Alice Johnson"
- `email`: "alice@example.com"
- `phone`: "555-1234"
- `age`: 28
- `verified`: True (boolean)

Access the contact's name and email. Then safely retrieve `notes` (which doesn't exist) with a default value of "No notes".

```python
# Write your solution here (remember union types!)
contact: dict[str, str | int | bool] = {
    # ... fill this in
}

# Access values and handle missing data
```

**Check your understanding**: What would happen if you tried `contact["notes"]` with bracket notation? How is `.get()` different?

## Try With AI: Dictionary Access Pattern Challenge

You've learned dict creation, bracket notation `dict['key']`, and safe access `dict.get('key', default)`. Now master when to use each access pattern—with AI as your safety validator.

### Part 1: Build User Profile (Your Turn First)

**Before asking AI**, create a user profile dictionary and access it TWO ways:

**Requirements**:
```python
# Create user profile with:
# - name: "Alice Smith"
# - email: "alice@example.com"
# - age: 28
# - premium: True

user: dict[str, str | int | bool] = ???

# Task 1: Get email using bracket notation
# Task 2: Get 'phone' (doesn't exist) using .get() with default "Not provided"
# Task 3: Try getting 'phone' with bracket notation - what happens?
```

**Your task**:
1. Create the dict with correct type hints
2. Predict: What happens with `user['phone']`?
3. Write down WHY you'd use `.get()` vs `[]`

---

### Part 2: AI Explains Access Patterns (Discovery)

Share your code with AI:

> "Here's my user profile dictionary: [paste code]. Explain:
> 1. Why does `user['phone']` raise KeyError?
> 2. When would I WANT a KeyError (bracket notation)?
> 3. When would I want a default (get method)?
> 4. Show me a real scenario where each pattern prevents bugs"

**Your task**: Evaluate AI's safety guidance.
- Does it explain KeyError is GOOD when data is required?
- Does it show `.get()` is GOOD for optional fields?
- Does it give examples (required: email, optional: phone)?
- Can you now choose the right access pattern?

---

### Part 3: Student Teaches AI (Type Hint Limitations)

AI explained access patterns. But does it understand type hint LIMITATIONS?

Challenge AI with runtime vs. static types:

> "I declared my dict as `dict[str, str | int | bool]` but then did:
> ```python
> user: dict[str, str | int | bool] = {'name': 'Alice', 'age': 28}
> user['scores'] = [95, 88, 92]  # Added a list! Type hint says str|int|bool!
> ```
>
> Questions:
> 1. Does Python stop me at runtime? (Try it!)
> 2. What does the type hint ACTUALLY do?
> 3. Would mypy catch this error?
> 4. Show me how to make the type hint MORE accurate for mixed data"

**Your task**: Understand type hint reality.
- Does AI explain that type hints are hints, not enforcement?
- Does it show that Python allows ANY value in dicts?
- Does it mention `Any` type or TypedDict for strict typing?
- Can you explain: "Type hints document intent, don't enforce it"?

---

### Part 4: Build Config System Together (Convergence)

Now apply dict patterns to a real system:

> "Build an app config system:
> 1. Required settings: `app_name` (str), `port` (int), `debug` (bool)
> 2. Optional settings: `log_file` (str, default='app.log'), `max_connections` (int, default=100)
> 3. Use bracket notation for required (crash if missing)
> 4. Use .get() for optional (provide defaults)
> 5. Type hint: `dict[str, str | int | bool]`
>
> Show code that:
> - Creates config dict
> - Accesses all settings safely
> - Explains WHY each access pattern"

**Your task**: Review the mixed access pattern.
- Does it use `config['app_name']` for required fields?
- Does it use `config.get('log_file', 'app.log')` for optional?
- Does it explain that required fields SHOULD crash if missing?
- Can you explain when defensive (.get) vs. strict ([]) makes sense?

Iterate if needed:
> "Add validation: If port exists but isn't an int, show error. Can you do this with type hints alone?"

---

**Time**: 25-30 minutes total
**Outcome**: You've mastered bracket notation for required data (fail-fast), .get() for optional data (defaults), understood type hint limitations, and built a production config pattern.

**Safety Note**: Dicts DON'T enforce type hints at runtime. Always validate critical data (especially user input) even with type hints. Type hints help editors/mypy, not Python itself.
