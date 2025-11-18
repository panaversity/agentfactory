---
title: "Introduction to Generics and Type Variables"
chapter: 27
lesson: 3
duration_minutes: 35-40

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Generic Function Creation"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can write a generic function that works with list[int], list[str], and list[User] while preserving type information"

  - name: "TypeVar Usage"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can create TypeVar and use it in function signatures to enable type parameterization"

  - name: "PEP 695 Syntax"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can write 'def func[T](x: T) -> T:' syntax and understand its advantage over legacy Generic imports"

  - name: "Type Safety Understanding"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can explain how Generics help IDEs catch bugs before running code and maintain type information through function calls"

learning_objectives:
  - objective: "Explain what Generics are and why they enable type-safe reusable code"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Student articulates the difference between generic and non-generic approaches with 2-3 examples"

  - objective: "Create generic functions using TypeVar and modern PEP 695 syntax"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student writes a working generic function that operates on multiple types"

  - objective: "Apply generic functions to multiple data types while preserving type information"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student demonstrates type preservation in IDE and creates usage examples"

  - objective: "Understand how Generics improve IDE autocomplete and error detection"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Student recognizes type benefits in IDE and contrasts with Any type usage"

cognitive_load:
  new_concepts: 10
  assessment: "10 new concepts (Generics definition, TypeVar, generic functions, PEP 695 syntax, type preservation, IDE benefits, type inference, legacy vs modern, type safety, reusability) at B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Research TypeVar bounds and constraints; create generic functions with upper bounds like T: Comparable"
  remedial_for_struggling: "Focus on single-type scenarios first (get_first_item[T]) before multiple-type examples; work through PEP 695 syntax step-by-step with interactive IDE"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/001-part-4-chapter-27/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Introduction to Generics and Type Variables

Imagine you've written a function that gets the first item from a list of integers. It works perfectly:

```python
def get_first_int(items: list[int]) -> int | None:
    return items[0] if items else None
```

Now you need the same function for strings. So you write another one:

```python
def get_first_str(items: list[str]) -> str | None:
    return items[0] if items else None
```

And another for custom User objects. And another for Book objects. You're copying the same logic over and over, just changing the type names. This is the duplication problem that **Generics** solve.

Generics let you write ONE function that works with ANY type while keeping full type safety. Instead of three separate functions, you get this:

```python
def get_first_item[T](items: list[T]) -> T | None:
    return items[0] if items else None
```

One function. Works with integers, strings, User objects, anything. And your IDE knows the types—it autocompletes correctly and catches type errors before you run the code.

This is the power of Generics: **reusable type-safe code**. In this lesson, you'll learn to write generic functions that scale to any data type while preserving the type information that makes your IDE smarter.

## What Are Generics? Type-Safe Code That Works with Any Type

Before we dive into syntax, let's understand the problem Generics solve. In Python, you have two options for flexible functions:

**Option 1: No Type Hints**
```python
def get_first_item(items: list) -> None:
    return items[0] if items else None
```

This works but loses all type information. Your IDE has no idea what type the item is. Did you get an int? A string? A User? The IDE can't help you—you're flying blind.

**Option 2: Use `Any` Type**
```python
from typing import Any

def get_first_item(items: list[Any]) -> Any:
    return items[0] if items else None
```

This says "this could be anything," which technically works but throws away type information. Your IDE won't autocomplete. If you try to call `.upper()` on the result, the IDE won't know if it's safe.

**Option 3: Generics (The Right Way)**
```python
def get_first_item[T](items: list[T]) -> T | None:
    return items[0] if items else None
```

Now the function says: "I accept any type T, and I return the same type T." When you call `get_first_item([1, 2, 3])`, the IDE knows T is `int`, so it knows the return type is `int | None`. When you call `get_first_item(["a", "b"])`, the IDE knows T is `str`, so it knows the return type is `str | None`.

**Why Python needs Generics**: Python is dynamically typed at runtime, but professionally-written Python uses static type checking with tools like mypy and Pylance (in your IDE). Generics bridge this gap: they preserve type information for tools while allowing flexible, reusable code.

#### 💬 AI Colearning Prompt
> "What's the difference between using Generics and using `Any` type in Python? Show examples of why Generics provide better type safety and IDE support."

---

## Section 1: Your First Generic Function

Let's build your first generic function and see how it preserves type information across different data types.

### Creating a Generic Function

Here's the canonical example—`get_first_item`—that works with any type:

```python
def get_first_item[T](items: list[T]) -> T | None:
    """Returns the first item from a list or None if empty.

    Type parameter T ensures the return type matches the input type.
    """
    return items[0] if items else None


# Works with integers
numbers: list[int] = [1, 2, 3]
first_num = get_first_item(numbers)  # Type: int | None

# Works with strings
names: list[str] = ["Alice", "Bob"]
first_name = get_first_item(names)  # Type: str | None

# Works with custom objects
class Book:
    def __init__(self, title: str):
        self.title = title

books: list[Book] = [Book("Python Guide"), Book("Web Dev")]
first_book = get_first_item(books)  # Type: Book | None
```

**What's happening here?**

- `T` is a **type variable**—a placeholder for any type
- When you call `get_first_item(numbers)`, Python and your IDE infer that `T = int`
- When you call `get_first_item(names)`, Python and your IDE infer that `T = str`
- The function works identically in all cases, but the type information flows through

### How IDEs Use This Type Information

Your IDE uses Generics to provide autocomplete and error detection:

```python
# IDE knows first_num is int | None
if first_num is not None:
    # IDE offers methods for int: bit_length(), to_bytes(), etc.
    print(first_num + 10)  # ✅ IDE allows this (int + int is valid)
    print(first_num.upper())  # ❌ IDE flags this error (int has no .upper())

# IDE knows first_name is str | None
if first_name is not None:
    # IDE offers methods for str: upper(), lower(), split(), etc.
    print(first_name.upper())  # ✅ IDE allows this (str has .upper())
    print(first_name + 10)  # ❌ IDE flags this error (str + int invalid)
```

The IDE catches bugs BEFORE you run the code. This is the real value of Generics—not just for runtime, but for your development experience.

#### 🎓 Expert Insight
> In AI-native development, Generics aren't about memorizing syntax—they're about preserving intent. When you write `Stack[User]`, you're telling both humans and AI agents: "This container holds Users." That clarity cascades through your codebase, making AI code generation more accurate and refactoring safer.

---

## Section 2: Modern PEP 695 Syntax

Python has two ways to write generics. One is old and verbose. One is modern and clean. You're already seeing the modern way—let's understand why.

### The Legacy Approach (Not Recommended)

Before Python 3.14, you had to import TypeVar from the typing module:

```python
from typing import TypeVar

T = TypeVar('T')  # Create a type variable

def get_first_item(items: list[T]) -> T | None:
    return items[0] if items else None
```

This works, but it's awkward. You define `T` separately from the function. You have to remember to import TypeVar. It's extra boilerplate.

### The Modern Approach (PEP 695 - Python 3.14+)

Python 3.14 introduced cleaner syntax:

```python
def get_first_item[T](items: list[T]) -> T | None:
    """Returns the first item from a list or None if empty."""
    return items[0] if items else None
```

The `[T]` goes directly in the function definition. No imports, no separate TypeVar definition. It's cleaner, more readable, and the direction Python is evolving.

**Key difference:**
- **Legacy**: TypeVar lives outside the function; function uses it
- **Modern**: Type parameters declared right in the function signature

**For this book, always use PEP 695 syntax.** It's simpler, more readable, and future-proof.


---

## Section 3: Type Inference in Action

One of the elegant features of Generics is that Python infers the type automatically. You don't have to tell Python what T is—it figures it out from how you call the function.

### Python Infers T from Context

```python
# Call 1: Python sees list[int] and infers T = int
result1 = get_first_item([1, 2, 3])  # T is inferred as int

# Call 2: Python sees list[str] and infers T = str
result2 = get_first_item(["hello", "world"])  # T is inferred as str

# Call 3: Python sees list[Book] and infers T = Book
result3 = get_first_item([Book("A"), Book("B")])  # T is inferred as Book
```

You never explicitly tell Python what T is. It infers it from the argument you pass. This is why the function "just works" with any type.

### Explicit Type Parameters (Rare)

In very rare cases, you might need to explicitly specify the type parameter:

```python
# Explicit specification (rarely needed)
result = get_first_item[int]([1, 2, 3])
```

This says "I'm explicitly telling you that T is int." In practice, inference handles 99% of cases. You'll almost never use explicit type parameters.

### Type Checkers Use Generics to Catch Errors

Your IDE and type checkers like mypy and Pylance use Generics to validate your code before running it:

```python
numbers: list[int] = [1, 2, 3]
first = get_first_item(numbers)  # Type checker knows this is int | None

# This is safe (int supports +)
if first is not None:
    value = first + 10  # ✅ Type checker allows this

# This is an error (int doesn't support .upper())
if first is not None:
    text = first.upper()  # ❌ Type checker flags this as error
```

The type checker runs without executing code. It catches the `.upper()` error immediately in your IDE.

#### 🤝 Practice Exercise

> **Ask your AI**: "Create a generic function `get_last_item[T]` that returns the last element from a list or None if empty. Show usage examples with both integers and strings, and explain how type information flows through the function."

**Expected Outcome**: You'll understand how type parameters flow through function signatures and return types, seeing the same generic pattern work for different container operations while maintaining full type safety.

---

## Section 4: Generics vs Dynamic Typing

Here's an important distinction that clarifies what Generics actually do:

**Generics are for tools (IDEs, type checkers), not for Python runtime.**

Python is still dynamically typed when your code runs. Generics don't enforce type checking at runtime—they provide type information for static analysis tools.

### Generics Don't Enforce Runtime Type Checking

```python
def get_first_item[T](items: list[T]) -> T | None:
    return items[0] if items else None

# At runtime, this works (Python is dynamically typed)
weird_list: list[int] = [1, 2, 3]  # Type says this is list[int]
first = get_first_item(weird_list)  # Works fine at runtime

# Python doesn't actually check that items is list[int]
# If you pass list[str], Python runs it anyway (runtime is dynamic)
```

**This is important**: Generics are NOT runtime enforcement. They're hints for tools.

### The Real Benefit: Catch Errors Before Running Code

The value of Generics is in your development workflow:

```python
numbers: list[int] = [1, 2, 3]
first = get_first_item(numbers)

# IDE knows first is int | None
if first is not None:
    text = first.upper()  # ❌ IDE shows red squiggly (int has no .upper())
```

You see the error in your editor BEFORE running the code. You fix it BEFORE it becomes a production bug. This is the real power—catch mistakes during development, not after deployment.


---

## Common Mistakes

### Mistake 1: Using `Any` When Generics Are Better

```python
# ❌ Don't do this (throws away type information)
from typing import Any
def get_first_item(items: list[Any]) -> Any:
    return items[0] if items else None

# ✅ Do this (preserves type information)
def get_first_item[T](items: list[T]) -> T | None:
    return items[0] if items else None
```

`Any` is a cop-out. It says "I don't care about type safety." Generics say "I maintain type safety for any type."

### Mistake 2: Over-Constraining with Unnecessary Type Bounds

You might think you need to constrain what types are allowed:

```python
# ❌ Overly restrictive (unnecessary bounds)
from typing import Sequence
def get_first[T: Sequence](items: list[T]) -> T | None:  # Why constrain T?
    return items[0] if items else None

# ✅ Simple and flexible
def get_first_item[T](items: list[T]) -> T | None:
    return items[0] if items else None
```

If your function doesn't require special methods on T, don't constrain it. Let it work with anything.

### Mistake 3: Thinking Generics Enforce Runtime Type Checking

```python
def get_first_item[T](items: list[T]) -> T | None:
    return items[0] if items else None

# ❌ Mistaken expectation
mixed_list = [1, "two", 3.0]  # Not actually a valid type at runtime
first = get_first_item(mixed_list)  # Python doesn't enforce T consistency at runtime
```

Generics don't enforce types at runtime. Python is still dynamically typed. Generics help your IDE, not the runtime.

### Mistake 4: Mixing Legacy and Modern Syntax

```python
# ❌ Don't mix them (confusing)
from typing import TypeVar
T = TypeVar('T')
def get_first[T](items: list[T]) -> T | None:  # This mixes styles
    return items[0] if items else None

# ✅ Use modern syntax exclusively
def get_first_item[T](items: list[T]) -> T | None:
    return items[0] if items else None
```

Pick one style and stick with it. Modern PEP 695 is cleaner.

---

## Try With AI: The Type Safety Discovery (4-Part Learning Challenge)

This challenge teaches you why Generics matter and how they preserve type information through your code.

---

### Part 1: You Discover (Student Discovers Problems)

**Your Turn** — Build utility functions WITHOUT Generics to experience loss of type information:

```python
# Part 1: Non-generic utilities (the fragile way)
# Build three separate functions that do the same thing:

def get_first_int(items: list[int]) -> int | None:
    """Get first integer from list."""
    return items[0] if items else None

def get_first_str(items: list[str]) -> str | None:
    """Get first string from list."""
    return items[0] if items else None

def get_first_any(items: list) -> any:
    """Get first item—type is lost."""
    return items[0] if items else None

# Problem to discover:
# - You're duplicating the same logic three times
# - get_first_any() loses type information (IDE can't help)
# - When you call get_first_int([1,2,3]), IDE knows return is int | None
# - But what if you have a User class? You'd need get_first_user()
# - This approach doesn't scale—you'd need a function for every type!

class User:
    def __init__(self, name: str):
        self.name = name

# Do you really need get_first_user()? Or can one function work for all?
```

**What You'll Realize**: Without Generics, you either duplicate code (one function per type) or lose type information (using `Any`). Neither is acceptable for professional code.

---

### Part 2: AI as Teacher (AI Explains Concepts)

**Ask your AI:**

> "I have this problem: I'm writing utility functions like 'get first item' that work with many types (int, str, User objects). I don't want to write separate functions for each type.
>
> Show me how Generics solve this. Explain:
> 1. Why Generics are better than writing one function per type
> 2. Why Generics are better than using `Any` type
> 3. How type information 'flows' through a generic function
> 4. Show me a generic `get_first[T]` function and how type information is preserved when calling it with int, str, and User"

**What AI Should Show You**:
- How PEP 695 syntax `def func[T](items: list[T]) -> T:` works
- How the IDE infers T from the argument type
- How type information flows through the return type
- Concrete examples: get_first([1,2,3]) → int | None, get_first(["a","b"]) → str | None
- Why this is better than Any: IDE provides autocomplete and catches errors

**Your Role**: Ask clarifying questions. "So the IDE knows the return type without me telling it?" "What happens if I pass a mixed-type list?" Push for understanding, not just examples.

---

### Part 3: You as Teacher (You Challenge AI's Approach)

**Challenge AI** — Ask it to handle edge cases and variations:

> "Your generic function works, but I have real-world needs:
> 1. What if I have a `find_all[T]` that returns ALL matching items instead of just first?
> 2. What if I need a generic function that works with BOTH lists AND tuples?
> 3. What if I have a custom User class—does the generic function work with User objects?
> 4. When I call find_all(["a","b","c"]), does the IDE know I'm getting back list[str] | None?
>
> Show me: a) Working code for each case, b) Type preservation in each example, c) How to demonstrate that IDE autocomplete works correctly"

**Your Role**: Push back on limitations. "Does this work with my custom User class without changing the code?" or "Can I pass a tuple instead of list?" This forces AI to clarify the flexibility of Generics.

**AI's Response Should Show**:
- How Generics work with different collection types
- How type information is preserved across function calls
- How to demonstrate IDE type inference actually works
- Patterns for more sophisticated generic functions

---

### Part 4: You Build (Production Artifact)

**Build a Type-Safe Utilities Library** — Synthesize into reusable components:

```python
# deliverable: generic_utils.py

from typing import Sequence, Callable

def get_first_item[T](items: Sequence[T]) -> T | None:
    """Get first item from any sequence (list, tuple, etc.).

    Type parameter T ensures type information flows through.
    Works with any type—int, str, User, etc.
    """
    return items[0] if items else None

def find_all[T](items: Sequence[T], predicate: Callable[[T], bool]) -> list[T]:
    """Find all items matching condition.

    Type parameter T is preserved in the return list[T].
    """
    return [item for item in items if predicate(item)]

def get_last_item[T](items: Sequence[T]) -> T | None:
    """Get last item from any sequence.

    Demonstrates that Generics work for different operations.
    """
    return items[-1] if items else None

class User:
    def __init__(self, name: str, age: int):
        self.name = name
        self.age = age

# Test implementation—IDE autocomplete works perfectly
if __name__ == "__main__":
    # Works with integers
    numbers = [1, 2, 3, 4, 5]
    first_num = get_first_item(numbers)  # IDE knows this is int | None
    if first_num is not None:
        print(f"First number: {first_num + 10}")  # IDE allows int operations

    # Works with strings
    words = ["hello", "world"]
    first_word = get_first_item(words)  # IDE knows this is str | None
    if first_word is not None:
        print(f"First word uppercase: {first_word.upper()}")  # IDE allows str operations

    # Works with custom objects
    users = [
        User("Alice", 30),
        User("Bob", 25),
        User("Charlie", 35)
    ]
    first_user = get_first_item(users)  # IDE knows this is User | None
    if first_user is not None:
        print(f"First user: {first_user.name}, age {first_user.age}")  # IDE autocompletes name/age

    # find_all preserves type
    adults = find_all(users, lambda u: u.age >= 30)  # Returns list[User]
    print(f"Found {len(adults)} adults")

    # Works with tuples too (any Sequence)
    tuple_data = ("a", "b", "c")
    first_from_tuple = get_first_item(tuple_data)  # IDE knows this is str | None
```

**Success Criteria**:
- Single generic function works with multiple types (int, str, custom objects)
- IDE provides correct autocomplete for each type
- Type information flows from input to output
- No code duplication—one function definition works for all types
- Works with different collection types (list, tuple, etc.)

---

## Time Estimate
**30-35 minutes** (6 min discover, 8 min AI teaches, 8 min you challenge, 8 min build)
