---
title: "Exception Fundamentals"
chapter: 21
lesson: 1
duration_minutes: 45

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Recognizing Exception Types"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Remember"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student identifies ValueError, TypeError, ZeroDivisionError by name from error messages and code context"

  - name: "Understanding Try/Except Flow"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student explains what code runs in try block vs except block and predicts behavior for different inputs"

  - name: "Reading Tracebacks"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student identifies error location, error type, and error message from Python traceback output"

  - name: "Writing Basic Exception Handlers"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student writes try/except block that catches specific exception types with appropriate recovery code"

learning_objectives:
  - objective: "Understand what exceptions are and why programs crash without error handling"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Explain the difference between exception occurring and exception being caught"

  - objective: "Write basic try/except blocks that prevent crashes"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Write try/except code catching at least 2 exception types with appropriate handlers"

  - objective: "Recognize common exception types by name (ValueError, TypeError, ZeroDivisionError)"
    proficiency_level: "A2"
    bloom_level: "Remember"
    assessment_method: "Match error messages to exception types and predict which exceptions could occur in given code"

  - objective: "Read Python tracebacks to find where errors occur"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Analyze traceback output and identify error type, location, and line number"

cognitive_load:
  new_concepts: 4
  assessment: "4 new concepts (exceptions as objects, try/except structure, common exception types, tracebacks) within A2 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Challenge: Write function that raises different exceptions for different invalid inputs; have peer predict which exception will occur"
  remedial_for_struggling: "Focus on ValueError from user input first (int(input())); practice this pattern before introducing other exception types"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/015-part-4-chapter-21/spec.md"
created: "2025-11-09"
last_modified: "2025-01-18"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "2.0.0"
---

# Exception Fundamentals

Every Python program you write will encounter situations that don't go as planned. A user might enter text when you expect a number. A calculation might divide by zero. A file you're trying to open might not exist. Without proper error handling, your program crashes and leaves the user confused.

In this lesson, you'll learn how to anticipate errors and handle them gracefully using Python's exception handling system. But here's the key: you'll start by intentionally breaking code to discover what exceptions look like, then work with AI to understand the exception hierarchy, challenge AI with edge cases, and finally build your own exception reference guide.

---

## Part 1: Intentionally Break Code to Discover Exception Types

**Your Role**: Active experimenter discovering exception patterns

Before you can handle errors, you need to see them. Professional developers don't memorize exception types—they learn to recognize patterns by experimenting.

### Discovery Exercise: What Breaks and How?

Run each code snippet and observe the error message. Your goal: identify the exception type and understand what triggered it.

**Experiment 1: String to Number Conversion**
```python
# What happens when you convert non-numeric text to integer?
def convert_to_integer(user_input: str) -> int:
    return int(user_input)

result = convert_to_integer("hello")
print(f"Number is: {result}")
```

**What you'll see**:
```
ValueError: invalid literal for int() with base 10: 'hello'
Traceback (most recent call last):
  File "example.py", line 5, in <module>
    result = convert_to_integer("hello")
  File "example.py", line 2, in convert_to_integer
    return int(user_input)
ValueError: invalid literal for int() with base 10: 'hello'
```

**Your task**: Read the traceback bottom-to-top:
- **Last line**: Exception type (ValueError) + message (what went wrong)
- **Middle lines**: Call chain (where in your code the error occurred)
- **Top line**: Header telling you "program stopped here"

**Experiment 2: Type Mismatch**
```python
# What happens when you add incompatible types?
def add_values(a: int, b: int) -> int:
    return a + b

result = add_values(5, "10")
print(f"Sum: {result}")
```

**What you'll see**: `TypeError: unsupported operand type(s) for +: 'int' and 'str'`

**Experiment 3: Division by Zero**
```python
# What happens when math goes wrong?
def divide_numbers(numerator: float, denominator: float) -> float:
    return numerator / denominator

result = divide_numbers(10, 0)
print(f"Result: {result}")
```

**What you'll see**: `ZeroDivisionError: division by zero`

### Recognition Pattern Building

After running all three experiments, you've discovered Python's three most common exception types:

| Exception Type | Trigger | What It Means |
|---------------|---------|---------------|
| **ValueError** | Correct type, wrong value | "This string isn't a valid number" |
| **TypeError** | Wrong type altogether | "You can't add int and str" |
| **ZeroDivisionError** | Math violation | "You can't divide by zero" |

**Deliverable**: Create a text file called `exception_discoveries.txt` documenting:
- Each experiment you ran
- The exception type produced
- Your hypothesis about what triggers each exception type
- 2 additional ways you could trigger the same exception

---

## Part 2: AI Explains Exception Hierarchy

**Your Role**: Student learning from AI Teacher

Now that you've seen exceptions in action, it's time to understand the bigger picture. Ask AI to teach you how Python organizes exceptions.

### AI Teaching Prompt

Ask your AI companion (Claude Code, Gemini CLI, or ChatGPT):

> "I've discovered three exception types: ValueError, TypeError, and ZeroDivisionError. Explain:
>
> 1. How does Python determine which exception type to raise?
> 2. What is the exception hierarchy (BaseException → Exception → specific types)?
> 3. Why does ValueError exist separately from TypeError?
> 4. Show me a simple diagram of how these three exceptions relate to each other in Python's exception hierarchy."

### What You'll Learn from AI

**Expected AI Response** (summary):
- **Exception Hierarchy**: All exceptions inherit from `BaseException`, most inherit from `Exception`
- **ValueError**: Raised when a function receives an argument of correct type but inappropriate value
- **TypeError**: Raised when an operation is applied to an object of inappropriate type
- **ZeroDivisionError**: Specific math error, inherits from `ArithmeticError`
- **Why separate types?**: Different exceptions communicate different problems, enabling targeted recovery strategies

### Convergence Activity

After AI explains, **test your understanding**:

Ask AI: "Show me code that could raise all three exception types in the same function. For each exception, explain what recovery strategy makes sense."

**Example of what AI might show**:
```python
def process_calculation(value_str: str, divisor_str: str) -> float:
    """
    Process calculation with multiple potential exception points.

    Could raise:
    - ValueError: if value_str or divisor_str aren't valid numbers
    - TypeError: if wrong types passed (unlikely with type hints, but possible)
    - ZeroDivisionError: if divisor is zero
    """
    try:
        value = int(value_str)        # ValueError if not numeric
        divisor = int(divisor_str)    # ValueError if not numeric
        result = value / divisor       # ZeroDivisionError if divisor is 0
        return result
    except ValueError as e:
        print(f"Conversion error: {e}")
        return 0.0
    except ZeroDivisionError:
        print("Cannot divide by zero")
        return 0.0
    except TypeError as e:
        print(f"Type error: {e}")
        return 0.0
```

**Your turn**: Review AI's code and explain back to AI:
- Why are there three separate except blocks?
- What happens if value_str = "abc"?
- What happens if divisor_str = "0"?
- What happens if you pass a list instead of a string?

**Deliverable**: Write a 1-paragraph summary explaining Python's exception hierarchy in your own words, referencing the diagram AI provided.

---

## Part 3: Student Challenges AI with Multiple Except Blocks

**Your Role**: Student teaching AI by identifying edge cases

Now reverse the roles. You'll design challenges that test whether AI understands exception handling nuances.

### Challenge Design Pattern

Create scenarios where AI must:
1. Predict which exception will occur
2. Handle multiple exception types in priority order
3. Deal with exception chaining

### Challenge 1: Predict the Exception

**Your prompt to AI**:
> "I have this function. What exception will occur for each test case? Explain your prediction BEFORE running the code.
>
> ```python
> def parse_age(age_str: str) -> int:
>     age = int(age_str)
>     if age < 0:
>         raise ValueError(f"Age cannot be negative, got {age}")
>     return age
> ```
>
> Test cases:
> 1. `parse_age("25")`
> 2. `parse_age("-5")`
> 3. `parse_age("abc")`
> 4. `parse_age("")`
>
> For each case, predict: Will it succeed? If it fails, which exception and why?"

**Expected AI Response**:
1. Success → returns 25
2. ValueError from `raise` statement (age < 0)
3. ValueError from `int()` call (invalid literal)
4. ValueError from `int()` call (empty string)

**Your follow-up**: "Now show me how to handle all these cases with appropriate except blocks. Which exception should be caught first and why?"

### Challenge 2: Exception Chaining

**Your prompt to AI**:
> "Explain exception chaining. Why would I use `raise ValueError(...) from e` instead of just `raise ValueError(...)`? Show me an example where exception chaining helps debugging."

**Expected learning**: AI will explain that `from e` preserves the original exception context, making debugging easier.

### Challenge 3: Catching vs Re-raising

**Your prompt to AI**:
> "Here's code with a try/except. Should this catch the exception and continue, or catch and re-raise? Explain your reasoning.
>
> ```python
> def read_configuration(filename: str) -> dict:
>     try:
>         with open(filename) as f:
>             return json.load(f)
>     except FileNotFoundError:
>         # What should happen here?
>         pass
> ```
>
> Give me three different strategies:
> 1. Catch and return default configuration
> 2. Catch and re-raise with better message
> 3. Don't catch, let caller handle
>
> When would you use each strategy?"

**Deliverable**: Document three challenging scenarios you posed to AI, AI's responses, and your analysis of whether AI's exception handling advice was sound.

---

## Part 4: Build Exception Reference Guide with Handling Strategies

**Your Role**: Knowledge synthesizer creating reusable reference

Now integrate everything you've learned into a practical reference guide you can use in future projects.

### Your Exception Reference Guide

Create a markdown file called `exception_reference_guide.md` with this structure:

```markdown
# Python Exception Handling Reference Guide
*Chapter 21, Lesson 1*

## Common Exception Types

### ValueError
**When it occurs**: Function receives correct type but inappropriate value
**Common triggers**:
- `int("abc")` — string isn't valid number
- `int("-5")` for age validation — value out of expected range

**Handling strategy**:
- Input validation: Prompt user to retry
- Data processing: Skip invalid record, log error
- Configuration: Use default value

**Code example**:
[Your example here]

---

### TypeError
**When it occurs**: Operation applied to inappropriate type
**Common triggers**:
- `5 + "10"` — can't add int and str
- `len(42)` — int has no length

**Handling strategy**:
- Input validation: Type checking before operation
- Function calls: Validate argument types
- API responses: Validate data structure

**Code example**:
[Your example here]

---

### ZeroDivisionError
**When it occurs**: Division or modulo by zero
**Common triggers**:
- `10 / 0`
- `x % 0`

**Handling strategy**:
- Check divisor before operation
- Return sentinel value (None, 0, or custom)
- Raise more specific exception with context

**Code example**:
[Your example here]

---

## Exception Handling Patterns

### Pattern 1: Specific Exception First
```python
try:
    risky_operation()
except ValueError:
    # Handle value errors
except TypeError:
    # Handle type errors
except Exception:
    # Catch-all (use sparingly)
```

### Pattern 2: Exception Chaining
```python
try:
    value = int(input_str)
except ValueError as e:
    raise ValueError(f"Invalid age: {input_str}") from e
```

### Pattern 3: Catch and Continue
```python
for item in data:
    try:
        process(item)
    except ValueError as e:
        log_error(e)
        continue  # Skip this item, process next
```

---

## Traceback Reading Guide

**Anatomy of a traceback** (read bottom-to-top):
```
Traceback (most recent call last):
  File "example.py", line 5, in <module>      ← Where you called the function
    result = convert_to_integer("hello")
  File "example.py", line 2, in convert_to_integer  ← Where error occurred
    return int(user_input)
ValueError: invalid literal for int() with base 10: 'hello'  ← Exception type + message
```

**Reading steps**:
1. Read exception type (last line)
2. Read exception message (last line)
3. Find where error occurred (second-to-last File line)
4. Trace call chain (work upward)

---

## Decision Tree: Which Exception to Catch?

**Question 1**: Is the value the wrong type?
- Yes → Expect `TypeError`
- No → Go to Question 2

**Question 2**: Is the value the right type but invalid?
- Yes → Expect `ValueError`
- No → Go to Question 3

**Question 3**: Is it a math operation error?
- Yes → Expect `ZeroDivisionError` or `ArithmeticError`
- No → Check Python docs for specific exception

---

## Testing Exception Handling

**Pattern**: Always test both success and failure paths
```python
# Test success
assert parse_age("25") == 25

# Test ValueError for negative
try:
    parse_age("-5")
    assert False, "Should have raised ValueError"
except ValueError:
    pass  # Expected

# Test ValueError for invalid string
try:
    parse_age("abc")
    assert False, "Should have raised ValueError"
except ValueError:
    pass  # Expected
```
```

### Guide Requirements

Your reference guide must include:
1. **Three exception types** with triggers, handling strategies, and code examples
2. **Three exception handling patterns** with realistic use cases
3. **Traceback reading guide** with step-by-step instructions
4. **Decision tree** for choosing which exception to catch
5. **Testing patterns** for verifying exception handling works

### Validation with AI

Once your guide is complete, validate it by asking AI:

> "Review my exception reference guide. Are my handling strategies appropriate? What common exceptions am I missing? What patterns should I add for professional code?"

**Deliverable**: Complete `exception_reference_guide.md` file that serves as your go-to resource for exception handling in future projects.

---

## Summary: Bidirectional Learning Pattern

In this lesson, you experienced all three roles:

**Part 1 (Student explores)**: You intentionally broke code to discover exception types
**Part 2 (AI teaches)**: AI explained the exception hierarchy and organization
**Part 3 (Student teaches)**: You challenged AI with edge cases and exception chaining
**Part 4 (Knowledge synthesis)**: You built a reusable reference guide

This pattern ensures you understand exceptions not just as syntax, but as a conceptual framework for robust error handling.

### What You've Built

1. `exception_discoveries.txt` — Your experimental findings
2. Exception hierarchy summary — Your understanding of Python's exception organization
3. Challenge documentation — Three edge cases you posed to AI
4. `exception_reference_guide.md` — Your comprehensive exception handling reference

### Next Steps

Lesson 2 will build on this foundation, exploring `except`, `else`, and `finally` blocks for more sophisticated error handling. You'll use this reference guide throughout Chapter 21.
