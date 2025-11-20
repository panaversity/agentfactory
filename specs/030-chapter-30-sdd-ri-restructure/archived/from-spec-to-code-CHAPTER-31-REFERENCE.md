---
title: "From Spec to Code: Turning Specifications into Working Implementations"
chapter: 30
lesson: 4
duration: "60-75 minutes"
skills:
  - name: "Specification-First Workflow"
    proficiency: "B1"
    category: "Technical"
  - name: "Code Generation and Validation"
    proficiency: "B1"
    category: "Technical"
  - name: "Specification-Code Alignment Testing"
    proficiency: "B1"
    category: "Technical"
  - name: "Debugging Specification Misalignments"
    proficiency: "B1"
    category: "Technical"
learning_objectives:
  - "Execute the specification-first workflow: spec.md → AI prompt → code generation → acceptance test validation (B1)"
  - "Validate that generated code matches specification intent by testing against predefined acceptance criteria (B1)"
  - "Diagnose and resolve specification-code misalignment by identifying whether the spec was unclear or the code is incorrect (B1)"
generated_by: "content-implementer v1.0.0 (Reasoning-Activated)"
source_spec: "specs/030-chapter-30-sdd-ri-restructure/lesson-plan.md"
source_whitepaper: "papers/SDD-RI Whitepaper.md (Sections 1.3, 5.3, 5.4, 6.3)"
created: "2025-01-18"
version: "1.0.0"
---

# From Spec to Code: Turning Specifications into Working Implementations

You've written your first specification collaboratively with AI (Lesson 3). Your calculator spec.md is complete—it contains user stories, acceptance criteria, edge cases, and error handling.

But now comes the most powerful moment: watching your specification transform into working code.

In this lesson, you'll experience **Specification Primacy**—the principle that in Spec-Driven Development, specifications are the source and code is the regenerable output. When your spec changes, you regenerate code. When you need a different implementation language, you regenerate from the same spec. The specification is permanent. The code is temporary.

This is a fundamental shift from traditional development, where code is primary and specifications are afterthoughts.

---

## The Paradigm Shift: Code as Regenerable Output

### A Brief Historical Perspective

Throughout computing history, abstraction levels have continuously risen:

| Era | What You Write | What Executes |
|-----|----------------|---------------|
| 1950s | Machine code (binary) | Machine code directly |
| 1960s | Assembly language | Machine code (assembled) |
| 1980s | High-level code (C, Python, JavaScript) | Machine code (compiled/interpreted) |
| **2020s** | **Specifications** | **Code (AI-generated) → Machine code** |

In each transition, what developers focus on becomes more abstract and intent-focused, while lower levels of implementation become automated.

**Key insight**: You no longer write assembly code manually—compilers do it. Similarly, you're increasingly not writing boilerplate by hand—AI does it. Your focus shifts upward: from "HOW do I implement this?" to "WHAT do I want to build?"

### Why This Matters for Specification-First Development

From the *SDD-RI Whitepaper* (Section 5.3):

> "Under SDD, implementation becomes an artifact derived from specifications. Code as Regenerable Output: implementation stays synchronized with intent, migrations become specification updates, and knowledge persists in structured, versioned specifications."

And crucially (Section 5.4):

> "In a SDD-RI methodology, the **durable asset is the specification**, not the generated code. Specs define the system's behavior and constraints. Code becomes a **rebuildable artifact**, while specifications become the primary locus of value."

**What this means for you**:
- Your spec.md is what you version control and maintain long-term
- Code is temporary (regenerated when specs evolve)
- Specifications apply across multiple languages (same spec → Python code, JavaScript code, test suites)
- Knowledge accumulates in specs, not in code archives

---

## The Specification-First Workflow

Working with specifications requires a new mental model. Here's the complete workflow you'll practice in this lesson:

### Step 1: Review Your Complete Specification

You already have one from Lesson 3: your calculator spec.md.

This specification should contain:
- **Intent**: What the calculator module does (four arithmetic operations)
- **User Stories**: Why users need this (express intent without prescribing implementation)
- **Acceptance Criteria**: Specific input/output examples (verifiable success conditions)
- **Constraints**: Type handling, error handling (division by zero), precision requirements
- **Non-Goals**: What's intentionally NOT included (no history, no parentheses, etc.)

Your specification is the contract between your intent and the implementation.

### Step 2: Generate Code from Specification

You'll provide your spec.md to your AI companion and ask for code that implements all requirements.

The prompt structure is simple:

```
Based on this specification:

[paste your complete calculator_spec.md]

Generate Python code that implements all requirements.
Include:
- Type hints for all function parameters and returns
- Docstrings explaining each function's behavior
- Error handling for all cases mentioned in Constraints
- All functions that match your user stories
```

Your AI companion will generate code that attempts to satisfy every requirement in your spec.

### Step 3: Validate Against Acceptance Criteria

**This is the critical step.** You don't just trust the generated code—you verify it matches your specification.

Run each acceptance criterion from your spec:

```python
# From your Acceptance Criteria section:
assert add(2, 3) == 5.0
assert subtract(10, 3) == 7.0
assert multiply(4, 5) == 20.0
assert divide(10, 2) == 5.0

# From your Constraints section:
try:
    divide(10, 0)
    assert False, "Should have raised error"
except ZeroDivisionError:
    pass  # Correct!
```

If all tests pass: The code matches your spec. Done.

If any test fails: You have a problem to diagnose.

### Step 4: Debug Specification-Code Misalignment

When code fails validation, **first ask: Is my spec clear?**

There are only two possibilities:

**A) Specification was unclear** (most common)
- The spec didn't explicitly state the requirement
- AI made a reasonable guess, but it wasn't what you meant
- **Solution**: Refine the specification, regenerate code

**B) Code is incorrect** (less common, especially with clear specs)
- The spec was clear, but AI made an error
- **Solution**: Show the error to AI, request regeneration

Example:

**Your spec says**: "Division by zero should be handled"
**Code does**: Returns `None` (AI guessed the error handling approach)
**Your test expects**: `ZeroDivisionError` exception

Diagnosis: Spec was unclear about HOW to handle division by zero.

Refined spec:
```
Constraint: Division by zero must raise ZeroDivisionError exception
(not return None, not return infinity, but raise an error that can be caught)
```

Regenerate code with refined spec → Code now raises the correct exception.

### The Feedback Loop

Real development is **iterative**:

```
Iteration 1: Spec v1 (vague) → Code v1 (makes assumptions) → Test (fails)
  → Diagnosis: Spec unclear

Iteration 2: Spec v2 (refined) → Code v2 (regenerated) → Test (passes mostly)
  → Diagnosis: Edge case missing in spec

Iteration 3: Spec v3 (edge case added) → Code v3 (regenerated) → Test (all pass)
  → Done!
```

Each iteration improves your specification. The code is disposable—it's regenerated each time. The specification is permanent—it gets better with each iteration.

**Key principle**: Don't waste time fixing code directly. Fix the specification, then regenerate.

---

## Practice 1: Generate Your Calculator Implementation

### Your Task (20 minutes)

You'll use your calculator spec from Lesson 3 to generate working Python code.

### Step 1: Locate Your Spec

Open your `calculator_spec.md` from Lesson 3.

It should look something like this:

```markdown
# Calculator Module Specification

## Intent
A Python module providing four basic arithmetic operations for programmatic use.

## User Stories
- As a developer, I want to add two numbers so I can compute sums
- As a developer, I want to divide two numbers so I can compute quotients
- ...

## Acceptance Criteria
- add(2, 3) returns 5.0
- subtract(10, 7) returns 3.0
- multiply(4, 5) returns 20.0
- divide(10, 2) returns 5.0
- divide(10, 0) raises ZeroDivisionError

## Constraints
- All function inputs must be numbers (int or float)
- Invalid inputs raise TypeError with clear message
- Division by zero raises ZeroDivisionError
- Results are returned as float
- Zero multiplied by anything is 0.0
- Negative numbers handled correctly

## Non-Goals
- No expression parsing (no "2+3*4")
- No history of operations
- No UI or interactive mode
```

### Step 2: Prompt Your AI Companion

Copy and paste this prompt into your AI tool (Claude, ChatGPT, etc.), replacing the bracket with your actual spec:

```
I'm practicing Specification-First Development with this calculator module specification:

[paste your complete calculator_spec.md here]

Please generate Python code that implements this specification completely.

The code should:
- Implement all operations in your user stories (add, subtract, multiply, divide)
- Satisfy every acceptance criterion listed
- Enforce every constraint
- Include type hints for all parameters and return values
- Include docstrings that explain each function
- Include proper error handling (TypeErrors, ZeroDivisionError)
- Be production-quality (clear, maintainable, well-structured)

Format the code as a complete, runnable Python module that I can copy and test immediately.
```

### Step 3: Review the Generated Code

Your AI companion will generate something like this:

```python
"""Calculator module providing basic arithmetic operations."""

def add(a: float, b: float) -> float:
    """Add two numbers.

    Args:
        a: First number (int or float)
        b: Second number (int or float)

    Returns:
        Sum as float

    Raises:
        TypeError: If either argument is not a number

    Examples:
        >>> add(2, 3)
        5.0
        >>> add(-1, 1)
        0.0
    """
    if not isinstance(a, (int, float)) or not isinstance(b, (int, float)):
        raise TypeError(f"Arguments must be numbers, got {type(a)} and {type(b)}")
    return float(a + b)


def subtract(a: float, b: float) -> float:
    """Subtract two numbers.

    Args:
        a: Minuend (int or float)
        b: Subtrahend (int or float)

    Returns:
        Difference as float

    Raises:
        TypeError: If either argument is not a number

    Examples:
        >>> subtract(10, 3)
        7.0
        >>> subtract(5, 10)
        -5.0
    """
    if not isinstance(a, (int, float)) or not isinstance(b, (int, float)):
        raise TypeError(f"Arguments must be numbers, got {type(a)} and {type(b)}")
    return float(a - b)


def multiply(a: float, b: float) -> float:
    """Multiply two numbers.

    Args:
        a: First number (int or float)
        b: Second number (int or float)

    Returns:
        Product as float

    Raises:
        TypeError: If either argument is not a number

    Examples:
        >>> multiply(4, 5)
        20.0
        >>> multiply(3, 0)
        0.0
    """
    if not isinstance(a, (int, float)) or not isinstance(b, (int, float)):
        raise TypeError(f"Arguments must be numbers, got {type(a)} and {type(b)}")
    return float(a * b)


def divide(a: float, b: float) -> float:
    """Divide two numbers.

    Args:
        a: Dividend (int or float)
        b: Divisor (int or float)

    Returns:
        Quotient as float

    Raises:
        TypeError: If either argument is not a number
        ZeroDivisionError: If divisor is zero

    Examples:
        >>> divide(10, 2)
        5.0
        >>> divide(7, 2)
        3.5
    """
    if not isinstance(a, (int, float)) or not isinstance(b, (int, float)):
        raise TypeError(f"Arguments must be numbers, got {type(a)} and {type(b)}")
    if b == 0:
        raise ZeroDivisionError("Cannot divide by zero")
    return float(a / b)
```

**Observations**:
- Every function has type hints
- Every function has a docstring
- Error handling matches your constraints
- Examples in docstrings demonstrate expected behavior

### Step 4: Validate Against Your Acceptance Criteria

Create a test file and run these validations:

```python
from calculator import add, subtract, multiply, divide

# Test each acceptance criterion from your spec
print("Testing add...")
assert add(2, 3) == 5.0, "add(2, 3) should be 5.0"
assert add(-1, 1) == 0.0, "add(-1, 1) should be 0.0"
print("✓ add tests pass")

print("Testing subtract...")
assert subtract(10, 3) == 7.0, "subtract(10, 3) should be 7.0"
assert subtract(3, 10) == -7.0, "subtract(3, 10) should be -7.0"
print("✓ subtract tests pass")

print("Testing multiply...")
assert multiply(4, 5) == 20.0, "multiply(4, 5) should be 20.0"
assert multiply(3, 0) == 0.0, "multiply(3, 0) should be 0.0"
print("✓ multiply tests pass")

print("Testing divide...")
assert divide(10, 2) == 5.0, "divide(10, 2) should be 5.0"
assert divide(7, 2) == 3.5, "divide(7, 2) should be 3.5"
print("✓ divide tests pass")

# Test constraints: error handling
print("Testing error handling...")
try:
    divide(10, 0)
    assert False, "divide(10, 0) should raise ZeroDivisionError"
except ZeroDivisionError:
    print("✓ ZeroDivisionError raised correctly")

try:
    add("5", 3)
    assert False, "add('5', 3) should raise TypeError"
except TypeError:
    print("✓ TypeError raised correctly for non-numeric input")

print("\n✅ All tests pass! Code matches your specification.")
```

If all tests pass, you've successfully:
1. Written a clear specification
2. Generated code from it
3. Validated that code matches intent

This is the specification-first workflow in action.

---

## Practice 2: Debugging Specification-Code Misalignment

### The Problem

Good specifications prevent bugs. Vague specifications cause bugs.

When code doesn't match what you wanted, 80% of the time it's because your specification was unclear. The AI generated exactly what you specified—just not what you *meant*.

In this practice, you'll experience this firsthand.

### A Vague Specification Example

Here's an intentionally incomplete specification:

```markdown
# Temperature Converter Module

## Intent
Convert temperatures between different units.

## User Stories
- As a scientist, I want to convert temperatures between units

## Acceptance Criteria
- convert_temperature(100, "C", "F") returns 212
- convert_temperature(32, "F", "C") returns 0

## Constraints
- Must validate inputs
- Must support common units

## Non-Goals
- No historical conversions
```

**What's missing?**
- Which units are supported? (Celsius? Fahrenheit? Kelvin? All three?)
- What counts as "validation"? (Just check types? Check value ranges?)
- What if invalid units are provided? (Return None? Raise error? Which error type?)
- What about edge cases? (Absolute zero violation for Kelvin?)
- What precision? (Decimal places? Rounding behavior?)

### Your Task (15 minutes)

1. **Copy the vague temperature spec above**
2. **Prompt your AI companion** to generate code
3. **Observe the assumptions** AI makes
4. **Identify the gaps** in your specification

Prompt to use:

```
Please generate Python code for this specification:

[paste vague temperature converter spec]

Make reasonable assumptions where the spec is unclear, and note those assumptions in comments.
```

### What Happens

Your AI companion will generate code like this:

```python
def convert_temperature(value: float, from_unit: str, to_unit: str) -> float:
    """Convert temperature between units."""
    # Assumption: Only supports Celsius (C) and Fahrenheit (F)
    # Assumption: Invalid units return None
    # Assumption: No validation of absolute zero

    if from_unit == "C" and to_unit == "F":
        return (value * 9/5) + 32
    elif from_unit == "F" and to_unit == "C":
        return (value - 32) * 5/9
    else:
        return None
```

**The Problems**:
- Only supports C/F (not Kelvin that you might have wanted)
- Returns `None` instead of raising an error (ambiguous)
- No validation of absolute zero
- No rounding behavior specified

This is exactly what the vague spec allowed. The AI made reasonable guesses—just not necessarily *your* guesses.

### Refining the Specification

Now write a **clear** version:

```markdown
# Temperature Converter Module (v2)

## Intent
Convert temperatures between Celsius, Fahrenheit, and Kelvin with rigorous validation.

## User Stories
- As a scientist, I want to convert Celsius to Fahrenheit
- As a scientist, I want to convert Fahrenheit to Celsius
- As a scientist, I want to convert between Kelvin and other units

## Acceptance Criteria
- convert_temperature(100, "C", "F") returns 212.0
- convert_temperature(32, "F", "C") returns 0.0
- convert_temperature(273.15, "K", "C") returns 0.0
- convert_temperature(-50, "K", "C") raises ValueError (absolute zero violation)
- convert_temperature(0, "X", "C") raises ValueError (invalid unit)

## Constraints
- Must support exactly: Celsius ("C"), Fahrenheit ("F"), Kelvin ("K")
- Must raise ValueError if from_unit or to_unit not in {C, F, K}
- Must raise ValueError if Kelvin value < 0 (absolute zero violation)
- Must return float rounded to 2 decimal places
- No unit conversions other than {C, F, K}

## Non-Goals
- No Rankine or other exotic units
- No temperature unit for non-temperature (e.g., color temperature)
```

Notice the differences:
- **Explicit units**: "Exactly: Celsius, Fahrenheit, Kelvin"
- **Explicit error handling**: "Must raise ValueError" (not return None)
- **Explicit edge cases**: "Kelvin < 0 = absolute zero violation"
- **Explicit rounding**: "2 decimal places"

### Regenerate Code with Refined Spec

Same prompt, but paste the refined spec:

```
Please generate Python code for this specification:

[paste refined temperature converter spec v2]

Implement all acceptance criteria exactly.
```

Your AI will now generate:

```python
def convert_temperature(value: float, from_unit: str, to_unit: str) -> float:
    """Convert temperature between Celsius, Fahrenheit, and Kelvin.

    Args:
        value: Temperature value
        from_unit: Source unit ("C", "F", or "K")
        to_unit: Target unit ("C", "F", or "K")

    Returns:
        Converted temperature rounded to 2 decimal places

    Raises:
        ValueError: If units invalid or Kelvin < 0
    """
    valid_units = {"C", "F", "K"}

    # Validate units
    if from_unit not in valid_units or to_unit not in valid_units:
        raise ValueError(f"Invalid unit. Must be one of: {valid_units}")

    # Validate absolute zero for Kelvin
    if from_unit == "K" and value < 0:
        raise ValueError("Kelvin cannot be negative (absolute zero violation)")

    # Convert to Celsius first (intermediate standard)
    if from_unit == "C":
        celsius = value
    elif from_unit == "F":
        celsius = (value - 32) * 5 / 9
    elif from_unit == "K":
        celsius = value - 273.15

    # Convert from Celsius to target unit
    if to_unit == "C":
        result = celsius
    elif to_unit == "F":
        result = (celsius * 9 / 5) + 32
    elif to_unit == "K":
        result = celsius + 273.15

    return round(result, 2)
```

**What changed?**
- ✅ All three units supported explicitly
- ✅ Clear ValueError for invalid units (not None)
- ✅ Absolute zero validation present
- ✅ Rounding to 2 decimal places

**The lesson**: When code fails to match your intent, ask "Was my spec clear?" before "Is the code wrong?" Clear specifications produce correct code. Vague specifications produce code that matches the vagueness.

---

## The Real-World Pattern: Specification Quality Determines Code Quality

This pattern repeats in production:

1. **Quick specs lead to wrong code** (spec doesn't express intent clearly)
2. **Good specs lead to correct code** (spec provides executable contract)
3. **Great specs enable multiple implementations** (same spec, different languages)

From the *SDD-RI Whitepaper* (Section 5.3):

> "Implementation stays synchronized with intent. When specifications evolve, code is regenerated. Knowledge persists in structured, versioned specifications."

The specification becomes your intellectual property. The code becomes a commodity.

---

## Key Principles from This Lesson

### 1. Specifications Are Primary, Code Is Regenerable

- **Durable asset**: Your spec.md (captured intent)
- **Temporary artifact**: Generated code (one implementation of intent)
- **When requirements change**: Update spec, regenerate code (not manually update code)

### 2. Validation Is Objective

You defined success criteria BEFORE code existed. Validation is:

✅ **Test against predefined acceptance criteria** (objective)
❌ **"Looks good to me"** (subjective, problematic)

### 3. Debugging Prioritizes Spec Clarity

When code fails tests:
1. **First question**: Is my spec clear and complete?
2. **Second question**: Does code match the (possibly unclear) spec?
3. **Resolution**: Usually fix spec, regenerate code

### 4. Clear Specs Enable Multiple Implementations

The same calculator spec could generate:
- Python implementation (what you did)
- JavaScript implementation (same spec, different language)
- Test suite (validate in isolation)
- Documentation (auto-generated from spec)

Different outputs, same specification source.

### 5. Iterative Refinement Is Normal

Spec v1 → Code v1 → Test → Refine spec → Code v2 → Test → etc.

Each iteration improves your specification. The code gets thrown away and regenerated. The spec gets better every time.

---

## Try With AI

Now that you understand the specification-first workflow, practice with these prompts in your AI tool:

### Prompt Set 1: Generate from Your Own Spec

**Your calculator spec (from Lesson 3)**:

```
I wrote this specification for a calculator module:

[paste your complete calculator_spec.md]

Please generate Python code that implements this specification completely.
Include type hints, docstrings, and error handling as specified.
Format as a complete, runnable module.
```

**Expected outcomes**: Working calculator code that passes all your acceptance criteria.

**Safety note**: Review generated code before running it. Verify it matches your specification intent.

### Prompt Set 2: Diagnose Vague Specifications

```
I wrote this specification, but the generated code doesn't behave as I expected:

[paste a vague specification]

What questions should I answer to make this specification complete?
What ambiguities might cause AI to generate code different from what I intended?
```

**Expected outcomes**: Clear list of specification gaps and refinement suggestions.

**Safety note**: Vague specs are the root cause of most implementation problems. Taking time to clarify specs saves debugging time later.

### Prompt Set 3: Validate Code Against Spec

```
Here's my specification:

[paste specification]

Here's the generated code:

[paste code]

Does the code satisfy all acceptance criteria in the spec?
Are there any specification-code misalignments?
What tests should I run to validate each acceptance criterion?
```

**Expected outcomes**: Point-by-point validation mapping code to spec requirements.

**Safety note**: This validation step is where you catch misalignments early, before code reaches production.

### Prompt Set 4: Iterative Refinement

```
My specification led to code that handles normal cases but fails on edge cases:

[paste specification]

Here's the test that failed:

[paste test that failed]

What edge cases am I missing in my specification?
Help me refine the spec to cover them.
```

**Expected outcomes**: Refined spec that captures edge cases you initially missed.

**Safety note**: This is the normal development pattern—iterate spec → test → refine spec.

---

## Moving Forward

You've now completed the core SDD workflow:
- **Lesson 1**: Why specs matter (vagueness costs time and money)
- **Lesson 2**: What specs look like (anatomy and structure)
- **Lesson 3**: Writing specs with AI (collaborative refinement)
- **Lesson 4** (this lesson): Converting specs to code (Specification Primacy)

The next lessons introduce more advanced patterns:
- **Lesson 5**: Evaluating spec quality (over/under specification tradeoffs)
- **Lesson 6**: Reusable Intelligence (when to encode recurring patterns)
- **Lesson 7**: Creating Skills and Subagents (Persona + Questions + Principles)

For now, practice the specification-first workflow until it becomes natural. Write specs that are clear enough that generated code needs minimal fixes. The discipline of clear specifications compounds—each spec you write gets better, and the code that emerges gets better with it.

Remember: In Specification-Driven Development, **you become a specification architect, not a code typist**.
