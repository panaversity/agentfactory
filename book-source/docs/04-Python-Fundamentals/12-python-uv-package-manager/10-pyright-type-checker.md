---
title: "Pyright Type Checker"
chapter: 12
lesson: 10
duration_minutes: 20

skills:
  - name: "Install and Use Pyright"
    proficiency_level: "A1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student installs Pyright and runs type checking successfully"

  - name: "Write and Understand Type Hints"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student adds type hints to functions and identifies type errors"

  - name: "Configure Type Checking Modes"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student configures Pyright mode and understands tradeoffs"

learning_objectives:
  - objective: "Understand why type hints exist: catch bugs before running code"
    proficiency_level: "A1"
    bloom_level: "Remember"
    assessment_method: "Student explains: types catch ~30-50% of bugs statically (no runtime needed)"

  - objective: "Write basic type hints in Python 3.13+ syntax"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student adds return type and parameter types to a function"

  - objective: "Run Pyright to catch type mismatches"
    proficiency_level: "A1"
    bloom_level: "Apply"
    assessment_method: "Student runs `uv run pyright` and identifies type error messages"

cognitive_load:
  new_concepts: 7
  assessment: "7 concepts: type hints syntax, return type annotations, parameter types, type errors vs runtime errors, checking modes (basic/standard/strict), static analysis benefit, LSP integration. Within B1 limit. ✓"

differentiation:
  extension_for_advanced: "Explore strict mode and advanced types (Union, Optional, Callable)"
  remedial_for_struggling: "Focus on basic: type hints are optional; function parameters and return types are the essential pieces"

generated_by: "content-implementer v3.0.0"
source_spec: "specs/001-chapter-12-lightning-python-stack/plan.md"
created: "2025-01-15"
last_modified: "2025-01-15"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Pyright Type Checker

## Why Type Checking? (Tier 1: Motivation)

Python is **dynamically typed**, meaning you can write:

```python
x = "hello"
x = 42
x = [1, 2, 3]
```

Same variable, three different types. Python doesn't complain—it just changes.

**But this causes bugs:**
```python
def greet(name):
    return f"Hello, {name}"

result = greet(42)  # Oops! Passing a number instead of a string
```

Python runs this fine until you expect `result` to be a string:
```python
upper = result.upper()  # Crash: int has no method .upper()
```

**Problem**: The bug is in `greet(42)` call, but the crash is at `.upper()`. You waste time debugging the wrong place.

**Solution**: **Type hints** tell Python what types to expect:
```python
def greet(name: str) -> str:
    return f"Hello, {name}"

result = greet(42)  # Type checker: ERROR! 42 is not a str
```

Now the error is **caught immediately**, before running code.

**Benefit**: Catch 30-50% of bugs statically (without running code). That's huge.

**Source**: Verified in intelligence/001-verified-tool-documentation.md

---

## What Are Type Hints? (Tier 1: Learn Syntax)

Type hints are **annotations** that tell Python what types variables/functions expect.

### **Function Parameter Types**

```python
def add(x: int, y: int) -> int:
    """Add two integers and return the result."""
    return x + y
```

Read as:
- `x: int` — Parameter `x` should be an integer
- `y: int` — Parameter `y` should be an integer
- `-> int` — Function returns an integer

### **Python 3.13+ Modern Syntax**

Python 3.13 introduced **new syntax** for optional types using `|` instead of `Union`:

```python
def process(data: str | None) -> int:
    """Accept string or None, return integer."""
    if data is None:
        return 0
    return len(data)
```

Read as:
- `data: str | None` — Accept string or None (called "union type")
- `-> int` — Returns an integer

**Older syntax** (Python 3.9-3.12) used `Union[str, None]`. We use 3.13+ syntax because it's cleaner.

**Note**: Type hints are **optional syntax**. Python still runs without them. They're for tools like Pyright to catch errors.

---

## Installing Pyright (Tier 1: Direct Command)

Just like Ruff, install via `uv`:

```bash
uv add pyright --dev
```

**Verify installation:**
```bash
uv run pyright --version
```

**Expected output:** `pyright 1.1.407` (or similar version)

---

## Running Pyright (Tier 1: Direct Command)

Let's catch a type error!

**Create a file `types.py`:**
```python
def greet(name: str) -> str:
    return f"Hello, {name}"

result = greet(42)  # ERROR: 42 is not a str
```

**Run Pyright:**
```bash
uv run pyright types.py
```

**Expected output:**
```
/path/to/types.py:4:21 - error: Argument of type "Literal[42]" cannot be assigned to parameter "name" of type "str" in function "greet"
  "Literal[42]" is not assignable to "str"
1 error, 0 warnings
```

**Read the error:**
- Line 4, column 21: The error location
- `"Literal[42]"` cannot be assigned to `"str"`: You passed an integer where a string was expected
- `1 error`: Pyright found one type problem

**Fix the code:**
```python
result = greet("Alice")  # Correct: pass a string
```

**Run Pyright again:**
```bash
uv run pyright types.py
```

**Expected output:**
```
0 errors, 0 warnings, 0 informations
```

Success! No type errors.

**Note**: This code demonstrates type checking, not Python programming. You'll learn more about functions and types in Chapter 13.

**Source**: Verified in intelligence/001-verified-tool-documentation.md

---

## Type Checking Modes (Tier 1: Understand Options)

Pyright has **four modes** controlling how strict type checking is:

| Mode | Strictness | Use Case |
|------|-----------|----------|
| **off** | No checking | Legacy code (not recommended) |
| **basic** | Standard checking | Most projects (default) |
| **standard** | Stricter checking | Type-aware projects |
| **strict** | Maximum safety | Type-first projects |

**Basic mode** (default) is usually right. **Strict mode** catches more potential issues but requires more type hints.

**Configure in `pyproject.toml`:**
```toml
[tool.pyright]
typeCheckingMode = "standard"  # Change to "strict" for more checking
pythonVersion = "3.13"
```

---

## Common Type Errors (Tier 1: Learn Patterns)

Here are errors Pyright catches:

### **Error 1: Type Mismatch**
```python
def add(x: int, y: int) -> int:
    return x + y

result = add("5", "10")  # ERROR: strings are not ints
```

### **Error 2: Return Type Wrong**
```python
def get_count() -> int:
    return "five"  # ERROR: returns str, not int
```

### **Error 3: Optional Handling**
```python
def process(data: str | None) -> int:
    return len(data)  # ERROR: data might be None; len(None) crashes
```

Fix by checking for None:
```python
def process(data: str | None) -> int:
    return len(data) if data else 0
```

---

## Tier 2: Configure Pyright in pyproject.toml

Tell your AI:

```
I want to configure Pyright for my Python project.
I want:
- Standard type checking (not too strict)
- Python 3.13
- Report missing imports as errors
- Report unknown variable types as warnings

Generate the [tool.pyright] section for pyproject.toml.
```

AI generates:
```toml
[tool.pyright]
typeCheckingMode = "standard"
pythonVersion = "3.13"
reportMissingImports = "error"
reportUnknownVariableType = "warning"
```

Copy-paste into your `pyproject.toml` and run:
```bash
uv run pyright
```

Pyright now uses your configuration.

**Source**: Verified in intelligence/001-verified-tool-documentation.md

---

## Pyright + Zed Integration (Tier 2: Editor Display)

You'll configure Zed to show **Pyright type errors inline** (next lesson, Lesson 11).

For now, just know: Pyright can run in your editor, showing errors as you type—no need to open terminal.

---

## Type Hints vs. Tests (Tier 1: Understand Relationship)

**Type hints and tests are complementary:**

| Tool | Catches | Doesn't Catch |
|------|---------|---------------|
| **Pyright** (type checking) | Type mismatches | Logic errors, runtime failures |
| **Tests** | Logic errors, edge cases | Type mismatches (without hints) |
| **Together** | Most bugs | Edge cases you didn't think of |

**Good practice**: Use both.

---

## Try With AI: Type-Safe Project Capstone

This capstone integrates ALL Chapter 12 concepts: UV project setup, dependency management, Ruff formatting, and Pyright type checking.

### Part 1: Design a Type-Safe Calculator (Your Turn First)

**Before asking AI**, plan a typed calculator module:

**Your design task**:
1. Create a UV project called `calc-app`
2. Design functions: `add()`, `subtract()`, `multiply()`, `divide()`
3. For EACH function, write type hints (parameters and return type)
4. Predict: What type error would occur if someone passes a string to `add()`?

**Write pseudocode with type hints** (on paper/notes):
```python
def add(x: ???, y: ???) -> ???:
    # What types should x, y, and return be?
```

---

### Part 2: AI Guides Project Setup (Discovery)

Now build the project with AI assistance:

> "Let's build a type-safe calculator project. I need:
>
> **Project Setup**:
> 1. Create UV project `calc-app`
> 2. Add Pyright and Ruff as dev dependencies
> 3. Configure Pyright for standard type checking (Python 3.13)
>
> **Calculator Module** (`calc.py`):
> 1. Functions: `add`, `subtract`, `multiply`, `divide`
> 2. All functions take two numbers, return a number
> 3. Use Python 3.13+ type hint syntax
> 4. For `divide`: Handle division by zero (return `None` if divisor is zero)
>
> Show me the complete setup commands and the `calc.py` code with type hints."

**Your evaluation task**:
- Does AI's `divide()` function use `int | None` or `float | None` as return type?
- Run `uv run pyright calc.py`. Do you get any type errors? Why or why not?
- Compare your pseudocode to AI's implementation. What did you miss?

---

### Part 3: Student Teaches AI (Break Type Safety)

Challenge AI with intentional type errors:

> "Let's intentionally break type safety to understand Pyright:
>
> **Create `test_errors.py`**:
> 1. Import `add` from `calc.py`
> 2. Call `add("5", "10")` (strings, not numbers)
> 3. Call `add(5, None)` (None is not a number)
> 4. Store result of `divide(10, 0)` in a variable, then try to do math with it (what happens if it's None?)
>
> For EACH error:
> - Show me the exact Pyright error message
> - Explain WHY Pyright caught it
> - Propose a fix WITHOUT changing type hints (handle edge cases properly)"

**Your debugging task**:
- Which error message is most confusing? Ask AI to clarify in simpler terms.
- Can you now explain to someone else the difference between a type error (Pyright catches) and a runtime error (crashes when running)?

---

### Part 4: Add Production Dependencies (Integration)

Extend the project with real dependencies:

> "Let's make this calculator interactive using `rich` library for beautiful terminal output:
>
> **Requirements**:
> 1. Add `rich` as a PRODUCTION dependency (why production, not dev?)
> 2. Create `main.py` with type hints
> 3. Use `rich.console.Console` to display results
> 4. Accept user input for two numbers and operation (+, -, *, /)
> 5. Handle invalid input gracefully (type hints should guide error handling)
>
> Show me:
> - The `uv add` command for `rich`
> - The complete `main.py` with type hints
> - How type hints help validate user input"

**Refinement**:
> "This doesn't handle when user enters text instead of numbers. Add type-safe input validation that converts strings to floats, handling ValueError."

---

### Part 5: Configure Full Toolchain (Convergence)

Create a production-ready project configuration:

> "Let's set up the complete toolchain in `pyproject.toml`:
>
> **Ruff Configuration**:
> - Line length 88 (Python community standard)
> - Format on save (preview mode)
>
> **Pyright Configuration**:
> - Standard mode
> - Python 3.13
> - Report missing type hints as warnings (not errors)
>
> **Testing Setup**:
> - Add pytest as dev dependency
> - Show me one test for `add()` function with type hints
>
> Then run this sequence:
> 1. `uv run ruff format .` (format all code)
> 2. `uv run pyright` (check types)
> 3. `uv run pytest` (run tests)
>
> All three should PASS. If not, debug together."

**Final validation**:
> "Create a checklist: What steps would a new teammate run to set up this project on their machine? Include environment sync, dependency install, and verification commands."

---

**Time**: 45-60 minutes total
**Outcome**: You've built a complete type-safe Python project using UV, Pyright, Ruff, and pytest—integrating ALL Chapter 12 tools into a cohesive workflow. You understand the relationship between type hints, testing, and code quality.

---

## Red Flags to Watch

**Problem**: "Pyright says type error but my code runs fine"
- **What it means**: Type hints catch static errors; not all runtime issues
- **What to do**: Understand: Pyright prevents bugs early, but tests still catch logic errors
- **Normal**: This is expected behavior; type hints are a safety net, not a guarantee

**Problem**: "Pyright says `str | None` is invalid syntax"
- **What it means**: You're using Python 3.12 or older; `|` syntax is Python 3.13+ only
- **What to do**: Upgrade Python to 3.13 (`uv.lock` should handle this), or use `Union[str, None]` syntax
- **Check version**: Run `python --version`

**Problem**: "Pyright shows 'LSP not connected' in Zed"
- **What it means**: Zed can't find Pyright LSP
- **What to do**: Verify Pyright installed (`uv add pyright --dev`); restart Zed
- **Not urgent**: You can still run `uv run pyright` from terminal; inline checking comes later

