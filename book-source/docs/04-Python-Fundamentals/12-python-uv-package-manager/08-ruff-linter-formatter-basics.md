---
title: "Ruff Linter & Formatter Basics"
chapter: 12
lesson: 8
duration_minutes: 15

skills:
  - name: "Install Ruff via uv"
    proficiency_level: "A1"
    category: "Technical"
    bloom_level: "Remember"
    digcomp_area: "Digital Literacy"
    measurable_at_this_level: "Student runs `uv add ruff --dev` successfully"

  - name: "Run Ruff Format Command"
    proficiency_level: "A1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student runs `uv run ruff format .` and sees files reformatted"

  - name: "Run Ruff Check (Linter)"
    proficiency_level: "A1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student runs `uv run ruff check .` and identifies lint errors"

learning_objectives:
  - objective: "Understand the difference between linting (finding errors) and formatting (fixing style)"
    proficiency_level: "A1"
    bloom_level: "Remember"
    assessment_method: "Student explains: linting catches bugs, formatting makes code look consistent"

  - objective: "Use `uv run ruff format` to automatically format Python files"
    proficiency_level: "A1"
    bloom_level: "Apply"
    assessment_method: "Student runs command and observes formatting changes"

  - objective: "Use `uv run ruff check` to find linting errors (unused imports, bad style)"
    proficiency_level: "A1"
    bloom_level: "Apply"
    assessment_method: "Student identifies at least one error (F401 unused import, E501 line too long, etc.)"

cognitive_load:
  new_concepts: 7
  assessment: "7 concepts: linting vs formatting distinction, Ruff replaces three tools, PEP 8 standards, unused imports detection, line length convention, --fix flag, pyproject.toml config location. Within A1 limit. ✓"

differentiation:
  extension_for_advanced: "Explore Ruff rule codes (F, E, B, I) and understand what each category detects"
  remedial_for_struggling: "Focus on two commands: `uv run ruff format .` (fixes style) and `uv run ruff check .` (finds problems)"

generated_by: "lesson-writer v3.0.0"
source_spec: "specs/001-chapter-12-lightning-python-stack/plan.md"
created: "2025-01-15"
last_modified: "2025-01-15"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Ruff Linter & Formatter Basics

## The Too-Many-Tools Problem

Before **Ruff**, Python developers needed three separate tools to write clean code:

| Tool | Job | Problem |
|------|-----|---------|
| **Black** | Format code style | Opinionated; no configuration |
| **isort** | Sort imports alphabetically | Does only one thing |
| **Flake8** | Find code errors and style violations | Slow |

A beginner would ask: "Which tool do I use?" Answer: "All three, in that order."

**Ruff solves this: One tool does the job of all three—and runs 10-100x faster.**

**Source**: Verified in intelligence/001-verified-tool-documentation.md

---

## What Ruff Does: Format + Lint (Tier 1: Understand)

Ruff has two jobs:

### **1. Formatting** — Make code look consistent
- Fix spacing: `x=1` becomes `x = 1`
- Fix quotes: `'hello'` becomes `"hello"` (configurable)
- Add blank lines between functions
- Proper indentation

**Result**: Your code looks professional, follows Python conventions (PEP 8)

### **2. Linting** — Find likely bugs and style problems
- Unused imports: `import os` (never used)
- Unused variables: `x = 5` (assigned but never read)
- Potential bugs: `if x = 5:` (assignment instead of comparison)
- Line too long: 120+ characters on one line

**Result**: You catch problems before they cause issues

---

## Installing Ruff (Tier 1: Direct Command)

You already have `uv`. Installing Ruff is one command:

```bash
uv add ruff --dev
```

The `--dev` flag means Ruff is a **development dependency**—you need it while coding, but not when your program runs in production.

**Verify installation:**
```bash
uv run ruff --version
```

**Expected output:** `ruff 0.14.5` (or similar version number)

---

## Formatting Your Code (Tier 1: Direct Command)

Let's create a messy Python file and watch Ruff fix it.

**Create a file `messy.py`:**
```python
print('hello')
def   foo(   x,y,z   ):
    return x+y+z
```

**Observe the mess:**
- Extra spaces around `def` and function parameters
- Inconsistent quotes (`'hello'` is single-quoted)
- No spaces around operators (`x+y+z`)

**Run Ruff formatter:**
```bash
uv run ruff format messy.py
```

**Expected output:**
```
1 file reformatted
```

**Look at your file now:**
```python
print("hello")


def foo(x, y, z):
    return x + y + z
```

**What changed:**
- Single quotes → double quotes
- Extra spaces removed
- Operators have spaces: `x + y + z`
- Two blank lines before function definition (PEP 8 standard)

**Note**: This code demonstrates Ruff's formatting, not Python programming. You'll learn Python functions in Chapter 13.

---

## Checking for Errors (Linting) (Tier 1: Direct Command)

Formatting makes code look good. **Linting finds actual bugs.**

**Create a file `buggy.py`:**
```python
import os
import sys

print("Hello")
```

**The problems:**
- You imported `os` but never used it
- You imported `sys` but never used it

**Run Ruff linter:**
```bash
uv run ruff check buggy.py
```

**Expected output:**
```
F401 [*] `os` imported but unused
F401 [*] `sys` imported but unused

Found 2 errors.
[*] 2 fixable with the `--fix` option.
```

**Read the output:**
- `F401`: Ruff code for "unused import"
- `[*]`: Means Ruff can auto-fix this error
- "2 fixable": Both errors can be automatically removed

---

## Auto-Fixing Errors (Tier 1: Direct Command)

You don't have to manually fix linting errors—Ruff can do it:

```bash
uv run ruff check buggy.py --fix
```

**Expected output:**
```
Fixed 2 errors.
```

**Check your file:**
```python
print("Hello")
```

**The unused imports are gone!** Ruff removed them automatically.

---

## Ruff Error Codes (Tier 1: Know What They Mean)

Ruff uses short codes to identify problem types:

| Code | Meaning | Fixable? |
|------|---------|----------|
| **F401** | Import not used | Yes |
| **E501** | Line too long | No (just warns) |
| **E225** | Missing whitespace around operator | Yes |
| **F841** | Variable assigned but not used | Yes |
| **E203** | Whitespace before colon | Yes |

**Key insight**: You don't memorize these codes. If you see an error, ask AI: "What does F401 mean and how do I fix it?"

**Source**: Verified in intelligence/001-verified-tool-documentation.md

---

## When to Format, When to Lint (Tier 1: Decision-Making)

**Format when:**
- You're done writing a section of code
- Before committing to git
- Every time you save (Zed can do this automatically, Lesson 9)

**Lint when:**
- You finish a function or module
- Before running tests
- As part of your review process

**Best practice**: Run both:
```bash
uv run ruff format .          # Fix all style issues
uv run ruff check .           # Find remaining problems
```

---

## Configuration: Where Rules Live (Tier 2: Preview)

Ruff reads configuration from **`pyproject.toml`** in your project root.

You haven't configured anything yet (Ruff works with defaults). But in **Lesson 9**, you'll customize which rules to enable/disable.

For now, just know:
- Default Ruff rules are sensible (follow PEP 8, catch common bugs)
- You'll configure specific rules when you need non-defaults

---

## Try With AI

Use your AI companion for these exercises.

### Prompt 1: Install & Run (Tier 1 — Direct)

```
1. Install Ruff in a uv project: `uv add ruff --dev`
2. Create a messy Python file with inconsistent spacing and quotes
3. Run `uv run ruff format .` and show me the before/after

Show me the exact commands and what the file looks like after formatting.
```

**Expected outcome:** Ruff installed, formatting works, you see code cleaned up.

### Prompt 2: Find Lint Errors (Tier 1 — Direct)

```
Create a Python file with:
- Unused import (import os but don't use it)
- A long line (>88 characters)

Run `uv run ruff check .` and show me the errors. What does each error code mean?
```

**Expected outcome:** Understand that Ruff catches unused imports (F401) and line length (E501).

### Prompt 3: Fix With AI (Tier 2 — AI Assistance)

```
I have lint errors in my Python file. Run `uv run ruff check .` and show me the errors.
Then run `uv run ruff check . --fix` and explain what changed.
Why can some errors be auto-fixed and others not?
```

**Expected outcome:** Understand `--fix` flag, learn which errors are auto-fixable, see real workflow.

---

## Red Flags to Watch

**Problem**: "Ruff says E501 (line too long) but I don't know how to fix it"
- **What it means**: Your line has >88 characters; Ruff wants it shorter
- **What to do**: Break the line into multiple lines (Lesson 9 will cover config for this)
- **For now**: Ask AI: "My line is too long: [paste line]. How should I break it up?"

**Problem**: "`uv run ruff` says 'command not found'"
- **What it means**: You forgot `--dev` when installing, or didn't run `uv add ruff --dev`
- **What to do**: Run `uv add ruff --dev` again
- **Key reminder**: Always use `uv run` to execute tools; don't try `ruff format` directly

**Problem**: "Ruff formatted my file but changed things I didn't want"
- **What it means**: Ruff's defaults don't match your style (happens with quotes, line length)
- **What to do**: This is why Lesson 9 teaches configuration. For now, accept Ruff's changes or ask AI to customize rules
- **Normal**: This is expected; Lesson 9 fixes it

