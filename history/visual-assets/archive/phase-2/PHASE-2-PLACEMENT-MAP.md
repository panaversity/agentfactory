# Phase 2: Visual Placement Map (Visuals 39-52)

**Purpose**: Detailed specifications for embedding 14 Phase 2 visuals in lesson files

---

## Visual 39: Python Operator Categories

**File**: `python-operator-categories-four-quadrants.png`
**Location**: `book-source/static/img/part-4/chapter-16/`
**Lesson File**: `apps/learn-app/docs/04-Python-Fundamentals/16-operators-keywords-variables/01-operators.md`
**Placement**: After the "## Operator Categories" or "## Types of Operators" section header
**Pedagogical Purpose**: Organize 20+ Python operators into 4 logical categories to prevent symbol confusion and provide mental model for operator types

**Alt Text**:

```
Four-quadrant diagram showing Python operator categories: Arithmetic operators (blue, +, -, *, /, //, %, ** for mathematical operations), Comparison operators (green, ==, !=, <, >, <=, >= for boolean comparisons), Logical operators (purple, and, or, not for combining conditions), and Assignment operators (orange, =, +=, -=, *=, /= for storing and modifying values), with example usage demonstrating each category's purpose in Python programming
```

---

## Visual 40: Variable Naming Rules & Conventions

**File**: `python-variable-naming-rules-conventions.png`
**Location**: `book-source/static/img/part-4/chapter-16/`
**Lesson File**: `apps/learn-app/docs/04-Python-Fundamentals/16-operators-keywords-variables/02-variables.md`
**Placement**: After the "## Variable Naming" or "## Naming Conventions" section header
**Pedagogical Purpose**: Distinguish between syntax rules (must follow) and style conventions (should follow) for Python variable names

**Alt Text**:

```
Three-section educational diagram showing Python variable naming: Rules section displays valid names (user_name, _temp, data2) versus invalid names marked with red X (2data, user-name, class keyword); Conventions section shows best practices in green (user_name, MAX_SIZE, calculate_total) versus poor style in yellow (userName, x, temp); Why It Matters section compares readable versus unreadable code examples demonstrating impact of good naming conventions
```

---

## Visual 41: String Methods Reference Card

**File**: `python-string-methods-reference-card.png`
**Location**: `book-source/static/img/part-4/chapter-17/`
**Lesson File**: `apps/learn-app/docs/04-Python-Fundamentals/17-strings-type-casting/01-strings.md`
**Placement**: After the "## Common String Methods" or "## String Manipulation" section header
**Pedagogical Purpose**: Provide quick-reference grid for 12 essential string methods students will use frequently

**Alt Text**:

```
Educational reference card displaying 12 essential Python string methods in a 3×4 grid: .upper() converts to uppercase, .lower() to lowercase, .strip() removes whitespace, .replace() substitutes text, .split() creates lists, .join() combines lists, .startswith() and .endswith() perform boolean checks, .find() locates substrings, .count() tallies occurrences, .isdigit() validates numbers, .format() formats strings—each cell shows method name in blue, syntax example in monospace, and result description in gray
```

---

## Visual 42: String Formatting Evolution

**File**: `python-string-formatting-evolution-three-eras.png`
**Location**: `book-source/static/img/part-4/chapter-17/`
**Lesson File**: `apps/learn-app/docs/04-Python-Fundamentals/17-strings-type-casting/01-strings.md`
**Placement**: After the "## String Formatting" or "## f-strings" section header
**Pedagogical Purpose**: Show historical progression of Python string formatting to help students understand why f-strings are preferred

**Alt Text**:

```
Timeline diagram showing Python string formatting evolution across three eras: Legacy % formatting in red ("Hello %s" % name, marked "Old style, avoid"), Transitional .format() method in yellow ("Hello {}".format(name), labeled "Intermediate"), and Modern f-strings in green (f"Hello {name}", marked "Preferred, Python 3.6+"), with arrows showing progression from left to right demonstrating Python's evolution toward more readable syntax
```

---

## Visual 43: Control Flow Decision Tree

**File**: `control-flow-decision-tree-if-elif-else.png`
**Location**: `book-source/static/img/part-4/chapter-18/`
**Lesson File**: `apps/learn-app/docs/04-Python-Fundamentals/18-control-flow-loops/01-control-flow.md`
**Placement**: After the "## if-elif-else Statements" or "## Conditional Logic" section header
**Pedagogical Purpose**: Visualize execution flow through conditional branches to clarify how Python evaluates conditions sequentially

**Alt Text**:

```
Flowchart diagram illustrating Python if-elif-else control flow: starting from top, first diamond decision box checks "if condition1?" leading to green action box if true or continuing down if false, second diamond checks "elif condition2?" with same branching pattern, finally reaching orange "else action" box if all conditions false, with arrows showing two distinct execution paths—success path (green arrows through if/elif to end) and fallback path (gray arrows to else block)
```

---

## Visual 44: Loop Types Comparison

**File**: `loop-types-comparison-for-vs-while.png`
**Location**: `book-source/static/img/part-4/chapter-18/`
**Lesson File**: `apps/learn-app/docs/04-Python-Fundamentals/18-control-flow-loops/02-loops.md`
**Placement**: After the "## for Loops vs while Loops" or "## Loop Types" section header
**Pedagogical Purpose**: Contrast use cases for for loops (known iterations) versus while loops (condition-based) to guide loop selection

**Alt Text**:

```
Side-by-side comparison of Python loop types: FOR loops section (blue) shows circular arrow icon labeled "known iterations", syntax "for item in sequence:", use case "Iterate over collections", example "for i in range(5):"; WHILE loops section (purple) shows circular arrow labeled "condition-based", syntax "while condition:", use case "Repeat until condition false", example "while x < 10:"; bottom guidance indicates FOR for countable iterations and collections, WHILE for unknown repetitions and condition-driven logic
```

---

## Visual 45: Collection Types Comparison Matrix

**File**: `python-collection-types-comparison-matrix.png`
**Location**: `book-source/static/img/part-4/chapter-19/`
**Lesson File**: `apps/learn-app/docs/04-Python-Fundamentals/19-lists-tuples-dictionary/01-collections-overview.md`
**Placement**: After the "## Python Collections" or "## Collection Types Overview" section header
**Pedagogical Purpose**: Compare properties of all four collection types in single table to help students choose appropriate data structure

**Alt Text**:

```
Comparison table for Python collection types showing four columns (LIST in blue, TUPLE in purple, SET in orange, DICT in green) across five property rows: Ordered (✓ for list/tuple/dict Python 3.7+, ✗ for set), Mutable (✓ for list/set/dict, ✗ for tuple), Duplicates allowed (✓ for list/tuple, ✗ for set, "Keys: ✗" for dict), Syntax ([], (), {}, {k:v}), and Use case (dynamic data, fixed data, unique items, key-value pairs), with green checkmarks and red X marks indicating support for each property
```

---

## Visual 46: Dictionary Structure

**File**: `python-dictionary-structure-key-value-pairs.png`
**Location**: `book-source/static/img/part-4/chapter-19/`
**Lesson File**: `apps/learn-app/docs/04-Python-Fundamentals/19-lists-tuples-dictionary/03-dictionaries.md`
**Placement**: After the "## Dictionary Basics" or "## Key-Value Pairs" section header
**Pedagogical Purpose**: Visualize dictionary structure to clarify relationship between keys and values with immutability requirement

**Alt Text**:

```
Dictionary structure diagram displaying Python dict {"name": "Alice", "age": 30, "city": "NYC", "active": True} with visual elements: keys on left in orange (name, age, city, active) connected by arrows to values on right in green (Alice, 30, NYC, True), three callouts explain "Keys must be immutable" pointing to keys, "Values can be any type" pointing to values, and "Accessed via keys: user['name']" at bottom demonstrating retrieval syntax
```

---

## Visual 47: Set Operations Venn Diagrams

**File**: `python-set-operations-venn-diagrams.png`
**Location**: `book-source/static/img/part-4/chapter-20/`
**Lesson File**: `apps/learn-app/docs/04-Python-Fundamentals/20-set-frozenset-gc/01-sets.md`
**Placement**: After the "## Set Operations" section header
**Pedagogical Purpose**: Use Venn diagrams to visualize four fundamental set operations students will perform on Python sets

**Alt Text**:

```
2×2 grid of Venn diagrams showing Python set operations with two overlapping circles (Set A and Set B): Union (top-left, blue, both circles filled, symbol A | B, combines all elements), Intersection (top-right, green, only overlap filled, symbol A & B, common elements only), Difference (bottom-left, orange, left circle non-overlap filled, symbol A - B, elements in A but not B), Symmetric Difference (bottom-right, purple, both circles filled except overlap, symbol A ^ B, elements in either set but not both)
```

---

## Visual 48: Mutability Spectrum

**File**: `python-mutability-spectrum-immutable-mutable.png`
**Location**: `book-source/static/img/part-4/chapter-20/`
**Lesson File**: `apps/learn-app/docs/04-Python-Fundamentals/20-set-frozenset-gc/02-mutability.md`
**Placement**: After the "## Mutable vs Immutable Types" section header
**Pedagogical Purpose**: Organize Python types along mutability spectrum to clarify which types can/cannot be modified after creation

**Alt Text**:

```
Horizontal spectrum diagram showing Python types arranged from immutable to mutable: left side (blue background) displays immutable types with lock icons—int (42), float (3.14), str ("hello"), tuple ((1, 2, 3)), frozenset (frozenset({1, 2})) labeled "Cannot change after creation"; right side (orange background) shows mutable types with unlock icons—list ([1, 2, 3]), set ({1, 2, 3}), dict ({"key": "value"}) labeled "Can modify in place", gradient background transitions blue to orange across center divide
```

---

## Visual 49: Function Signature Anatomy

**File**: `python-function-signature-anatomy.png`
**Location**: `book-source/static/img/part-4/chapter-21/`
**Lesson File**: `apps/learn-app/docs/04-Python-Fundamentals/21-modules-functions/02-functions.md`
**Placement**: After the "## Function Definition" or "## Function Syntax" section header
**Pedagogical Purpose**: Break down function signature components to teach proper function declaration syntax with type hints

**Alt Text**:

```
Annotated diagram of Python function signature showing "def calculate_total(items: list, tax: float = 0.08) -> float:" with six labeled callouts: "def" keyword highlighted in blue (Function keyword), "calculate_total" in green (Function name), "items: list" in purple (Parameter with type hint), "tax: float = 0.08" in orange (Default parameter), "-> float" in purple (Return type hint), docstring """Calculate total price with tax.""" in gray (Documentation), demonstrating complete function anatomy with syntax highlighting and monospace code font
```

---

## Visual 50: Module Import Patterns

**File**: `python-module-import-patterns.png`
**Location**: `book-source/static/img/part-4/chapter-21/`
**Lesson File**: `apps/learn-app/docs/04-Python-Fundamentals/21-modules-functions/01-modules.md`
**Placement**: After the "## Importing Modules" section header
**Pedagogical Purpose**: Compare four import patterns with trade-offs to help students choose appropriate import style

**Alt Text**:

```
Vertical stack showing four Python import patterns: (1) Full module import in blue—"import math", usage "math.sqrt(16)", pros: clear origin and no conflicts, cons: verbose; (2) Specific imports in green—"from math import sqrt, pi", usage "sqrt(16)", pros: concise and explicit, cons: potential conflicts; (3) Import with alias in purple—"import numpy as np", usage "np.array([1,2,3])", pros: short and conventional, cons: requires knowing alias; (4) Wildcard import in red—"from math import *", usage "sqrt(16)", pros: everything imported, cons: unclear origin and conflicts, marked "Not recommended"
```

---

## Visual 51: Try-Except-Finally Flow

**File**: `python-try-except-finally-flow.png`
**Location**: `book-source/static/img/part-4/chapter-22/`
**Lesson File**: `apps/learn-app/docs/04-Python-Fundamentals/22-exception-handling/01-exceptions.md`
**Placement**: After the "## try-except-finally" or "## Exception Handling Blocks" section header
**Pedagogical Purpose**: Show two execution paths (success vs exception) through try-except-else-finally blocks to clarify when each block executes

**Alt Text**:

```
Flowchart showing Python exception handling flow with two color-coded execution paths: Success path (green arrows)—try block (blue) executes without error → else block (green) runs → finally block (purple) executes → END; Exception path (red arrows)—try block encounters error → except block (red) handles exception → finally block (purple) always executes → END; callouts explain "Code that might fail" for try, "Handle errors" for except, "Runs if no exception" for else, "Always runs (cleanup)" for finally
```

---

## Visual 52: Exception Hierarchy

**File**: `python-exception-hierarchy-tree.png`
**Location**: `book-source/static/img/part-4/chapter-22/`
**Lesson File**: `apps/learn-app/docs/04-Python-Fundamentals/22-exception-handling/02-common-exceptions.md`
**Placement**: After the "## Python Exception Hierarchy" or "## Common Exceptions" section header
**Pedagogical Purpose**: Show inheritance structure of Python exceptions to teach exception catching specificity

**Alt Text**:

```
Tree diagram showing Python exception hierarchy across three tiers: root BaseException (gray) branches to Exception (blue) which branches to seven common exceptions with descriptions—ValueError in red ("Invalid value type"), TypeError in red ("Wrong data type"), KeyError in orange ("Missing dictionary key"), IndexError in orange ("List index out of range"), AttributeError in yellow ("Missing attribute"), FileNotFoundError in purple ("File doesn't exist"), ZeroDivisionError in red ("Division by zero")—connecting lines show inheritance relationships, colors indicate severity (red for critical, orange for common, yellow/purple for specific errors)
```

---

## Embedding Instructions

For each visual, insert the markdown reference at the specified placement location:

```markdown
![Alt text from above](/img/part-4/chapter-NN/filename.png)
```

**Ensure**:

- Blank line before and after the image reference
- Alt text matches exactly from this document
- File path is correct (`/img/part-4/chapter-NN/filename.png`)
- Placement is after the specified section header

---

**Created**: 2025-11-22
**Total Visuals**: 14 (V39-V52)
**Chapters Covered**: 16-22
**Status**: Ready for embedding
