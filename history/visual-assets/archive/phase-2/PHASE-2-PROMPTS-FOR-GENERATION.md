# Phase 2: Image Generation Prompts (Visuals 39-52)

**Instructions**: Copy each prompt below and paste into a new Imagen 4 tab. Generate all 14 images, then Claude will handle downloading and placement.

---

## VISUAL 39: Python Operator Categories
**Filename**: `python-operator-categories-four-quadrants.png`
**Directory**: `book-source/static/img/part-4/chapter-16/`

**PROMPT**:
```
Create a professional educational diagram showing a 2×2 grid layout for Python operator categories.

Four quadrants labeled:
- Top-left: "ARITHMETIC" (blue #3b82f6) with operators: +, -, *, /, //, %, **
- Top-right: "COMPARISON" (green #10b981) with operators: ==, !=, <, >, <=, >=
- Bottom-left: "LOGICAL" (purple #8b5cf6) with operators: and, or, not
- Bottom-right: "ASSIGNMENT" (orange #f97316) with operators: =, +=, -=, *=, /=

Typography hierarchy:
- Largest: Category labels (ARITHMETIC, COMPARISON, LOGICAL, ASSIGNMENT)
- Medium: Operator symbols
- Smallest: Example usage (5 + 3, x == 5, True and False, x = 10)

Clean, organized layout with clear visual separation between quadrants. Minimal, professional design suitable for educational content. White background. Sans-serif font.
```

---

## VISUAL 40: Variable Naming Rules & Conventions
**Filename**: `python-variable-naming-rules-conventions.png`
**Directory**: `book-source/static/img/part-4/chapter-16/`

**PROMPT**:
```
Create a professional educational diagram showing Python variable naming rules and conventions in a three-section vertical layout.

Section 1: RULES (Valid vs Invalid) with red (#ef4444) for invalid
- Valid examples: user_name, _temp, data2
- Invalid examples (with X marks): 2data, user-name, class

Section 2: CONVENTIONS (Good vs Poor) with green (#10b981) for good, yellow (#fbbf24) for poor
- Good: user_name, MAX_SIZE, calculate_total()
- Poor: userName, x, temp

Section 3: WHY CONVENTIONS MATTER
- Side-by-side code comparison showing readable vs unreadable variable names

Typography hierarchy:
- Largest: Section headers (RULES, CONVENTIONS, WHY)
- Medium: Valid/Invalid, Good/Poor labels
- Smallest: Code examples

Clean, organized layout. Professional design for educational content. White background. Sans-serif font.
```

---

## VISUAL 41: String Methods Reference Card
**Filename**: `python-string-methods-reference-card.png`
**Directory**: `book-source/static/img/part-4/chapter-17/`

**PROMPT**:
```
Create a professional educational reference card showing 12 essential Python string methods in a 3×4 grid layout.

Methods to include (each in its own cell):
- .upper() → "HELLO"
- .lower() → "hello"
- .strip() → removes whitespace
- .replace(old, new) → substitution
- .split() → creates list
- .join() → combines list
- .startswith() → boolean check
- .endswith() → boolean check
- .find() → index position
- .count() → occurrences
- .isdigit() → validation
- .format() → string formatting

Each cell contains:
- Method name (largest, blue #3b82f6)
- Syntax example (medium, monospace)
- Result/purpose (smallest, gray)

Clean grid layout with borders between cells. Professional design for educational content. White background. Sans-serif font for labels, monospace for code.
```

---

## VISUAL 42: String Formatting Evolution
**Filename**: `python-string-formatting-evolution-three-eras.png`
**Directory**: `book-source/static/img/part-4/chapter-17/`

**PROMPT**:
```
Create a professional educational timeline showing Python string formatting evolution across three horizontal sections.

Three eras (left to right):
1. LEGACY (red #ef4444): % Formatting
   - Example: "Hello %s" % name
   - Label: "Old style (avoid)"

2. TRANSITIONAL (yellow #fbbf24): .format() Method
   - Example: "Hello {}".format(name)
   - Label: "Intermediate"

3. MODERN (green #10b981): f-strings
   - Example: f"Hello {name}"
   - Label: "Preferred (Python 3.6+)"

Each section shows:
- Era label (largest)
- Code example (medium, monospace)
- Status indicator (smallest)

Arrow progression from left to right showing evolution. Clean, organized layout. Professional design for educational content. White background.
```

---

## VISUAL 43: Control Flow Decision Tree (if/elif/else)
**Filename**: `control-flow-decision-tree-if-elif-else.png`
**Directory**: `book-source/static/img/part-4/chapter-18/`

**PROMPT**:
```
Create a professional educational flowchart showing Python if/elif/else control flow as a decision tree.

Flow structure (top to bottom):
1. START (blue circle)
2. Diamond: "if condition1?"
   - YES → Action 1 (green box) → END
   - NO ↓
3. Diamond: "elif condition2?"
   - YES → Action 2 (green box) → END
   - NO ↓
4. Rectangle: "else: Action 3" (orange box) → END

Color coding:
- Diamonds (decisions): blue #3b82f6
- Action boxes (true paths): green #10b981
- Else action: orange #f97316
- Flow arrows: gray

Typography:
- Largest: Condition text
- Medium: Action labels
- Smallest: YES/NO on arrows

Clean flowchart layout. Professional design for educational content. White background.
```

---

## VISUAL 44: Loop Types Comparison (for vs while)
**Filename**: `loop-types-comparison-for-vs-while.png`
**Directory**: `book-source/static/img/part-4/chapter-18/`

**PROMPT**:
```
Create a professional educational diagram comparing Python for loops vs while loops in a side-by-side layout.

Left side: FOR LOOPS (blue #3b82f6)
- Icon: circular arrow with "known iterations"
- Syntax: for item in sequence:
- Use case: "Iterate over collections"
- Example: for i in range(5):

Right side: WHILE LOOPS (purple #8b5cf6)
- Icon: circular arrow with "condition-based"
- Syntax: while condition:
- Use case: "Repeat until condition false"
- Example: while x < 10:

Bottom section: WHEN TO USE
- FOR: "Count known, iterating collections"
- WHILE: "Unknown iterations, condition-driven"

Typography hierarchy:
- Largest: "FOR" and "WHILE" headers
- Medium: Syntax and use cases
- Smallest: Examples (monospace)

Clean, balanced layout with vertical divider. Professional design for educational content. White background.
```

---

## VISUAL 45: Collection Types Comparison Matrix
**Filename**: `python-collection-types-comparison-matrix.png`
**Directory**: `book-source/static/img/part-4/chapter-19/`

**PROMPT**:
```
Create a professional educational comparison table for Python collection types.

Table with 5 rows (properties) × 4 columns (types):

Columns: LIST | TUPLE | SET | DICT
Properties rows:
1. Ordered: ✓ | ✓ | ✗ | ✓ (Python 3.7+)
2. Mutable: ✓ | ✗ | ✓ | ✓
3. Duplicates: ✓ | ✓ | ✗ | Keys: ✗
4. Syntax: [] | () | {} | {k:v}
5. Use Case: Dynamic data | Fixed data | Unique items | Key-value pairs

Color coding:
- LIST: blue #3b82f6
- TUPLE: purple #8b5cf6
- SET: orange #f97316
- DICT: green #10b981

✓ = green checkmark
✗ = red X

Typography:
- Largest: Column headers (collection types)
- Medium: Property labels and cell content
- Smallest: Use case descriptions

Clean table with gridlines. Professional design for educational content. White background.
```

---

## VISUAL 46: Dictionary Structure (Key-Value Pairs)
**Filename**: `python-dictionary-structure-key-value-pairs.png`
**Directory**: `book-source/static/img/part-4/chapter-19/`

**PROMPT**:
```
Create a professional educational diagram showing Python dictionary structure with 3-4 key-value pairs and callouts.

Central dictionary visualization:
{
  "name": "Alice",
  "age": 30,
  "city": "NYC",
  "active": True
}

Visual elements:
- Keys (left side, orange #f97316): "name", "age", "city", "active"
- Arrows/connections pointing to
- Values (right side, green #10b981): "Alice", 30, "NYC", True

Callouts with labels:
- "Keys must be immutable" (pointing to keys)
- "Values can be any type" (pointing to values)
- "Accessed via keys: user['name']" (bottom)

Typography hierarchy:
- Largest: "DICTIONARY STRUCTURE" header
- Medium: Keys and values in the dict
- Smallest: Callout explanations

Clean, organized layout with clear visual connections. Professional design for educational content. White background. Monospace font for code.
```

---

## VISUAL 47: Set Operations Venn Diagrams
**Filename**: `python-set-operations-venn-diagrams.png`
**Directory**: `book-source/static/img/part-4/chapter-20/`

**PROMPT**:
```
Create a professional educational diagram showing 4 Python set operations using Venn diagrams in a 2×2 grid.

Each quadrant shows a different operation with two overlapping circles (Set A and Set B):

1. UNION (top-left, blue #3b82f6)
   - Both circles filled
   - Symbol: A | B or A.union(B)
   - Result: All elements from both sets

2. INTERSECTION (top-right, green #10b981)
   - Only overlap filled
   - Symbol: A & B or A.intersection(B)
   - Result: Common elements only

3. DIFFERENCE (bottom-left, orange #f97316)
   - Only left circle (non-overlap) filled
   - Symbol: A - B or A.difference(B)
   - Result: Elements in A but not B

4. SYMMETRIC DIFFERENCE (bottom-right, purple #8b5cf6)
   - Both circles filled except overlap
   - Symbol: A ^ B or A.symmetric_difference(B)
   - Result: Elements in A or B but not both

Typography hierarchy:
- Largest: Operation names
- Medium: Set operation symbols
- Smallest: Result descriptions

Clean grid layout. Professional design for educational content. White background.
```

---

## VISUAL 48: Mutability Spectrum (Immutable → Mutable)
**Filename**: `python-mutability-spectrum-immutable-mutable.png`
**Directory**: `book-source/static/img/part-4/chapter-20/`

**PROMPT**:
```
Create a professional educational diagram showing Python types arranged on a horizontal spectrum from immutable to mutable.

Horizontal layout with gradient background (blue to orange):

LEFT SIDE - IMMUTABLE (blue #3b82f6 background):
- int: 42
- float: 3.14
- str: "hello"
- tuple: (1, 2, 3)
- frozenset: frozenset({1, 2})

RIGHT SIDE - MUTABLE (orange #f97316 background):
- list: [1, 2, 3]
- set: {1, 2, 3}
- dict: {"key": "value"}

Center divider with labels:
- Left: "Cannot change after creation"
- Right: "Can modify in place"

Each type shows:
- Type name (largest)
- Example (medium, monospace)
- Lock icon (immutable) or Unlock icon (mutable)

Typography hierarchy:
- Largest: "IMMUTABLE" and "MUTABLE" headers
- Medium: Type names
- Smallest: Examples

Clean, organized spectrum layout. Professional design for educational content.
```

---

## VISUAL 49: Function Signature Anatomy
**Filename**: `python-function-signature-anatomy.png`
**Directory**: `book-source/static/img/part-4/chapter-21/`

**PROMPT**:
```
Create a professional educational diagram showing Python function signature anatomy with labeled callouts.

Central function example:
def calculate_total(items: list, tax: float = 0.08) -> float:
    """Calculate total price with tax."""
    return sum(items) * (1 + tax)

Callouts pointing to each component:
1. "def" → "Function keyword" (blue #3b82f6)
2. "calculate_total" → "Function name" (green #10b981)
3. "items: list" → "Parameter with type hint" (purple #8b5cf6)
4. "tax: float = 0.08" → "Default parameter" (orange #f97316)
5. "-> float" → "Return type hint" (purple #8b5cf6)
6. """...""" → "Docstring" (gray)

Typography hierarchy:
- Largest: "FUNCTION SIGNATURE ANATOMY" header
- Medium: Component labels in callouts
- Code: Monospace font
- Smallest: Explanatory text in callouts

Clean layout with arrows from labels to code. Professional design for educational content. White background.
```

---

## VISUAL 50: Module Import Patterns
**Filename**: `python-module-import-patterns.png`
**Directory**: `book-source/static/img/part-4/chapter-21/`

**PROMPT**:
```
Create a professional educational diagram showing 4 Python import patterns in a vertical stack layout.

Four sections, each showing:

1. FULL MODULE IMPORT (blue #3b82f6)
   - Code: import math
   - Usage: math.sqrt(16)
   - Pros: ✓ Clear origin | ✓ No conflicts
   - Cons: ✗ Verbose

2. SPECIFIC IMPORTS (green #10b981)
   - Code: from math import sqrt, pi
   - Usage: sqrt(16)
   - Pros: ✓ Concise | ✓ Clear what's imported
   - Cons: ✗ Potential conflicts

3. IMPORT WITH ALIAS (purple #8b5cf6)
   - Code: import numpy as np
   - Usage: np.array([1, 2, 3])
   - Pros: ✓ Short | ✓ Convention
   - Cons: ✗ Requires knowing alias

4. WILDCARD IMPORT (red #ef4444)
   - Code: from math import *
   - Usage: sqrt(16)
   - Pros: ✓ Everything imported
   - Cons: ✗ Unclear origin | ✗ Name conflicts | ✗ Not recommended

Typography hierarchy:
- Largest: Pattern names
- Medium: Code (monospace)
- Smallest: Pros/cons

Clean vertical layout with dividers. Professional design for educational content. White background.
```

---

## VISUAL 51: Try-Except-Finally Flow
**Filename**: `python-try-except-finally-flow.png`
**Directory**: `book-source/static/img/part-4/chapter-22/`

**PROMPT**:
```
Create a professional educational flowchart showing Python try-except-else-finally execution flow.

Flow structure (top to bottom):

1. START → "try:" block (blue #3b82f6)
   - If NO ERROR → "else:" block (green #10b981) → "finally:" block
   - If ERROR ↓

2. "except:" block (red #ef4444)
   - Catches exception → "finally:" block

3. "finally:" block (purple #8b5cf6)
   - Always executes → END

Two execution paths clearly shown:
- Success path (green arrows): try → else → finally → END
- Exception path (red arrows): try → except → finally → END

Callouts:
- try: "Code that might fail"
- except: "Handle errors"
- else: "Runs if no exception"
- finally: "Always runs (cleanup)"

Typography hierarchy:
- Largest: Block names (try, except, else, finally)
- Medium: Path labels
- Smallest: Callout explanations

Clean flowchart layout with color-coded paths. Professional design for educational content. White background.
```

---

## VISUAL 52: Exception Hierarchy (Common Exceptions)
**Filename**: `python-exception-hierarchy-tree.png`
**Directory**: `book-source/static/img/part-4/chapter-22/`

**PROMPT**:
```
Create a professional educational tree diagram showing Python exception hierarchy with 3 tiers and 7-10 common exception types.

Tree structure (top to bottom):

TIER 1: BaseException (root, gray)
  |
TIER 2: Exception (blue #3b82f6)
  |
TIER 3 (branches):
- ValueError (red #ef4444): "Invalid value type"
- TypeError (red #ef4444): "Wrong data type"
- KeyError (orange #f97316): "Missing dictionary key"
- IndexError (orange #f97316): "List index out of range"
- AttributeError (yellow #fbbf24): "Missing attribute"
- FileNotFoundError (purple #8b5cf6): "File doesn't exist"
- ZeroDivisionError (red #ef4444): "Division by zero"

Each exception shows:
- Exception name (largest)
- Brief description (smallest)
- Connecting lines to parent

Color coding by severity/frequency:
- Critical errors: red
- Common errors: orange
- Attribute/lookup: yellow/purple

Typography hierarchy:
- Largest: Exception class names
- Medium: Tier labels
- Smallest: Descriptions

Clean tree layout with hierarchical connections. Professional design for educational content. White background.
```

---

## SUMMARY

**Total Prompts**: 14 (Visuals 39-52)
**Chapters Covered**: 16-22
**Estimated Generation Time**: 2-3.5 hours

**After you paste and generate all images, Claude will**:
1. Download each image
2. Rename to correct filename
3. Move to appropriate chapter directory
4. Create placement map
5. Embed in lesson files
6. Verify all embeddings

**Ready to proceed!**
