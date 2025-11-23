# Phase 2: Downloaded File Renaming & Placement Guide

**After downloading all 14 images from Gemini tabs**, use this guide to rename and move them to the correct locations.

---

## Downloaded Files Location

Your downloaded images are likely in: `~/Downloads/`

Files will be named something like: `Gemini_Generated_Image_XXXXX.png`

---

## Renaming and Moving Instructions

### Tab 1: Visual 39 - Python Operator Categories
**Downloaded as**: `Gemini_Generated_Image_*.png`
**Rename to**: `python-operator-categories-four-quadrants.png`
**Move to**: `book-source/static/img/part-4/chapter-16/`

### Tab 2: Visual 41 - String Methods Reference
**Downloaded as**: `Gemini_Generated_Image_*.png`
**Rename to**: `python-string-methods-reference-card.png`
**Move to**: `book-source/static/img/part-4/chapter-17/`

### Tab 3: Visual 42 - String Formatting Evolution
**Downloaded as**: `Gemini_Generated_Image_*.png`
**Rename to**: `python-string-formatting-evolution-three-eras.png`
**Move to**: `book-source/static/img/part-4/chapter-17/`

### Tab 4: Visual 43 - Control Flow Decision Tree
**Downloaded as**: `Gemini_Generated_Image_*.png`
**Rename to**: `control-flow-decision-tree-if-elif-else.png`
**Move to**: `book-source/static/img/part-4/chapter-18/`

### Tab 5: Visual 44 - Loop Types Comparison
**Downloaded as**: `Gemini_Generated_Image_*.png`
**Rename to**: `loop-types-comparison-for-vs-while.png`
**Move to**: `book-source/static/img/part-4/chapter-18/`

### Tab 6: Visual 45 - Collection Types Matrix
**Downloaded as**: `Gemini_Generated_Image_*.png`
**Rename to**: `python-collection-types-comparison-matrix.png`
**Move to**: `book-source/static/img/part-4/chapter-19/`

### Tab 7: Visual 46 - Dictionary Structure
**Downloaded as**: `Gemini_Generated_Image_*.png`
**Rename to**: `python-dictionary-structure-key-value-pairs.png`
**Move to**: `book-source/static/img/part-4/chapter-19/`

### Tab 8: Visual 47 - Set Operations Venn Diagrams
**Downloaded as**: `Gemini_Generated_Image_*.png`
**Rename to**: `python-set-operations-venn-diagrams.png`
**Move to**: `book-source/static/img/part-4/chapter-20/`

### Tab 9: Visual 48 - Mutability Spectrum
**Downloaded as**: `Gemini_Generated_Image_*.png`
**Rename to**: `python-mutability-spectrum-immutable-mutable.png`
**Move to**: `book-source/static/img/part-4/chapter-20/`

### Tab 10: Visual 49 - Function Signature Anatomy
**Downloaded as**: `Gemini_Generated_Image_*.png`
**Rename to**: `python-function-signature-anatomy.png`
**Move to**: `book-source/static/img/part-4/chapter-21/`

### Tab 11: Visual 50 - Module Import Patterns
**Downloaded as**: `Gemini_Generated_Image_*.png`
**Rename to**: `python-module-import-patterns.png`
**Move to**: `book-source/static/img/part-4/chapter-21/`

### Tab 12: Visual 51 - Try-Except-Finally Flow
**Downloaded as**: `Gemini_Generated_Image_*.png`
**Rename to**: `python-try-except-finally-flow.png`
**Move to**: `book-source/static/img/part-4/chapter-22/`

**NOTE**: We only have 12 tabs (0-11), but we need 14 visuals. Visual 40 and Visual 52 are missing.

---

## Missing Visuals (Need Generation)

### Visual 40: Variable Naming Rules & Conventions
**Filename**: `python-variable-naming-rules-conventions.png`
**Directory**: `book-source/static/img/part-4/chapter-16/`
**Prompt**: See PHASE-2-PROMPTS-FOR-GENERATION.md (Visual 40)

### Visual 52: Exception Hierarchy
**Filename**: `python-exception-hierarchy-tree.png`
**Directory**: `book-source/static/img/part-4/chapter-22/`
**Prompt**: See PHASE-2-PROMPTS-FOR-GENERATION.md (Visual 52)

---

## Automated Moving Script

Run this command from the project root after renaming:

```bash
# This assumes you've renamed files in ~/Downloads/

# Visual 39
mv ~/Downloads/python-operator-categories-four-quadrants.png book-source/static/img/part-4/chapter-16/

# Visual 41
mv ~/Downloads/python-string-methods-reference-card.png book-source/static/img/part-4/chapter-17/

# Visual 42
mv ~/Downloads/python-string-formatting-evolution-three-eras.png book-source/static/img/part-4/chapter-17/

# Visual 43
mv ~/Downloads/control-flow-decision-tree-if-elif-else.png book-source/static/img/part-4/chapter-18/

# Visual 44
mv ~/Downloads/loop-types-comparison-for-vs-while.png book-source/static/img/part-4/chapter-18/

# Visual 45
mv ~/Downloads/python-collection-types-comparison-matrix.png book-source/static/img/part-4/chapter-19/

# Visual 46
mv ~/Downloads/python-dictionary-structure-key-value-pairs.png book-source/static/img/part-4/chapter-19/

# Visual 47
mv ~/Downloads/python-set-operations-venn-diagrams.png book-source/static/img/part-4/chapter-20/

# Visual 48
mv ~/Downloads/python-mutability-spectrum-immutable-mutable.png book-source/static/img/part-4/chapter-20/

# Visual 49
mv ~/Downloads/python-function-signature-anatomy.png book-source/static/img/part-4/chapter-21/

# Visual 50
mv ~/Downloads/python-module-import-patterns.png book-source/static/img/part-4/chapter-21/

# Visual 51
mv ~/Downloads/python-try-except-finally-flow.png book-source/static/img/part-4/chapter-22/

# After adding Visual 40 and 52:
# mv ~/Downloads/python-variable-naming-rules-conventions.png book-source/static/img/part-4/chapter-16/
# mv ~/Downloads/python-exception-hierarchy-tree.png book-source/static/img/part-4/chapter-22/
```

---

## Verification

After moving all files, verify with:

```bash
find book-source/static/img/part-4/chapter-{16..22}/ -name "*.png" | wc -l
# Expected: 14 (after generating and moving Visual 40 and 52)
```

---

**Once files are in place, let me know and I'll create the placement map and embed all visual references in the lesson files.**
