# Phase 2 Generation Plan: Visuals 39-52 (Chapters 16-22)

**Date**: 2025-11-22
**Scope**: 14 professional educational visuals
**Status**: 📋 Ready for execution
**Creative Briefs**: ✅ Complete in `PART-4-PYTHON-CONTINUATION.md`

---

## Overview

Phase 2 generates visuals for intermediate Python fundamentals (Chapters 16-22), covering:
- Operators and variables
- String manipulation
- Control flow
- Data structures (lists, tuples, dictionaries, sets)
- Modules and functions
- Exception handling

All creative briefs follow the professional Story+Intent+Metaphor structure established in Phase 1.

---

## Visual Inventory (14 Visuals)

### Chapter 16: Operators, Keywords, Variables (2 visuals)

**VISUAL 39: Python Operator Categories (Arithmetic, Comparison, Logical, Assignment)**
- **Filename**: `python-operator-categories-four-quadrants.png`
- **Type**: 2×2 Category Grid
- **Elements**: 4 quadrants, 5-7 operators per category
- **Proficiency**: A2
- **Creative Brief**: Lines 13-70 in PART-4-PYTHON-CONTINUATION.md

**VISUAL 40: Variable Naming Rules & Conventions**
- **Filename**: `python-variable-naming-rules-conventions.png`
- **Type**: Three-Section Comparison
- **Elements**: Rules (valid/invalid), Conventions (good/poor), Why section
- **Proficiency**: A2
- **Creative Brief**: Lines 72-131 in PART-4-PYTHON-CONTINUATION.md

### Chapter 17: Strings and Type Casting (2 visuals)

**VISUAL 41: String Methods Reference Card (Common Operations)**
- **Filename**: `python-string-methods-reference-card.png`
- **Type**: 3×4 Reference Grid
- **Elements**: 12 essential string methods with syntax and examples
- **Proficiency**: A2
- **Creative Brief**: Lines 139-201 in PART-4-PYTHON-CONTINUATION.md

**VISUAL 42: String Formatting Evolution (% → .format() → f-strings)**
- **Filename**: `python-string-formatting-evolution-three-eras.png`
- **Type**: Chronological Comparison
- **Elements**: 3 formatting methods (legacy, transitional, modern)
- **Proficiency**: A2
- **Creative Brief**: Lines 204-260 in PART-4-PYTHON-CONTINUATION.md

### Chapter 18: Control Flow and Loops (2 visuals)

**VISUAL 43: Control Flow Decision Tree (if/elif/else)**
- **Filename**: `control-flow-decision-tree-if-elif-else.png`
- **Type**: Decision Flowchart
- **Elements**: 3 branches (if, elif, else) with execution path highlighting
- **Proficiency**: A2
- **Creative Brief**: Lines 269-326 in PART-4-PYTHON-CONTINUATION.md

**VISUAL 44: Loop Types Comparison (for vs while)**
- **Filename**: `loop-types-comparison-for-vs-while.png`
- **Type**: Side-by-Side Comparison
- **Elements**: 2 loop types with use cases and examples
- **Proficiency**: A2
- **Creative Brief**: Lines 329-387 in PART-4-PYTHON-CONTINUATION.md

### Chapter 19: Lists, Tuples, Dictionary (2 visuals)

**VISUAL 45: Collection Types Comparison Matrix**
- **Filename**: `python-collection-types-comparison-matrix.png`
- **Type**: Comparison Table
- **Elements**: 4 collections × 5 properties
- **Proficiency**: A2
- **Creative Brief**: Lines 396-452 in PART-4-PYTHON-CONTINUATION.md

**VISUAL 46: Dictionary Structure (Key-Value Pairs)**
- **Filename**: `python-dictionary-structure-key-value-pairs.png`
- **Type**: Structure Diagram
- **Elements**: 3-4 key-value pairs with callouts
- **Proficiency**: A2
- **Creative Brief**: Lines 455-515 in PART-4-PYTHON-CONTINUATION.md

### Chapter 20: Set, Frozen Set, GC (2 visuals)

**VISUAL 47: Set Operations Venn Diagrams**
- **Filename**: `python-set-operations-venn-diagrams.png`
- **Type**: 2×2 Venn Diagram Grid
- **Elements**: 4 set operations (union, intersection, difference, symmetric difference)
- **Proficiency**: B1
- **Creative Brief**: Lines 524-585 in PART-4-PYTHON-CONTINUATION.md

**VISUAL 48: Mutability Spectrum (Immutable → Mutable Types)**
- **Filename**: `python-mutability-spectrum-immutable-mutable.png`
- **Type**: Horizontal Spectrum
- **Elements**: 2 categories, 6-9 Python types
- **Proficiency**: A2
- **Creative Brief**: Lines 588-646 in PART-4-PYTHON-CONTINUATION.md

### Chapter 21: Module and Functions (2 visuals)

**VISUAL 49: Function Signature Anatomy**
- **Filename**: `python-function-signature-anatomy.png`
- **Type**: Annotated Code Diagram
- **Elements**: 6 components (def, name, params, types, docstring, body)
- **Proficiency**: B1
- **Creative Brief**: Lines 655-729 in PART-4-PYTHON-CONTINUATION.md

**VISUAL 50: Module Import Patterns**
- **Filename**: `python-module-import-patterns.png`
- **Type**: Four-Row Pattern Comparison
- **Elements**: 4 import patterns with pros/cons
- **Proficiency**: B1
- **Creative Brief**: Lines 732-803 in PART-4-PYTHON-CONTINUATION.md

### Chapter 22: Exception Handling (2 visuals)

**VISUAL 51: Try-Except-Finally Flow**
- **Filename**: `python-try-except-finally-flow.png`
- **Type**: Execution Flowchart
- **Elements**: 4 blocks (try, except, else, finally), 2 execution paths
- **Proficiency**: B1
- **Creative Brief**: Lines 811-873 in PART-4-PYTHON-CONTINUATION.md

**VISUAL 52: Exception Hierarchy (Common Exceptions)**
- **Filename**: `python-exception-hierarchy-tree.png`
- **Type**: Hierarchy Tree
- **Elements**: 3-tier tree, 7-10 exception types
- **Proficiency**: B1
- **Creative Brief**: Lines 876-937 in PART-4-PYTHON-CONTINUATION.md
- **Note**: Requires Google Search grounding for current Python exception hierarchy

---

## Generation Workflow

### Step 1: Prepare Generation Environment

**Browser Setup**:
1. Navigate to aistudio.google.com
2. Ensure logged into account with Gemini 2.0 Flash Experimental access
3. Open PART-4-PYTHON-CONTINUATION.md for reference

**Directory Setup**:
```bash
# Create Phase 2 directories if they don't exist
mkdir -p book-source/static/img/part-4/chapter-{16..22}
```

**Verification**:
```bash
ls -la book-source/static/img/part-4/
# Should show: chapter-13/ chapter-14/ chapter-15/ chapter-16/ ... chapter-22/
```

### Step 2: Generate Visuals (Autonomous Browser Workflow)

**For Each Visual (V39-V52)**:

1. **Read Creative Brief**
   - Open `PART-4-PYTHON-CONTINUATION.md`
   - Locate visual brief (line numbers provided above)
   - Copy complete brief text

2. **Generate with Gemini**
   - Paste creative brief into Gemini 2.0 Flash Experimental
   - Wait for image generation (typically 10-30 seconds)
   - Review generated visual against 5 quality gates:
     - ✅ Spelling accuracy (99%+)
     - ✅ Layout precision
     - ✅ Color accuracy (semantic colors)
     - ✅ Typography hierarchy
     - ✅ Teaching effectiveness

3. **Quality Validation**
   - If visual fails any gate: provide correction prompt
   - Regenerate until all 5 gates pass
   - Document any regeneration prompts for future reference

4. **Download and Save**
   - Download PNG file
   - Rename to match filename convention (from brief)
   - Save to correct directory:
     ```
     book-source/static/img/part-4/chapter-{NN}/{filename}.png
     ```

5. **Verify File**
   ```bash
   ls -lh book-source/static/img/part-4/chapter-{NN}/{filename}.png
   # Confirm file exists and has reasonable size (typically 100-500KB)
   ```

### Step 3: Create Placement Map for Chapters 16-22

**For Each Visual**:

1. **Identify Target Lesson File**
   - Determine which lesson in Chapter NN discusses the concept
   - Example: V39 (Operators) → likely `01-operators-keywords.md` or similar

2. **Find Pedagogical Placement**
   - Read lesson content
   - Apply placement principles:
     - Conceptual visuals: After section header introducing concept
     - Workflow visuals: Mid-lesson where process explained
     - Comparison visuals: Where contrasting approaches discussed
     - Summary visuals: End of section consolidating concepts

3. **Write Placement Spec**
   ```markdown
   ### Visual 39: Python Operator Categories
   - **Lesson File**: `book-source/docs/04-Python-Fundamentals/16-operators-keywords-variables/01-operators.md`
   - **Placement**: After "## Operator Categories" header
   - **Pedagogical Purpose**: Organize 20+ operators into 4 categories to prevent symbol confusion
   - **Alt Text**: (from creative brief)
   ```

4. **Document in Placement Map**
   - Create or append to `visual-placement-map-phase-2.md`
   - Follow same structure as Phase 1 placement map

### Step 4: Embed Visual References

**For Each Visual**:

1. **Locate Insertion Point**
   ```bash
   # Search for section mentioned in placement map
   grep -n "## Operator Categories" book-source/docs/04-Python-Fundamentals/16-operators-keywords-variables/01-operators.md
   ```

2. **Read Context**
   - Verify placement makes pedagogical sense
   - Ensure visual adds value at this moment in learning

3. **Insert Markdown Reference**
   - Use Edit tool to add:
     ```markdown
     ![Alt text from brief](/img/part-4/chapter-NN/filename.png)
     ```
   - Ensure blank lines before and after visual reference

4. **Verify Embedding**
   ```bash
   grep "filename.png" book-source/docs/04-Python-Fundamentals/16-operators-keywords-variables/01-operators.md
   ```

### Step 5: Batch Verification

**After Completing All 14 Visuals**:

1. **Count PNG Files**
   ```bash
   find book-source/static/img/part-4/chapter-{16..22}/ -name "*.png" | wc -l
   # Expected: 14
   ```

2. **Count Markdown Embeddings**
   ```bash
   grep -r "!\[.*\](/img/part-4/chapter-1[6-9]\|chapter-2[0-2]/" book-source/docs/04-Python-Fundamentals/ | wc -l
   # Expected: 14
   ```

3. **Verify Each Visual**
   ```bash
   # Run updated verification script
   bash /tmp/verify-phase-2-visuals.sh
   ```

4. **Create Verification Script**
   ```bash
   #!/bin/bash
   echo "=== Phase 2 Visual Verification ==="
   echo "Date: $(date)"
   echo ""

   total=$(grep -r "!\[.*\](/img/part-4/chapter-1[6-9]\|chapter-2[0-2]/" book-source/docs/04-Python-Fundamentals/ | wc -l | tr -d ' ')
   echo "Total Phase 2 visual references: $total/14"

   # Per-chapter counts
   c16=$(grep -r "!\[.*\](/img/part-4/chapter-16/" book-source/docs/04-Python-Fundamentals/16-operators-keywords-variables/ | wc -l | tr -d ' ')
   echo "Chapter 16: $c16/2"

   c17=$(grep -r "!\[.*\](/img/part-4/chapter-17/" book-source/docs/04-Python-Fundamentals/17-strings-type-casting/ | wc -l | tr -d ' ')
   echo "Chapter 17: $c17/2"

   # ... (repeat for chapters 18-22)

   if [ "$total" -eq 14 ]; then
       echo "✅ SUCCESS: All 14 Phase 2 visuals embedded"
   else
       echo "⚠️  ATTENTION: Found $total visuals (expected 14)"
   fi
   ```

---

## Quality Gates (Apply to Each Visual)

### Gate 1: Spelling Accuracy (99%+)
- Check all text labels for typos
- Verify technical terms spelled correctly
- Confirm Python syntax accurate (import, def, class, etc.)

### Gate 2: Layout Precision
- Elements aligned on grid
- Visual balance (not all weight on one side)
- Adequate whitespace
- Consistent spacing

### Gate 3: Color Accuracy (Semantic)
- Red (#ef4444) = Friction/problems/old way
- Green (#10b981) = Success/solutions/new way
- Blue (#3b82f6) = Foundation/core concepts
- Purple (#8b5cf6) = Advanced/special concepts
- Orange (#f97316) = Assignment/modification
- Yellow (#fbbf24) = Caution/transition

### Gate 4: Typography Hierarchy
- Largest: Concept/section labels
- Medium: Component names/operators
- Smallest: Example code/descriptions
- Monospace for code elements

### Gate 5: Teaching Effectiveness
- Concept clearer WITH visual than without?
- Appropriate complexity for proficiency (A2 vs B1)?
- No extraneous decorative elements?
- Mental model provided?

---

## Expected Time Investment

**Per Visual**:
- Generation: 1-3 minutes (including quality validation)
- Placement research: 2-5 minutes
- Embedding: 1-2 minutes
- Verification: 1 minute

**Total Estimated Time**:
- 14 visuals × 5-11 minutes = ~70-154 minutes (1.2-2.6 hours)
- Additional: Placement map creation (30-45 minutes)
- Additional: Batch verification and documentation (15-20 minutes)

**Overall**: 2-3.5 hours for complete Phase 2 execution

---

## Completion Criteria

Phase 2 is complete when:

- ✅ All 14 PNG files exist in correct directories
- ✅ All 14 markdown references embedded in lesson files
- ✅ Placement map created documenting pedagogical reasoning
- ✅ Automated verification script confirms 14/14 embeddings
- ✅ Documentation updated (COMPLETION-REPORT.md, VISUAL-ASSET-INVENTORY.md)
- ✅ Overall project status: 52/67 visuals (77.6%)

---

## Next Steps After Phase 2

Upon Phase 2 completion, proceed to **Phase 3**:
1. Expand 15 visual outlines (Chapters 23-30) into full creative briefs
2. Generate 15 visuals using expanded briefs
3. Embed all 15 visuals
4. Complete final verification (67/67)
5. Update all documentation to 100% status

---

**Plan Created**: 2025-11-22
**Ready for Execution**: ✅ YES
**Estimated Duration**: 2-3.5 hours
**Dependencies**: None (all creative briefs exist)
