---
title: "Capstone: Personal Introduction Program"
chapter: 13
lesson: 5
duration_minutes: 30
proficiency_level: "CEFR A2"
blooms_level: "Create"

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Integrating Multiple Programming Concepts"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can design and build a program combining print(), variables, and input() to solve a real problem"

  - name: "Planning Programs with Pseudocode"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can write step-by-step pseudocode describing program logic before writing Python syntax"

  - name: "Debugging and Troubleshooting"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can identify common errors (missing quotes, wrong variable names, syntax mistakes), use error messages to locate problems, and fix issues independently"

learning_objectives:
  - objective: "Design a complete program integrating all Chapter 13 concepts"
    proficiency_level: "A2"
    bloom_level: "Create"
    assessment_method: "Student builds working personal introduction program using print(), variables, and input() without assistance"

  - objective: "Plan program logic using pseudocode before coding"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student writes 5-7 step pseudocode outline describing program flow before writing Python code"

cognitive_load:
  new_concepts: 0
  assessment: "0 new concepts (synthesis only - integrating L1-L4 concepts) at A2 limit of 5-7 ✓"

differentiation:
  extension_for_advanced: "Add formatted output with separators, multiple sections, calculation preview (birth year from age)"
  remedial_for_struggling: "Start with 3-field version (name, age, city); gradually add fields"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/014-chapter-13-redesign/spec.md"
created: "2025-01-18"
last_modified: "2025-01-18"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "2.0.0"
---

# Capstone: Personal Introduction Program

## Your Challenge

Build a complete **Personal Introduction Program** that:
- Asks users for their name, age, city, and favorite hobby
- Stores all information in variables
- Displays a formatted digital introduction card

**No new concepts.** You already know everything you need from Lessons 1-4. This is about **integration**—combining what you've learned into one working program.

---

## Project Requirements

### Required Features

Your program must:

1. **Display a welcome message** when the program starts
2. **Ask four questions** and store the answers:
   - Name
   - Age
   - City
   - Favorite hobby
3. **Display a formatted introduction card** showing all the information

### Expected Output

When complete, your program should produce output similar to this:

```
========================================
  PERSONAL INTRODUCTION CARD CREATOR
========================================

Let's create your introduction card!

What is your name? Sarah
How old are you? 28
Where do you live? Chicago
What is your favorite hobby? Photography

========================================
        INTRODUCTION CARD
========================================
Name:   Sarah
Age:    28 years old
City:   Chicago
Hobby:  Photography
========================================

Thank you, Sarah! Your card is complete.
```

---

## Planning First: Pseudocode

Before writing any Python, plan your program in plain English.

**What is pseudocode?** A step-by-step outline describing what your program should do—no syntax, just logic.

Write your pseudocode now. It should have approximately 10-12 steps covering:
- Welcome message
- Gathering input
- Displaying output

---

## Implementation

Create a new file: `introduction_card.py`

Now write the Python code to implement your pseudocode.

### Hints (only if stuck)

<details>
<summary>Hint 1: Creating separator lines</summary>

You can repeat characters using multiplication:
```python
print("=" * 40)  # Prints 40 equals signs
```
</details>

<details>
<summary>Hint 2: Blank lines for spacing</summary>

Call print with no arguments:
```python
print()  # Creates a blank line
```
</details>

<details>
<summary>Hint 3: Displaying multiple values</summary>

Use commas to print multiple items:
```python
print("Name:", name)
```
</details>

---

## Testing Your Program

Run your program multiple times with different inputs:

**Test 1: Normal input**
- Name: Alex, Age: 25, City: Boston, Hobby: Reading

**Test 2: Names with spaces**
- Name: María José, Age: 30, City: São Paulo, Hobby: Rock climbing

**Test 3: Edge cases**
- What happens with empty input?
- What happens with very long names?

---

## Common Errors

If you encounter errors, check for:

- **Missing quotes** — All text needs quotes around it
- **Missing parentheses** — `print()` and `input()` need `()`
- **Variable typos** — `name` vs `Name` are different variables
- **Using variables before creating them** — Define variables before using them

---

## Extension Challenges (Optional)

Once your basic program works, try these:

1. **Add more fields** — Favorite color, dream job, etc.
2. **Better formatting** — Align the output columns
3. **Multiple sections** — Separate "Personal Info" from "Interests"
4. **Fancy borders** — Use Unicode characters for visual appeal

---

## Reflection

Before moving to Chapter 14, consider:

1. How did combining `print()`, `variables`, and `input()` create something more useful than each alone?
2. Did planning with pseudocode help you write the code?
3. What was the most challenging part?
4. Compare this to Lesson 1—how much have you learned?

---

## Success Criteria

You've completed the capstone when:

- ✅ Program runs without errors
- ✅ All four pieces of information are collected
- ✅ Output is formatted and readable
- ✅ You built it yourself (not copied)

**Congratulations!** You've completed Chapter 13 and built your first complete Python program. You're ready for Chapter 14: Data Types.
