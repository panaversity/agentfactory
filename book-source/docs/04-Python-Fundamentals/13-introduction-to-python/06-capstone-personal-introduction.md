---
title: "Capstone: Personal Introduction Program"
chapter: 13
lesson: 6
duration_minutes: 50
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
    measurable_at_this_level: "Student can design and build a program combining print(), variables, type hints, and input() to solve a real problem"

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
    assessment_method: "Student builds working personal introduction program using print(), variables with type hints, and input() without assistance"

  - objective: "Plan program logic using pseudocode before coding"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student writes 5-7 step pseudocode outline describing program flow before writing Python code"

  - objective: "Debug and troubleshoot program errors independently"
    proficiency_level: "A2"
    bloom_level: "Analyze"
    assessment_method: "Student identifies and fixes 3-5 deliberately introduced errors using error messages and logical reasoning"

cognitive_load:
  new_concepts: 0
  assessment: "0 new concepts (synthesis only - integrating L1-L5 concepts) at A2 limit of 5-7 ✓"

differentiation:
  extension_for_advanced: "Add formatted output with separators, multiple sections (about, hobbies, goals), calculation preview (birth year from age), colorful ASCII art borders"
  remedial_for_struggling: "Start with 3-field version (name, age, city); gradually add fields; provide skeleton code with TODOs"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/014-chapter-13-redesign/spec.md"
created: "2025-01-18"
last_modified: "2025-01-18"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Capstone: Personal Introduction Program

## What You'll Build

A complete **Personal Introduction Program** that:
- Asks users for their name, age, city, and favorite hobby
- Stores all information in properly typed variables
- Displays a beautifully formatted digital introduction card
- Demonstrates mastery of all Chapter 13 concepts

**No new concepts in this lesson.** You already know everything you need. This is about **integration**—combining what you've learned into one working program.

---

## Opening Hook: Your Robot Introduces Itself

Our robot's journey is complete. It has learned:
- **Lesson 1:** What programming is (giving instructions)
- **Lesson 2:** How to speak (`print()`)
- **Lesson 3:** How to remember (`variables`)
- **Lesson 4:** How to organize knowledge (`type hints`)
- **Lesson 5:** How to listen (`input()`)

Now it's time for the final test: **Can the robot introduce itself to strangers?**

Imagine your robot at a networking event. It needs to:
1. Ask someone's name
2. Ask their age
3. Ask where they're from
4. Ask their interests
5. Create a personalized introduction based on their answers

This isn't a single skill—it's **all five lessons working together**.

That's exactly what you're going to build. By the end of this capstone, you'll have created a complete program from scratch that showcases everything you've learned in Chapter 13.

**This is your proof of mastery.**

---

## Project Overview: Personal Introduction Program

### What It Does

Your program will:
1. **Welcome the user** with a friendly message
2. **Ask four questions**: name, age, city, hobby
3. **Store answers** in properly typed variables
4. **Display a formatted introduction card** with all their information

### Example Interaction

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

Thank you, Sarah! Your introduction card is complete.
```

### Skills You'll Demonstrate

- ✅ Using `print()` for output and formatting
- ✅ Creating variables with descriptive names
- ✅ Adding type hints for documentation
- ✅ Using `input()` to gather user data
- ✅ Combining variables in output
- ✅ Planning with pseudocode
- ✅ Debugging errors independently

---

## Phase 1: Planning (Pseudocode)

**Professional developers plan before coding.** Pseudocode is plain-English description of what your program should do.

### What Is Pseudocode?

Pseudocode is a step-by-step outline written in everyday language, not Python syntax.

**Example:**
```
1. Display welcome message
2. Ask user for their name
3. Store name in a variable
4. Ask user for their age
5. Store age in a variable
6. Display formatted introduction using stored variables
```

Notice: No `print()`, no `input()`, no syntax. Just logic.

### Pseudocode for Our Capstone

Write this in your notebook (or a comment in your code):

```
1. Display program title and welcome message
2. Add blank line for spacing
3. Ask for user's name, store in "name" variable (string)
4. Ask for user's age, store in "age" variable (string for now)
5. Ask for user's city, store in "city" variable (string)
6. Ask for user's hobby, store in "hobby" variable (string)
7. Add blank line before output
8. Display introduction card header with separators
9. Display each field: Name, Age, City, Hobby
10. Display closing separator
11. Display thank you message using user's name
```

**This is your roadmap.** Every line of pseudocode becomes a few lines of Python.

---

## Phase 2: Implementation (Step-by-Step)

Now let's turn pseudocode into working Python code.

### Step 1: Set Up the File

Create a new file: `introduction_card.py`

### Step 2: Add the Header

```python
# Personal Introduction Card Creator
# Chapter 13 Capstone Project
```

Comments help document your code. They're for humans, not Python.

### Step 3: Display Welcome Message

```python
print("=" * 40)
print("  PERSONAL INTRODUCTION CARD CREATOR")
print("=" * 40)
print()
print("Let's create your introduction card!")
print()
```

**What's happening:**
- `"=" * 40` creates a line of 40 equals signs
- `print()` with no arguments creates a blank line for spacing

**Run it now** to test: `python introduction_card.py`

You should see:
```
========================================
  PERSONAL INTRODUCTION CARD CREATOR
========================================

Let's create your introduction card!
```

✅ **Checkpoint 1 passed!**

---

### Step 4: Gather User Input

Add this code:

```python
name: str = input("What is your name? ")
age: str = input("How old are you? ")
city: str = input("Where do you live? ")
hobby: str = input("What is your favorite hobby? ")
```

**What's happening:**
- Four `input()` statements gather data
- Each result is stored in a typed variable
- All type hints are `str` because `input()` returns strings

**Run it now** and answer the questions. Nothing displays yet—we haven't added output.

✅ **Checkpoint 2 passed!**

---

### Step 5: Display the Introduction Card

Add this code:

```python
print()
print("=" * 40)
print("        INTRODUCTION CARD")
print("=" * 40)
print("Name:   " + name)
print("Age:    " + age + " years old")
print("City:   " + city)
print("Hobby:  " + hobby)
print("=" * 40)
print()
print("Thank you, " + name + "! Your introduction card is complete.")
```

**What's happening:**
- Blank line separates input from output
- Header with separator lines
- Each field displayed with spacing for alignment
- Footer separator
- Personalized thank you message

**Run the complete program:**

```
python introduction_card.py
```

Answer the questions, then see your introduction card!

✅ **Checkpoint 3 passed! Your capstone works!**

---

## The Complete Code

Here's your full program:

```python
# Personal Introduction Card Creator
# Chapter 13 Capstone Project

# Display welcome message
print("=" * 40)
print("  PERSONAL INTRODUCTION CARD CREATOR")
print("=" * 40)
print()
print("Let's create your introduction card!")
print()

# Gather user information
name: str = input("What is your name? ")
age: str = input("How old are you? ")
city: str = input("Where do you live? ")
hobby: str = input("What is your favorite hobby? ")

# Display introduction card
print()
print("=" * 40)
print("        INTRODUCTION CARD")
print("=" * 40)
print("Name:   " + name)
print("Age:    " + age + " years old")
print("City:   " + city)
print("Hobby:  " + hobby)
print("=" * 40)
print()
print("Thank you, " + name + "! Your introduction card is complete.")
```

**Congratulations!** You just wrote a complete Python program from scratch.

---

## Phase 3: Testing and Debugging

### Test Case 1: Normal Input

Run your program with typical answers:
```
Name: Alex
Age: 25
City: Boston
Hobby: Reading
```

Does the output look correct?

---

### Test Case 2: Unusual Names

Try special cases:
```
Name: María José
Age: 30
City: São Paulo
Hobby: Rock climbing
```

Python handles international characters. Does your program work?

---

### Test Case 3: Numbers in Text

```
Name: Alex123
Age: 25
City: New York
Hobby: Gaming
```

Even though the name has numbers, it's still text (string). Does it display correctly?

---

## Common Challenges and Solutions

### Challenge 1: "SyntaxError: unterminated string literal"

**Cause:** Missing closing quote or parenthesis

**Example:**
```python
name = input("What is your name?
```

**Solution:**
```python
name = input("What is your name? ")  # Add " and )
```

---

### Challenge 2: "NameError: name 'X' is not defined"

**Cause:** Using a variable before creating it, or typo in variable name

**Example:**
```python
print("Hello,", name)  # name doesn't exist yet
name = input("What is your name? ")
```

**Solution:** Create variables before using them:
```python
name = input("What is your name? ")
print("Hello,", name)
```

---

### Challenge 3: Output Not Aligned

**Problem:**
```
Name:Alex
Age:25
City:Boston
```

**Cause:** Missing spaces in string concatenation

**Solution:**
```python
print("Name:   " + name)  # Extra spaces for alignment
print("Age:    " + age + " years old")
```

---

## Extension Ideas (Optional Challenges)

Once your basic program works, try these enhancements:

### Extension 1: Add More Fields

Add two more questions:
- Favorite color
- Favorite programming language (even if they're just learning!)

Update your introduction card to display all six fields.

---

### Extension 2: Better Formatting

Make your card more visually appealing:

```python
print("╔" + "═" * 38 + "╗")
print("║     INTRODUCTION CARD              ║")
print("╠" + "═" * 38 + "╣")
print("║ Name:   " + name.ljust(28) + "║")
print("║ Age:    " + age.ljust(28) + "║")
print("╚" + "═" * 38 + "╝")
```

(Unicode box-drawing characters create a fancy border)

---

### Extension 3: Multiple Sections

Create sections in your card:

```
========================================
     PERSONAL INFORMATION
========================================
Name:   Alex
Age:    25
City:   Boston

========================================
     INTERESTS & HOBBIES
========================================
Hobby:  Reading
...
```

---

### Extension 4: Personalized Messages

Add conditional-like logic (preview of Chapter 17):

If the user's name is "Alex", display:
```
Special message: Hello, fellow Alex! 👋
```

(You can hard-code this for now—real conditionals come later)

---

## Reflection Questions

Before moving to Chapter 14, reflect on what you've learned:

**1. Integration**
- How does combining `print()`, `variables`, `type hints`, and `input()` make programs more powerful than using each alone?

**2. Problem-Solving**
- What was the hardest part of building this program?
- How did you overcome challenges (error messages, debugging, testing)?

**3. Planning**
- Did pseudocode help you organize your thoughts before coding?
- Would you use pseudocode for future projects?

**4. Growth**
- Compare this to Lesson 1. How much have you learned?
- What programming skill do you want to develop next?

---

## Try With AI

### Prompt 1: Code Review

```
I built a Personal Introduction Card program for my Python capstone.

Here's my code:
[paste your complete code]

Review my code and provide feedback on:
1. Code quality (naming, spacing, readability)
2. Any bugs or issues you spot
3. Suggestions for improvement
4. What I did well

Be specific with line numbers if you reference problems.
```

**What to look for:** AI provides constructive feedback on your work.

**Reflection:** Which suggestion will you implement first?

---

### Prompt 2: Extension Help

```
I want to extend my introduction card program.

Current version:
[paste your code]

Help me add:
1. Two more input fields (favorite color, dream job)
2. Better formatting with aligned columns
3. A separator between sections

Show me the updated code with explanations of what changed.
```

**What to look for:** AI shows how to extend your program systematically.

**Reflection:** Can you implement the extensions yourself after seeing the example?

---

### Prompt 3: Create a Variation

```
I built a Personal Introduction Card program.

Now help me create a DIFFERENT program using the same concepts (print, variables, type hints, input):

Ideas:
- A simple quiz program (ask 3 questions, store answers, display score)
- A story generator (ask for nouns/verbs, create a story)
- A to-do list creator (gather 5 tasks, display formatted list)

Pick one and show me the complete code.
```

**What to look for:** AI demonstrates how Chapter 13 skills apply to different problems.

**Reflection:** Which variation interests you most? Build it!

---

### Prompt 4: Capstone Self-Assessment

```
I completed the Chapter 13 capstone project.

Quiz me on what I learned:
1. How did I integrate all four concepts (print, variables, type hints, input)?
2. What does "integration" mean in programming?
3. Why is the capstone important for learning?
4. What should I focus on next in Chapter 14?

After each answer, give me feedback.
```

**What to look for:** AI helps you articulate your learning.

**Reflection:** Can you explain Chapter 13 concepts to someone who hasn't programmed before?

---

**Safety Note:** When AI reviews your code or suggests extensions, understand *why* each change is made. Don't just copy—learn the reasoning.

---

## You Did It! 🎉

**Congratulations on completing Chapter 13: Introduction to Python!**

You've learned:
- ✅ What programming is (giving instructions to computers)
- ✅ How to speak with `print()`
- ✅ How to remember with `variables`
- ✅ How to organize with `type hints`
- ✅ How to listen with `input()`
- ✅ How to integrate everything into a complete program

**This is a huge milestone.** You're no longer a "non-programmer"—you're a programmer who's just getting started.

---

## What's Next?

**Chapter 14: Data Types** awaits you!

You'll learn:
- Deep dive into `int`, `float`, `str`, `bool`
- Collections: `list`, `dict`, `tuple`, `set`
- Type utilities: `type()`, `isinstance()`, type casting
- Real-world decision frameworks for choosing types

**The foundation you built in Chapter 13 makes Chapter 14 much easier.** You already understand type hints—now you'll master the types themselves.

**Keep going. You're doing great!** 🚀
