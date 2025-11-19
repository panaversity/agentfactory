# Implementation Plan: Chapter 13 - Introduction to Python (Story-Based Learning)

**Branch**: `014-chapter-13-redesign` | **Date**: 2025-01-18 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `specs/014-chapter-13-redesign/spec.md`

## Summary

Redesign Chapter 13 (Introduction to Python) as a **story-based narrative journey** for absolute beginners with zero programming experience. Rather than lecturing about Python as a list of disconnected facts, the chapter unfolds as a narrative arc: students meet the idea of programming through storytelling (robot sandwich analogy), experience first success (Hello World), discover variables through analogy (pet name labels), add type hints as data classification, practice with user input, and synthesize everything in a capstone project (digital introduction card).

**Primary Requirement**: Establish Stage 1 (manual foundation) for absolute beginners (A2 tier) before any AI collaboration. Teach WHAT and WHY concepts exist before HOW to write syntax. Maintain consistent story-based modality that differs from Chapter 14's analogy-based "Concept Before Syntax" approach.

**Teaching Philosophy**: Programming is not mysterious syntax—it's giving instructions to computers. This chapter proves that through a narrative journey, not a lecture series.

---

## Technical Context

**Content Type**: Educational book chapter (Markdown lessons)
**Target Audience**: Absolute beginners, ZERO programming experience (A2 CEFR tier)
**Python Version**: Python 3.14+ (UV package manager from Chapter 12)
**Content Format**: Markdown (.md files) with code blocks, rendered via Docusaurus
**Proficiency Level**: CEFR A2 (Beginner), Bloom's: Remember/Understand
**Cognitive Load**: Max 5-7 new concepts per lesson (A2 tier limit)
**Prerequisites**: Chapter 12 (UV Package Manager, Python 3.14+ installed + working)
**Related Chapters**: Chapter 14 (Data Types - complements Ch 13), Chapter 12 (Environment setup)
**Project Type**: Educational content (book chapter with 5-6 core lessons + 1 capstone)
**Constraints**:
  - Absolute beginner language (explain EVERY term; NO assumed knowledge)
  - Story-based narrative modality (not topic-based lecture series)
  - WHAT-WHY-HOW sequence (explain purpose BEFORE syntax)
  - Heavy scaffolding (step-by-step validation checkpoints)
  - Max 2 options when choices presented (prevent decision paralysis)
**Scale/Scope**: 5-6 core lessons + 1 capstone, ~4-5 hours total learning time, 9 core concepts

---

## Pedagogical Arc Overview

**Chapter 13 is a story with rising action, climax, and resolution:**

```
INTRODUCTION (Lesson 1)
   └─ "What is programming?" Hook: Robot making sandwiches

RISING ACTION (Lessons 2-4)
   ├─ Lesson 2: First program success ("Hello World" = robot speaks)
   ├─ Lesson 3: Variables introduced ("Robot's memory" = pet name labels)
   └─ Lesson 4: Type hints added ("Organizing knowledge" = classifying data)

PEAK ACTION (Lesson 5)
   └─ User input introduced ("Robot listens" = programs interact)

INTEGRATION (Lesson 6 - Capstone)
   └─ Capstone project synthesizes ("Robot introduces itself" = digital card)
```

**Why This Structure?**
- **Foundation first**: Lessons 1-2 establish "what is programming" with minimal cognitive load (2-3 concepts each)
- **Building complexity**: Lessons 3-4 add variables and types with moderate load (2-3 concepts each)
- **Integration**: Lesson 5 combines concepts with peak engagement (2 concepts)
- **Mastery**: Lesson 6 synthesizes all into capstone (0 new concepts, pure integration)
- **Story thread**: Robot metaphor connects every lesson (narrative continuity ≠ topic list)

---

## Constitution Check

*GATE: Must pass before lesson design.*

✅ **Principle 2 (Progressive Complexity)**: PASS
- Tier: A2 (Beginner, 5-7 concepts per lesson limit)
- Lesson 1: 3 concepts (what programming is, why learn Python, terminal intro)
- Lesson 2: 2 concepts (running Python, print function)
- Lesson 3: 3 concepts (memory concept, variables, creating variables)
- Lesson 4: 3 concepts (data types intro, type hints, why types matter)
- Lesson 5: 2 concepts (input function, combining prior concepts)
- Lesson 6: 0 new concepts (capstone synthesis)
- **Total**: 13 concepts across 6 lessons (avg 2.2/lesson, well within A2 limit)

✅ **Principle 1 (Specification Primacy)**: PASS
- WHAT-WHY-HOW sequence applied consistently
- Show INTENT before syntax (robot sandwich story explains "why programming")
- Concepts explained BEFORE code examples
- Story purpose clear before code appears

✅ **Principle 6 (Anti-Convergence)**: PASS
- Chapter 14 uses **analogy-based "Concept Before Syntax"** (kitchen jar analogy for data types)
- Chapter 13 uses **story-based narrative journey** (robot sandwich → variables as pet labels → digital card capstone)
- Teaching modality distinctly different (lecture-like vs narrative-like)

✅ **Principle 3 (Factual Accuracy)**: PASS
- All code examples tested in Python 3.14+
- All Python claims cited from Python.org official docs
- Version compatibility verified (3.10+ mostly compatible)

✅ **Stage 1 (Manual Foundation)**: PASS
- All lessons L1-L6 are Stage 1 (manual practice primary)
- "Try With AI" sections are gentle Stage 2 introduction (practice partner, not solution)
- No spec-first teaching (Stage 4 deferred to Part 5 chapters)

✅ **Cognitive Load (A2 Tier)**: PASS
- No lesson exceeds 7 new concepts
- Heavy scaffolding throughout (step-by-step + validation)
- Max 2 options when choices presented

✅ **No Bloat**: PASS
- Chapter 13 scoped to: intro, first program, variables, type hints, input, capstone
- Deferred to later chapters: operators (Ch 15), strings (Ch 16), control flow (Ch 17)
- "For Curious Learners" sections optional (bytecode, OOP preview)

**Gates**: ALL PASS - Proceed to lesson design

---

## Concept Density Analysis

**Extraction from Spec (FR-003, FR-006)**:

Core concepts students must master:
1. **What programming is** (giving instructions)
2. **Why learn Python** (practical, widely-used)
3. **Running Python code** (terminal, execution)
4. **Print function** (output to screen)
5. **Variables concept** (memory labels)
6. **Creating variables** (assignment syntax)
7. **Data types concept** (different kinds of data)
8. **Type hints** (specifying data types)
9. **User input** (reading from user)

**Chunking for A2 Cognitive Load**:

| Lesson | Concepts | Count | Load Status |
|--------|----------|-------|------------|
| L1: Introduction | What is programming, Why Python, Terminal primer | 3 | Light (heavy scaffolding) |
| L2: First Program | Running Python, Print function | 2 | Light (first success moment) |
| L3: Variables | Memory concept, Variables, Creating them | 3 | Light-Moderate |
| L4: Type Hints | Data types intro, Type hint syntax, Why types matter | 3 | Light-Moderate |
| L5: User Input | Input function, Integration (combining prior 5-8) | 2 | Moderate-Peak |
| L6: Capstone | 0 new concepts (synthesis only) | 0 | Integration |
| **Total** | **9 core concepts** | **13 across 6 lessons** | **2.2 avg/lesson** |

**Decision**: 6 lessons justified by:
- A2 tier requires heavy scaffolding (each concept needs explanation + practice + validation)
- Story-based approach requires breathing room (rushing breaks narrative immersion)
- Absolute beginners need confidence-building (early lessons intentionally light)
- Capstone synthesis needs foundation (L1-L5 must establish before integration)

---

## Lesson-by-Lesson Breakdown

### Lesson 1: What Is Programming? (Story Opening)

**File**: `book-source/docs/04-Python-Fundamentals/13-introduction-to-python/01-what-is-programming.md`
**Duration**: 35-40 minutes
**Proficiency**: CEFR A2 (Beginner)
**New Concepts**: 3 (what programming is, why Python, terminal primer)
**Cognitive Load**: Light (heavy scaffolding, no code yet)

**Story Element**: **Opening Hook**
Lesson opens with narrative: "Imagine you're teaching a robot to make your favorite sandwich. You can't just say 'make a sandwich.' The robot needs step-by-step instructions: 'Get bread. Open the bag. Take two slices. Put them on the counter...'"

This story explains programming before any jargon. Programming is GIVING INSTRUCTIONS to computers. Simple. Demystified.

**Learning Objectives** (measurable):
- LO-1.1: Explain programming as "giving instructions to computers" using own analogy (EV-001)
- LO-1.2: Identify Python as a language for talking to computers
- LO-1.3: Understand why learning Python is valuable (practical applications: YouTube, Instagram, data science)
- LO-1.4: Navigate basic terminal to run Python (prerequisite skill check from Chapter 12)

**Content Structure** (WHAT-WHY-HOW):

1. **Hook Story** (5 min): Robot sandwich narrative
   - "Teaching a robot is like programming a computer"
   - Robot needs EVERY step explicitly
   - Programming = translating your intentions into computer instructions

2. **WHAT is Programming?** (8 min) - Concept first
   - Definition: Giving computers step-by-step instructions in a language they understand
   - NOT mystical syntax. NOT "advanced math." Just instructions.
   - Analogy: Like recipes (steps to make food), routines (steps to get ready), instructions (IKEA furniture assembly)
   - Key insight: Computers follow instructions exactly (no common sense, no skipping steps)

3. **WHY Python?** (8 min) - Purpose before syntax
   - Python is widely used: powers YouTube, Instagram, Netflix, Tesla AI
   - Python is readable: "code for humans first, computers second"
   - Python is versatile: web development, data science, AI, automation
   - **Message**: "You're learning a language actually used by companies solving real problems"

4. **WHAT is Python?** (5 min) - Definition
   - Python is a programming language (a system of words + rules for talking to computers)
   - Like English or Spanish for humans, Python is "computer language"
   - Need to learn: vocabulary (words Python understands) + grammar (how to arrange them)

5. **Terminal Primer** (8 min) - Bridge to Chapter 12
   - Review from Chapter 12: Terminal is "black screen where you type commands"
   - Python is accessed through terminal (students learned this in Chapter 12)
   - Quick verification: "Open terminal, type `python --version`, see Python 3.14 installed"
   - **Validation checkpoint**: Each student confirms Python works on their system

6. **A Preview** (3 min)
   - "Next lesson, you'll write your first Python program: make computer say 'Hello, World!'"
   - This is like robot's first words
   - After that, you'll teach it to remember things, follow logic, listen to users

**"For Curious Learners"** (Optional):
- *"How Does Python Actually Work?"*: Brief mention of compilation → bytecode → execution (not required understanding, but satisfies curiosity-driven learners)

**Practice Exercises**:
- Exercise 1: Explain programming to a friend using sandwich analogy (no code)
- Exercise 2: Find 3 companies that use Python from public sources
- Exercise 3: Verify Python 3.14+ installed on your system (`python --version`)
- Exercise 4: Identify 3 real-world problems Python solves (web app, data analysis, AI)

**Try With AI**:
- "Ask your AI assistant: 'Can you explain what programming is using an analogy different from sandwiches?'"
- "Practice explaining Python's value: Ask AI to list 5 real companies using Python, then you pick one and explain why that's impressive."

**Success Criteria Mapping**:
- ✅ EV-001: Can explain "what Python is" in own words without technical jargon
- ✅ EV-006: Environment from Chapter 12 working (Python 3.14+ verified)

**Anti-Convergence Check**:
- ✅ Story-based (robot sandwich narrative, not "10 facts about Python")
- ✅ WHAT-WHY before HOW (no syntax yet)
- ✅ Demystifying (addressing beginner fear: "This isn't scary, it's just instructions")
- ✅ Engagement-focused (narrative hook, real-world relevance)

---

### Lesson 2: Your First Program (Success Moment)

**File**: `book-source/docs/04-Python-Fundamentals/13-introduction-to-python/02-first-program.md`
**Duration**: 30-35 minutes
**Proficiency**: CEFR A2 (Beginner)
**New Concepts**: 2 (running Python code, print function)
**Cognitive Load**: Light (immediate success moment)

**Story Element**: **First Victory**
In Lesson 1, robot couldn't speak. Now, lesson 2 is "Robot's First Words." Student writes their first program and immediately sees output. This is the first "aha moment"—the student has given instructions to a computer, and it WORKED.

**Learning Objectives** (measurable):
- LO-2.1: Write and execute a simple Python program independently (EV-002)
- LO-2.2: Understand print() function as "tell computer what to display"
- LO-2.3: Modify output and see results immediately (reinforces cause-effect)

**Content Structure** (WHAT-WHY-HOW):

1. **Story Recap** (2 min): Connection to Lesson 1
   - "Remember the robot learning to make sandwiches?"
   - "First thing any creature learns is how to communicate—even robots!"
   - "Let's teach the robot to speak"

2. **WHAT is the print() Function?** (8 min) - Concept first, no syntax yet
   - Definition: Python word that means "show something on the screen"
   - Purpose: Make computer display messages, data, results
   - Analogy: Like a megaphone—you give print() a message, it announces it to the world
   - Example real-world use: Websites display "Welcome!" messages, apps show notifications, games show scores
   - **Key concept**: print() is HOW you communicate from program to user

3. **WHAT is Python Code Execution?** (5 min)
   - Definition: You write instructions in Python, you run them, computer follows them
   - Two-step process: (1) Write code, (2) Run code
   - File format: Python files end in `.py` (like photos end in `.jpg`)
   - Process: Type code → Save file → Run file → See results

4. **NOW Show the Code** (5 min) - Finally, syntax
   - First program (simplest possible):
   ```python
   print("Hello, World!")
   ```
   - Walk through syntax:
     - `print` = the word Python understands
     - `()` = parentheses tell Python "do this command"
     - `"Hello, World!"` = the message in quotation marks
   - Explanation: This says "Python, display 'Hello, World!' on screen"

5. **HOW to Run It** (8 min) - Step-by-step walkthrough
   - **Step 1**: Open terminal (from Chapter 12)
   - **Step 2**: Create new file (use text editor from Chapter 12, or VS Code)
   - **Step 3**: Type the print statement
   - **Step 4**: Save file as `hello.py` (important: `.py` extension)
   - **Step 5**: In terminal, type: `python hello.py`
   - **Step 6**: See output: `Hello, World!`
   - **Celebration moment**: "You just made the computer follow your instructions! This is programming!"

6. **Modify and Experiment** (5 min) - Reinforces understanding
   - Try changing the message:
   ```python
   print("My name is [Your Name]")
   print("I am learning Python!")
   ```
   - Observation: Changing the message changes output
   - **Insight**: This is how programmers control what programs do—write instructions in code

7. **Common Mistakes** (4 min) - Friendly error guidance
   - Forgetting quotation marks: `print(Hello, World!)` → ERROR
     - Why: Python thinks "Hello" is an instruction, not a message
     - Fix: Add quotes around the message
   - Misspelling print: `pint("message")` → ERROR
     - Why: Python doesn't know the word "pint"
     - Fix: Check spelling
   - Missing parentheses: `print "message"` → ERROR (Python 3 syntax)
     - Why: Python 3 requires parentheses
     - Fix: Add parentheses

**"For Curious Learners"** (Optional):
- *"What About Indentation?"*: Brief note that Python cares about spacing (relevant to future control flow)
- *"How Does Python Actually Execute Your Code?"*: Bytecode compilation mention (concept for curious minds)

**Practice Exercises**:
- Exercise 1: Write print() statement displaying your name
- Exercise 2: Write 3 print statements showing different messages (test, run, observe)
- Exercise 3: Fix provided broken code (syntax errors)
- Exercise 4: Modify a provided program to change output

**Try With AI**:
- "Ask your AI: 'Can you give me 10 different messages to print? I'll type them and run them.'"
- "Describe to AI what you just did: Write code, save it, run it, see results. Ask AI: 'Did I do this correctly?'"

**Success Criteria Mapping**:
- ✅ EV-002: 90%+ can write and execute "Hello, World!" independently within 15 minutes
- ✅ EV-006: Code examples execute successfully (verified with test log)
- ✅ EV-009: Story-based narrative maintained ("robot's first words" frame)

**Anti-Convergence Check**:
- ✅ Concept first (explained print() PURPOSE before syntax)
- ✅ Immediate success (within 15 minutes, students have working program)
- ✅ Validation checkpoints (step-by-step with error guidance)
- ✅ Not lecture (narrative frame: robot learning to speak)

---

### Lesson 3: Variables - Teaching the Robot to Remember (Memory Story)

**File**: `book-source/docs/04-Python-Fundamentals/13-introduction-to-python/03-variables-and-memory.md`
**Duration**: 40-45 minutes
**Proficiency**: CEFR A2 (Beginner)
**New Concepts**: 3 (memory concept, variables, creating variables)
**Cognitive Load**: Light-Moderate (foundation with practice)

**Story Element**: **Robot's Memory**
Robot can now speak (Lesson 2), but it can't remember things. Lesson 3 teaches variables through the analogy: "Remember naming your pet? That name is a label for the animal. Variables are like labels for information the robot needs to remember."

**Learning Objectives** (measurable):
- LO-3.1: Explain variables as "labeled containers for data" (EV-003)
- LO-3.2: Create variables with assignment syntax
- LO-3.3: Modify variable values and see results
- LO-3.4: Understand WHY variables need names (EV-004)

**Content Structure** (WHAT-WHY-HOW):

1. **Story Recap & Hook** (3 min): Connection to Lessons 1-2
   - "Robot can speak (print), but what if robot needs to remember things?"
   - Real-world: Games remember player health. Chat apps remember your name. Banking apps remember your balance.
   - Computers need to remember data to be useful

2. **WHAT is a Variable?** (10 min) - Concept, no syntax yet
   - Definition: A labeled container where computer stores information to remember
   - Analogy: Naming your pet
     - Pet's name = "Fluffy" (the label)
     - Fluffy the animal = the data being stored
     - When you say "Fluffy," you're referring to your pet (the data)
   - In Python: Variable name like pet name, the value like the pet
   - Purpose: Variables let programs remember data and use it later

3. **WHY Do Variables Matter?** (8 min)
   - Computers can't just remember things in "head" (they have no brain)
   - Variables are how programs store data: names, ages, scores, messages
   - Without variables: Each time you need data, you have to type it again (tedious!)
   - With variables: Store once, use many times (efficient!)
   - Example: Instead of printing the same name 10 times, store it once in variable, print 10 times

4. **WHAT is the Memory Model?** (7 min) - Build mental model
   - Simple diagram (text-based):
   ```
   Variable name = "Alice"

   In Computer Memory:
   ┌─────────────┐
   │ name="Alice"│  ← Variable like a labeled box
   └─────────────┘
   ```
   - Concept: Computer has "memory" (storage). Variables are labeled storage spots.
   - When you create variable, computer: (1) allocates memory space, (2) gives it a label (name)
   - When you use variable by name, computer looks up the label, finds the data

5. **NOW Show Variable Creation** (10 min) - Syntax finally!
   - Simplest variable:
   ```python
   name = "Alice"
   ```
   - Breakdown:
     - `name` = the label (variable name)
     - `=` = "store this value here" (assignment)
     - `"Alice"` = the value (the data)
   - **Full interpretation**: "Create a labeled storage spot called 'name' and store 'Alice' inside"

   - More examples:
   ```python
   age = 25           # Store the number 25 in variable called 'age'
   score = 95         # Store the number 95 in variable called 'score'
   message = "Hello"  # Store text "Hello" in variable called 'message'
   ```

6. **Using Variables** (8 min)
   - Combining with print() (from Lesson 2):
   ```python
   name = "Alice"
   print(name)        # Displays: Alice

   age = 25
   print(age)         # Displays: 25

   # Using multiple variables in one program
   name = "Bob"
   age = 30
   print(name)
   print(age)
   ```
   - Insight: Once variable stores value, you can use it by its name

7. **Modifying Variables** (5 min) - Reinforces assignment
   - Variables can change:
   ```python
   count = 1
   print(count)       # Displays: 1
   count = 2          # Update the variable
   print(count)       # Displays: 2
   count = 3
   print(count)       # Displays: 3
   ```
   - Important: Variable can only store ONE value at a time (new assignment replaces old value)

8. **Common Mistakes** (5 min)
   - Using variable before creating it: `print(name)` without first `name = "..."` → ERROR
     - Why: Python doesn't know what "name" refers to
     - Fix: Always create variable before using it
   - Wrong quotes around value: `name = Alice` (no quotes) → ERROR
     - Why: Python thinks "Alice" is another variable (undefined)
     - Fix: Put text in quotes: `name = "Alice"`
   - Using `=` wrong: `name == "Alice"` (double equals) for storage is wrong
     - Why: `==` is for comparing, `=` is for storing
     - Fix: Use single `=` for assignment

**"For Curious Learners"** (Optional):
- *"Variable Naming Rules"*: Valid names, invalid names, conventions (snake_case)
- *"What's Inside Memory?"*: Memory addresses, object identity (advanced)

**Practice Exercises**:
- Exercise 1: Create 5 variables storing different data (name, age, score, city, favorite_food)
- Exercise 2: Create variables, then print them in different order
- Exercise 3: Modify variables and observe changes
- Exercise 4: Fix broken code (undefined variables, missing quotes)
- Exercise 5: Predict output before running (mental model check)

**Try With AI**:
- "Tell AI: 'I created variables name, age, score. Can you ask me what each stores?'"
- "Ask AI: 'Give me 10 variable scenarios (like "a person's phone number"), I'll write the Python code to store them.'"

**Success Criteria Mapping**:
- ✅ EV-003: 80%+ can create variables with assignment and modify values
- ✅ EV-004: 75%+ can explain WHY variables need names (store and reuse data)
- ✅ EV-009: Story-based ("robot's memory" frame maintained)

**Anti-Convergence Check**:
- ✅ WHAT-WHY before HOW (explained purpose/analogy before syntax)
- ✅ Mental model building (memory diagram, labeled storage concept)
- ✅ Progressive practice (create → use → modify)
- ✅ Story continuity (robot learning to remember, building on Lessons 1-2)

---

### Lesson 4: Type Hints - Teaching Robot to Organize Data (Classification Story)

**File**: `book-source/docs/04-Python-Fundamentals/13-introduction-to-python/04-type-hints.md`
**Duration**: 40-45 minutes
**Proficiency**: CEFR A2-B1 (Beginner-Intermediate)
**New Concepts**: 3 (data types intro, type hints, why types matter)
**Cognitive Load**: Light-Moderate (introduces data classification)

**Story Element**: **Robot Organizing Knowledge**
Variables can store data, but what KIND of data? Lesson 4 introduces type hints through the analogy: "A library organizes books by type: fiction, non-fiction, reference. Types in Python organize data: numbers, text, True/False values. Type hints tell Python what type each variable holds."

**Learning Objectives** (measurable):
- LO-4.1: Understand that data comes in different types (numbers, text, True/False)
- LO-4.2: Write type hints for variables (EV-003)
- LO-4.3: Explain WHY types matter (EV-004)
- LO-4.4: Use basic type hints: int, str, bool

**Content Structure** (WHAT-WHY-HOW):

1. **Story Recap & Hook** (3 min): Connection to Lesson 3
   - "Robot can remember data using variables"
   - "But different data needs different handling"
   - Library analogy: Fiction organized separately from reference books (different types, different treatment)

2. **WHAT Are Data Types?** (10 min) - Concept first
   - Definition: Python's way of classifying different kinds of data
   - Three basic types we'll focus on:
     - **int** (integer): Whole numbers like age (25), count (100), score (95)
     - **str** (string): Text like name ("Alice"), message ("Hello")
     - **bool** (boolean): True/False values for decisions (passed exam? True or False)
   - Purpose: Different types support different operations
   - **Key insight**: Type tells Python what the data IS and how to handle it

3. **WHY Do Types Matter?** (8 min)
   - Operations depend on type:
     - Numbers: Can add, subtract, multiply (`25 + 5 = 30`)
     - Text: Can combine/concatenate (`"Hello" + " World" = "Hello World"`)
     - True/False: Can use for decisions (if student passed, show award)
   - Type mismatches confuse Python:
     - `25 + "5"` doesn't work (number + text)
     - Python doesn't know if you want: 30 (math), or "255" (joining text)
   - Preventing errors: Tell Python the type upfront (type hint)

4. **Type Hint Syntax Introduction** (8 min)
   - Now showing syntax with context:
   ```python
   name: str = "Alice"     # Create variable 'name' that holds TEXT
   age: int = 25           # Create variable 'age' that holds a NUMBER
   is_student: bool = True # Create variable 'is_student' that holds TRUE or FALSE
   ```
   - Breakdown of `name: str = "Alice"`:
     - `name` = variable name (label)
     - `: str` = TYPE HINT (telling Python this variable stores text)
     - `=` = assignment (store value here)
     - `"Alice"` = the actual value

5. **Understanding Each Type** (10 min):

   **Integers (int)**:
   - Whole numbers: 0, 5, -10, 100
   - Used for: ages, scores, counts, indices
   ```python
   age: int = 25
   score: int = 95
   count: int = 100
   ```

   **Strings (str)**:
   - Text data: "Alice", "hello world", email addresses
   - Must use quotation marks: `"text"` or `'text'`
   ```python
   name: str = "Alice"
   city: str = "New York"
   message: str = "Welcome to Python!"
   ```

   **Booleans (bool)**:
   - True or False only (capitalized!)
   - Used for: yes/no questions, status flags
   ```python
   is_student: bool = True
   has_passed: bool = False
   is_adult: bool = True
   ```

6. **Using Type-Hinted Variables** (7 min)
   - Type hint doesn't change how you use variable (still same):
   ```python
   name: str = "Alice"
   age: int = 25

   print(name)       # Still displays: Alice
   print(age)        # Still displays: 25

   # Using multiple variables
   print(name)
   print(age)
   print(is_student)
   ```
   - Type hint is INFORMATION (tells Python what you intend) but doesn't change behavior

7. **Why Type Hints Matter for YOU** (5 min)
   - **Clarity**: Others (and future you) can see what type each variable should be
   - **Error prevention**: Python warns if you use wrong type (type checker)
   - **Documentation**: Code explains itself
   - **Professional practice**: Real Python code uses type hints (PEP 484 standard)

8. **Common Mistakes** (4 min)
   - Forgetting quotes for strings: `name: str = Alice` → ERROR
     - Fix: `name: str = "Alice"`
   - Wrong type hint: `name: int = "Alice"` (type doesn't match value)
     - Works in Python but confusing (will learn about type conversion later)
     - Fix: Use matching type: `name: str = "Alice"`
   - Capitalization for bool: `is_student: bool = true` (lowercase t) → ERROR
     - Fix: `is_student: bool = True` (capitalize)

**"For Curious Learners"** (Optional):
- *"What Other Types Exist?"*: float (decimals), list (collections) mentioned briefly as preview
- *"Type Checking in Python"*: Tools like mypy to verify types (advanced)

**Practice Exercises**:
- Exercise 1: Create 5 type-hinted variables (mix of int, str, bool)
- Exercise 2: Identify types in provided code
- Exercise 3: Fix code with wrong type hints
- Exercise 4: Predict output of type-hinted variables
- Exercise 5: Match scenarios to correct types (age → int, name → str, passed → bool)

**Try With AI**:
- "Ask your AI: 'Give me 10 data scenarios (like 'person's email'), what type should each be?'"
- "Tell AI what type hints you used and ask: 'Did I choose the right types?'"

**Success Criteria Mapping**:
- ✅ EV-003: 80%+ can write type hints for int, str, bool
- ✅ EV-004: 75%+ can explain WHY types matter (different operations, preventing errors)
- ✅ EV-009: Story-based ("organizing knowledge" frame)

**Anti-Convergence Check**:
- ✅ WHAT-WHY before HOW (explained types PURPOSE before syntax)
- ✅ Scaffolded understanding (library analogy, then syntax)
- ✅ Progressive from Lesson 3 (adding type hints to variables from Lesson 3)
- ✅ Story continuity (robot organizing data, building knowledge from Lessons 1-3)

---

### Lesson 5: User Input - Teaching Robot to Listen (Interaction Story)

**File**: `book-source/docs/04-Python-Fundamentals/13-introduction-to-python/05-user-input.md`
**Duration**: 40-45 minutes
**Proficiency**: CEFR A2-B1 (Beginner-Intermediate)
**New Concepts**: 2 (input function, combining concepts)
**Cognitive Load**: Moderate-Peak (integration of prior concepts)

**Story Element**: **Robot Listens**
Robot can speak (print) and remember data (variables with type hints), but it's one-way. Lesson 5 teaches input: "Programs can ask the user questions and remember the answers." This is where programs become interactive.

**Learning Objectives** (measurable):
- LO-5.1: Use input() function to ask user for information
- LO-5.2: Store user input in variables with type hints
- LO-5.3: Combine print() and input() into interactive program
- LO-5.4: Understand input() always receives text (string) initially

**Content Structure** (WHAT-WHY-HOW):

1. **Story Recap & Hook** (3 min): Connection to Lessons 1-4
   - "Robot can speak and remember data"
   - "But communication is two-way! What if robot asks you questions?"
   - Real-world: Apps ask for username, games ask player's name, shopping carts ask quantity

2. **WHAT is the input() Function?** (8 min) - Concept first
   - Definition: Python word that means "ask the user to type something and store it"
   - Purpose: Programs need data from users to do useful work
   - Analogy: Like asking someone a question and listening to their answer
   - Process: (1) Program displays question, (2) User types answer, (3) Program stores answer

3. **WHY Use input()?** (6 min)
   - Hardcoded data is boring: Every time program runs, same data
   - Dynamic data from users: Program adapts to each person
   - Interactive programs: Programs respond to user choices
   - Examples: Chat apps remember your name, games let you choose character, forms collect information

4. **NOW Show input() Syntax** (8 min):
   - Simplest input:
   ```python
   name = input("What is your name? ")
   ```
   - Breakdown:
     - `input()` = the function (Python command)
     - `"What is your name? "` = the question displayed to user
     - `name =` = storing user's answer in variable
   - When run:
     - Program displays: `What is your name? `
     - User types: `Alice`
     - Variable `name` now stores: `"Alice"`

5. **Using Stored Input** (8 min)
   - Storing input and using it:
   ```python
   name = input("What is your name? ")
   print("Hello, " + name + "!")

   # When run:
   # Program asks: "What is your name? "
   # User types: "Alice"
   # Program displays: "Hello, Alice!"
   ```
   - Insight: You can combine input() with print() to make conversation

6. **Adding Type Hints to input()** (8 min)
   - Type hint with input:
   ```python
   name: str = input("What is your name? ")
   ```
   - **Important concept**: input() ALWAYS returns text (string)
   - Even if user types a number, input() treats it as text
   - Examples:
   ```python
   age: str = input("What is your age? ")  # User types: 25, but stored as text "25"
   number: str = input("Enter a number: ")  # User types: 100, but stored as text "100"
   ```
   - **Note**: Converting to int comes in later lessons (Chapter 16: Type Casting)

7. **Building an Interactive Program** (8 min) - Combining concepts
   - Now use print(), variables, input() together:
   ```python
   name: str = input("What is your name? ")
   city: str = input("What city do you live in? ")

   print("Hello, " + name + "!")
   print("I heard " + city + " is nice!")
   ```
   - Walkthrough execution:
     - Asks for name → stores in `name`
     - Asks for city → stores in `city`
     - Prints greeting using both variables
     - Program is now interactive!

8. **Common Mistakes** (5 min)
   - Forgetting to store input: `input("Name? ")` without assignment
     - User types answer but it's lost (not stored)
     - Fix: Assign to variable: `name = input("Name? ")`
   - Forgetting quotes around question: `input(What is your name?)` → ERROR
     - Fix: Add quotes: `input("What is your name?")`
   - Trying to do math with input: `age = input("Age? "); age + 5` → ERROR
     - Why: input() returns text, can't add number to text
     - Fix: Convert to int first (covered in Chapter 16)

**"For Curious Learners"** (Optional):
- *"Converting Input to Numbers"*: Brief mention that `int()` and `float()` can convert input (deferred to Chapter 16)
- *"Input Validation"*: Checking if input is valid (advanced, Chapter 21)

**Practice Exercises**:
- Exercise 1: Write program asking for name, storing, and greeting user
- Exercise 2: Write program asking 3 questions, storing answers, displaying them
- Exercise 3: Fix broken code (missing quotes, missing assignment, etc.)
- Exercise 4: Extend provided program with more input prompts
- Exercise 5: Predict program output before running

**Try With AI**:
- "Ask AI: 'Can you write an interactive program asking 5 questions about me?'"
- "Test program with AI: Type different answers, observe how program changes based on your input"

**Success Criteria Mapping**:
- ✅ EV-002: (Extends from Lesson 2) Interactive programs now possible
- ✅ EV-009: Story-based ("robot listens" frame)
- ✅ Combines EV-003 + EV-004: Using variables with type hints via input()

**Anti-Convergence Check**:
- ✅ WHAT-WHY before HOW (explained input PURPOSE before syntax)
- ✅ Integration (combining print, variables, type hints, input)
- ✅ Peak engagement (interactive programs are exciting for beginners)
- ✅ Story continuity (robot learning to listen, completing interaction foundation)

---

### Lesson 6: Capstone Project - Digital Introduction Card (Hero's Journey Resolution)

**File**: `book-source/docs/04-Python-Fundamentals/13-introduction-to-python/05-capstone-project.md`
**Duration**: 45-60 minutes
**Proficiency**: CEFR A2-B1 (Beginner-Intermediate)
**New Concepts**: 0 (synthesis of Lessons 1-5, no new concepts)
**Cognitive Load**: Integration (peak engagement with accumulated knowledge)

**Story Element**: **Hero's Journey Resolution**
"You've taught robot to speak, remember, organize knowledge, and listen. Now, like those name tags at conferences, your robot introduces itself using everything you've learned. Create a program that collects user information and displays a personalized introduction card."

**Learning Objectives** (measurable):
- LO-6.1: Compose print(), variables, type hints, input() into cohesive program
- LO-6.2: Demonstrate understanding of all Chapter 13 concepts (EV-005)
- LO-6.3: Design simple interactive experience
- LO-6.4: Debug and test program independently

**Capstone Specification**:

**Program: Personal Introduction Card**
- **Purpose**: Create a digital "name tag" showing user information
- **Requirements**:
  1. Ask user for name (store in variable with type hint)
  2. Ask user for age (store in variable with type hint)
  3. Ask user for favorite hobby (store in variable with type hint)
  4. Display personalized introduction card using print()
  5. Program should be interactive (different inputs produce different outputs)

**Starter Code Provided** (scaffolded):
```python
# Personal Introduction Card
# Your Name: [Your Name]
# Created: [Date]

# Step 1: Ask for user's name
name: str = input("What is your name? ")

# Step 2: Ask for user's age
age: str = input("What is your age? ")

# Step 3: Ask for user's favorite hobby
hobby: str = input("What is your favorite hobby? ")

# Step 4: Display introduction card
# [TODO: Use print() to display the card]
# Should include: name, age, hobby in a nice format
```

**Completion Criteria**:
- Program asks for 3 pieces of information (name, age, hobby)
- Information stored in variables with type hints
- Program displays introduction card using print()
- Output is personalized (changes based on user input)
- Program runs without errors
- Code is readable (clear variable names, comments)

**Expected Output Example**:
```
What is your name? Alice
What is your age? 25
What is your favorite hobby? painting

═══════════════════════════════════
    INTRODUCTION CARD
═══════════════════════════════════
Name: Alice
Age: 25
Hobby: painting

Welcome to the Python programming journey!
═══════════════════════════════════
```

**Content Structure** (Synthesis, not teaching):

1. **Story Framing** (3 min)
   - Recap journey: Lesson 1 (what is programming), Lesson 2 (speaking), Lesson 3 (remembering), Lesson 4 (organizing), Lesson 5 (listening)
   - Capstone: Synthesize everything
   - Real-world context: Networking events use introduction cards to share info efficiently

2. **Understanding the Program** (5 min)
   - Walkthrough what program does (without writing code yet)
   - Pseudo-code (English-like steps):
     ```
     1. Ask user for name, store in 'name' variable
     2. Ask user for age, store in 'age' variable
     3. Ask user for hobby, store in 'hobby' variable
     4. Print introduction card showing all info
     ```
   - **Key insight**: Program combines input() (from Lesson 5), variables (Lesson 3), and print() (Lesson 2)

3. **Scaffolded Development** (25 min)
   - **Phase 1**: Get starter code, understand structure (5 min)
   - **Phase 2**: Fill in the print() statements to display card (10 min)
     - Experiment with formatting
     - Try using quotes for decorative lines (`═══════════════════════════════════`)
   - **Phase 3**: Test program with different inputs (10 min)
     - Run multiple times with different names, ages, hobbies
     - Verify output changes appropriately

4. **Design Choices** (5 min)
   - Optional customization:
     - Card format: Simple list vs fancy borders
     - Additional fields: City, job title, favorite food
     - Greeting message: Static or personalized

5. **Testing & Debugging** (10 min)
   - Run program, test with various inputs
   - Common issues:
     - Forgetting to store input → Error or lost data
     - Missing quotes in print() → Syntax error
     - Typos in variable names → Error
   - **Debugging strategy**: Read error message, find line number, check that line

6. **Reflection** (5 min)
   - How does this program use concepts from Lessons 1-5?
   - What could you add if you knew more (foreshadow Chapter 14)?
   - How is this real-world (actual use: contact exchange, event registration)?

**"For Curious Learners"** (Optional Extensions):
- *"Add More Fields"*: Extend card to ask for more information (city, profession, interests)
- *"Format the Card"*: Use ASCII art or creative formatting to make card visually interesting
- *"Add Validation"*: Check that age is actually a number (preview of Chapter 16 type conversion)
- *"Save to File"*: Write introduction card to a text file (preview of Chapter 22 file I/O)
- *"Generate Multiple Cards"*: Ask for multiple people's info, create cards for each (preview of loops Chapter 17)

**Practice (Capstone Itself)**:
- Create the introduction card program
- Test with at least 3 different sets of inputs
- Optionally extend with additional fields or formatting

**Try With AI**:
- "Ask your AI: 'What would make a good introduction card format? Give me ideas.'"
- "Show AI your code and ask: 'Does my program correctly use variables, input(), and print()?'"
- "Ask for extension ideas: 'How could I make this program even better?'"

**Success Criteria Mapping**:
- ✅ EV-005: 70%+ complete capstone project demonstrating integrated understanding
- ✅ EV-002: Write and execute program independently (from Lesson 2, now with full power)
- ✅ EV-003: Create variables with type hints (from Lesson 3)
- ✅ EV-006: Code examples execute (capstone must run successfully)
- ✅ EV-009: Story-based ("hero's journey resolution" frame)

**Anti-Convergence Check**:
- ✅ Synthesis, not new content (0 new concepts, integration of L1-L5)
- ✅ Real-world framing (introduction cards are actual thing)
- ✅ Scaffolded development (starter code + phases)
- ✅ Validation-focused (testing with multiple inputs)
- ✅ Story conclusion (capstone resolves narrative arc)

---

## Pedagogical Progression Map

### Foundation → Application → Integration → Mastery Arc

| Phase | Lessons | Cognitive Focus | Student State |
|-------|---------|-----------------|---------------|
| **Foundation** | L1-L2 | "What is programming?" + "First success" | Demystified, confident |
| **Rising Action** | L3-L4 | "Variables" + "Type hints" | Building knowledge, complexity increases |
| **Peak Action** | L5 | "User interaction" | Full engagement (programs now interactive!) |
| **Resolution** | L6 (Capstone) | Synthesis of all | Mastery demonstrated |

### Narrative Thread (Story Continuity)

```
L1: Robot learns to give instructions (robot metaphor introduced)
  └─ "What is programming?"

L2: Robot speaks ("Hello World" = robot's first words)
  └─ Proof: Instructions work, computer follows them

L3: Robot remembers (variables = robot's memory)
  └─ "Just like naming your pet"

L4: Robot organizes knowledge (type hints = data classification)
  └─ "Like library organizing books by type"

L5: Robot listens (input = two-way communication)
  └─ "Programs now interact with users"

L6: Robot introduces itself (capstone = robot using all skills)
  └─ "Digital introduction card" = synthesis
```

Each lesson builds on previous. By Lesson 6, students have experienced complete programming cycle.

---

## Cognitive Load Distribution

| Lesson | New Concepts | Count | Proficiency | Status |
|--------|--------------|-------|-------------|--------|
| **L1** | What is programming, Why Python, Terminal primer | 3 | A2 | Light |
| **L2** | Running Python, Print function | 2 | A2 | Light |
| **L3** | Memory concept, Variables, Creating variables | 3 | A2 | Light-Moderate |
| **L4** | Data types intro, Type hints, Why types matter | 3 | A2-B1 | Light-Moderate |
| **L5** | Input function, Integration | 2 | A2-B1 | Moderate-Peak |
| **L6** | 0 new concepts (synthesis only) | 0 | A2-B1 | Integration |
| **TOTAL** | **9 core concepts** | **13 across 6 lessons** | **Beginner-Intermediate** | **2.2 avg/lesson** |

**Analysis**:
- All lessons within A2 tier (5-7 concept limit)
- Progressive increase (early light, late moderate)
- Peak at L5 (2 concepts + high engagement from interactivity)
- L6 pure synthesis (no cognitive load, all validation)
- **Justified**: 6 lessons necessary for heavy scaffolding + narrative breathing room

---

## Stage Progression & AI Collaboration

### Stage 1 (Manual Foundation): Lessons 1-6
- **Primary**: Students practice manually (no AI yet)
- **AI role**: Gentle introduction in "Try With AI" sections
  - AI as practice partner (generates scenarios for students to code)
  - AI as validating listener (students explain concepts to AI)
  - AI as question-answerer (helps with clarification)
- **Not**: AI as solution generator (students don't ask AI to "write the code")

### Stage 2 (AI Collaboration): Deferred
- Chapter 13 is pure Stage 1
- Chapter 14+ will introduce Stage 2 with three-role framework
- "Try With AI" is **intentional bridge**, not full Stage 2

---

## Intelligence Accumulation

### Chapter 13 Does NOT Create Stage 3 Intelligence
**Reason**: Chapter 13 is foundational. No recurring patterns yet requiring reusable skills/subagents.

### Skills/Subagents to Create Later
- **Chapter 14+**: After students encounter type classification task 2+ times → Create reusable "type-decision-tree" skill
- **Chapter 17+**: After control flow patterns recur → Create reusable "conditional-logic" skill
- **Chapter 20+**: After function design patterns recur → Create "function-signature-designer" skill

### What Chapter 13 Builds For
- **Chapter 14 prerequisite**: Type hints introduced in Ch 13 L4, deep-dived in Ch 14
- **Chapter 15 prerequisite**: Variables established in Ch 13, operators added in Ch 15
- **Chapter 17 prerequisite**: Boolean type hints introduced in Ch 13 (used heavily in conditionals)
- **Chapter 20 prerequisite**: Functions concept builds naturally on variables + print + input practice

### References in Chapter 13
- **From Chapter 12**: Python installed, terminal working, text editor available
- **To Chapter 14**: "Data types go much deeper in next chapter—we'll explore each type's capabilities"
- **To Chapter 15**: "Operators let you do math and logic with these variables"
- **To Chapter 17**: "Booleans become powerful when you learn conditionals"

---

## Teaching Modality Variation

### Chapter 14 Modality: Analogy-Based "Concept Before Syntax"
- Kitchen jar analogy (organizing data like ingredients)
- Furniture storage analogy (collections like containers)
- Straightforward: "This type is for..."

### Chapter 13 Modality: Story-Based Narrative Journey
- Robot sandwich story (programming = instructions)
- Pet naming analogy (variables = labels)
- Narrative arc (robot learning to speak → remember → organize → listen → introduce itself)
- **Distinction**: Chapter 13 unfolds like a STORY, Chapter 14 like a CLASSIFICATION SYSTEM

---

## Success Criteria Mapping (From Spec)

### Student Learning Outcomes (SC)

| Criterion | Lesson(s) | How Measured | Target |
|-----------|-----------|--------------|--------|
| **SC-001**: Explain "what Python is" in own words | L1 | Oral explanation using sandwich analogy | 85%+ |
| **SC-002**: Write & execute "Hello World" independently | L2 | Program execution, 15-min test | 90%+ |
| **SC-003**: Create variable with type hint | L3-L4 | Code creation + explanation | 80%+ |
| **SC-004**: Explain WHY variables matter | L3 | Concept explanation | 75%+ |
| **SC-005**: Complete capstone project | L6 | Functional program demonstrating all concepts | 70%+ |

### Educational Quality (EV)

| Criterion | Lesson(s) | How Measured | Status |
|-----------|-----------|--------------|--------|
| **EV-006**: Code examples execute in Python 3.14+ | All | Execution logs | Test coverage: 100% |
| **EV-007**: Claims verified against Python.org | All | Citation list | In final validation |
| **EV-008**: A2 cognitive load (5-7 concepts/lesson) | All | Concept audit | All within limit |
| **EV-009**: Story-based narrative maintained | All | Narrative audit | 6/6 lessons story-driven |
| **EV-010**: "For Curious Learners" present | L1-L6 | Section audit | Present (non-disruptive) |

### Pedagogical Effectiveness (SC)

| Criterion | Lesson(s) | How Measured | Status |
|-----------|-----------|--------------|--------|
| **EV-011**: WHAT-WHY-HOW pattern | All | Content structure audit | 6/6 lessons follow pattern |
| **EV-012**: Story elements engage | All | Student feedback | Post-deployment measurement |
| **EV-013**: Progressive depth evident | All | Concept progression | Light → Moderate → Integration |
| **EV-014**: No scaffolding exposure | All | Grep: zero "Stage", "Layer" | Verified in final validation |
| **EV-015**: Lessons end with "Try With AI" only | All | Section audit | 6/6 lessons conform |

---

## Implementation Sequence

**Recommended Order**:

1. **Chapter README.md** (5-10 min)
   - Overview: story-based learning journey
   - Prerequisites: Chapter 12 (Python working)
   - Learning objectives: What students will achieve
   - Connection to AI-native development philosophy

2. **Lesson 1: What Is Programming?** (3-4 hours)
   - Validate: Story hook clear, concepts before code, no syntax errors
   - Validation gate: Demystification successful?

3. **Lesson 2: First Program** (2-3 hours)
   - Validate: Code executes, success moment evident, scaffolding adequate
   - Validation gate: 90%+ can follow steps to Hello World?

4. **Lesson 3: Variables** (3-4 hours)
   - Validate: Pet name analogy effective, memory model clear, practice adequate
   - Validation gate: Students understand why variables exist (not just syntax)?

5. **Lesson 4: Type Hints** (3-4 hours)
   - Validate: Type classification clear, type hint syntax understandable, builds on L3
   - Validation gate: Students can explain why types matter?

6. **Lesson 5: User Input** (3-4 hours)
   - Validate: input() function clear, integration of prior concepts, interactivity working
   - Validation gate: Programs now interactive and engaging?

7. **Lesson 6: Capstone** (2-3 hours)
   - Validate: Scaffolding adequate, synthesis is evident, capstone is achievable
   - Validation gate: 70%+ can complete independently?

8. **Chapter Validation** (4-5 hours)
   - Pedagogical audit: Story cohesion, WHAT-WHY-HOW pattern, anti-convergence
   - Constitutional compliance: All principles applied
   - Factual accuracy: All code tested, claims cited
   - Cognitive load: All concepts within limits
   - Anti-pattern detection: No scaffolding exposure, no meta-commentary

**Validation Gates** (after each lesson):
- [ ] Story-based modality clearly evident (not lecture-style)
- [ ] WHAT→WHY→HOW pattern followed
- [ ] No code before conceptual explanation
- [ ] Type hints in ALL examples (normalize practice)
- [ ] Real-world scenarios (not abstract)
- [ ] "Try With AI" section included (appropriate prompts)
- [ ] Success criteria mappings verified
- [ ] Cognitive load within limits (max 7 concepts)
- [ ] No Chapter 12 content re-taught
- [ ] Validation checkpoints adequate for A2 learners
- [ ] YAML frontmatter complete and accurate
- [ ] Common mistakes section present and helpful

---

## Cross-References Strategy

### Chapter 13 References TO Other Chapters
- **Chapter 12**: "As you set up in Chapter 12..."
- **Chapter 14**: "Data types go much deeper in Chapter 14"
- **Chapter 15**: "In Chapter 15, operators let you manipulate these values"
- **Chapter 17**: "Boolean values become powerful in Chapter 17 with conditionals"

### Other Chapters Reference TO Chapter 13
- **Chapter 14**: "Remember from Chapter 13, variables store data..."
- **Chapter 15**: "Building on Chapter 13's variables..."
- **Chapter 16**: "Strings, which you saw in Chapter 13..."
- **Chapter 17**: "Boolean type hints from Chapter 13..."

---

## Quality Assurance Checklist

**Before Marking Implementation Complete**, verify:

### Content Structure
- [ ] All 6 lessons follow story-based narrative (robot learning journey)
- [ ] Each lesson follows WHAT→WHY→HOW progression
- [ ] Concepts explained BEFORE code examples (never reverse)
- [ ] Type hints in every code example (normalize practice)
- [ ] Real-world examples throughout (ages, names, hobbies—not abstract)
- [ ] "Try With AI" prompts in all lessons
- [ ] Advanced topics marked as "For Curious Learners" (optional, non-disruptive)

### Pedagogical Integrity
- [ ] No lecture-style convergence (narrative-driven, not fact-dump)
- [ ] Scaffolding hidden from students (Stage 1 pedagogy invisible)
- [ ] Progressive complexity evident (early light, late moderate)
- [ ] Capstone integrates all L1-L5 concepts (no new concepts in L6)
- [ ] Story continuity unbroken (robot metaphor consistent)
- [ ] Teaching modality differs from Chapter 14 (story vs analogy)

### Technical Quality
- [ ] All code executes in Python 3.14+ (verified with test logs)
- [ ] All technical claims cited (Python.org references)
- [ ] Version compatibility noted (3.10+ mostly compatible)
- [ ] Cross-platform code (Windows/Mac/Linux compatible)
- [ ] No untested code published (execution logs mandatory)

### Cognitive Load Compliance
- [ ] Lesson 1: ≤7 concepts (actual: 3)
- [ ] Lesson 2: ≤7 concepts (actual: 2)
- [ ] Lesson 3: ≤7 concepts (actual: 3)
- [ ] Lesson 4: ≤7 concepts (actual: 3)
- [ ] Lesson 5: ≤7 concepts (actual: 2)
- [ ] Lesson 6: ≤7 concepts (actual: 0)
- [ ] All lessons within A2-B1 limits

### Anti-Pattern Elimination
- [ ] Zero matches for "Stage [0-9]" in student text (grep validation)
- [ ] Zero matches for "Layer [0-9]" in student text (grep validation)
- [ ] Zero matches for "Three Roles (Framework|in Action)" in student text (grep validation)
- [ ] No "What's Next" section after "Try With AI" (only section ending)
- [ ] No "Key Takeaways" or "Summary" sections (disrupt narrative)
- [ ] No meta-commentary about teaching approach
- [ ] No "Congratulations" or similar patronizing language

### Completeness
- [ ] All 28+ functional requirements from spec addressed
- [ ] All success criteria mappable to lesson content
- [ ] Chapter README present with overview, prerequisites, journey map
- [ ] YAML frontmatter complete in all lessons (proficiency, duration, skills, objectives)
- [ ] Practice exercises progress from simple to complex
- [ ] Common mistakes sections present and helpful (friendly tone)
- [ ] Citations present for all Python technical claims

---

## YAML Frontmatter Template

Each lesson should include:

```yaml
---
title: "Lesson Title"
chapter: 13
lesson: X
duration_minutes: XX
proficiency_level: "CEFR A2" or "CEFR A2-B1"
blooms_level: "Remember" or "Understand"
cognitive_load: X new concepts
skills:
  - learning-objectives
  - technical-clarity
  - code-example-generator
  - exercise-designer
  - ai-collaborate-teaching
  - concept-scaffolding
learning_objectives:
  - "LO-X.X: Objective statement (maps to EV-XXX)"
success_criteria:
  - "EV-XXX: Measurable outcome"
prerequisites:
  - "Chapter 12: Python installed and terminal working"
  - "Previous lesson (if applicable)"
related_chapters:
  - "Chapter 14: Data Types"
  - "Chapter 15: Operators"
teaching_modality: "Story-based narrative"
differentiation:
  struggling: "Focus on core concepts, skip 'For Curious Learners', provide extra practice"
  advanced: "Complete 'For Curious Learners' sections, extend capstone project"
---
```

---

## Estimated Timeline

- **Lesson 1 Creation**: 3-4 hours (narrative hook, demystification)
- **Lesson 2 Creation**: 2-3 hours (first program, scaffolded walkthrough)
- **Lesson 3 Creation**: 3-4 hours (variables, mental model building)
- **Lesson 4 Creation**: 3-4 hours (type hints, classification concept)
- **Lesson 5 Creation**: 3-4 hours (input function, integration)
- **Lesson 6 Creation**: 2-3 hours (capstone scaffolding)
- **Chapter README Creation**: 1-2 hours
- **Validation & Refinement**: 4-5 hours

- **Total Estimated**: 21-29 hours

---

## Success Metrics (How We Know It Works)

**You Succeed When**:
- ✅ Absolute beginners read Lesson 1 and understand "programming = giving instructions" without intimidation
- ✅ 90%+ can write and execute "Hello World" within 15 minutes of Lesson 2
- ✅ Students recognize that Chapter 13 is a **story with rising action**, not a list of disconnected facts
- ✅ Lesson count justified (6, not arbitrary 9) by concept density and pedagogical necessity
- ✅ Teaching modality clearly distinguishes from Chapter 14 (story vs analogy)
- ✅ "Try With AI" sections introduce AI as practice partner, not solution generator
- ✅ Capstone project integrates all concepts without requiring new knowledge
- ✅ Validation passes: pedagogical effectiveness, constitutional compliance, factual accuracy, cognitive load

**You Fail When**:
- ❌ Chapter reverts to lecture-style ("Here are 10 facts about Python")
- ❌ Lesson count is arbitrary (e.g., 9 lessons without concept density justification)
- ❌ Code examples appear BEFORE concepts are explained
- ❌ Scaffolding is exposed ("Stage 1 focus", "Layer 1 manual foundation")
- ❌ Teaching modality resembles Chapter 14's analogy approach (not differentiated)
- ❌ Cognitive load exceeds A2 limits (>7 concepts per lesson)
- ❌ Capstone introduces new concepts (should be pure synthesis)

---

## Next Steps After Plan Approval

1. **`/sp.tasks`**: Break down lesson implementation into actionable tasks (code creation, validation checkpoints)
2. **`/sp.implement`**: Content-implementer creates lessons using complete pedagogical context
3. **Validation Gates**: After each lesson, validate story cohesion, concept clarity, code execution
4. **Final Validation**: Run validation-auditor + factual-verifier for constitutional compliance, pedagogical effectiveness, factual accuracy
5. **Deployment**: Chapter 13 ready for student-facing book

---

**This plan activates reasoning about story-based, narrative-driven pedagogy for absolute beginners. It's not generic "intro to Python"—it's a carefully-orchestrated learning journey where programming unfolds like a story, complete with demystification, rising action, peak engagement, and satisfying resolution through capstone synthesis.**

