# Feature Specification: Chapter 13 - Introduction to Python

**Feature Branch**: `016-part-4-chapter-13`
**Created**: 2025-11-08
**Status**: Draft
**Chapter**: 13 - Introduction to Python
**Part**: 4 (Python Fundamentals)
**Complexity Tier**: Beginner (A1-A2 CEFR)
**Learning Pattern**: AI-Native Learning (describe intent → explore → validate → learn from errors)

## Success Evals *(FIRST - business-goal-aligned)*

### Reader Comprehension
- **EVAL-001**: 75%+ of students can explain what Python is and why it's important for AI development (measured via chapter quiz)
- **EVAL-002**: 80%+ of students can explain why type hints matter and how they help AI collaboration (measured via reflective prompts in "Try With AI" sections)
- **EVAL-003**: 70%+ of students can distinguish between variable names, values, and types without confusion (measured via interactive exercises)

### Skill Acquisition
- **EVAL-004**: 85%+ of students successfully install Python 3.14+ and verify installation (measured via capstone completion)
- **EVAL-005**: 80%+ of students can write variables with correct type hint syntax (measured via code examples in capstone)
- **EVAL-006**: 75%+ of students can run Python programs and interpret output correctly (measured via execution exercises)

### Engagement
- **EVAL-007**: 70%+ of students complete the hands-on capstone project (measured via submission tracking)
- **EVAL-008**: 80%+ of students engage with "Try With AI" sections (measured via prompt usage in AI tools)

### Accessibility
- **EVAL-009**: Reading level maintained at Grade 7-8 (measured via automated readability tools: Flesch-Kincaid)
- **EVAL-010**: Zero use of gatekeeping language ("easy", "simply", "obvious") (measured via editorial review)
- **EVAL-011**: All code examples include beginner-friendly explanations (measured via technical review)

## Topic Summary

Chapter 13 introduces students to Python as the language of AI agents and modern software development. This chapter bridges the AI-Driven Development mindset (learned in Chapters 1-11) with hands-on Python programming, positioning Python not as "a programming language to memorize" but as "a tool for describing intent that AI agents can execute."

Students learn foundational Python concepts through AI-Native Learning: they describe what they want their code to do (using type hints and clear variable names), explore concepts with their AI companion (Claude Code or Gemini CLI), validate their understanding through interactive programs, and learn from errors by asking AI "why did this happen?" This chapter establishes the pattern students will use throughout Part 4: AI as co-reasoning partner, not coding assistant.

**Connection to AIDD Principles**: This chapter applies the AI-Driven Development methodology learned in Chapters 1-11 to learning Python. Students use prompt engineering (Chapter 10) to ask AI about Python concepts, context engineering (Chapter 11) to provide code context to AI, and AI collaboration (Chapters 5-6) to explore and validate their understanding. Type hints are introduced as "describing intent" (preparing for Specification-Driven Development in Part 5) rather than optional annotations.

## Prerequisites *(explicit list)*

**Required Knowledge** (from previous chapters):
- **Chapter 1-4**: AI-Driven Development mindset and philosophy (understanding AI as co-reasoning partner)
- **Chapter 5-6**: AI tool literacy (proficiency with Claude Code or Gemini CLI for code exploration)
- **Chapter 10**: Prompt engineering (writing clear, specific questions to AI)
- **Chapter 11**: Context engineering (providing code context to AI for better responses)
- **Chapter 12**: Python UV package manager (installed and configured)

**Technical Prerequisites**:
- Computer with Windows, macOS, or Linux
- Terminal/command line access (learned in Chapter 7: Bash Essentials)
- Text editor or IDE (VS Code, Cursor, or any code editor)
- Internet connection for Python download and AI tool access

**No Prior Programming Experience Required**: This chapter assumes zero Python knowledge and zero programming background.

## Learning Objectives *(3-5 measurable, aligned with evals)*

By the end of this chapter, students will be able to:

1. **LO-001 (Comprehension)**: Explain what Python is, why it's used for AI development, and how it differs from other programming languages (aligned with EVAL-001, EVAL-002)

2. **LO-002 (Installation)**: Install Python 3.14+ on their operating system, verify the installation using terminal commands, and troubleshoot common installation errors with AI assistance (aligned with EVAL-004)

3. **LO-003 (Type Hints)**: Write variables with type hints (int, str, float, bool) to describe intent, and explain why type hints improve AI collaboration and code clarity (aligned with EVAL-002, EVAL-005)

4. **LO-004 (Execution)**: Run Python programs using terminal commands, interpret program output, and validate that programs behave as expected (aligned with EVAL-006)

5. **LO-005 (Application)**: Build an interactive program that collects user information with typed variables, demonstrating all concepts learned in this chapter (aligned with EVAL-007)

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Understanding What Python Is (Priority: P1)

**As a** complete beginner to programming,
**I want to** understand what Python is and why it's important for AI development,
**So that** I can make an informed decision about investing time in learning Python and understand how it connects to the AI-Native Development methodology I've learned.

**Why this priority**: Foundation understanding is critical. Without knowing "what" Python is and "why" it matters, students lack motivation and context. This is the conceptual anchor that everything else builds on.

**Independent Test**: Can be fully tested by asking students to explain Python in 2-3 sentences and describe one real-world AI use case. Delivers value by building confidence and motivation.

**Acceptance Scenarios**:

1. **Given** a student with no Python knowledge, **When** they read Lesson 1, **Then** they can explain "Python is a programming language used for AI" in their own words
2. **Given** a student learning AIDD methodology, **When** they complete Lesson 1, **Then** they can connect Python to at least one AIDD principle (e.g., "Python helps me describe intent that AI executes")
3. **Given** a student exploring AI applications, **When** they review Python use cases, **Then** they can name at least 2 real-world AI applications built with Python

---

### User Story 2 - Installing and Verifying Python (Priority: P1)

**As a** student ready to start coding,
**I want to** install Python 3.14+ on my computer and verify it's working correctly,
**So that** I have a working development environment and can run Python programs.

**Why this priority**: No working environment = no hands-on learning. This is a hard blocker for all subsequent lessons. Must be P1 to enable everything else.

**Independent Test**: Can be fully tested by running `python --version` and seeing Python 3.14+ output. Delivers immediate value: working development environment.

**Acceptance Scenarios**:

1. **Given** a Windows/Mac/Linux computer, **When** I follow installation instructions with AI assistance, **Then** Python 3.14+ is installed successfully
2. **Given** Python is installed, **When** I run `python --version` in terminal, **Then** I see "Python 3.14.x" (or higher) displayed
3. **Given** an installation error occurs, **When** I ask my AI companion for help, **Then** AI provides step-by-step troubleshooting guidance
4. **Given** Python is verified, **When** I run `python -c "print('Hello')"`, **Then** I see "Hello" printed to terminal

---

### User Story 3 - Writing Variables with Type Hints (Priority: P2)

**As a** student learning Python fundamentals,
**I want to** create variables with type hints to store different kinds of data,
**So that** I can describe my intent clearly and practice the "describe intent first" pattern that will be essential for AI-Native Development.

**Why this priority**: Type hints are the bridge between Python basics and specification-driven thinking. This is the first hands-on coding experience, making it P2 (after understanding and setup).

**Independent Test**: Can be fully tested by writing 5 different typed variables and asking AI to verify correctness. Delivers value: first working Python code + specification-first thinking pattern.

**Acceptance Scenarios**:

1. **Given** a Python environment, **When** I write `age: int = 25`, **Then** I understand this creates an integer variable named "age" with value 25
2. **Given** I want to store text, **When** I write `name: str = "Alice"`, **Then** I understand this creates a string variable with type hint
3. **Given** I want to store decimal numbers, **When** I write `price: float = 19.99`, **Then** I understand float is for decimal values
4. **Given** I want to store true/false values, **When** I write `is_valid: bool = True`, **Then** I understand bool is for boolean logic
5. **Given** I've written typed variables, **When** I ask AI "Check if my type hints are correct", **Then** AI validates my syntax and explains any errors

---

### User Story 4 - Running Python Programs (Priority: P2)

**As a** student with basic Python knowledge,
**I want to** create and run Python programs that print output,
**So that** I can see my code in action and validate that I understand execution.

**Why this priority**: Running code validates understanding. This is where theory becomes practice. P2 because it requires variables (P2 dependency).

**Independent Test**: Can be fully tested by writing a .py file, running it with `python filename.py`, and seeing expected output. Delivers value: tangible proof of learning.

**Acceptance Scenarios**:

1. **Given** I've written Python code in a file, **When** I run `python hello.py`, **Then** I see the program output in terminal
2. **Given** I write `print("Hello")`, **When** I run the program, **Then** "Hello" appears in terminal
3. **Given** I write variables with values, **When** I print those variables, **Then** I see the variable values in output
4. **Given** I want to explore output formats, **When** I use f-strings like `print(f"Name: {name}")`, **Then** I see formatted output

---

### User Story 5 - Building Interactive Programs (Priority: P3)

**As a** student ready to apply all concepts,
**I want to** build an interactive program that asks for user input and responds with typed variables,
**So that** I can demonstrate comprehensive understanding of Python fundamentals in a real, working program.

**Why this priority**: This is the integration/capstone experience. P3 because it requires all previous learning (P1 + P2 dependencies). This is "proof of mastery" rather than foundational learning.

**Independent Test**: Can be fully tested by running the capstone program, providing input, and verifying output matches specification. Delivers value: portfolio-worthy first Python project.

**Acceptance Scenarios**:

1. **Given** I want user interaction, **When** I use `input("Enter name: ")`, **Then** the program waits for user input
2. **Given** I collect user input, **When** I store it in typed variables, **Then** type hints describe the expected data
3. **Given** I have user data, **When** I print formatted output, **Then** the program responds conversationally
4. **Given** my interactive program is complete, **When** I run it end-to-end, **Then** all features work as specified in my intent description

---

### Edge Cases

- **Installation Errors**: What happens when Python download fails or installation is blocked by permissions?
  - Student asks AI: "I got this error during installation: [error]. How do I fix it?"
  - AI provides platform-specific troubleshooting steps

- **Wrong Python Version**: What happens when student has Python 2.x or very old Python 3.x installed?
  - Student verifies version with `python --version`
  - If wrong version, AI guides upgrade or installation of correct version

- **Type Mismatch**: What happens when student writes `age: int = "twenty"`?
  - Python doesn't enforce type hints at runtime (explain this explicitly)
  - Student learns to use `isinstance()` to validate types
  - AI helps student understand the difference between type hints (documentation) and runtime behavior

- **Syntax Errors**: What happens when student forgets quotes around strings or colons in type hints?
  - Student sees Python error message
  - Student asks AI: "What does this error mean?" and learns to interpret Python errors

- **Print vs Return**: What happens when student confuses `print()` with variable assignment?
  - Lesson clarifies: `print()` displays output, assignment stores values
  - AI helps student understand the difference through exploration

## Requirements *(mandatory)*

### Functional Requirements

**Lesson Structure Requirements:**

- **FR-001**: Chapter MUST include 4 foundational lessons + 1 capstone lesson (5 lessons total)
- **FR-002**: Each lesson MUST introduce maximum 5 new concepts (cognitive load limit for A1-A2 level)
- **FR-003**: Each lesson MUST end with "Try With AI" section containing exactly 4 prompts (no summaries, checklists, or other closure content)
- **FR-004**: All lessons MUST apply AI-Native Learning pattern (describe intent → explore → validate → learn from errors)

**Content Requirements:**

- **FR-005**: Lesson 1 MUST explain what Python is, its use cases for AI, and its role in modern development
- **FR-006**: Lesson 2 MUST guide Python 3.14+ installation on Windows/Mac/Linux with AI-assisted troubleshooting
- **FR-007**: Lesson 3 MUST teach variables, naming conventions, and type hints (int, str, float, bool)
- **FR-008**: Lesson 4 MUST teach basic syntax (indentation, comments, print statements, f-strings)
- **FR-009**: Lesson 5 (Capstone) MUST integrate all concepts in an interactive program students build

**Code Standards Requirements:**

- **FR-010**: All code examples MUST use Python 3.14+ syntax and features
- **FR-011**: All code examples MUST include type hints on every variable (zero exceptions)
- **FR-012**: All code examples MUST run correctly when copy-pasted (verified via testing)
- **FR-013**: All code examples MUST include beginner-friendly comments explaining intent

**Pedagogical Requirements:**

- **FR-014**: Chapter MUST apply CoLearning pedagogy throughout (💬 AI Colearning Prompts, 🎓 Instructor Commentary, 🚀 CoLearning Challenges, ✨ Teaching Tips)
- **FR-015**: Chapter MUST use conversational tone (you, your, we) NOT documentation style
- **FR-016**: Chapter MUST position type hints as core concept (not optional or advanced)
- **FR-017**: Chapter MUST teach "syntax is cheap — semantics is gold" philosophy

**Constraint Requirements:**

- **FR-018**: Chapter MUST NOT teach future topics in detail (OOP, duck typing, control flow, functions); MAY include brief preview of collection types (list, dict, tuple, set) as awareness-only (no syntax or examples - detailed coverage in Chapters 18-19)
- **FR-019**: Chapter MUST NOT use formal SDD terminology ("write specifications") - use "describe intent" instead
- **FR-020**: Chapter MUST NOT include more than 2 options for any decision (cognitive load rule)
- **FR-021**: Chapter MUST NOT assume programming background (explain all jargon on first use)
- **FR-022**: Chapter MUST introduce every built-in function/method with 1-2 line explanation BEFORE first use (e.g., print(), input(), type(), isinstance(), int()) - no function used without introduction

**Validation Requirements:**

- **FR-023**: All code examples MUST be tested on Windows, Mac, and Linux (cross-platform verification)
- **FR-024**: Reading level MUST be Grade 7-8 (verified via Flesch-Kincaid readability tool)
- **FR-025**: Chapter MUST include CEFR proficiency levels in lesson plan (A1-A2 progression)

### Key Entities *(data model for this chapter)*

**Since this is an educational chapter (not a software feature), entities represent learning artifacts:**

- **Lesson**: Represents one unit of learning (5 total). Attributes: title, learning objectives, concepts taught (max 5), CEFR level, "Try With AI" prompts (4)

- **Code Example**: Represents a runnable Python program. Attributes: purpose, complexity (beginner), code snippet, expected output, AI prompt used to generate, type hints present (boolean)

- **Exercise**: Represents hands-on practice. Attributes: title, concepts applied, acceptance criteria, capstone flag (boolean)

- **CoLearning Element**: Represents interactive AI collaboration point. Attributes: type (prompt/commentary/challenge/tip), content, placement in lesson, Bloom's taxonomy level

## Content Outline *(2-3 major sections + Common Mistakes + Try With AI)*

### Lesson 1: What Is Python? (A1 Level)

**Learning Objective**: Understand what Python is and why it's important for AI development

**Concepts Introduced** (max 5):
1. Python as a programming language
2. Python's role in AI and data science
3. Python vs other languages (high-level overview)
4. Why Python is beginner-friendly
5. Python in the AI-Native Development workflow

**Structure**:
- What is Python? (plain language definition)
- Python's superpowers for AI (real-world use cases)
- Why learn Python in the AI era? (motivation and relevance)
- 💬 AI Colearning Prompt: "Explain Python to a 10-year-old"
- 🎓 Instructor Commentary: "Syntax is cheap — semantics is gold"
- Try With AI (4 prompts: Remember → Understand → Apply → Analyze)

---

### Lesson 2: Installing Python 3.14+ (A1-A2 Level)

**Learning Objective**: Successfully install Python 3.14+ and verify installation

**Concepts Introduced** (max 5):
1. Downloading Python from python.org
2. Installation process (OS-specific)
3. Verifying installation with terminal commands
4. Understanding Python versions
5. Troubleshooting with AI assistance

**Structure**:
- Why Python 3.14+? (version importance)
- Installation guide (Windows/Mac/Linux with screenshots)
- Verification steps (`python --version`, test program)
- 💬 AI Colearning Prompt: "I got this installation error: [error]. Help me fix it."
- ✨ Teaching Tip: "Use AI for platform-specific troubleshooting"
- Try With AI (4 prompts: Remember → Understand → Apply → Troubleshoot)

---

### Lesson 3: Variables and Type Hints (A2 Level)

**Learning Objective**: Create variables with type hints to store data

**Concepts Introduced** (max 6):
1. What is a variable? (naming and assignment)
2. Core primitive types (int, str, float, bool)
3. Type hints syntax (`:` notation)
4. Why type hints matter (describing intent)
5. Variable naming conventions
6. Collection types preview (list, dict, tuple, set exist - detailed coverage in Chapters 18-19)

**Structure**:
- Variables: Names for Values (concept introduction)
- Type Hints: Describing Intent (why they help AI collaboration)
- Core Primitive Types in Action (int, str, float, bool examples)
- Collection Types Awareness (brief preview: list, dict, tuple, set - no syntax, full coverage later)
- 💬 AI Colearning Prompt: "Explain how type hints help AI generate better code"
- 🚀 CoLearning Challenge: "Create 5 variables with different types"
- Try With AI (4 prompts: Remember → Understand → Apply → Validate)

---

### Lesson 4: Basic Syntax and First Programs (A2 Level)

**Learning Objective**: Write and run simple Python programs

**Concepts Introduced** (max 5):
1. Indentation rules (Python's unique syntax)
2. Comments (# for documentation)
3. `print()` function (displaying output)
4. F-strings (formatted output)
5. Running Python files (`.py` execution)

**Structure**:
- Indentation: Python's Way (why whitespace matters)
- Print and Display (showing output)
- Your First Program (Hello World with type hints)
- Making Programs Interactive (input/output basics)
- 🎓 Instructor Commentary: "AI handles syntax details — you focus on intent"
- ✨ Teaching Tip: "Ask AI to explain any error you see"
- Try With AI (4 prompts: Remember → Understand → Apply → Debug)

---

### Lesson 5: Capstone - Personal Info Collector (B1 Level)

**Learning Objective**: Build an interactive program integrating all concepts

**Concepts Applied** (integration, not new concepts):
- Variables with type hints
- User input with `input()`
- Formatted output with f-strings
- Program structure and execution

**Capstone Specification**:

**Program Purpose**: Interactive program that asks user for personal information (name, age, favorite color) and displays a personalized greeting.

**Requirements**:
1. Use type hints for all variables (name: str, age: int, etc.)
2. Ask user for at least 3 pieces of information
3. Validate that age is a number (using isinstance check)
4. Display formatted output with user's information
5. Include comments explaining each section

**Structure**:
- Capstone Introduction (what you'll build)
- Step-by-step build guide (with AI collaboration)
- 🚀 CoLearning Challenge: "Extend the program with 2 more questions"
- 💬 AI Colearning Prompt: "Review my code and suggest improvements"
- Try With AI (4 prompts: Understand → Build → Validate → Reflect)

---

### Common Mistakes Section (in each lesson)

**Lesson 1 Common Mistakes**:
- Thinking you need to memorize everything (you don't — AI is your reference)
- Confusing Python with other languages (focus on Python concepts only)

**Lesson 2 Common Mistakes**:
- Installing wrong Python version (verify with --version)
- Not adding Python to PATH (affects terminal access)

**Lesson 3 Common Mistakes**:
- Forgetting colons in type hints (`age int = 5` instead of `age: int = 5`)
- Using quotes around numbers (`age: int = "5"` instead of `age: int = 5`)
- Confusing type hints with type enforcement (Python doesn't enforce at runtime)

**Lesson 4 Common Mistakes**:
- Inconsistent indentation (mixing tabs and spaces)
- Forgetting quotes around strings
- Confusing `print()` with variable assignment

**Lesson 5 Common Mistakes**:
- Not handling type mismatches in user input
- Forgetting to test the complete program end-to-end

---

## Code Examples Specifications *(3-8 examples with purpose, complexity, prompts)*

### Example 1: Hello World (Beginner)

**Purpose**: Demonstrate simplest Python program with type hints

**Complexity**: A1 level (absolute beginner)

**Code Specification**:
```python
# Create a greeting variable with type hint
greeting: str = "Hello, Python!"
print(greeting)
```

**AI Prompt for Generation**:
```
Generate a Hello World program in Python 3.14 that:
1. Uses a variable with type hint for the greeting
2. Prints the greeting to terminal
3. Includes a comment explaining what the code does
```

**Expected Output**: `Hello, Python!`

**Learning Focus**: Type hints, print statement, comments

---

### Example 2: Typed Variables (Beginner)

**Purpose**: Show all 4 core types with type hints

**Complexity**: A1-A2 level

**Code Specification**:
```python
# Core data types with type hints
name: str = "Alice"
age: int = 25
height: float = 5.7
is_student: bool = True

print(f"Name: {name}")
print(f"Age: {age}")
print(f"Height: {height}")
print(f"Student: {is_student}")
```

**AI Prompt for Generation**:
```
Create a Python program demonstrating all 4 core types (str, int, float, bool) with:
1. Type hints on every variable
2. Meaningful variable names
3. F-string formatted output for each variable
4. Comments explaining each type
```

**Expected Output**:
```
Name: Alice
Age: 25
Height: 5.7
Student: True
```

**Learning Focus**: Type hints for different types, f-strings

---

### Example 3: Interactive Input (Intermediate)

**Purpose**: Collect user input and display it

**Complexity**: A2 level

**Code Specification**:
```python
# Get user input
name: str = input("What is your name? ")
age_str: str = input("What is your age? ")

# Note: input() always returns string, so we convert
age: int = int(age_str)

# Display greeting
print(f"Hello, {name}! You are {age} years old.")
```

**AI Prompt for Generation**:
```
Create an interactive Python program that:
1. Asks user for name and age
2. Stores input in typed variables
3. Converts age from string to integer
4. Displays a personalized greeting
5. Uses type hints throughout
6. Includes comments explaining type conversion
```

**Expected Output** (example):
```
What is your name? Alice
What is your age? 25
Hello, Alice! You are 25 years old.
```

**Learning Focus**: `input()`, type conversion, user interaction

---

### Example 4: Type Validation (Intermediate)

**Purpose**: Check variable types at runtime

**Complexity**: A2-B1 level

**Code Specification**:
```python
# Create variables with type hints
name: str = "Bob"
age: int = 30

# Validate types
print(f"Is name a string? {isinstance(name, str)}")
print(f"Is age an integer? {isinstance(age, int)}")

# Show type
print(f"Type of name: {type(name)}")
print(f"Type of age: {type(age)}")
```

**AI Prompt for Generation**:
```
Create a Python program that:
1. Defines typed variables (str and int)
2. Uses isinstance() to validate types
3. Uses type() to display type information
4. Prints results in readable format
5. Includes comments explaining isinstance vs type()
```

**Expected Output**:
```
Is name a string? True
Is age an integer? True
Type of name: <class 'str'>
Type of age: <class 'int'>
```

**Learning Focus**: Type checking, isinstance(), type()

---

### Example 5: Capstone - Personal Info Collector (Application)

**Purpose**: Integrate all concepts in working program

**Complexity**: B1 level (application/integration)

**Code Specification**:
```python
# Personal Info Collector - Capstone Project
# This program collects user information and displays a summary

# Collect information
print("=== Personal Information Collector ===\n")

name: str = input("What is your name? ")
age_str: str = input("How old are you? ")
favorite_color: str = input("What is your favorite color? ")

# Convert and validate age
age: int = int(age_str)

# Validate age is reasonable
is_age_valid: bool = isinstance(age, int) and age > 0

# Display summary
print("\n=== Your Profile ===")
print(f"Name: {name}")
print(f"Age: {age}")
print(f"Favorite Color: {favorite_color}")
print(f"Age Valid: {is_age_valid}")

print(f"\nThank you, {name}! Your information has been recorded.")
```

**AI Prompt for Generation**:
```
Create a complete interactive Python program that:
1. Collects name, age, and favorite color from user
2. Uses type hints for all variables
3. Converts age to integer
4. Validates age using isinstance
5. Displays formatted summary of user information
6. Uses clear section headers
7. Includes comprehensive comments
```

**Expected Output** (example):
```
=== Personal Information Collector ===

What is your name? Alice
How old are you? 25
What is your favorite color? Blue

=== Your Profile ===
Name: Alice
Age: 25
Favorite Color: Blue
Age Valid: True

Thank you, Alice! Your information has been recorded.
```

**Learning Focus**: Integration of all concepts, program structure, user experience

---

## Success Criteria *(measurable outcomes, technology-agnostic)*

### Learning Outcomes

- **SC-001**: 75% of students can explain what Python is and name 2 AI applications built with Python (measured via quiz)
- **SC-002**: 85% of students successfully install Python 3.14+ and verify installation within 30 minutes (measured via capstone prerequisite check)
- **SC-003**: 80% of students can write 5 different typed variables without AI assistance (measured via in-lesson exercises)
- **SC-004**: 75% of students can run Python programs and interpret error messages (measured via debugging exercises)
- **SC-005**: 70% of students complete the capstone project demonstrating all learned concepts (measured via submission tracking)

### Engagement Outcomes

- **SC-006**: 80% of students engage with at least 3 out of 4 "Try With AI" prompts per lesson (measured via AI tool analytics if available)
- **SC-007**: Chapter completion time averages 3-4 hours (measured to validate cognitive load is appropriate)
- **SC-008**: 90% of students report feeling more confident about learning Python with AI collaboration after Chapter 13 (measured via post-chapter survey)

### Quality Outcomes

- **SC-009**: Reading level remains Grade 7-8 across all lessons (measured via Flesch-Kincaid tool)
- **SC-010**: Zero instances of gatekeeping language in published content (measured via editorial review)
- **SC-011**: All code examples run without errors on Windows, Mac, and Linux (measured via cross-platform testing)

### Preparation Outcomes

- **SC-012**: Students demonstrate readiness for Chapter 14 (Data Types) by correctly using type hints in capstone (measured via technical review)
- **SC-013**: Students can explain "describe intent first" philosophy in their own words (measured via reflective prompts)

## Acceptance Criteria *(checklist to validate quality)*

### Content Completeness
- [ ] All 5 lessons written (4 foundational + 1 capstone)
- [ ] Each lesson introduces ≤ 5 new concepts
- [ ] Each lesson ends with "Try With AI" section (4 prompts, progressive Bloom's levels)
- [ ] No additional closure content after "Try With AI" (no summaries, checklists)
- [ ] All lessons apply AI-Native Learning pattern (describe → explore → validate → learn from errors)

### Code Quality
- [ ] All code examples use Python 3.14+ syntax
- [ ] Type hints present on every variable (100% coverage)
- [ ] All code examples tested and run correctly
- [ ] All code examples include beginner-friendly comments
- [ ] Code examples tested on Windows, Mac, and Linux

### Pedagogical Quality
- [ ] CoLearning elements present throughout (💬🎓🚀✨ in all lessons)
- [ ] Conversational tone used (you, your, we)
- [ ] "Syntax is cheap — semantics is gold" reinforced
- [ ] Type hints positioned as core concept (not advanced/optional)
- [ ] Graduated Teaching Pattern followed (Book teaches foundation, AI companion handles complexity)

### Constraint Compliance
- [ ] Zero forward references (no OOP, duck typing, control flow, functions, collections)
- [ ] Zero formal SDD terminology (use "describe intent" not "write specifications")
- [ ] Maximum 2 options for any decision (cognitive load rule)
- [ ] Zero assumptions about programming background
- [ ] All jargon explained on first use

### CEFR and Cognitive Load
- [ ] CEFR proficiency levels assigned to each lesson (A1-A2 progression)
- [ ] Cognitive load ≤ 5 concepts per lesson
- [ ] Lesson progression validates A1 → A2 → B1 (capstone) trajectory
- [ ] Skills proficiency mapping applied via `skills-proficiency-mapper`

### Accessibility and Quality
- [ ] Reading level Grade 7-8 (verified via Flesch-Kincaid)
- [ ] Zero gatekeeping language ("easy", "simply", "obvious")
- [ ] Platform-specific guidance provided (Windows/Mac/Linux)
- [ ] Multiple explanation styles (text, code, analogies)
- [ ] Error literacy taught (distinguish normal output vs actual errors)

### Alignment with Evals
- [ ] Content supports all 11 success evals (EVAL-001 through EVAL-011)
- [ ] Each learning objective maps to specific eval(s)
- [ ] Assessment mechanisms defined for each eval
- [ ] Business outcomes clearly connected to content

## Complexity Tier

**Tier**: Beginner (Chapters 12-16)
**CEFR Range**: A1-A2 (with B1 capstone)
**Cognitive Load Limit**: Maximum 5 concepts per lesson
**Scaffolding Level**: Maximum (clear instructions, AI support, beginner-friendly language)
**Options Limit**: Maximum 2 choices for any decision (prevent overwhelm)

**Tier Implications**:
- Heavy use of analogies and real-world examples
- Step-by-step instructions with screenshots
- Extensive AI collaboration prompts (exploration is encouraged)
- Simple, clear language (avoid technical jargon unless explained)
- Confidence-building exercises (success is designed in)
- "Your AI handles complexity" framing throughout

## Assumptions

**Technology Assumptions**:
1. Students have completed Chapter 12 (Python UV) and have package manager configured
2. Students have terminal/command line access from Chapter 7 (Bash Essentials)
3. Students have stable internet connection for Python download and AI tool access
4. Students have at least one AI tool configured (Claude Code or Gemini CLI) from Chapters 5-6

**Learning Assumptions**:
1. Students are familiar with AI-Driven Development philosophy from Chapters 1-11
2. Students are comfortable asking AI questions (prompt engineering from Chapter 10)
3. Students are motivated to learn (completed 12 prior chapters successfully)
4. Zero programming background (we assume nothing about coding knowledge)

**Pedagogical Assumptions**:
1. Type hints can be taught as "describing intent" (introductory framing appropriate for beginners)
2. Students will learn better by building (hands-on capstone is valuable)
3. AI collaboration improves learning outcomes (constitution assumption)
4. Conversational tone is more accessible than documentation style

**Content Decisions (Documented)**:
1. We exclude control flow, functions, and OOP to prevent scope creep (taught in future chapters)
2. We introduce isinstance() for type validation even though it's slightly advanced (necessary for error handling)
3. We teach f-strings over older string formatting (modern best practice)
4. We position type hints as mandatory, not optional (aligns with professional development and SDD preparation)

## Notes

**For Lesson Writers**:
- This is the FIRST hands-on Python chapter. Students have zero Python experience.
- Balance theory (what/why) with practice (how) — aim for 40% conceptual, 60% hands-on.
- AI collaboration is not optional — embed it throughout, not just at the end.
- Type hints are CORE, not advanced. Frame them as "how you describe intent to AI."

**For Technical Reviewers**:
- Validate all code on Python 3.14+ specifically (not just "Python 3.x")
- Check for forward references aggressively (easy to slip in OOP concepts unconsciously)
- Verify "Try With AI" sections have exactly 4 prompts (no more, no less)
- Ensure no summaries/checklists after "Try With AI" (common violation)

**For Planners**:
- This chapter sets the pattern for all Part 4 chapters (12-29)
- Lesson structure and CoLearning pedagogy established here become the template
- Proficiency progression (A1 → A2 → B1) is the model for future chapters
- Skills metadata (CEFR levels) MUST be included in plan.md
