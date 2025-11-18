# Chapter 13: Introduction to Python — Task Checklist

**Status**: Ready for Development
**Feature Branch**: `016-part-4-chapter-13`
**Chapter Type**: Technical/Code-Focused (Beginner AI-Native Learning)
**Complexity Tier**: Beginner (CEFR A1-A2, with B1 capstone)
**Estimated Total Effort**: 40-50 hours (implementation + review + validation)
**Owner**: To be assigned
**Created**: 2025-11-08

---

## Overview

This task checklist breaks down Chapter 13 implementation into atomic, testable, prioritized tasks. All tasks align with the approved specification and detailed plan. Each task is designed to complete in 1-2 hours and has clear acceptance criteria.

**Task Philosophy**: MUST-HAVE tasks ensure chapter is functional and meets learning objectives. SHOULD-HAVE tasks improve pedagogical quality. NICE-TO-HAVE tasks enhance engagement or extend learning.

---

## PHASE 1: Chapter Structure & Foundational Elements

### Task 1.1: Create Chapter README.md
**Priority**: MUST-HAVE | **Effort**: 1.5h | **Status**: ✅ COMPLETED

**Description**: Create overview document for Chapter 13 that appears first when students encounter the chapter.

**Acceptance Criteria**:
- [ ] File created at: `book-source/docs/04-Part-4-Python-Fundamentals/13-introduction-to-python/readme.md`
- [ ] Title: `# Chapter 13: Introduction to Python`
- [ ] Includes 2-3 paragraph introduction explaining chapter purpose
- [ ] "What You'll Learn" section lists 5 learning objectives (from plan)
- [ ] Estimated time: 4-5 hours mentioned
- [ ] Note about AI-Native Learning methodology
- [ ] Preview of what students will build (capstone project)
- [ ] References to required tools (Python 3.14+, AI tool like Claude Code or Gemini CLI)
- [ ] Follows canonical output style (reference: `.claude/output-styles/chapters.md`)

**Reference**: `.claude/output-styles/chapters.md` for chapter README structure

**Dependencies**: None (prerequisite for all lessons)

---

### Task 1.2: Create YAML Frontmatter Template for All Lessons
**Priority**: MUST-HAVE | **Effort**: 1h | **Status**: ✅ COMPLETED

**Description**: Prepare YAML frontmatter structure for lessons with skills proficiency metadata.

**Acceptance Criteria**:
- [ ] Template created with all required fields:
  ```yaml
  ---
  title: "[Lesson Title]"
  chapter: 13
  lesson: [1-5]
  duration_minutes: [estimated]

  # HIDDEN SKILLS METADATA (Institutional Integration Layer)
  skills:
    - name: "[Skill Name]"
      proficiency_level: "[A1|A2|B1]"
      category: "[Technical|Conceptual|Soft]"
      bloom_level: "[Remember|Understand|Apply|Analyze|Evaluate|Create]"
      digcomp_area: "[Information|Communication|Content|Safety|Problem-Solving]"
      measurable_at_this_level: "[What student demonstrates]"

  learning_objectives:
    - objective: "[Statement]"
      bloom_level: "[Level]"
      assessment_method: "[How we measure]"

  cognitive_load:
    new_concepts: [number]
    concepts_list: "[List of 4-5 concepts]"
    cefr_level: "[A1|A2|B1]"
  ---
  ```
- [ ] Template saved as reference for lesson writers
- [ ] All fields documented with examples
- [ ] Skills metadata includes CEFR levels (from plan)
- [ ] Bloom's levels aligned with learning objectives

**Reference**: `.claude/output-styles/lesson.md` for full lesson structure

**Dependencies**: None (prerequisite for all lesson writing)

---

## PHASE 2: Lesson 1 – What Is Python?

### Task 2.1: Outline Lesson 1 Structure
**Priority**: MUST-HAVE | **Effort**: 1h | **Status**: ✅ COMPLETED

**Description**: Create detailed outline for Lesson 1 before writing full content.

**Acceptance Criteria**:
- [ ] Outline includes all 3 sections from plan:
  - What Exactly Is Python?
  - Why Python for AI Development?
  - How Python Fits Into This Book
- [ ] CoLearning elements mapped to sections:
  - 💬 AI Colearning Prompt (After Section 2)
  - 🎓 Instructor Commentary (After Section 3)
  - 🚀 CoLearning Challenge (Optional, before "Try With AI")
  - ✨ Teaching Tips (Throughout)
- [ ] Common mistakes listed with corrections
- [ ] "Try With AI" structure outlined (4 prompts with Bloom's levels)
- [ ] Estimated time per section calculated
- [ ] Approved by human before proceeding to writing

**Reference**: `specs/016-part-4-chapter-13/plan.md` Section "Lesson 1: What Is Python?"

**Dependencies**: Task 1.1, Task 1.2

---

### Task 2.2: Write Lesson 1 Core Content
**Priority**: MUST-HAVE | **Effort**: 3h | **Status**: ✅ COMPLETED

**Description**: Write full content for Lesson 1 (3 main sections + common mistakes).

**Acceptance Criteria**:
- [ ] 3 main sections written (What is Python, Why Python for AI, How Python Fits)
- [ ] Real-world examples included (Spotify, Tesla, ChatGPT)
- [ ] Connection to AIDD methodology explicit in Section 3
- [ ] Reading level Grade 7-8 (no overly complex sentences)
- [ ] All jargon explained on first use ("programming language", "AI", etc.)
- [ ] Conversational tone (you, your, we) throughout
- [ ] No gatekeeping language ("easy", "simple", "obvious")
- [ ] Common mistakes section addresses:
  - Myth: "I need to memorize all of Python"
  - Myth: "Python is only for data science"
  - Myth: "Using AI means I'm not really learning"
- [ ] Follows output style for lesson structure
- [ ] Adheres to cognitive load (5 concepts max): What is programming language, Python as language, Python's superpower for AI, Why Python for book, Python in dev workflow

**Reference**:
- `.claude/output-styles/lesson.md` for structure
- Plan Section "Lesson 1: What Is Python?" for content structure
- CLAUDE.md for conversational tone guidelines

**Dependencies**: Task 2.1

---

### Task 2.3: Create Lesson 1 "Try With AI" Prompts
**Priority**: MUST-HAVE | **Effort**: 1.5h | **Status**: ✅ COMPLETED

**Description**: Write 4 progressive prompts for "Try With AI" section (Lesson 1 closure).

**Acceptance Criteria**:
- [ ] **Prompt 1 (Recall)**: One-sentence Python definition
  - Prompt text: Exact, concrete, specific
  - Expected outcome: Documented
  - Bloom's level verified: Remember

- [ ] **Prompt 2 (Understand)**: Explain Python's readability and library ecosystem
  - Prompt text: Multi-part question about syntax + library example
  - Expected outcome: Student understands connection to AI collaboration
  - Bloom's level verified: Understand

- [ ] **Prompt 3 (Apply)**: Connect Python to personal goal
  - Prompt text: Specific about relating Python to learner's aspirations
  - Expected outcome: Personalized application; relevance established
  - Bloom's level verified: Apply

- [ ] **Prompt 4 (Analyze)**: Compare Python to other languages (cognitive closure)
  - Prompt text: Asks for tradeoff analysis; "why this question matters"
  - Expected outcome: Broader perspective; professional decision-making
  - Bloom's level verified: Analyze

- [ ] All prompts are CONCRETE (not "ask about X" but specific text)
- [ ] All prompts reference AI tool explicitly (Claude Code or Gemini CLI)
- [ ] Closing statement emphasizes synthesis/connection

**Reference**: Plan Section "Lesson 1: Try With AI (4 Prompts - Progressive Bloom's)"

**Dependencies**: Task 2.2

---

### Task 2.4: Integrate CoLearning Elements in Lesson 1
**Priority**: SHOULD-HAVE | **Effort**: 1h | **Status**: Not Started

**Description**: Embed CoLearning elements (💬🎓🚀✨) into Lesson 1 content.

**Acceptance Criteria**:
- [ ] **💬 AI Colearning Prompt** placed after Section 2
  - Exact prompt: "Explain Python to a 10-year-old in 2-3 sentences..."
  - Context provided: "Now that you understand Python's strengths..."
  - Expected outcome documented: Student validates understanding through AI's explanation

- [ ] **🎓 Instructor Commentary** placed after Section 3
  - Message: "Syntax is cheap—semantics is gold"
  - Explanation: Why understanding matters more than memorization
  - Connection to AI: AI handles syntax details

- [ ] **🚀 CoLearning Challenge** (Optional, enrichment)
  - Challenge: Find 3 real Python AI applications
  - Purpose: Connect to real-world systems
  - Estimated time: 5 min

- [ ] **✨ Teaching Tips** placed throughout
  - Tip 1: "Your AI tool knows Python deeply..."
  - Tip 2: (Context-specific teaching advice)

- [ ] All elements formatted with clear visual markers (emoji + title)
- [ ] Each element has clear purpose statement
- [ ] CoLearning elements don't duplicate content

**Reference**: `ai-collaborate-teaching` skill output + Plan Section "Lesson 1: CoLearning Elements"

**Dependencies**: Task 2.2

---

### Task 2.5: Lesson 1 Quality Review
**Priority**: SHOULD-HAVE | **Effort**: 1h | **Status**: Not Started

**Description**: Internal quality check before seeking external review.

**Acceptance Criteria**:
- [ ] Reading level check: Run Flesch-Kincaid tool (target: Grade 7-8)
- [ ] Grammar check: Proofread for typos, sentence structure
- [ ] Jargon check: Every technical term has beginner-friendly explanation
- [ ] Tone check: Conversational (you, your, we) throughout
- [ ] No gatekeeping language ("easy", "simple", "obvious")
- [ ] Learning objective verified: Can student explain what Python is?
- [ ] CEFR level verified: A1 (Recognition/Understanding)
- [ ] Cognitive load verified: Exactly 5 concepts
- [ ] "Try With AI" section present with 4 prompts only (no other closure content)
- [ ] No forward references (no OOP, functions, control flow, collections)
- [ ] Plan alignment: Content matches outline from Task 2.1

**Reference**: Quality Assurance Checklist in plan.md

**Dependencies**: Tasks 2.2, 2.3, 2.4

---

## PHASE 3: Lesson 2 – Installing Python 3.14+

### Task 3.1: Outline Lesson 2 & Create OS-Specific Guides
**Priority**: MUST-HAVE | **Effort**: 1.5h | **Status**: Not Started

**Description**: Plan Lesson 2 structure and create installation guides for Windows, Mac, Linux.

**Acceptance Criteria**:
- [ ] Lesson 2 sections outlined:
  - Why Installation Matters
  - Installation Steps (OS-specific: Windows, Mac, Linux)
  - Verifying Installation
  - Troubleshooting with AI Assistance

- [ ] **Windows installation guide**:
  - Download from python.org link (exact URL provided)
  - Step-by-step installer clicks (with screenshots descriptions)
  - Add to PATH checkbox emphasis
  - Test: `python --version`
  - Test: `python -c "print('Hello')"`

- [ ] **Mac installation guide**:
  - Download from python.org link
  - Installation steps
  - PATH handling (usually automatic)
  - Tests same as Windows

- [ ] **Linux installation guide**:
  - Package manager approach (apt, brew, yum)
  - Commands provided
  - Tests same as Windows

- [ ] Common errors section:
  - "Python command not found"
  - "Python 2.x instead of 3.14"
  - "Installation blocked by permissions"

- [ ] CoLearning elements mapped:
  - 💬 AI Colearning Prompt (if error occurs)
  - ✨ Teaching Tips (platform-specific troubleshooting)
  - 🚀 CoLearning Challenge (after successful installation)

- [ ] Approved before proceeding to full writing

**Reference**: Plan Section "Lesson 2: Installing Python 3.14+"

**Dependencies**: Tasks 1.1, 1.2

---

### Task 3.2: Write Lesson 2 Core Content
**Priority**: MUST-HAVE | **Effort**: 3.5h | **Status**: ✅ COMPLETED

**Description**: Write full Lesson 2 content with OS-specific installation guides.

**Acceptance Criteria**:
- [ ] 4 sections written:
  - Why Installation Matters (1 min read)
  - Installation Steps with OS differentiation (5 min per OS)
  - Verifying Installation (2 min)
  - Troubleshooting with AI Assistance (2 min)

- [ ] **Windows section**:
  - Direct link to python.org Windows download
  - Screenshots or detailed step descriptions
  - Critical: "Add Python to PATH" checkbox highlighted
  - Verification commands provided

- [ ] **Mac section**:
  - Direct link to python.org Mac download
  - Steps similar to Windows
  - Note about PATH (usually handled automatically)
  - Verification commands provided

- [ ] **Linux section**:
  - Package manager option (apt for Ubuntu/Debian, brew for MacOS, yum for Red Hat)
  - Installation commands (copy-paste ready)
  - Verification commands provided

- [ ] Troubleshooting section includes:
  - How to diagnose errors (copy exact error message)
  - Template for asking AI: "I got this error... How do I fix it?"
  - Common solutions for each error type
  - When to ask AI for platform-specific help

- [ ] Reading level: Grade 7-8
- [ ] Conversational tone throughout
- [ ] Cognitive load: 4 new concepts (Installer, python.org, version importance, terminal verification)
- [ ] CEFR level: A1-A2 (guided application)
- [ ] No gatekeeping language

**Reference**:
- Plan Section "Lesson 2: Installing Python 3.14+"
- `.claude/output-styles/lesson.md` for lesson structure

**Dependencies**: Task 3.1

---

### Task 3.3: Create Lesson 2 "Try With AI" Prompts
**Priority**: MUST-HAVE | **Effort**: 1h | **Status**: ✅ COMPLETED

**Description**: Write 4 progressive "Try With AI" prompts for Lesson 2 (closure section).

**Acceptance Criteria**:
- [ ] **Prompt 1 (Recall)**: Where did you download Python? Why python.org?
  - Concrete, specific question
  - Expected outcome: Student recalls official source; security awareness

- [ ] **Prompt 2 (Understand)**: What does Python 3.14+ requirement mean?
  - Multi-part: explain versioning, what "+" means, how you verified
  - Expected outcome: Understanding of semantic versioning

- [ ] **Prompt 3 (Apply)**: Troubleshoot a friend's error (apply troubleshooting skill)
  - Scenario given: "python: command not found"
  - Expected outcome: Student applies knowledge to real problem

- [ ] **Prompt 4 (Analyze)**: Why use two verification commands?
  - Asks about difference between `--version` and `print()` test
  - Expected outcome: Understanding verification strategies; cognitive closure

- [ ] All prompts concrete and specific
- [ ] Bloom's progression: Remember → Understand → Apply → Analyze
- [ ] AI tool explicitly referenced

**Reference**: Plan Section "Lesson 2: Try With AI (4 Prompts - Progressive Bloom's)"

**Dependencies**: Task 3.2

---

### Task 3.4: Integrate CoLearning Elements in Lesson 2
**Priority**: SHOULD-HAVE | **Effort**: 45m | **Status**: Not Started

**Description**: Embed CoLearning elements (💬🎓🚀✨) into Lesson 2.

**Acceptance Criteria**:
- [ ] **💬 AI Colearning Prompt** (conditional, if error occurs)
  - Template: "I tried to install Python on [OS] and got this error: [error]. How do I fix it?"
  - Teaches context provision (OS + full error message)
  - Expected outcome: AI provides platform-specific troubleshooting

- [ ] **✨ Teaching Tips**:
  - Tip 1: Different computers have different setups—AI is perfect for this
  - Tip 2: Copy exact error message, not summary
  - Tip 3: Specific error messages → better AI help

- [ ] **🚀 CoLearning Challenge** (after success):
  - Challenge: Run both verification commands; describe what happened
  - Challenge: Ask AI what the `-c` flag does
  - Purpose: Encourages exploration; validates installation

- [ ] All elements clearly marked and purposeful
- [ ] No duplication of content sections

**Reference**: `ai-collaborate-teaching` skill + Plan Section "Lesson 2: CoLearning Elements"

**Dependencies**: Tasks 3.2, 3.3

---

### Task 3.5: Lesson 2 Quality Review
**Priority**: SHOULD-HAVE | **Effort**: 1h | **Status**: Not Started

**Description**: Internal quality check for Lesson 2.

**Acceptance Criteria**:
- [ ] Reading level: Grade 7-8 (Flesch-Kincaid)
- [ ] Grammar and spelling: Proofread
- [ ] Technical accuracy: Installation steps verified (or marked "screenshots needed")
- [ ] Platform coverage: Windows, Mac, Linux all addressed
- [ ] CEFR level: A1-A2 (guided application)
- [ ] Cognitive load: 4 concepts
- [ ] Learning objective met: Student can install Python 3.14+ and verify
- [ ] "Try With AI" section complete with 4 prompts only
- [ ] No forward references
- [ ] Common mistakes addressed

**Reference**: Quality Assurance Checklist in plan.md

**Dependencies**: Tasks 3.2, 3.3, 3.4

---

## PHASE 4: Lesson 3 – Variables and Type Hints

### Task 4.1: Outline Lesson 3 & Create Code Examples
**Priority**: MUST-HAVE | **Effort**: 1.5h | **Status**: Not Started

**Description**: Plan Lesson 3 structure and design code examples for each concept.

**Acceptance Criteria**:
- [ ] Lesson 3 structure outlined:
  - Section 1: Variables – Names for Values
  - Section 2: The Four Core Primitive Types (int, str, float, bool)
  - Section 3: Type Hints – Describing Intent
  - Section 4: Collection Types Awareness (brief preview: list, dict, tuple, set - no syntax)
  - Section 5: Working With Variables (type(), isinstance())

- [ ] Code examples designed:
  - Example 1: Basic variable with type hint (`age: int = 25`)
  - Example 2: All 4 types demonstrated (int, str, float, bool)
  - Example 3: Type validation with `isinstance()`
  - Example 4: `type()` function usage

- [ ] Each example:
  - Tested to run correctly (Python 3.14+)
  - Includes type hints on ALL variables
  - Includes comments explaining intent
  - Shows expected output

- [ ] CoLearning elements mapped:
  - 💬 AI Colearning Prompt (explain type hints + AI)
  - 🚀 CoLearning Challenge (create 5 variables, validate with AI)
  - ✨ Teaching Tips (type hints as core, not optional)

- [ ] Common mistakes identified:
  - Forgetting colons in type hints
  - Quotes around numbers
  - Confusing type hints with enforcement
  - Non-descriptive variable names
  - Invalid variable names (starts with number, has spaces, uses keywords)

- [ ] Approved before full writing

**Reference**: Plan Section "Lesson 3: Variables and Type Hints"

**Dependencies**: Tasks 1.1, 1.2

---

### Task 4.2: Write Lesson 3 Core Content
**Priority**: MUST-HAVE | **Effort**: 3h | **Status**: ✅ COMPLETED

**Description**: Write full Lesson 3 content with variable and type hint concepts.

**Acceptance Criteria**:
- [ ] 5 sections written:
  - Section 1: Variables as labeled containers (with analogy)
  - Section 2: All 4 core primitive types with examples
  - Section 3: Type hints syntax and purpose
  - Section 4: Collection types awareness (brief preview: list, dict, tuple, set - no syntax/examples)
  - Section 5: Working with type() and isinstance()

- [ ] Each primitive type explained:
  - **int**: Whole numbers (no decimals) — Example: `age: int = 25`
  - **str**: Text (goes in quotes) — Example: `name: str = "Alice"`
  - **float**: Decimal numbers — Example: `price: float = 19.99`
  - **bool**: True/False values — Example: `is_student: bool = True`

- [ ] Python naming conventions explained (PEP 8):
  - **Use lowercase with underscores**: `user_name`, `total_price`, `is_valid`
  - **Be descriptive**: `age` not `a`, `customer_name` not `cn`
  - **Start with letter or underscore**: `name` ✓, `_temp` ✓, `2name` ✗
  - **No spaces**: `user name` ✗, `user_name` ✓
  - **Avoid Python keywords**: `class` ✗, `user_class` ✓
  - Examples: `age`, `user_name`, `total_price`, `is_student`, `favorite_color`
  - Show error when invalid name used (e.g., `2age`, `user name`)

- [ ] Type hints explained:
  - Purpose: Describe what data a variable should hold
  - Syntax: `: TypeName` (colon + space + type)
  - Not enforced by Python (but AI uses them to understand intent)
  - Core to specification-first thinking

- [ ] Collection types preview (awareness only):
  - Brief introduction: "Python has types for storing multiple values"
  - Names mentioned: list, dict, tuple, set
  - Forward reference: "We'll learn these in detail in Chapters 18-19"
  - No syntax or examples provided
  - 1-2 paragraphs maximum

- [ ] Built-in functions introduced before use (FR-022):
  - **print()**: "The `print()` function displays output to your screen. It shows you what's inside variables." (1-2 lines BEFORE first use)
  - **type()**: "The `type()` function tells you what kind of data a variable holds." (1-2 lines BEFORE first use)
  - **isinstance()**: "The `isinstance()` function checks if a variable is a specific type. It returns True or False." (1-2 lines BEFORE first use)

- [ ] isinstance() and type() usage explained:
  - `isinstance(variable, int)` checks if variable is an int
  - `type(variable)` returns the actual type
  - Useful for validation

- [ ] Code examples integrated throughout (not separate section)
- [ ] Reading level: Grade 7-8
- [ ] Cognitive load: 7 concepts (variable, primitive types, type hints, naming conventions, invalid variable names and error messages, collection awareness, type validation)
- [ ] CEFR level: A2 (simple application with scaffolding)
- [ ] No gatekeeping language

**Reference**:
- Plan Section "Lesson 3: Variables and Type Hints"
- `.claude/output-styles/lesson.md`

**Dependencies**: Task 4.1

---

### Task 4.3: Create Code Examples for Lesson 3
**Priority**: MUST-HAVE | **Effort**: 1.5h | **Status**: ✅ COMPLETED

**Description**: Write, test, and finalize all code examples for Lesson 3.

**Acceptance Criteria**:
- [ ] **Example 1: Basic Variable**
  ```python
  greeting: str = "Hello, Python!"
  print(greeting)
  ```
  - Runs without error ✓
  - Type hints present ✓
  - Comments explain intent ✓
  - Tested on Windows, Mac, Linux ✓

- [ ] **Example 2: All Four Types**
  ```python
  name: str = "Alice"
  age: int = 25
  height: float = 5.7
  is_student: bool = True
  # [print statements and f-strings for each]
  ```
  - All 4 types represented ✓
  - Type hints on all variables ✓
  - f-strings for formatted output ✓
  - Expected output documented ✓

- [ ] **Example 3: isinstance() Validation**
  ```python
  name: str = "Bob"
  age: int = 30
  print(isinstance(name, str))
  print(isinstance(age, int))
  ```
  - Demonstrates validation ✓
  - Shows correct and incorrect checks ✓
  - Output explained ✓

- [ ] **Example 4: type() Function**
  ```python
  # Shows type() returns <class 'type_name'>
  ```
  - Demonstrates type() usage ✓
  - Output explained ✓

- [ ] All examples:
  - Follow PEP 8 style ✓
  - Have type hints on 100% of variables ✓
  - Include comments explaining intent ✓
  - Tested to run correctly ✓
  - Cross-platform verified (Windows/Mac/Linux) ✓
  - Expected output documented ✓

**Reference**: Plan Section "Lesson 3: Code Examples"

**Dependencies**: Task 4.1

---

### Task 4.4: Create Lesson 3 "Try With AI" Prompts
**Priority**: MUST-HAVE | **Effort**: 1h | **Status**: ✅ COMPLETED

**Description**: Write 4 progressive "Try With AI" prompts (Lesson 3 closure).

**Acceptance Criteria**:
- [ ] **Prompt 1 (Recall)**: Type hint syntax pattern
  - Asks to fill in blanks: `name: _______ = _______`
  - Asks for 3 examples with different types
  - Expected outcome: Syntax memorization

- [ ] **Prompt 2 (Understand)**: Why type hints matter
  - Asks: What's difference between `age = 25` and `age: int = 25`?
  - Asks: Why is type hint better from AI perspective?
  - Expected outcome: Understanding intent communication

- [ ] **Prompt 3 (Apply)**: Write 5 typed variables
  - Scenario: Store person's name, years coding, skill level, Python preference, something personal
  - With type hints for each
  - Validate with AI
  - Expected outcome: Correct syntax; demonstrates mastery

- [ ] **Prompt 4 (Analyze)**: Type validation puzzle (cognitive closure)
  - Puzzle: Python allows `age: int = "twenty-five"` (wrong data for type hint)
  - Questions: Why? What's difference between hints and enforcement? How to check with isinstance()?
  - Why violate type hints is bad?
  - Expected outcome: Understanding distinction; importance of consistency

- [ ] All prompts concrete and specific
- [ ] Bloom's progression verified
- [ ] AI tool referenced explicitly

**Reference**: Plan Section "Lesson 3: Try With AI (4 Prompts - Progressive Bloom's)"

**Dependencies**: Task 4.2

---

### Task 4.5: Integrate CoLearning Elements in Lesson 3
**Priority**: SHOULD-HAVE | **Effort**: 1h | **Status**: Not Started

**Description**: Embed CoLearning elements (💬🎓🚀✨) into Lesson 3.

**Acceptance Criteria**:
- [ ] **💬 AI Colearning Prompt** (after Section 3)
  - Prompt: "Explain how type hints help AI generate better code. Give a specific example..."
  - Purpose: Student learns that type hints improve AI collaboration
  - Expected outcome: Connection to AIDD methodology

- [ ] **🚀 CoLearning Challenge** (before exercises)
  - Challenge: Create 5 variables with different types (name, age, height, learning AI, personal choice)
  - Ask AI to validate type hints
  - Purpose: Scaffolded practice; AI validates work

- [ ] **✨ Teaching Tips**:
  - Tip: "Type hints are not optional—they're core to professional Python and AI collaboration"
  - Tip: "Every variable gets a type hint. No exceptions."

- [ ] All elements clearly marked
- [ ] No content duplication

**Reference**: `ai-collaborate-teaching` skill + Plan Section "Lesson 3: CoLearning Elements"

**Dependencies**: Tasks 4.2, 4.3, 4.4

---

### Task 4.6: Lesson 3 Quality Review
**Priority**: SHOULD-HAVE | **Effort**: 1h | **Status**: Not Started

**Description**: Internal quality check for Lesson 3.

**Acceptance Criteria**:
- [ ] Reading level: Grade 7-8
- [ ] Grammar and spelling: Proofread
- [ ] Code examples: All tested and run correctly
- [ ] Type hints: 100% coverage in all examples
- [ ] Comments: Clear intent explanations
- [ ] CEFR level: A2
- [ ] Cognitive load: 5 concepts
- [ ] Learning objective: Student can write typed variables
- [ ] "Try With AI" section complete with 4 prompts
- [ ] No forward references (no OOP, control flow, functions)
- [ ] Common mistakes addressed

**Reference**: Quality Assurance Checklist in plan.md

**Dependencies**: Tasks 4.2, 4.3, 4.4, 4.5

---

## PHASE 5: Lesson 4 – Basic Syntax and First Programs

### Task 5.1: Outline Lesson 4 & Design Code Examples
**Priority**: MUST-HAVE | **Effort**: 1.5h | **Status**: Not Started

**Description**: Plan Lesson 4 structure with code examples for all 5 syntax concepts.

**Acceptance Criteria**:
- [ ] Lesson 4 sections outlined:
  - Section 1: Indentation (Python's unique syntax)
  - Section 2: Comments (# for explanation)
  - Section 3: print() Function (output to screen)
  - Section 4: F-Strings (modern text formatting)
  - Section 5: Creating and Running .py Files

- [ ] Code examples designed:
  - Example 1: Hello World with type hints
  - Example 2: Variables and print statements
  - Example 3: F-strings in action
  - (Potentially more per section)

- [ ] Each example:
  - Tested to run (Python 3.14+)
  - Type hints on all variables
  - Comments explaining intent
  - Expected output shown
  - Can be copy-pasted by beginners

- [ ] CoLearning elements:
  - 🎓 Instructor Commentary (syntax is cheap, semantics is gold)
  - ✨ Teaching Tips (error messages, indentation, AI for debugging)
  - 🚀 CoLearning Challenge (write simple program with f-strings)

- [ ] Common mistakes:
  - Indentation errors (tabs vs spaces)
  - Forgetting quotes around strings
  - Confusing print() with assignment
  - Old string formatting vs f-strings

- [ ] Approved before full writing

**Reference**: Plan Section "Lesson 4: Basic Syntax and First Programs"

**Dependencies**: Tasks 1.1, 1.2

---

### Task 5.2: Write Lesson 4 Core Content
**Priority**: MUST-HAVE | **Effort**: 3h | **Status**: ✅ COMPLETED

**Description**: Write full Lesson 4 content with syntax concepts.

**Acceptance Criteria**:
- [ ] 5 sections written:
  - Indentation explained (unique to Python; 4 spaces standard)
  - Comments explained (`#` for code explanation)
  - print() function explained (basic and formatted output)
  - F-strings explained (modern preferred approach)
  - .py files explained (how to save and run)

- [ ] Indentation section:
  - Explains whitespace significance in Python
  - Standard: 4 spaces per level
  - Common mistake: mixing tabs and spaces
  - Tool recommendation: Configure editor to show whitespace

- [ ] Comments section:
  - `# This is a comment` syntax
  - Purpose: Explain what code does and why
  - Philosophy: Code for computers, comments for humans

- [ ] print() section:
  - **Introduce print() first** (FR-022): "The `print()` function displays text and data to your screen. It's how you see what your program is doing." (1-2 lines BEFORE examples)
  - Basic: `print("Hello")`
  - Variables: `print(name)`
  - Multiple items: `print(name, age)`
  - Output goes to terminal

- [ ] F-strings section:
  - Modern approach: `f"Name: {name}"`
  - vs. old way: `"Name: " + name` or `"Name: {}".format(name)`
  - Why f-strings preferred: readable, concise, professional
  - Key insight: Syntax changes; patterns matter

- [ ] .py files section:
  - Create file with .py extension
  - Edit in text editor (VS Code, Cursor, etc.)
  - Run: `python filename.py`
  - Output appears in terminal

- [ ] Code examples integrated throughout
- [ ] Reading level: Grade 7-8
- [ ] Cognitive load: 5 concepts (indentation, comments, print, f-strings, .py files)
- [ ] CEFR level: A2 (application to familiar contexts)
- [ ] No gatekeeping language

**Reference**:
- Plan Section "Lesson 4: Basic Syntax and First Programs"
- `.claude/output-styles/lesson.md`

**Dependencies**: Task 5.1

---

### Task 5.3: Create Code Examples for Lesson 4
**Priority**: MUST-HAVE | **Effort**: 1.5h | **Status**: ✅ COMPLETED

**Description**: Write, test, and finalize code examples for Lesson 4.

**Acceptance Criteria**:
- [ ] **Example 1: Hello World with Type Hints**
  ```python
  greeting: str = "Hello, Python!"
  print(greeting)
  ```
  - Runs correctly ✓
  - Type hints present ✓
  - Comments explain intent ✓
  - Cross-platform tested ✓

- [ ] **Example 2: Variables and Print**
  ```python
  name: str = "Alice"
  age: int = 25
  city: str = "Portland"
  print(name)
  print(age)
  print(city)
  ```
  - All variables typed ✓
  - Demonstrates multiple print statements ✓
  - Expected output shown ✓

- [ ] **Example 3: F-Strings**
  ```python
  name: str = "Bob"
  age: int = 30
  favorite_language: str = "Python"
  print(f"Name: {name}")
  print(f"Age: {age}")
  print(f"Favorite Language: {favorite_language}")
  ```
  - F-strings used (not concatenation) ✓
  - Modern syntax demonstrated ✓
  - Expected output shown ✓

- [ ] All examples:
  - PEP 8 compliant ✓
  - Type hints 100% coverage ✓
  - Comments explaining intent ✓
  - Tested to run ✓
  - Cross-platform verified ✓
  - Expected output documented ✓

**Reference**: Plan Section "Lesson 4: Code Examples for This Lesson"

**Dependencies**: Task 5.1

---

### Task 5.4: Create Lesson 4 "Try With AI" Prompts
**Priority**: MUST-HAVE | **Effort**: 1h | **Status**: ✅ COMPLETED

**Description**: Write 4 progressive "Try With AI" prompts (Lesson 4 closure).

**Acceptance Criteria**:
- [ ] **Prompt 1 (Recall)**: Syntax elements
  - Fill-in-blanks for indentation, comments, print()
  - Bonus: F-string syntax
  - Expected outcome: Memorization of core syntax

- [ ] **Prompt 2 (Understand)**: F-strings vs. concatenation
  - Compare two approaches; explain why f-strings recommended
  - Ask AI for advantages of f-strings
  - Expected outcome: Understanding readability preference

- [ ] **Prompt 3 (Apply)**: Write your own program
  - Create program with 3 typed variables, print, f-strings, comments
  - Save as `.py` file and run
  - Show code to AI for review
  - Expected outcome: Working program; demonstrates mastery

- [ ] **Prompt 4 (Analyze)**: Debug broken code (cognitive closure)
  - Present code with error: `age int = 30` (missing colon in type hint)
  - Questions: What's wrong? Why? How to fix? What would error message say?
  - Ask AI to explain error
  - Expected outcome: Error interpretation; debugging process; analytical thinking

- [ ] All prompts concrete and specific
- [ ] Bloom's progression verified
- [ ] AI tool referenced

**Reference**: Plan Section "Lesson 4: Try With AI (4 Prompts - Progressive Bloom's)"

**Dependencies**: Task 5.2

---

### Task 5.5: Integrate CoLearning Elements in Lesson 4
**Priority**: SHOULD-HAVE | **Effort**: 1h | **Status**: Not Started

**Description**: Embed CoLearning elements (💬🎓🚀✨) into Lesson 4.

**Acceptance Criteria**:
- [ ] **🎓 Instructor Commentary** (after Section 4)
  - Message: "Syntax is cheap—semantics is gold"
  - Explanation: Why f-strings are temporary, patterns permanent
  - Teaching: AI handles syntax; focus on intent

- [ ] **✨ Teaching Tips**:
  - Tip 1: When you see error, copy message and ask AI
  - Tip 2: Indentation errors are common; configure editor to show whitespace
  - Tip 3: Use AI for platform-specific troubleshooting

- [ ] **🚀 CoLearning Challenge** (before exercises)
  - Challenge: Write simple program with variables, print, f-strings
  - Ask AI: "Does this look correct? Any improvements?"
  - Purpose: Scaffolded practice; AI feedback

- [ ] All elements clearly marked
- [ ] No content duplication

**Reference**: `ai-collaborate-teaching` skill + Plan Section "Lesson 4: CoLearning Elements"

**Dependencies**: Tasks 5.2, 5.3, 5.4

---

### Task 5.6: Lesson 4 Quality Review
**Priority**: SHOULD-HAVE | **Effort**: 1h | **Status**: Not Started

**Description**: Internal quality check for Lesson 4.

**Acceptance Criteria**:
- [ ] Reading level: Grade 7-8
- [ ] Grammar and spelling: Proofread
- [ ] Code examples: All tested and run
- [ ] Type hints: 100% coverage
- [ ] Comments: Clear intent explanations
- [ ] CEFR level: A2
- [ ] Cognitive load: 5 concepts
- [ ] Learning objective: Student can write and run programs
- [ ] "Try With AI" section complete with 4 prompts
- [ ] No forward references
- [ ] Common mistakes addressed

**Reference**: Quality Assurance Checklist in plan.md

**Dependencies**: Tasks 5.2, 5.3, 5.4, 5.5

---

## PHASE 6: Lesson 5 – Capstone Project

### Task 6.1: Outline Capstone and Design Specification
**Priority**: MUST-HAVE | **Effort**: 1.5h | **Status**: Not Started

**Description**: Plan capstone structure with clear specification and design.

**Acceptance Criteria**:
- [ ] Capstone program specification clear:
  - Purpose: Collect personal information from user
  - Input: Name, age, favorite color (at least)
  - Output: Formatted summary with validation
  - Requirements: Type hints, validation, f-strings

- [ ] Content sections outlined:
  - Section 1: Understanding the Capstone (why? what?)
  - Section 2: Program Design Before Coding (specification-first thinking)
  - Section 3: Step-by-Step Build (implementation with AI)
  - Section 4: Testing and Validation (verification)

- [ ] Code template designed:
  - Section headers (comments)
  - Input collection with `input()`
  - Type conversion: `int(input(...))`
  - Validation with `isinstance()`
  - Formatted output with f-strings

- [ ] CoLearning elements:
  - 🚀 CoLearning Challenge (design phase with AI)
  - 💬 AI Colearning Prompt (code review)
  - ✨ Teaching Tips (type hints essential, test end-to-end)

- [ ] Acceptance criteria listed:
  - Runs without error ✓
  - Collects 3+ pieces of information ✓
  - Type hints on all variables ✓
  - Uses f-strings ✓
  - Includes comments ✓
  - Validates age with isinstance() ✓
  - Displays formatted summary ✓

- [ ] Approved before full writing

**Reference**: Plan Section "Lesson 5: Capstone Project"

**Dependencies**: Tasks 1.1, 1.2

---

### Task 6.2: Write Lesson 5 Core Content
**Priority**: MUST-HAVE | **Effort**: 3h | **Status**: ✅ COMPLETED

**Description**: Write full Lesson 5 content including capstone guide.

**Acceptance Criteria**:
- [ ] 4 sections written:
  - Understanding the Capstone (what you'll build and why)
  - Program Design First (specification-driven approach)
  - Step-by-Step Build (implementation walkthrough)
  - Testing and Validation (verification process)

- [ ] Understanding the Capstone section:
  - Explain this integrates all Chapter 13 concepts
  - This is specification-driven development in miniature
  - Process: Understand WHAT → Design HOW → Code → Validate

- [ ] Program Design section:
  - Design BEFORE coding
  - Answer: What will it do? What input? What output? What could go wrong?
  - Create design document in plain English first
  - Reference this as "specification-first thinking"

- [ ] Step-by-Step Build section:
  - **Introduce input() first** (FR-022): "The `input()` function asks the user to type something. It shows a prompt, waits for them to type and press Enter, then gives you what they typed as a string." (1-2 lines BEFORE first use)
  - Use `input()` to ask questions
  - **Introduce int() first** (FR-022): "The `int()` function converts text (strings) to numbers. Since `input()` gives you a string, use `int()` to turn it into a number you can work with." (1-2 lines BEFORE first use)
  - Convert string to int with `int()`
  - Validate with `isinstance()` (already introduced in Lesson 3)
  - Build output with f-strings (already introduced in Lesson 4)
  - Add section comments for clarity
  - Include AI collaboration prompts ("Ask your AI to review this...")

- [ ] Testing and Validation section:
  - Run program multiple times
  - Test normal case: valid inputs
  - Test edge case: non-numeric age
  - Verify program matches specification
  - Ask: "Does this work exactly as I designed?"

- [ ] Code template provided (copy-paste ready)
- [ ] Reading level: Grade 7-8
- [ ] Cognitive load: 0 new concepts (integration only)
- [ ] CEFR level: B1 (independent application to real context)
- [ ] Emphasis on specification-first thinking

**Reference**:
- Plan Section "Lesson 5: Capstone Project"
- `.claude/output-styles/lesson.md`

**Dependencies**: Task 6.1

---

### Task 6.3: Create Capstone Code Template
**Priority**: MUST-HAVE | **Effort**: 1.5h | **Status**: ✅ COMPLETED

**Description**: Write, test, and refine the capstone code template for students.

**Acceptance Criteria**:
- [ ] **Full working template provided**:
  ```python
  # Personal Information Collector
  # This program collects user info and displays summary

  # ===== COLLECT INFORMATION =====
  print("=== Personal Information Collector ===\n")

  name: str = input("What is your name? ")
  age_str: str = input("How old are you? ")
  favorite_color: str = input("What is your favorite color? ")

  # ===== VALIDATE AND CONVERT =====
  age: int = int(age_str)
  is_age_valid: bool = isinstance(age, int) and age > 0

  # ===== DISPLAY SUMMARY =====
  print("\n=== Your Profile ===")
  print(f"Name: {name}")
  print(f"Age: {age}")
  print(f"Favorite Color: {favorite_color}")
  print(f"Age Valid: {is_age_valid}")
  print(f"\nThank you, {name}! Your information has been recorded.")
  ```

- [ ] All requirements demonstrated:
  - Collects 3 pieces of information ✓
  - Type hints on all variables ✓
  - Uses `input()` for user interaction ✓
  - Converts string to int ✓
  - Validates with `isinstance()` ✓
  - Uses f-strings for formatted output ✓
  - Section comments for clarity ✓

- [ ] Template tested:
  - Runs without error (Python 3.14+) ✓
  - Expected output shown ✓
  - Cross-platform verified ✓
  - Can be copy-pasted by students ✓

- [ ] Alternative/extension versions:
  - Version with additional questions (3+ fields)
  - Version with more complex validation
  - Comments explain each section

**Reference**: Plan Section "Lesson 5: Code Example (Capstone Template)"

**Dependencies**: Task 6.1

---

### Task 6.4: Create Lesson 5 "Try With AI" Prompts
**Priority**: MUST-HAVE | **Effort**: 1.5h | **Status**: ✅ COMPLETED

**Description**: Write 4 progressive "Try With AI" prompts for capstone (closure section).

**Acceptance Criteria**:
- [ ] **Prompt 1 (Understand)**: Program components
  - Fill-in blanks: 4 main components (collect, validate, display, comments)
  - List in order of execution
  - Expected outcome: Understanding program structure

- [ ] **Prompt 2 (Understand)**: Why validate age
  - Question: Why use isinstance() to check age?
  - Question: What goes wrong if we skip validation?
  - Ask AI: Difference between string input and integer needed?
  - Expected outcome: Understanding type conversion necessity

- [ ] **Prompt 3 (Apply)**: Extend the program
  - Add one more question to the program
  - Add type hint for new variable
  - Include in output summary
  - Test the program
  - Ask AI to review extended version
  - Expected outcome: Independent implementation; gains confidence

- [ ] **Prompt 4 (Analyze/Create)**: Reflection on learning (cognitive closure)
  - Reflection questions:
    1. What was hardest? How did you solve it?
    2. Where did you use AI? Why was it helpful?
    3. How would you explain this program to a beginner?
    4. How is this "describing intent first, coding second"?
    5. What would you build next? First step?
  - Ask AI for insights on learning journey
  - Expected outcome: Metacognitive reflection; understanding of AIDD methodology; clarity on next steps

- [ ] All prompts concrete and specific
- [ ] Bloom's progression: Understand → Understand → Apply → Analyze/Create
- [ ] Final prompt provides cognitive closure for entire chapter
- [ ] AI tool referenced

**Reference**: Plan Section "Lesson 5: Try With AI (4 Prompts - Progressive Bloom's)"

**Dependencies**: Task 6.2

---

### Task 6.5: Integrate CoLearning Elements in Lesson 5
**Priority**: SHOULD-HAVE | **Effort**: 1h | **Status**: Not Started

**Description**: Embed CoLearning elements (💬🎓🚀✨) into Lesson 5.

**Acceptance Criteria**:
- [ ] **🚀 CoLearning Challenge** (Start of capstone)
  - Challenge: Design program in plain English before coding
  - Questions: What will you ask? What display? What could go wrong?
  - Ask AI: "Does my design make sense? Should I change anything?"
  - Purpose: Specification-first thinking; AI validates design before code

- [ ] **💬 AI Colearning Prompt** (Mid-capstone, code review)
  - Prompt: "Here's my capstone program. Can you review it? Check: 1) All variables typed? 2) Output looks good? 3) Any improvements? 4) One enhancement suggestion?"
  - Purpose: Code review skill; professional feedback; AI-assisted learning

- [ ] **✨ Teaching Tips**:
  - Tip 1: Type hints describe intent—every variable gets one
  - Tip 2: If program crashes on non-numeric age, that's expected—handle with isinstance()
  - Tip 3: Specification-first thinking: Design FIRST (plain English), code SECOND

- [ ] All elements clearly marked
- [ ] No content duplication

**Reference**: `ai-collaborate-teaching` skill + Plan Section "Lesson 5: CoLearning Elements"

**Dependencies**: Tasks 6.2, 6.3, 6.4

---

### Task 6.6: Lesson 5 Quality Review
**Priority**: SHOULD-HAVE | **Effort**: 1.5h | **Status**: Not Started

**Description**: Internal quality check for Lesson 5 (capstone).

**Acceptance Criteria**:
- [ ] Reading level: Grade 7-8
- [ ] Grammar and spelling: Proofread
- [ ] Code template: Tested and runs correctly
- [ ] Type hints: 100% coverage in template
- [ ] Comments: Clear structure explanation
- [ ] CEFR level: B1 (independent application)
- [ ] Cognitive load: 0 new concepts (integration focus)
- [ ] Learning objective: Student builds working program demonstrating all concepts
- [ ] "Try With AI" section complete with 4 prompts
- [ ] Capstone acceptance criteria clear and testable
- [ ] Connection to specification-first thinking explicit
- [ ] Reflection prompts encourage metacognitive awareness

**Reference**: Quality Assurance Checklist in plan.md

**Dependencies**: Tasks 6.2, 6.3, 6.4, 6.5

---

## PHASE 7: Chapter Integration and Validation

### Task 7.1: Chapter Cross-Reference Review
**Priority**: SHOULD-HAVE | **Effort**: 1h | **Status**: Not Started

**Description**: Verify all lessons reference each other appropriately and flow logically.

**Acceptance Criteria**:
- [ ] Lesson 1 introduces concepts that Lesson 2 builds on ✓
- [ ] Lesson 2 prerequisite for Lesson 3 ✓
- [ ] Lesson 3 builds on Lessons 1-2 ✓
- [ ] Lesson 4 uses variables (Lesson 3) and concepts from Lesson 1 ✓
- [ ] Lesson 5 (capstone) integrates all Lessons 1-4 ✓
- [ ] Cross-references are explicit ("You learned in Lesson X...")
- [ ] No forward references to future chapters
- [ ] CEFR progression clear: A1 → A1-A2 → A2 → A2 → B1
- [ ] Connection to Chapter 14 (Data Types) mentioned at end (without dependency)

**Reference**: Chapter-Level Architecture in plan.md

**Dependencies**: Tasks 2.5, 3.5, 4.6, 5.6, 6.6

---

### Task 7.2: Content Style and Tone Consistency Review
**Priority**: SHOULD-HAVE | **Effort**: 1h | **Status**: Not Started

**Description**: Verify consistent voice, tone, and style across all lessons.

**Acceptance Criteria**:
- [ ] Conversational tone throughout ("you", "your", "we")
- [ ] No inconsistent terminology (same concept called same thing everywhere)
- [ ] Same level of technical detail across all lessons
- [ ] CoLearning elements consistent in style and placement
- [ ] Common mistakes section present in all lessons
- [ ] "Try With AI" prompts follow same structure pattern
- [ ] No sudden shift to formal/technical language
- [ ] Jargon consistent (e.g., "type hint" used same way everywhere)

**Reference**: Output style from `.claude/output-styles/lesson.md` + `CLAUDE.md` tone guidelines

**Dependencies**: Tasks 2.5, 3.5, 4.6, 5.6, 6.6

---

### Task 7.3: Reading Level Verification (All Lessons)
**Priority**: MUST-HAVE | **Effort**: 1h | **Status**: Not Started

**Description**: Run Flesch-Kincaid readability tool on all lessons to verify Grade 7-8 level.

**Acceptance Criteria**:
- [ ] All lessons scanned with Flesch-Kincaid tool
- [ ] **Lesson 1**: Grade 7-8 (target: 7.5)
- [ ] **Lesson 2**: Grade 7-8 (target: 7.5)
- [ ] **Lesson 3**: Grade 7-8 (target: 8.0)
- [ ] **Lesson 4**: Grade 7-8 (target: 8.0)
- [ ] **Lesson 5**: Grade 7-8 (target: 8.0)
- [ ] Any lesson >Grade 9: Simplify sentence structure, reduce complex vocabulary
- [ ] Any lesson <Grade 7: Acceptable (slightly easier is fine)
- [ ] Report generated documenting all results

**Tool**: Flesch-Kincaid readability checker (available in various editors/online)

**Reference**: EVAL-009 in spec.md (Grade 7-8 reading level requirement)

**Dependencies**: Tasks 2.5, 3.5, 4.6, 5.6, 6.6

---

### Task 7.4: Gatekeeping Language Audit
**Priority**: MUST-HAVE | **Effort**: 45m | **Status**: Not Started

**Description**: Search all lessons for gatekeeping language and remove it.

**Acceptance Criteria**:
- [ ] Search for and remove:
  - "easy" (e.g., "Easy installation" → "Installation")
  - "simple" (e.g., "Simple syntax" → "Python syntax")
  - "obvious" (e.g., "Obviously, type hints..." → "Type hints...")
  - "just" (e.g., "Just add a type hint" → "Add a type hint")
  - "simply" (e.g., "Simply run the program" → "Run the program")
  - "basically" (e.g., "Basically, this is..." → "This is...")

- [ ] If any instance found:
  - Replace with direct, non-gatekeeping language
  - Document replacements

- [ ] Zero instances of gatekeeping language in final content

**Reference**: EVAL-010 in spec.md; CLAUDE.md accessibility guidelines

**Dependencies**: Tasks 2.5, 3.5, 4.6, 5.6, 6.6

---

### Task 7.5: Code Example Cross-Platform Testing
**Priority**: MUST-HAVE | **Effort**: 2h | **Status**: Not Started

**Description**: Test all code examples on Windows, Mac, and Linux.

**Acceptance Criteria**:
- [ ] **Lesson 1**: No code examples (content only)

- [ ] **Lesson 2**: Installation verified on all platforms
  - [ ] Windows: Python 3.14+ installed, `python --version` works
  - [ ] Mac: Python 3.14+ installed, `python --version` works
  - [ ] Linux: Python 3.14+ installed, `python --version` works

- [ ] **Lesson 3 Examples**: All 4 code examples tested
  - [ ] Example 1 (basic variable): Runs on Windows ✓, Mac ✓, Linux ✓
  - [ ] Example 2 (all 4 types): Runs on Windows ✓, Mac ✓, Linux ✓
  - [ ] Example 3 (isinstance): Runs on Windows ✓, Mac ✓, Linux ✓
  - [ ] Example 4 (type()): Runs on Windows ✓, Mac ✓, Linux ✓

- [ ] **Lesson 4 Examples**: All 3 code examples tested
  - [ ] Example 1 (Hello World): Runs on Windows ✓, Mac ✓, Linux ✓
  - [ ] Example 2 (variables and print): Runs on Windows ✓, Mac ✓, Linux ✓
  - [ ] Example 3 (f-strings): Runs on Windows ✓, Mac ✓, Linux ✓

- [ ] **Lesson 5 Template**: Capstone code tested
  - [ ] Capstone template: Runs on Windows ✓, Mac ✓, Linux ✓
  - [ ] With sample input: Produces expected output on all platforms ✓

- [ ] All tests documented with:
  - Platform (Windows version, Mac version, Linux distro)
  - Python version (3.14.x exactly)
  - Output screenshot or confirmation
  - Any platform-specific notes

**Reference**: FR-022 in spec.md (cross-platform verification requirement)

**Dependencies**: Tasks 4.3, 5.3, 6.3

---

### Task 7.6: CEFR Proficiency Progression Validation
**Priority**: MUST-HAVE | **Effort**: 1h | **Status**: Not Started

**Description**: Verify CEFR proficiency levels progress correctly across lessons.

**Acceptance Criteria**:
- [ ] **Lesson 1**: A1 (Recognition/Understanding only)
  - [ ] Students can recognize and understand concepts
  - [ ] No application expected yet
  - [ ] Verified: Content limited to explanation, examples, no practice

- [ ] **Lesson 2**: A1-A2 (Recognition → Guided Application)
  - [ ] Students recognize installation steps
  - [ ] Guided application with instructions
  - [ ] Scaffolding present (step-by-step)
  - [ ] Verified: Installation with explicit guidance

- [ ] **Lesson 3**: A2 (Simple Application with Scaffolding)
  - [ ] Students apply concepts to simple, familiar contexts
  - [ ] Type hint syntax practiced
  - [ ] Exercise with type validation
  - [ ] Verified: Create variables with AI validation

- [ ] **Lesson 4**: A2 (Application to Familiar Contexts)
  - [ ] Students run provided programs
  - [ ] Write simple programs (guided)
  - [ ] Interpret output and errors (with AI help)
  - [ ] Verified: Run and modify programs

- [ ] **Lesson 5**: B1 (Independent Application to Real Context)
  - [ ] Students design and build without step-by-step guidance
  - [ ] Apply concepts to own specification (personal info collector)
  - [ ] Handle variations and edge cases independently
  - [ ] Verified: Capstone is student-designed and built

- [ ] No regression: Proficiency never decreases across lessons ✓
- [ ] Progression is clear and intentional ✓

**Reference**: CEFR Proficiency Progression in plan.md

**Dependencies**: Tasks 2.5, 3.5, 4.6, 5.6, 6.6

---

### Task 7.7: Cognitive Load Validation
**Priority**: SHOULD-HAVE | **Effort**: 1h | **Status**: Not Started

**Description**: Verify each lesson stays within cognitive load limits.

**Acceptance Criteria**:
- [ ] **Lesson 1 Concept Count**: 5 new concepts maximum
  - [ ] Concept 1: What is programming language ✓
  - [ ] Concept 2: Python as language ✓
  - [ ] Concept 3: Python's superpower for AI ✓
  - [ ] Concept 4: Why Python for book ✓
  - [ ] Concept 5: Python in dev workflow ✓
  - [ ] Total: 5 ✓ (within A1 limit)

- [ ] **Lesson 2 Concept Count**: 4-5 new concepts maximum
  - [ ] Concept 1: What is installer ✓
  - [ ] Concept 2: Python.org as source ✓
  - [ ] Concept 3: Version importance ✓
  - [ ] Concept 4: Terminal verification ✓
  - [ ] Total: 4 ✓ (within A1-A2 limit)

- [ ] **Lesson 3 Concept Count**: 6 new concepts (includes awareness-only preview)
  - [ ] Concept 1: What is variable ✓
  - [ ] Concept 2: 4 core primitive types (int, str, float, bool) ✓
  - [ ] Concept 3: Type hint syntax ✓
  - [ ] Concept 4: Naming conventions ✓
  - [ ] Concept 5: Why type hints matter ✓
  - [ ] Concept 6: Collection types awareness (preview only - no syntax/examples) ✓
  - [ ] Total: 6 ✓ (acceptable - preview concept has minimal cognitive load)

- [ ] **Lesson 4 Concept Count**: 5 new concepts maximum
  - [ ] Concept 1: Indentation ✓
  - [ ] Concept 2: Comments ✓
  - [ ] Concept 3: print() function ✓
  - [ ] Concept 4: F-strings ✓
  - [ ] Concept 5: .py files and execution ✓
  - [ ] Total: 5 ✓ (within A2 limit)

- [ ] **Lesson 5 Concept Count**: 0 new concepts (integration only)
  - [ ] No new concepts taught ✓
  - [ ] All concepts from Lessons 1-4 applied ✓
  - [ ] Total: 0 ✓ (within B1 limit)

- [ ] All lessons within limits ✓

**Reference**: Cognitive Load Validation in plan.md

**Dependencies**: Tasks 2.5, 3.5, 4.6, 5.6, 6.6

---

### Task 7.8: Forward Reference Check
**Priority**: MUST-HAVE | **Effort**: 1h | **Status**: Not Started

**Description**: Search all lessons for forbidden forward references.

**Acceptance Criteria**:
- [ ] Search for and remove any mentions of:
  - Control flow (if/else, for/while loops, break, continue)
  - Functions (def, function definitions, function calls beyond print and input)
  - OOP (class, object, inheritance, methods beyond type hints)
  - Collections (list[], dict{}, set, tuple)
  - Advanced concepts (duck typing, decorators, generators, etc.)

- [ ] Allowed concepts only:
  - Variables and type hints ✓
  - print() and input() functions ✓
  - type() and isinstance() functions ✓
  - F-strings ✓
  - Basic syntax (indentation, comments) ✓

- [ ] If any forbidden concept found:
  - Remove or defer to later chapter
  - Document removal

- [ ] Zero forward references in final content ✓

**Reference**: FR-018 in spec.md (No forward references constraint)

**Dependencies**: Tasks 2.5, 3.5, 4.6, 5.6, 6.6

---

### Task 7.9: Built-in Function Introduction Check
**Priority**: MUST-HAVE | **Effort**: 45m | **Status**: Not Started

**Description**: Verify all built-in functions are introduced with 1-2 line explanation BEFORE first use (FR-022).

**Acceptance Criteria**:
- [ ] Search all lessons for built-in functions used:
  - print() (used in Lessons 3, 4, 5)
  - input() (used in Lesson 5)
  - type() (used in Lesson 3)
  - isinstance() (used in Lessons 3, 5)
  - int() (used in Lesson 5)

- [ ] Verify each function has introduction BEFORE first use:
  - [ ] **Lesson 3**: print(), type(), isinstance() all introduced before examples
  - [ ] **Lesson 4**: print() introduced before examples (not assumed from Lesson 3)
  - [ ] **Lesson 5**: input() and int() introduced before capstone code

- [ ] Introduction pattern verified:
  - [ ] Each introduction is 1-2 lines
  - [ ] Pattern: "The `function_name()` does [what]. [Why useful]."
  - [ ] Introduction appears IMMEDIATELY before first use (not buried earlier)
  - [ ] Beginner-friendly language (no jargon without explanation)

- [ ] If any function used without introduction:
  - [ ] Add 1-2 line introduction before first use
  - [ ] Document addition

- [ ] Zero instances of unexplained built-in functions ✓

**Reference**: FR-022 in spec.md (Built-in function introduction requirement)

**Dependencies**: Tasks 2.5, 3.5, 4.6, 5.6, 6.6

---

## PHASE 8: Technical Review and Final Approval

### Task 8.1: Technical Reviewer – Code Quality Check
**Priority**: MUST-HAVE | **Effort**: 2h | **Status**: Not Started

**Description**: Technical reviewer validates all code examples for correctness and standards.

**Acceptance Criteria**:
- [ ] All code examples:
  - [ ] Run without errors on Python 3.14+ ✓
  - [ ] Have type hints on 100% of variables ✓
  - [ ] Follow PEP 8 style guide ✓
  - [ ] Include comments explaining intent ✓
  - [ ] Are appropriate for beginner level ✓
  - [ ] Cross-platform compatible (Windows/Mac/Linux) ✓

- [ ] Specific checks:
  - [ ] Lesson 3: All 4 type examples correct (int, str, float, bool)
  - [ ] Lesson 4: print() and f-strings used correctly
  - [ ] Lesson 5: Capstone includes input(), type conversion, isinstance()

- [ ] No prohibited patterns:
  - [ ] No control flow (if/else, loops)
  - [ ] No function definitions (beyond built-in functions)
  - [ ] No OOP (class definitions)
  - [ ] No collections (lists, dicts, sets)

- [ ] Critical issues: 0 ✓
- [ ] Major issues: 0 (or documented with remediation plan) ✓
- [ ] Minor issues: Documented and reviewed ✓

**Report**: Technical review passes with verdict PASS/FAIL

**Dependencies**: Tasks 4.3, 5.3, 6.3

---

### Task 8.2: Pedagogical Review – Learning Effectiveness
**Priority**: SHOULD-HAVE | **Effort**: 1.5h | **Status**: Not Started

**Description**: Pedagogical reviewer assesses learning effectiveness and clarity.

**Acceptance Criteria**:
- [ ] All lessons:
  - [ ] Have exactly 1 clear learning objective ✓
  - [ ] Objective is measurable and testable ✓
  - [ ] Aligns with appropriate Bloom's level ✓
  - [ ] Content matches objective ✓
  - [ ] "Try With AI" section reinforces objective ✓

- [ ] CEFR progression:
  - [ ] A1 → A1-A2 → A2 → A2 → B1 progression clear ✓
  - [ ] Scaffolding appropriate for each level ✓
  - [ ] No cognitive leaps without support ✓

- [ ] Engagement:
  - [ ] CoLearning elements present and purposeful ✓
  - [ ] Tone conversational and accessible ✓
  - [ ] Real-world connections explicit ✓
  - [ ] AI collaboration positioned as partnership ✓

- [ ] Common mistakes:
  - [ ] Addressed proactively ✓
  - [ ] Explanations clear ✓

- [ ] "Try With AI" sections:
  - [ ] Exactly 4 prompts per lesson ✓
  - [ ] Progressive Bloom's levels ✓
  - [ ] Concrete and specific (not "ask about X") ✓
  - [ ] Expected outcomes clear ✓
  - [ ] No additional closure content (no summaries, checklists) ✓

**Report**: Pedagogical review with verdict PASS/CONDITIONAL/NEEDS WORK

**Dependencies**: Tasks 2.5, 3.5, 4.6, 5.6, 6.6

---

### Task 8.3: Specification Alignment Check
**Priority**: MUST-HAVE | **Effort**: 1h | **Status**: Not Started

**Description**: Verify all content aligns with approved specification.

**Acceptance Criteria**:
- [ ] **Learning objectives met**:
  - [ ] LO-001 (Explain Python) addressed in Lesson 1 ✓
  - [ ] LO-002 (Install Python) addressed in Lesson 2 ✓
  - [ ] LO-003 (Type hints) addressed in Lesson 3 ✓
  - [ ] LO-004 (Run programs) addressed in Lesson 4 ✓
  - [ ] LO-005 (Capstone) addressed in Lesson 5 ✓

- [ ] **Success evals supported**:
  - [ ] EVAL-001: Content helps students explain Python ✓
  - [ ] EVAL-002: Type hints importance emphasized ✓
  - [ ] EVAL-003: Concept distinctions clear ✓
  - [ ] EVAL-004: Installation process detailed ✓
  - [ ] EVAL-005: Type hint syntax taught ✓
  - [ ] EVAL-006: Program execution explained ✓
  - [ ] EVAL-007: Capstone included ✓
  - [ ] EVAL-008: "Try With AI" prompts provided ✓
  - [ ] EVAL-009: Reading level meets requirement ✓
  - [ ] EVAL-010: No gatekeeping language ✓
  - [ ] EVAL-011: Code examples work ✓

- [ ] **Functional requirements met**:
  - [ ] FR-001: 5 lessons (4 + capstone) ✓
  - [ ] FR-002: Each lesson ≤ 5 concepts ✓
  - [ ] FR-003: "Try With AI" sections present (4 prompts each) ✓
  - [ ] FR-004: AI-Native Learning pattern applied ✓
  - [ ] FR-005–FR-009: Content requirements addressed ✓
  - [ ] FR-010–FR-013: Code standards met ✓
  - [ ] FR-014–FR-017: Pedagogical requirements met ✓
  - [ ] FR-018–FR-021: Constraints respected ✓
  - [ ] FR-022–FR-024: Validation and quality met ✓

**Report**: Specification alignment with PASS/FAIL verdict

**Dependencies**: Tasks 2.5, 3.5, 4.6, 5.6, 6.6, 7.1-7.8

---

### Task 8.4: Human Final Review & Approval
**Priority**: MUST-HAVE | **Effort**: 1.5h | **Status**: Not Started

**Description**: Human subject matter expert reviews entire chapter and approves for publication.

**Acceptance Criteria**:
- [ ] All prior tasks completed and documented ✓
- [ ] Technical review: PASS ✓
- [ ] Pedagogical review: PASS ✓
- [ ] Specification alignment: PASS ✓
- [ ] Reading level verified: Grade 7-8 ✓
- [ ] Code examples tested: All platforms ✓
- [ ] No forward references ✓
- [ ] CEFR progression validated ✓
- [ ] Cognitive load verified ✓
- [ ] Content flows logically ✓
- [ ] "Try With AI" prompts effective ✓
- [ ] CoLearning elements purposeful ✓
- [ ] Connection to AIDD methodology clear ✓
- [ ] Ready for publication ✓

**Sign-off**: Human reviewer provides final approval (APPROVED / REQUEST CHANGES)

**If changes requested**: Document specific issues and return to relevant lesson task for remediation

**Dependencies**: Tasks 8.1, 8.2, 8.3

---

## COMPLETION CRITERIA

Chapter 13 is considered **COMPLETE** and **READY FOR PUBLICATION** when:

**All Phases Complete**:
- [ ] Phase 1: Chapter structure created ✓
- [ ] Phase 2: Lesson 1 written and reviewed ✓
- [ ] Phase 3: Lesson 2 written and reviewed ✓
- [ ] Phase 4: Lesson 3 written and reviewed ✓
- [ ] Phase 5: Lesson 4 written and reviewed ✓
- [ ] Phase 6: Lesson 5 written and reviewed ✓
- [ ] Phase 7: Chapter integration validated ✓
- [ ] Phase 8: Technical and final review passed ✓

**Quality Verification**:
- [ ] Reading level: Grade 7-8 (verified with tool)
- [ ] Code: All examples tested on Windows, Mac, Linux
- [ ] No gatekeeping language
- [ ] No forward references
- [ ] CEFR progression: A1 → A1-A2 → A2 → A2 → B1
- [ ] Cognitive load: All lessons ≤ 5 concepts
- [ ] "Try With AI": Exactly 4 prompts per lesson, no other closures

**Alignment Verification**:
- [ ] Specification: All requirements met
- [ ] Learning objectives: All 5 taught and measurable
- [ ] Success evals: All 11 supported
- [ ] CoLearning pedagogy: Embedded throughout
- [ ] AI-Native Learning: Spec → Generate → Validate pattern clear

**Final Approval**:
- [ ] Technical reviewer: PASS
- [ ] Pedagogical reviewer: PASS
- [ ] Specification alignment: PASS
- [ ] Human final review: APPROVED FOR PUBLICATION

---

## Summary

This task checklist breaks Chapter 13 implementation into **6 distinct phases** and **35 specific tasks**, totaling **40-50 hours of effort**. Each task has:
- Clear purpose and acceptance criteria
- Specific file locations and references
- Dependencies documented
- Priority level (MUST/SHOULD/NICE-TO-HAVE)
- Effort estimation

**Next Steps**:
1. Assign owner for Chapter 13 implementation
2. Create feature branch: `016-part-4-chapter-13`
3. Begin Phase 1 tasks (Chapter structure)
4. Move sequentially through remaining phases
5. Maintain quality standards per acceptance criteria
6. Complete all reviews before marking DONE

**Support Resources**:
- Approved specification: `specs/016-part-4-chapter-13/spec.md`
- Detailed plan: `specs/016-part-4-chapter-13/plan.md`
- Output styles: `.claude/output-styles/lesson.md`, `.claude/output-styles/chapters.md`
- Constitutional guidance: `.specify/memory/constitution.md`
- Pedagogical skills: `.claude/skills/` (learning-objectives, concept-scaffolding, ai-collaborate-teaching, etc.)

---

**Status**: ✅ Task Checklist Ready for Implementation
**Created**: 2025-11-08
**Maintained By**: Chapter Planning Agent

