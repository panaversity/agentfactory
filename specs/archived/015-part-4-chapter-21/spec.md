# Chapter 21: Exception Handling – Specification

**Feature Branch**: `015-part-4-chapter-21`
**Created**: 2025-11-09
**Status**: Draft
**Chapter**: 21
**Part**: 4 (Python Fundamentals)
**Complexity Tier**: Intermediate (A2-B1 CEFR)

---

## Overview

Chapter 21 teaches students how to write robust Python programs by handling errors gracefully. Rather than letting programs crash, students learn to anticipate errors, handle them strategically, and communicate them effectively. This chapter bridges foundational Python concepts (Chapters 12-20) toward production-ready error handling patterns (used in Chapters 22-29).

**Core Philosophy**: Errors are information. Handling exceptions transforms errors from program killers into teachable moments. Students learn to think defensively—predicting what could go wrong and building resilience into their code.

**AI-Native Learning Approach**:
- Students **describe intent** using type hints and clear error-handling goals
- **Explore with AI** by asking questions like "What exceptions can this raise?"
- **Validate together** by testing code with intentional errors
- **Learn from errors** by asking AI "Why did this happen?"

---

## Success Evals (Business Goals First)

Before specifications, define what success looks like:

| Eval | Business Goal | Measurement |
|------|---------------|-------------|
| **Comprehension** | Students understand why error handling matters | 80%+ can explain "What happens if try/except didn't exist?" |
| **Skill Acquisition** | Students can write exception handlers for real code | 75%+ write valid try/except for at least 2 different exception types |
| **Application** | Students apply error handling to realistic scenarios | 70%+ complete capstone project (robust file parser) |
| **Confidence** | Students feel comfortable encountering errors | 80%+ believe "errors are solvable with AI help" |
| **Accessibility** | Content is readable and engageable | Grade 7-8 reading level, engagement rate > 70% |

---

## User Scenarios & Testing

### User Story 1 - Master Exception Handling Fundamentals (Priority: P1)

A student learns to write basic error-handling code using try/except/else/finally blocks. They understand program flow when errors occur and can gracefully handle the most common errors (TypeError, ValueError, ZeroDivisionError).

**Why this priority**: Foundation for all exception handling. Without understanding try/except flow, students can't progress to advanced patterns.

**Independent Test**: Student can write a try/except block that catches two different exception types and handles each appropriately.

**Acceptance Scenarios**:

1. **Given** a function that divides two numbers, **When** user provides zero as divisor, **Then** ZeroDivisionError is caught and user sees helpful message
2. **Given** a function that converts strings to integers, **When** user provides non-numeric string, **Then** ValueError is caught and user is prompted to retry
3. **Given** code with try/except/else/finally, **When** no exception occurs, **Then** else block executes and finally always runs

---

### User Story 2 - Raise Exceptions and Create Custom Exceptions (Priority: P2)

A student learns to write functions that raise exceptions intentionally when preconditions aren't met. They create custom exception classes to represent domain-specific errors (e.g., "InvalidAgeError", "FileNotFoundError").

**Why this priority**: Enables students to build defensive functions. Custom exceptions let code communicate intent clearly.

**Independent Test**: Student can write a function that raises a custom exception and a caller that catches it with meaningful error recovery.

**Acceptance Scenarios**:

1. **Given** a function that requires positive integers, **When** negative number is provided, **Then** function raises custom exception with descriptive message
2. **Given** a custom exception class, **When** caught in try/except, **Then** specific error handling applies (vs generic Exception)
3. **Given** function that raises exception, **When** called without error handling, **Then** traceback clearly shows where error occurred

---

### User Story 3 - Apply Error Handling to Real Programs (Priority: P3)

A student builds a robust file parser capstone project that handles multiple error scenarios: missing files, invalid formats, permission errors, unexpected data. They apply try/except patterns systematically and validate output.

**Why this priority**: Solidifies understanding through realistic application. Demonstrates how error handling prevents catastrophic failures.

**Independent Test**: Student's file parser handles at least 4 different error scenarios without crashing, providing helpful feedback for each.

**Acceptance Scenarios**:

1. **Given** parser attempts to open non-existent file, **When** FileNotFoundError occurs, **Then** user is told file name and location
2. **Given** parser encounters unexpected data format, **When** ValueError occurs, **Then** parser skips malformed line with log entry
3. **Given** parser completes, **When** some data was skipped due to errors, **Then** summary shows how many lines were processed vs skipped

---

### Edge Cases

- What happens when exception handler itself raises an exception?
- How does exception chaining work (raising from another exception)?
- When should context managers (with statement) be used instead of try/finally?
- What's the difference between catching Exception vs catching specific exception types?

---

## Requirements

### Learning Objectives

By the end of Chapter 21, students will be able to:

1. **Understand (A2)**: Explain what exceptions are, why they occur, and how try/except prevents program crashes
2. **Apply (A2-B1)**: Write try/except blocks that handle specific exception types with appropriate recovery strategies
3. **Raise (B1)**: Write functions that raise custom exceptions when preconditions fail, with descriptive error messages
4. **Design (B1)**: Plan error handling strategy for realistic scenarios (file I/O, type conversion, calculations)
5. **Validate (B1)**: Test code with intentional errors to verify exception handling works as intended

---

### Core Concepts (7 Maximum for Intermediate)

1. **Exceptions as Objects** (A2): Errors in Python are objects with type, message, and traceback. Understanding this foundation enables strategic handling.

2. **try/except/else/finally Control Flow** (A2): The four-block structure determines which code runs when. Students understand each block's purpose and typical use cases.

3. **Exception Types and Hierarchy** (A2-B1): Python's built-in exceptions (ValueError, TypeError, ZeroDivisionError, etc.) form a hierarchy. Catching specific types enables precise error handling.

4. **Raising Exceptions Intentionally** (B1): Functions raise exceptions to signal errors to callers. Students write functions that check preconditions and raise custom exceptions when violated.

5. **Custom Exception Classes** (B1): Students create their own exception classes inheriting from Exception. Custom exceptions communicate domain-specific errors clearly.

6. **Error Handling Strategies** (B1): Beyond just catching errors, students learn patterns: retry logic, fallback values, graceful degradation, logging errors for debugging.

7. **Context Managers (with statement) Overview** (B1): Awareness of context managers for resource management (files, database connections, locks). Deep dive deferred to Chapter 22 (IO & File Handling).

---

### Functional Requirements

- **FR-001**: Chapter MUST teach 7 core concepts on exception handling aligned with CEFR A2-B1
- **FR-002**: Chapter MUST include 5 lessons progressing from foundational to capstone
- **FR-003**: Each lesson MUST end with "Try With AI" section containing exactly 4 prompts
- **FR-004**: Chapter MUST include AI-Native CoLearning elements (💬🎓🚀✨) throughout lessons
- **FR-005**: All code examples MUST run on Python 3.14+ with modern type hints
- **FR-006**: Chapter MUST include real capstone project (CSV file parser with error handling)
- **FR-007**: Chapter MUST be teachable in 3-4 hours for intermediate learners (A2-B1)
- **FR-008**: Chapter MUST contain no forward references to Chapters 22+
- **FR-009**: Chapter MUST use "describe intent" language, not formal SDD terminology

---

## Success Criteria

### Measurable Outcomes

- **SC-001**: 80%+ of students can explain "What happens without try/except?" after Lesson 1
- **SC-002**: 75%+ of students write valid try/except for 2+ exception types after Lesson 2
- **SC-003**: 70%+ of students write custom exception class after Lesson 3
- **SC-004**: 75%+ of students apply error handling strategy to realistic scenario after Lesson 4
- **SC-005**: 70%+ of students complete capstone project (file parser handling 4+ error types) after Lesson 5
- **SC-006**: Content is readable at Grade 7-8 level (Flesch-Kincaid score 45-55)
- **SC-007**: Each lesson includes all 4 "Try With AI" prompts with expected outcomes
- **SC-008**: Zero forward references to Chapters 22-29 (chapter scope only)

---

## Lesson Structure (5 Lessons)

### Lesson 1: Exception Fundamentals (A2 level)
- **Topics**: What exceptions are, try/except flow, common exception types (ValueError, TypeError, ZeroDivisionError)
- **Practice**: Write try/except for simple errors (division by zero, invalid conversion)
- **CoLearning**: Ask AI "What types of errors can my code raise?" and "What does the traceback tell me?"
- **"Try With AI"**: 4 prompts (Recall → Understand → Apply → Analyze)

### Lesson 2: Except, Else, and Finally (A2-B1 level)
- **Topics**: Multiple except blocks, else (runs if no exception), finally (always runs)
- **Practice**: Handle multiple exception types; understand program flow with all 4 blocks
- **CoLearning**: Ask AI "Why do I need else/finally when except already handles errors?"
- **"Try With AI"**: 4 prompts (progressive Bloom's levels)

### Lesson 3: Raising and Custom Exceptions (B1 level)
- **Topics**: raise statement, custom exception classes, meaningful error messages
- **Practice**: Write functions that validate inputs and raise custom exceptions
- **CoLearning**: Ask AI "How do I design custom exceptions?" and "What makes a good error message?"
- **"Try With AI"**: 4 prompts (Apply → Analyze → Evaluate → Create)

### Lesson 4: Error Handling Strategies (B1 level)
- **Topics**: Defensive programming patterns, retry logic, fallback values, logging
- **Practice**: Apply multiple strategies to realistic scenarios
- **CoLearning**: Ask AI "What's the best error handling strategy for file I/O?" and "How do professionals handle network errors?"
- **"Try With AI"**: 4 prompts (Apply → Analyze → Design → Criticize)

### Lesson 5: Capstone - Robust File Parser (B1 level)
- **Project**: Build file parser that handles FileNotFoundError, ValueError, PermissionError gracefully
- **Spec**: Parser reads CSV, validates data, logs errors, reports summary
- **CoLearning**: Ask AI "What edge cases might my parser miss?" and "How do I validate my error handling is working?"
- **Validation**: Parser handles at least 4 error types; provides helpful user feedback
- **"Try With AI"**: 4 prompts (Understand spec → Implement → Test → Debug issues)

---

## Pedagogical Direction

### What the Book Teaches (Tier 1 - Foundation)
- What exceptions are and why they matter
- Basic try/except structure and flow
- Common built-in exception types
- Writing functions that raise exceptions
- Why custom exceptions help code clarity

### What AI Companion Handles (Tier 2 - Syntax & Complexity)
- Complex exception handling patterns (multiple except blocks, nested try/except)
- Context managers syntax (with statement)
- Exception chaining details (raise from, __cause__, __context__)
- Designing exception hierarchies

### What Capstone Demonstrates (Tier 3 - Integration)
- Real scenario: CSV parser with validation
- Multiple error types handled strategically
- Logging and error reporting
- Specification → validation → implementation cycle

---

## AI-Native CoLearning Elements

Every lesson includes these conversational, exploratory elements:

**💬 AI Colearning Prompt**: After foundational concepts, encourage deeper conceptual understanding with AI
**🎓 Instructor Commentary**: Emphasize reasoning > syntax ("Syntax is cheap — semantics is gold")
**🚀 CoLearning Challenge**: Practice specification-driven thinking with AI collaboration
**✨ Teaching Tip**: Build AI tool literacy and effective collaboration patterns

---

## Capstone Project: Robust CSV File Parser

**Specification** (Student Describes Intent):

Build a Python program that:
1. Reads a CSV file containing user data (name, age, email)
2. Validates each row:
   - Age is a positive integer
   - Email contains '@' symbol
   - Name is non-empty string
3. Handles errors gracefully:
   - FileNotFoundError: Tell user file location
   - ValueError: Log malformed row, continue processing
   - PermissionError: Tell user they lack permissions
4. Reports summary:
   - Total rows processed
   - Rows with errors (skipped)
   - Successfully validated rows

**Expected Outcome**:
- Parser handles at least 4 error scenarios
- User gets helpful feedback for each error
- No program crashes; all errors logged
- Summary shows what was processed

---

## Code Examples (Validated, Modern Python 3.14+)

All examples use modern type hints and are tested on Python 3.14+.

### Example 1: Basic try/except (Lesson 1)
```python
try:
    result = int(input("Enter a number: "))
except ValueError:
    print("That's not a valid number!")
```

### Example 2: Multiple except blocks (Lesson 2)
```python
try:
    numbers = [1, 2, 3]
    result = 10 / numbers[5]
except IndexError:
    print("Index out of range")
except ZeroDivisionError:
    print("Can't divide by zero")
```

### Example 3: Custom exception (Lesson 3)
```python
class InvalidAgeError(Exception):
    pass

def set_age(age: int) -> None:
    if age < 0 or age > 150:
        raise InvalidAgeError(f"Age must be 0-150, got {age}")
    print(f"Age set to {age}")
```

### Example 4: Error handling strategy (Lesson 4)
```python
def read_file_with_retry(filename: str, max_retries: int = 3) -> str:
    for attempt in range(max_retries):
        try:
            with open(filename) as f:
                return f.read()
        except FileNotFoundError:
            print(f"File not found: {filename}")
            return ""
        except PermissionError:
            print(f"Permission denied: {filename}")
            return ""
```

### Example 5: Capstone foundation (Lesson 5)
```python
def validate_row(row: dict) -> bool:
    try:
        age = int(row["age"])
        if age < 0 or age > 150:
            raise ValueError(f"Invalid age: {age}")
        if "@" not in row.get("email", ""):
            raise ValueError("Invalid email format")
        return True
    except ValueError as e:
        print(f"Row error: {e}")
        return False
```

---

## Acceptance Criteria

- [ ] All 5 lessons created with AI-Native Learning pattern
- [ ] Each lesson includes 💬🎓🚀✨ CoLearning elements throughout (not just at end)
- [ ] Each lesson ends with "Try With AI" section ONLY (4 prompts, no summaries)
- [ ] All code examples run on Python 3.14+
- [ ] Type hints used consistently (function signatures show parameter and return types)
- [ ] No SDD terminology (use "describe intent" not "write specification")
- [ ] No forward references (all concepts from Lessons 1-4 available in Lesson 5)
- [ ] Capstone project includes specification, validation steps, expected outcomes
- [ ] All 7 core concepts defined and taught across lessons
- [ ] Chapter is teachable in 3-4 hours for A2-B1 learners
- [ ] Tone is conversational (you, your, we), not documentation style

---

## Assumptions & Non-Goals

### Assumptions
- Students have completed Chapters 1-20 (understand variables, functions, control flow)
- Python 3.14+ is installed (type hints with | syntax available)
- Students have Claude Code or Gemini CLI for AI collaboration
- Reading level: Grade 7-8 (intermediate, conversational)

### Non-Goals (Deferred to Later Chapters)
- Deep dive into context managers (introduction only; detailed coverage in Chapter 22)
- Exception chaining edge cases (covered in Chapter 22 when file I/O needed)
- Logging module configuration (basic printing in Chapter 21; full logging in production chapters)
- Async exception handling (deferred to Chapter 28 Asyncio)
- Subclassing Exception with complex behavior (covered in Chapter 25 OOP Part II)

---

## CEFR Proficiency Levels (Intermediate Tier)

| Lesson | CEFR Level | What Students Can Do |
|--------|-----------|---------------------|
| 1 | A2 | Recognize exception types; write simple try/except |
| 2 | A2-B1 | Use multiple except blocks; understand flow |
| 3 | B1 | Write custom exceptions; design for validation |
| 4 | B1 | Apply strategies to realistic scenarios |
| 5 | B1 | Apply integrated error handling in capstone |

**Progression**: A2 (recognition + simple application) → B1 (independent application in unfamiliar problems)

---

## Dependencies & Prerequisite Knowledge

**Must Know (Chapters 1-20)**:
- Variables and types (Chapter 14)
- Control flow: if/elif/else (Chapter 17)
- Functions: definition, parameters, return (Chapter 20)
- Loops and iteration (Chapter 17)

**Will Use In Later Chapters**:
- File I/O (Chapter 22 builds on exception handling for file operations)
- Object-Oriented Programming (Chapter 24+ uses custom exceptions in classes)

---

## Next Steps After Chapter 21

- **Chapter 22 (IO & File Handling)**: Exception handling is primary tool for file operations
- **Chapter 23 (Math, Date, Time)**: Exceptions occur during invalid calculations/conversions
- **Chapters 24-25 (OOP)**: Custom exceptions used in class design for domain-specific errors
