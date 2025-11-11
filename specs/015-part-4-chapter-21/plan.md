# Chapter 21: Exception Handling – Implementation Plan

**Status**: Ready for Implementation
**Chapter**: 21
**Part**: 4 (Python Fundamentals)
**Complexity Tier**: Intermediate (A2-B1 CEFR)
**Total Duration**: 3-4 hours (approx. 45 minutes per lesson)
**Source Spec**: `specs/015-part-4-chapter-21/spec.md`

---

## Plan Overview

Chapter 21 teaches exception handling as a core defensive programming skill—transforming errors from program killers into teachable moments. The chapter progresses through five lessons, each building on prior knowledge:

1. **Lesson 1 (A2)**: Exception fundamentals—what exceptions are, basic try/except, recognizing error types
2. **Lesson 2 (A2-B1)**: Multi-block control flow—except, else, finally, understanding when each runs
3. **Lesson 3 (B1)**: Raising and custom exceptions—functions that validate inputs and signal errors
4. **Lesson 4 (B1)**: Error handling strategies—defensive patterns for real-world scenarios
5. **Lesson 5 (B1)**: Capstone project—robust CSV file parser integrating all concepts

**CEFR Progression**: A2 (recognition + simple application with scaffolding) → B1 (independent application in unfamiliar problems)

**Key Philosophy**: Book teaches error-handling fundamentals. AI companion handles syntax complexity. Capstone demonstrates integration.

---

## CEFR Proficiency & Cognitive Load Validation

| Lesson | CEFR Level | Bloom's | New Concepts | Status |
|--------|-----------|--------|-------------|--------|
| 1 | A2 | Understand | 4 (within limit) | ✓ |
| 2 | A2-B1 | Apply | 3 (within limit) | ✓ |
| 3 | B1 | Apply/Analyze | 3 (within limit) | ✓ |
| 4 | B1 | Apply/Analyze | 2 (within limit) | ✓ |
| 5 | B1 | Apply | 0 (integration) | ✓ |

**Progression**: A2 → B1 (no zigzag) ✓ | **Concept Distribution**: 4+3+3+2+0=14 total ✓ | **All 7 spec concepts taught** ✓

---

## Lesson-by-Lesson Architecture

### Lesson 1: Exception Fundamentals (A2 Level)

**Learning Objective**: Understand what exceptions are, why they occur, how try/except prevents crashes

**Skills Taught**:
- Recognizing Exception Types — A2 — Technical — Student identifies ValueError, TypeError, ZeroDivisionError by name/context
- Understanding Try/Except Flow — A2 — Technical — Student explains what code runs when
- Reading Tracebacks — A2 — Technical — Student identifies error location from traceback

**Concepts** (4 total):
1. Exceptions as objects (type, message, traceback)
2. try/except block structure (attempt code, catch if error)
3. Common exception types (ValueError, TypeError, ZeroDivisionError, KeyError)
4. Tracebacks (reading error messages to find source)

**Prerequisites**: Variables, functions, control flow (Chapters 12-20)

**Duration**: 45 minutes

**Content Structure**:
- Opening: Why errors happen and why try/except matters
- What are exceptions? (errors as objects)
- Anatomy of a traceback (reading error messages)
- Basic try/except structure
- Three common exception types with examples
- Simple exercises catching exceptions
- **CoLearning**: Ask AI "What exceptions can my function raise?"
- **"Try With AI"**: 4 prompts (Remember → Understand → Apply → Analyze)

**Code Examples** (4):
1. Uncaught exception crash → Same with try/except
2. ValueError from `int(input())` with non-numeric input
3. TypeError using wrong operation on type
4. ZeroDivisionError with recovery

**Exercises** (3):
- Catch ValueError from string-to-int conversion
- Catch ZeroDivisionError; provide fallback
- Read traceback and identify error source

**AI-Native CoLearning** (throughout lesson):
- 💬 After try/except intro: Explore how Python determines exception type
- 🎓 After exceptions shown: Emphasize semantics not syntax
- 🚀 Mid-lesson: Specification-driven exception handling
- ✨ Before exercises: Use Claude Code to test exception handling

---

### Lesson 2: Except, Else, and Finally (A2-B1 Level)

**Learning Objective**: Understand multi-block exception handling; know when each block executes

**Skills Taught**:
- Multiple Exception Handling — A2-B1 — Technical — Write 2+ except blocks for different types
- Control Flow Understanding — A2-B1 — Technical — Predict which block runs for each error
- Else/Finally Patterns — B1 — Technical — Use else/finally for cleanup and guaranteed execution

**Concepts** (3 new):
1. Multiple except blocks (different errors, different handling)
2. else block (runs only if NO exception)
3. finally block (runs regardless, guaranteed)

**Prerequisites**: Lesson 1

**Duration**: 45 minutes

**Content Structure**:
- Review single try/except
- Why multiple except blocks? (different error types)
- finally block: cleanup code that always runs
- else block: code that runs only if no exception
- Order matters (except first/last, finally always last)
- Real-world pattern: file operations with finally
- **CoLearning**: Why do I need else when except works?
- **"Try With AI"**: 4 prompts (Remember → Understand → Apply → Analyze)

**Code Examples** (4):
1. Single except → Multiple except blocks
2. try/except/else showing when else runs
3. try/except/finally showing guaranteed execution
4. Complete four-block structure

**Exercises** (3):
- Multiple except blocks for different errors
- try/except/else where else runs on success
- try/except/finally with cleanup

**AI-Native CoLearning**:
- 💬 After multi-except: Show me two exceptions; why catch separately?
- 🎓 After finally intro: Semantics not syntax; when to use finally
- 🚀 Mid-lesson: Ask AI about file operations with finally
- ✨ Before exercises: Test different exception scenarios

---

### Lesson 3: Raising and Custom Exceptions (B1 Level)

**Learning Objective**: Write functions that raise exceptions; create custom exception classes

**Skills Taught**:
- Raising Exceptions Strategically — B1 — Technical — Functions check preconditions, raise when violated
- Custom Exception Design — B1 — Technical — Create exception classes for domain errors
- Error Message Clarity — B1 — Conceptual — Write messages that explain what and why

**Concepts** (3 new):
1. raise statement (explicitly signal error)
2. Custom exception classes (inherit from Exception)
3. Meaningful error messages (what failed, why)

**Prerequisites**: Lessons 1-2

**Duration**: 45 minutes

**Content Structure**:
- Why raise exceptions? (defensive programming)
- raise statement and syntax
- Creating custom exception classes
- When raise vs. return error codes
- Meaningful error messages (context matters)
- **CoLearning**: How do I design custom exceptions?
- **"Try With AI"**: 4 prompts (Remember → Understand → Apply → Analyze)

**Code Examples** (4):
1. Function without validation → With raise
2. Custom exception class definition
3. Raising custom exception with message
4. Function with multiple validations

**Exercises** (3):
- Validate age, raise custom exception
- Validate email format, raise custom exception
- Multiple validations, multiple custom exceptions

**AI-Native CoLearning**:
- 💬 After raise intro: Raising vs. return codes; when use each?
- 🎓 After custom exceptions: Think about errors your function encounters
- 🚀 Mid-lesson: Ask AI about banking system exceptions
- ✨ Before exercises: Should exception store just message or more data?

---

### Lesson 4: Error Handling Strategies (B1 Level)

**Learning Objective**: Apply defensive patterns to realistic scenarios; choose appropriate strategies

**Skills Taught**:
- Strategic Error Recovery — B1 — Technical — Choose retry/fallback/degradation/logging
- Real-World Analysis — B1 — Conceptual — Identify what errors could occur
- Logging for Debugging — B1 — Technical — Include context for error diagnosis

**Concepts** (2 new):
1. Error handling strategies (retry, fallback, degradation, logging)
2. Defensive programming patterns

**Prerequisites**: Lessons 1-3

**Duration**: 45 minutes

**Content Structure**:
- Beyond catching: what happens after exception?
- Strategy 1: Retry logic (attempt multiple times)
- Strategy 2: Fallback values (use default if fails)
- Strategy 3: Graceful degradation (partial success)
- Strategy 4: Logging errors (record context)
- Real-world: file I/O with multiple errors
- **CoLearning**: When retry vs. give up?
- **"Try With AI"**: 4 prompts (Remember → Understand → Apply → Analyze)

**Code Examples** (4):
1. Simple exception handling → Retry logic
2. Graceful degradation (skip bad lines)
3. Fallback values (use default)
4. Logging with context

**Exercises** (3):
- Implement retry logic (attempt 3 times)
- Implement graceful degradation (skip bad data)
- Combine multiple strategies

**AI-Native CoLearning**:
- 💬 After strategies intro: When retry, when give up?
- 🎓 After patterns: Your job is choosing strategy, not memorizing syntax
- 🚀 Mid-lesson: Ask AI about CSV parser error handling
- ✨ Before exercises: Use Claude Code to compare strategies

---

### Lesson 5: Capstone – Robust CSV File Parser (B1 Level)

**Learning Objective**: Integrate all exception handling concepts in realistic project

**Skills Taught**:
- Comprehensive Error Handling — B1 — Technical — Apply multiple strategies in one project
- Specification to Implementation — B1 — Technical — Translate requirements to code
- Testing and Validation — B1 — Technical — Validate error handling works

**Concepts** (0 new):
- Integration only; applies Lessons 1-4

**Prerequisites**: Lessons 1-4

**Duration**: 60 minutes

**Project Specification**:

```
Build a Python program that:

1. Reads CSV file with user records (name, age, email)
2. Validates each row:
   - Name: non-empty string
   - Age: positive integer, 0-150 range
   - Email: contains '@' symbol
3. Handles errors gracefully:
   - FileNotFoundError: tell user file location
   - ValueError: log row, skip, continue
   - PermissionError: tell user permissions issue
4. Reports summary:
   - Total rows, successful rows, rows skipped
   - Log entries for debugging
```

**Content Structure**:
- Project overview and requirements
- Decomposing into parts (file open, validation, errors, summary)
- Building piece by piece with testing
- **CoLearning**: What edge cases might I miss?
- **"Try With AI"**: 4 prompts (Understand spec → Implement → Test → Debug)

**Code Examples** (4):
1. Validation functions (age, email)
2. Row processing with error handling
3. File operations with try/except/finally
4. Summary generation

**Test Cases** (5):
1. Valid data → all processed
2. Missing file → FileNotFoundError caught
3. Bad data rows → skipped gracefully
4. Permission denied → PermissionError caught
5. Mixed valid/invalid → correct counts

**AI-Native CoLearning**:
- 💬 Project start: What errors could occur?
- 🎓 After building: Test error handling, not just happy path
- 🚀 During testing: Ask AI for edge-case test data
- ✨ Validation: How do I test that errors work as expected?

---

## Concept Prerequisites & Dependencies

```
Lesson 1: Exceptions, try/except, types, traceback
  ↓ (required for Lesson 2)
Lesson 2: Multiple except, else, finally
  ↓ (required for Lesson 3)
Lesson 3: raise, custom exceptions, messages
  ↓ (required for Lesson 4)
Lesson 4: Strategies, retry, fallback, logging
  ↓ (required for Lesson 5)
Lesson 5: Integration project (applies all)
```

**Critical**: No forward references. Lesson N only uses concepts from Lessons 1-N.

---

## Scaffolding Strategy

### Complexity Progression

- **Lesson 1 (A2)**: Foundation—what exceptions are, simple catching, recognition
- **Lesson 2 (A2-B1)**: Expansion—multiple blocks, control flow, completeness
- **Lesson 3 (B1)**: Design—validating functions, creating exceptions, messages
- **Lesson 4 (B1)**: Application—real scenarios, strategy choices, tradeoffs
- **Lesson 5 (B1)**: Integration—realistic project, testing, reflection

### Cognitive Load Management

- Lesson 1: 4 concepts introduced one at a time ✓
- Lesson 2: 3 new blocks, each explained before combining ✓
- Lesson 3: 3 patterns, simple examples before complex validations ✓
- Lesson 4: 2 strategy concepts, practiced separately then combined ✓
- Lesson 5: 0 new concepts, all review and integration ✓

### Pedagogical Elements

- **Worked Examples**: Every concept has complete working code
- **Conceptual First**: Book explains "why" before "how"
- **Gradual Complexity**: Simple → variations → real-world
- **Error Messages**: All examples show helpful messages (modeling)
- **Testing Pattern**: Every lesson shows how to test exception handling

---

## Graduated Teaching Pattern (Constitution Principle 13)

### Tier 1: Book Teaches (Foundational)

**What book explains directly**:
- What exceptions are and why they matter
- Basic try/except structure and control flow
- Common exception types and recognition
- Why custom exceptions help clarity
- When to raise exceptions (defensive philosophy)

**Rationale**: Stable, foundational concepts requiring deep understanding

### Tier 2: AI Companion Handles (Complex Syntax)

**What AI handles**:
- All 60+ Python built-in exception types
- Exception hierarchy and inheritance
- Exception chaining (raise from, __cause__)
- Context managers and with statement
- Advanced filtering and re-raising patterns

**Rationale**: Complex syntax students don't need to memorize; AI shows variations

### Tier 3: Capstone Demonstrates (Integration)

**What capstone shows**:
- Real scenario: CSV file parsing with errors
- Multiple error types handled strategically
- Logging and error reporting
- Testing error-handling code
- Specification → implementation → testing cycle

**Rationale**: Integration at realistic scale; not toy examples

---

## AI-Native CoLearning Distribution

### 💬 AI Colearning Prompts (Exploration)

- **Lesson 1**: "Explain how Python determines exception type"
- **Lesson 2**: "Show difference between catching multiple vs. all exceptions"
- **Lesson 3**: "Explain difference between raising and returning error codes"
- **Lesson 4**: "When should you retry vs. give up?"
- **Lesson 5**: "What edge cases might my parser miss?"

**Placement**: After book introduces concept, AI explores deeper

### 🎓 Instructor Commentary (Semantics First)

- **Lesson 1**: "Semantics is gold. Understand WHEN to expect errors."
- **Lesson 2**: "Your job: understand control flow, not memorize blocks."
- **Lesson 3**: "Design WHAT errors your function encounters."
- **Lesson 4**: "Choose strategy; syntax is implementation details."
- **Lesson 5**: "Test is: does it handle EVERY error gracefully?"

**Placement**: After code examples, before exercises

### 🚀 CoLearning Challenges (Application)

- **Lesson 1**: "Ask AI to identify all exceptions YOUR function could raise"
- **Lesson 2**: "Ask AI to show 2 exceptions in one function; catch which first?"
- **Lesson 3**: "Design custom exception; ask AI: store extra data?"
- **Lesson 4**: "Compare retry vs. fallback for different error types"
- **Lesson 5**: "Ask AI for edge-case test data; does parser handle all?"

**Placement**: Mid-lesson when students ready for application

### ✨ Teaching Tips (AI Tool Literacy)

- **Lesson 1**: "Use Claude Code to test: intentionally provide bad input"
- **Lesson 2**: "Ask AI to compare: catch multiple vs. catch Exception"
- **Lesson 3**: "Use Claude Code to design exception classes"
- **Lesson 4**: "Ask AI to compare strategies for different error scenarios"
- **Lesson 5**: "Use Claude Code to design tests: simulate FileNotFoundError"

**Placement**: Before exercises, showing specific AI collaboration

---

## "Try With AI" Closure Structure

Every lesson ends with single "Try With AI" section (no additional summaries).

### Pattern (4 Prompts per Lesson)

| # | Bloom's | Example |
|----|---------|---------|
| 1 | Remember | Recall key terms/structures |
| 2 | Understand | Explain concepts in own words |
| 3 | Apply | Demonstrate application to real example |
| 4 | Analyze | Reason about when/why to use concept |

### Tool Selection (Chapter 21 Post-Tools)

- **Primary**: Claude Code (CLI) with code examples
- **Alternative**: Gemini CLI
- **Web Option**: ChatGPT or Claude web for plain-text
- **Note**: "Use your preferred AI companion (Claude Code, Gemini CLI, or ChatGPT web)"

---

## Implementation Checklist

### For Lesson-Writer Subagent

#### Lesson 1
- [ ] 4 code examples (all working, modern type hints)
- [ ] 3 exercises (progressive difficulty)
- [ ] 💬🎓🚀✨ CoLearning elements throughout
- [ ] "Try With AI": 4 prompts (Remember → Understand → Apply → Analyze)
- [ ] No summary sections after "Try With AI"
- [ ] Grade 7-8 reading level, clear explanations

#### Lesson 2
- [ ] 4 code examples (multiple except, else, finally, complete)
- [ ] 3 exercises (except blocks, else, finally)
- [ ] 💬🎓🚀✨ throughout (not just end)
- [ ] "Try With AI": 4 prompts
- [ ] CEFR A2-B1 appropriate (more complex than Lesson 1)
- [ ] 3 new concepts within cognitive load limit

#### Lesson 3
- [ ] 4 code examples (validation pattern, custom exception, multiple validations)
- [ ] 3 exercises (age validation, email validation, multiple)
- [ ] 💬🎓🚀✨ throughout
- [ ] "Try With AI": 4 prompts
- [ ] CEFR B1 level (independent application)
- [ ] 3 new concepts within limit
- [ ] Error messages are descriptive and helpful

#### Lesson 4
- [ ] 4 code examples (retry, fallback, graceful degradation, logging)
- [ ] 3 exercises (retry logic, graceful degradation, combined strategies)
- [ ] 💬🎓🚀✨ throughout
- [ ] "Try With AI": 4 prompts
- [ ] CEFR B1 level (less familiar contexts)
- [ ] Real-world file I/O scenarios included
- [ ] 2 new concepts within limit

#### Lesson 5 (Capstone)
- [ ] Project specification clear
- [ ] Starter code (file opening structure)
- [ ] 4 code examples (validation functions, row processing, error handling)
- [ ] 5+ test cases (valid data, missing file, bad data, permission, mixed)
- [ ] 💬🎓🚀✨ throughout project phases
- [ ] "Try With AI": 4 prompts (Understand → Implement → Test → Debug)
- [ ] CEFR B1 level (integration)
- [ ] 0 new concepts (all review)
- [ ] Student can explain error handling choices

---

## Quality Gates (MUST PASS)

- [ ] All 5 lessons have measurable learning objectives ✓
- [ ] CEFR progression is A2 → B1 (no zigzag) ✓
- [ ] Cognitive load: ≤7 new concepts per lesson ✓
- [ ] No forward references (Lesson N uses only concepts 1-N) ✓
- [ ] All 7 spec concepts assigned to lessons ✓
- [ ] CoLearning elements (💬🎓🚀✨) specified for each lesson ✓
- [ ] "Try With AI" pattern specified (4 prompts, Bloom's progression) ✓
- [ ] Capstone integrates Lessons 1-4 (no new concepts) ✓
- [ ] Constitution Principle 13 (Graduated Teaching) addressed ✓
- [ ] Lesson dependencies clear and testable ✓

---

## Validation Strategy

**Lesson 1**: Write try/except preventing crash; read traceback
**Lesson 2**: Multiple except blocks; predict which runs for error
**Lesson 3**: Create custom exception; function raises it appropriately
**Lesson 4**: Implement error handling strategy; explain choice
**Lesson 5**: Parser handles 4+ error types; provides helpful messages; shows summary

---

## Next Steps

1. Approval of this plan
2. Run `/sp.tasks` to generate detailed task checklist
3. Lesson-writer subagent implements all 5 lessons
4. Technical review (Python 3.14+, type hints, working examples)
5. Validation (CEFR levels, cognitive load, CoLearning elements, closure pattern)
6. Ready for Chapter 22 (IO & File Handling) which builds on this foundation
