# Chapter 17: Control Flow and Loops — Detailed Lesson Plan

**Chapter Type**: Technical (Python code-focused)
**Part**: 4 - Python Fundamentals
**Complexity Tier**: Intermediate (A2-B1 CEFR Proficiency)
**Estimated Total Time**: 4-5 hours (60-75 minutes per lesson)
**Branch**: 001-part-4-chapter-17
**Date**: November 8, 2025

---

## Summary

Chapter 17 teaches decision-making and repetition in Python—core programming concepts that enable students to write programs that respond to conditions and automate repeated tasks. Students progress from simple conditionals (`if/else`) through pattern matching (`match-case`) and loop structures (`for`, `while`) to nested control flow combining both. The chapter builds sequentially on Chapters 13-16 (Python basics, data types, operators, strings) and emphasizes **AI-Native Learning**: students describe intent using clear code structure and type hints, explore patterns through dialogue with AI, validate logic through testing, and learn from errors by asking AI diagnostic questions.

**Key Pedagogical Approach**:
- **Tier 1 (Book Teaches)**: Foundational syntax and simple patterns (`if/else`, `for`, `while` basics)
- **Tier 2 (AI Companion)**: Complex patterns and conversions (`match-case`, nested structures, control flow optimization)
- **Student Role**: Describe intent clearly; validate and learn from AI-generated code

---

## Technical Context

**Language**: Python 3.14+
**No External Dependencies**: Uses only Python standard library
**Prerequisites**: Chapters 13-16 completed (Python basics, data types, operators, strings)
**Type Hints**: Mandatory on all function examples
**Code Testing**: All examples tested on Python 3.14.0
**AI Tools**: Claude Code, Gemini CLI (per learner preference)

---

## Constitution Alignment Check

**✓ Specification-First**: Spec document (spec.md) defines 5 lessons, learning objectives, code examples
**✓ Evals-First**: Chapter defines 13 success evals aligned to business goals (comprehension, skill acquisition, engagement, accessibility)
**✓ Graduated Teaching Pattern**: Lessons progress from Book Teaches (Tier 1) → AI Companion (Tier 2) across lessons
**✓ AI as Co-Reasoning Partner**: "Try With AI" sections integrate dialogue with AI for explanation, validation, and diagnostic learning
**✓ Validation-First Safety**: Every lesson includes Red Flags section and edge-case validation guidance
**✓ No Forward References**: All examples stay within Chapter 17 scope; no references to Chapters 30+ or advanced topics
**✓ Cognitive Load Management**: Each lesson respects A2 (max 7 concepts) and B1 (max 10 concepts) limits from CEFR research
**✓ Lesson Closure Pattern**: Every lesson ends with ONLY a "Try With AI" section (no separate "Key Takeaways" or "What's Next")

---

## Lesson Architecture Summary

| Lesson | Title | CEFR Level | Time | Key Concepts | Skills Count |
|--------|-------|-----------|------|--------------|--------------|
| 1 | Making Decisions with Conditionals | A2 | 45-60 min | if/elif/else, logical operators | 6 |
| 2 | Pattern Matching with match-case | A2-B1 | 50-65 min | match-case, literal patterns, wildcard | 5 |
| 3 | Repetition with Loops | A2-B1 | 55-70 min | for, while, range(), termination | 6 |
| 4 | Controlling Loops | A2-B1 | 55-70 min | break, continue, for...else, while...else | 6 |
| 5 | Nested Control Structures | B1 | 60-75 min | nested if, nested loops, combinations | 6 |
| **TOTAL** | | **A2→B1** | **4-5 hours** | | **29 total skills** |

---

## Detailed Lesson Plans

### Lesson 1: Making Decisions with Conditionals (A2 - Basic Application)

**Learning Objective**: Students will write conditional logic using `if`, `elif`, and `else` statements to make decisions based on multiple conditions, and apply logical operators in decision expressions.

**CEFR Proficiency**: A2 (Basic) — Student can apply conditional logic in simple, guided contexts; understand when to use multiple conditions.

**Skills Taught** (CEFR Levels & Categories):
- Conditional Logic (if/elif/else) — A2, Technical, Apply level
- Boolean Expression Evaluation — A2, Technical, Understand level
- Indentation & Code Structure — A2, Technical, Understand level
- Type-Safe Decision Making — A2, Technical, Apply level
- Comparison Operators Review — A2, Technical, Understand level
- Logical Operators (and/or/not) — A2, Technical, Apply level

**Cognitive Load**: 7 concepts (at A2 limit) ✓

**Content Elements**:
- Quick review of comparison and logical operators (1-2 min, callback to Ch 15)
- Book teaches: `if` statement, `if-else`, `if-elif-else` chaining (20 min)
- Code Examples 1-3 (Tier 1): Age verification, even/odd checker, grade classifier
- AI Companion: Complex conditions (8 min)
- Code Examples 4-5 (Tier 2): Discount eligibility, multi-criteria validation with nested if

**Try With AI** (4 Prompts):
1. **Recall**: "What's the difference between `if` and `elif`?"
2. **Understand**: "Explain how this if-elif-else chain evaluates: [grade example]. What happens when score=85?"
3. **Apply**: "Generate a conditional that checks if a number is positive, negative, or zero. Include type hints."
4. **Analyze**: "What edge cases should I test for age verification logic? List 5 test cases."

**Red Flags**: IndentationError, unreachable code, logical errors (impossible conditions), type mismatches

**Duration**: 45-60 minutes | **Prerequisites**: Ch 13-16 | **Estimated Total Time for Chapter**: 4-5 hours

---

### Lesson 2: Pattern Matching with match-case (A2-B1 - Transitional)

**Learning Objective**: Students will use match-case pattern matching (Python 3.10+) as a modern alternative to if/elif chains, and recognize when match-case improves code readability.

**CEFR Proficiency**: A2-B1 (Transitional) — Student understands `match-case` syntax; can apply to simple patterns; beginning to recognize readability benefits.

**Skills Taught** (CEFR Levels & Categories):
- Pattern Matching Syntax — A2, Technical, Apply level
- Wildcard Pattern Recognition — A2, Technical, Understand level
- Code Readability Assessment — A2-B1, Soft, Analyze level
- Python 3.10+ Syntax Awareness — A2, Technical, Understand level
- Intent Clarity in Code — B1, Soft, Analyze level

**Cognitive Load**: 5 concepts (well under A2-B1 limit) ✓

**Content Elements**:
- Motivation: Readability comparison (if/elif vs match-case)
- Book teaches: `match-case` syntax, literal patterns, wildcard pattern `_` (15 min)
- Code Examples 1-2 (Tier 1): HTTP status codes, menu selection
- AI Companion: Converting if/elif to match-case (12 min)
- Code Examples 3-4 (Tier 2): Traffic light logic, calculator operations
- Readability discussion and tradeoffs

**Try With AI** (4 Prompts):
1. **Recall**: "What's the purpose of the wildcard pattern `_` in match-case?"
2. **Understand**: "Explain how this match-case evaluates when color='yellow': [traffic light example]"
3. **Apply**: "Generate match-case for days of the week (1-7 → day name). Use type hints and include default."
4. **Evaluate**: "Compare readability: this if/elif chain vs. match-case equivalent. When is each better?"

**Red Flags**: Python < 3.10 incompatibility, missing colon, unreachable patterns, type mismatches, missing default case

**Duration**: 50-65 minutes | **Prerequisites**: Lesson 1 | **Estimated Time from Chapter Start**: 1.75 hours

---

### Lesson 3: Repetition with Loops (A2-B1 - Transitional)

**Learning Objective**: Students will implement for loops for definite iteration and while loops for indefinite iteration, and recognize when to use each approach.

**CEFR Proficiency**: A2-B1 (Transitional) — Student writes basic loops; understands `for` for known count and `while` for condition-based repetition; can trace loop execution.

**Skills Taught** (CEFR Levels & Categories):
- For Loop with range() — A2, Technical, Apply level
- Range() Function Mastery — A2, Technical, Apply level
- While Loop Implementation — A2-B1, Technical, Apply level
- Loop Termination Logic — A2-B1, Technical, Understand level
- For vs While Decision — B1, Soft, Analyze level
- Infinite Loop Prevention — A2-B1, Technical, Understand level

**Cognitive Load**: 7 concepts (at A2-B1 limit) ✓

**Content Elements**:
- Motivation: Automation of repetition (5 min)
- Book teaches: `for` loop, `range()` with start/stop/step variations (20 min)
- Code Examples 1-3 (Tier 1): Print 1-10, even numbers, countdown
- Book teaches: `while` loop and termination conditions (10 min)
- Code Example 4 (Tier 1): User input validation with while
- AI Companion: For vs while comparison and infinite loop demonstration (10 min)
- Code Examples 5-6 (Tier 2): Same task both ways, infinite loop + fix

**Try With AI** (4 Prompts):
1. **Recall**: "What's the difference between for and while loops?"
2. **Understand**: "Trace this for loop step-by-step: `for i in range(2, 6, 1): print(i)`. What values does i take?"
3. **Apply**: "Generate a while loop that counts down from 10 to 1. Include type hints and termination logic."
4. **Analyze**: "What happens if I forget to update the loop counter in this while loop? Show the error and fix."

**Red Flags**: Infinite loops, off-by-one errors, loop variable scope confusion, step zero error

**Duration**: 55-70 minutes | **Prerequisites**: Lesson 1-2 | **Estimated Time from Chapter Start**: 2.75 hours

---

### Lesson 4: Controlling Loops (A2-B1 - Transitional)

**Learning Objective**: Students will control loop execution using break and continue statements and recognize when for...else and while...else patterns are appropriate.

**CEFR Proficiency**: A2-B1 (Transitional) — Student implements `break`, `continue`, and `for...else`; understands when each is used; applies to search and validation problems.

**Skills Taught** (CEFR Levels & Categories):
- Break Statement — A2, Technical, Apply level
- Continue Statement — A2, Technical, Apply level
- For...Else Pattern — A2-B1, Technical, Understand-Apply level
- While...Else Pattern — A2-B1, Technical, Apply level
- Loop Control Flow Mastery — B1, Technical, Analyze level
- When to Use Each — B1, Soft, Analyze level

**Cognitive Load**: 7 concepts (at A2-B1 limit) ✓

**Content Elements**:
- Motivation: Early exit, skipping, retry with limit scenarios (5 min)
- Book teaches: `break` statement and search scenarios (10 min)
- Code Example 1 (Tier 1): Search for number in list
- Book teaches: `continue` statement and filtering (8 min)
- Code Example 2 (Tier 1): Skip even numbers
- Book teaches: `for...else` and `while...else` patterns (15 min)
- Code Examples 3, 5 (Tier 1-2): Search with "not found" detection, retry with limit
- AI Companion: Complex loop control (10 min)
- Code Examples 4, 6 (Tier 2): Input validation loop, game logic

**Try With AI** (4 Prompts):
1. **Recall**: "What's the difference between break and continue?"
2. **Understand**: "Trace this for...else execution: [search example]. Does else run if item found?"
3. **Apply**: "Generate a while loop that retries 3 times before giving up. Include break and else."
4. **Evaluate**: "When is for...else more elegant than using a flag variable? Give an example."

**Red Flags**: Misunderstanding when else executes, continue outside loop, break unreachable, overuse of break, break in nested loops

**Duration**: 55-70 minutes | **Prerequisites**: Lesson 1-3 | **Estimated Time from Chapter Start**: 3.5 hours

---

### Lesson 5: Nested Control Structures (B1 - Integration/Synthesis)

**Learning Objective**: Students will nest conditionals and loops to solve multi-step problems requiring combined decision-making and repetition, and recognize when nesting increases or decreases code clarity.

**CEFR Proficiency**: B1 (Intermediate - Integration) — Student applies all Lessons 1-4 concepts in combination; traces complex nested logic; recognizes nesting depth limits.

**Skills Taught** (CEFR Levels & Categories):
- Nested If Statements — B1, Technical, Apply level
- Nested Loops — B1, Technical, Apply level
- Conditionals Inside Loops — B1, Technical, Apply level
- Loops Inside Conditionals — B1, Technical, Apply level
- Loop Invariants & Complexity — B1, Conceptual, Understand-Analyze level
- Refactoring for Clarity — B1, Soft, Analyze level

**Cognitive Load**: 6 concepts (within B1 limit) ✓ — All previous concepts reviewed, no new syntax introduced

**Content Elements**:
- Integration review: Callback to Lessons 1-4 (8 min)
- Book teaches: Nested if statements, decision trees (10 min)
- Code Example 1 (Tier 1): Multi-criteria eligibility with nested if
- Book teaches: Nested loops, 2D iteration (12 min)
- Code Example 2 (Tier 1): Multiplication table
- Book teaches: Conditionals in loops, loops in conditionals (10 min)
- Code Examples 3-5 (Tier 1-2): Filter while iterating, sum positives, process list conditionally
- AI Companion: Complex nested structures (10 min)
- Code Example 6 (Tier 2): Game grid logic
- Refactoring discussion: When nesting is too deep (preview of functions)

**Try With AI** (4 Prompts):
1. **Recall**: "Why is indentation critical in nested structures?"
2. **Understand**: "Trace this nested loop step-by-step: [multiplication table]. What's the output for row=2?"
3. **Apply**: "Generate nested loops to find all pairs (i, j) where i + j = 10 (i from 1-9, j from 1-9)."
4. **Synthesize**: "Describe a real-world problem requiring nested control flow. Ask AI to implement it, then trace one scenario."

**Red Flags**: Indentation errors, loop variable confusion, logic errors, infinite nested loops, deep nesting reducing readability

**Duration**: 60-75 minutes | **Prerequisites**: Lessons 1-4 | **Total Chapter Time**: 4-5 hours ✓

---

## Skills Proficiency Mapping Summary

**Total Skills Taught**: 29 across 5 lessons

**CEFR Proficiency Progression**:
- **Lesson 1**: A2 (6 skills)
- **Lesson 2**: A2-B1 (5 skills)
- **Lesson 3**: A2-B1 (6 skills)
- **Lesson 4**: A2-B1 (6 skills)
- **Lesson 5**: B1 (6 skills)

**Cognitive Load Validation**:
- A2 lessons (Lesson 1): 7 concepts (max limit) ✓
- A2-B1 lessons (Lessons 2-4): 5-7 concepts (well under B1 limit of 10) ✓
- B1 lesson (Lesson 5): 6 concepts (all review, no new syntax) ✓

**Skills by Category**:
- **Technical Skills**: 22 (if/elif/else, match-case, for, while, break, continue, range(), nested structures)
- **Soft Skills**: 4 (Readability Assessment, Decision-Making, Refactoring for Clarity, When to Use Each)
- **Conceptual Skills**: 3 (Boolean Expressions, Loop Invariants, Indentation Understanding)

---

## Implementation Notes for Lesson Writer

### Output Structure
- **Directory**: `book-source/docs/04-Part-4-Python-Fundamentals/17-control-flow-loops/`
- **Files**: `README.md` (overview) + 5 lesson markdown files with descriptive titles
- **YAML Frontmatter**: sidebar_position, title, description, reading_level (Grade 7-9), time_estimate
- **Tone**: Conversational; address student directly; avoid documentation-style language

### Code Standards (Constitution Section IX)
- Python 3.14+ required for all examples
- **Type Hints Mandatory**: All variables and parameters have type annotations
- **No External Libraries**: Only standard library (except `random` in Lesson 4 if needed)
- **Inline Comments**: Explain logic; avoid over-commenting obvious code
- **Copy-Pasteable**: All examples are complete, runnable, and testable

### Graduated Teaching Pattern Application
- **Tier 1 (Book Teaches)**: Examples in main lesson text with full explanation
- **Tier 2 (AI Companion)**: Examples marked clearly; AI prompts specified for code generation
- **Student Validates**: Every example includes expected output and validation approach

### Lesson Closure Requirement (Critical)
- **EVERY lesson ends with "Try With AI" section ONLY**
- No "Key Takeaways", "Summary", "What's Next", or other closing sections
- 4 prompts per lesson following Bloom's progression: Recall → Understand → Apply → Analyze/Evaluate/Synthesize

### Pre-Implementation Checklist
- [ ] All 5 lessons planned (this document)
- [ ] 25+ code examples specified (3-6 per lesson)
- [ ] AI prompts written for all Tier 2 examples
- [ ] "Try With AI" prompts drafted for all lessons
- [ ] Red Flags documented for each lesson
- [ ] Chapter prerequisites verified (Ch 13-16 status)
- [ ] No forward references to Part 5+ chapters
- [ ] CEFR proficiency levels assigned
- [ ] Cognitive load validated

---

## Validation Checklist (For Lesson Writer & Technical Reviewer)

### Pedagogical Quality
- [ ] All 5 lessons exist with detailed learning flows
- [ ] Each lesson has 3-6 code examples (mix Tier 1 + Tier 2)
- [ ] Each lesson ends with "Try With AI" (4 prompts, Bloom's progression)
- [ ] No "Key Takeaways" or "What's Next" sections
- [ ] Conversational tone; Grade 7-9 reading level
- [ ] Learning objectives aligned to CEFR and Bloom's

### Technical Correctness
- [ ] All code examples tested on Python 3.14.0
- [ ] Type hints present on all variables and functions
- [ ] Expected output documented for each example
- [ ] Edge cases identified and tested
- [ ] Red Flags section present with AI troubleshooting prompts

### Constitution Compliance
- [ ] No forward references to Ch 30+ or SDD terminology
- [ ] AI as co-reasoning partner, not coding assistant
- [ ] Validation-first approach emphasized
- [ ] Type hints mandatory; modern Python syntax
- [ ] Each lesson ends ONLY with Try With AI section

### Success Eval Alignment
- [ ] EVAL-001 through EVAL-010 assessment methods specified
- [ ] Comprehension quizzes (80%+ pass rate target) included
- [ ] Skill acquisition exercises with model solutions provided
- [ ] Edge case validation guidance integrated throughout
- [ ] Try With AI prompts support engagement (85% completion target)

---

## Risks & Dependencies

### Technical Risks
- **Python 3.10+ Requirement**: Lesson 2 (match-case) requires Python 3.10+. Mitigation: Note in lesson with fallback to if/elif.
- **Type Hints Learning Curve**: Beginners may find type hints distracting. Mitigation: Explain they're optional during learning, mandatory in professional code.

### Pedagogical Risks
- **Nesting Complexity** (Lesson 5): Combining all concepts can overwhelm. Mitigation: Start with 2-level nesting; use visual examples (grids); mention functions as future simplification.
- **Infinite Loops**: Common student error. Mitigation: Red Flags section; AI troubleshooting prompt ready; clear examples of proper termination.

### Dependencies
- **Chapter 13-16**: Must be completed before Chapter 17
  - Ch 13: Variables and assignment
  - Ch 14: Boolean type and type hints
  - Ch 15: Comparison and logical operators (CRITICAL)
  - Ch 16: String type and input() function
- **AI Tool Access**: Students need Claude Code or Gemini CLI (available from Parts 1-3)
- **Python 3.14+**: All code examples require Python 3.14; assume `uv` is installed

### Dependency Validation
- [ ] Chapter 13 status: ✅ Implemented
- [ ] Chapter 14 status: ✅ Implemented & Validated
- [ ] Chapter 15 status: 📋 Planned (verify before linking)
- [ ] Chapter 16 status: 📋 Planned (verify before linking)

**Note**: Chapters 15-16 listed as "Planned" in chapter-index.md, but Chapter 17 spec assumes they're complete. **Action**: Verify status before content-implementer starts; add quick review sections if needed.

---

## Project File Structure

**Chapter Output Directory**: `book-source/docs/04-Part-4-Python-Fundamentals/17-control-flow-loops/`

**Expected Files After Implementation**:
```
17-control-flow-loops/
  ├── README.md (chapter overview, learning outcomes, structure)
  ├── 01-making-decisions-with-conditionals.md
  ├── 02-pattern-matching-with-match-case.md
  ├── 03-repetition-with-loops.md
  ├── 04-controlling-loops.md
  └── 05-nested-control-structures.md
```

**README.md Content**:
- Chapter overview (what students will learn)
- Learning outcomes (5 aligned to learning objectives)
- Prerequisites (Chapters 13-16)
- Time estimate (4-5 hours total)
- Table of contents (5 lessons with time per lesson)
- Success metrics (aligned to evals)

---

## Conclusion

This plan provides a complete roadmap for implementing Chapter 17 with:
- **5 progressively complex lessons** (Lessons 1-5, A2→B1 proficiency)
- **25-30 code examples** (Tier 1 book teaches + Tier 2 AI companion)
- **AI prompts specified** for all Tier 2 examples and Try With AI sections
- **Red Flags documented** (common errors, fixes, AI troubleshooting)
- **CEFR proficiency validation** (cognitive load checked; progression confirmed)
- **Lesson closure compliance** (every lesson ends ONLY with Try With AI)
- **Constitution alignment** (no forward references, AI as partner, validation-first)

**Status**: ✅ Complete and Ready for Human Review
**Next Step**: Human approval → `/sp.tasks` to generate detailed development tasks
