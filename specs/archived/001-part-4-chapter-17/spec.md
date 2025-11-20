# Feature Specification: Chapter 17 - Control Flow and Loops

**Feature Branch**: `001-part-4-chapter-17`
**Created**: 2025-11-08
**Status**: Draft
**Part**: 4 (Python Fundamentals)
**Chapter Number**: 17
**Complexity Tier**: Intermediate (A2-B1)
**Input**: Chapter 17: "Control Flow and Loops" in Part 4 - Teaching decision-making and repetition in Python with AI-Native Learning methodology

---

## Success Evals *(FIRST - Business-Goal-Aligned Success Criteria)*

### Pedagogical Success Metrics (Measurable Outcomes)

**Comprehension Evals** (can students explain concepts?):
- **EVAL-001**: 80%+ of students can explain the difference between `if/elif/else` and `match-case` after reading Lesson 2
- **EVAL-002**: 75%+ of students can identify when to use `for` vs `while` loops in a given scenario
- **EVAL-003**: 85%+ of students can predict the output of nested control structures (comprehension quiz)

**Skill Acquisition Evals** (can students apply techniques?):
- **EVAL-004**: 80%+ of students can write correct conditional logic for real-world decision problems (exercise completion)
- **EVAL-005**: 75%+ of students can implement loops with appropriate control flow (break/continue) for given tasks
- **EVAL-006**: 70%+ of students can describe control flow intent clearly enough for AI to generate working code (AI collaboration eval)
- **EVAL-007**: 80%+ of students can validate control flow logic by testing edge cases (validation skill assessment)

**Engagement Evals** (did students complete chapter?):
- **EVAL-008**: 85%+ completion rate for all "Try With AI" exercises (tracked via learning platform)
- **EVAL-009**: Average time-on-chapter: 3.5-4.5 hours (indicates appropriate pacing for intermediate level)
- **EVAL-010**: 75%+ of students report feeling confident using conditionals and loops after chapter (exit survey)

**Accessibility Evals** (reading level appropriate?):
- **EVAL-011**: Flesch-Kincaid Grade Level: 7-9 (intermediate appropriate, not too simple, not too complex)
- **EVAL-012**: Technical jargon introduced with clear definitions on first use (automated check: 100% compliance)
- **EVAL-013**: Code examples include inline comments explaining logic (manual review: all examples must pass)

**Why These Evals Connect to Business Goals**:
- Comprehension/skill metrics → Students can apply concepts professionally → Higher job placement rates → Book reputation/sales
- Engagement metrics → Chapter completion correlates with course completion → Lower dropout → Better student outcomes
- Accessibility metrics → Wider audience reach → More students can succeed → Larger market impact

---

## Topic Summary

Chapter 17 teaches control flow and decision-making in Python—fundamental skills for writing programs that make decisions and repeat actions. Students learn to use conditional statements (`if/elif/else`, `match-case`) to direct program logic based on conditions, and loops (`for`, `while`) to automate repetition. This chapter builds on Chapters 13-16 (Python basics, data types, operators, strings) and prepares students for more complex programs in subsequent chapters.

The chapter emphasizes AI-Native Learning principles: students describe their intent using clear code structure and type hints, explore concepts through AI dialogue, validate their understanding through testing, and learn from errors by asking AI diagnostic questions. This intermediate-level chapter (A2-B1 proficiency) introduces 4-5 core concepts per lesson with progressive complexity, culminating in nested control structures that combine conditionals and loops.

**Key Pedagogical Approach**: Book teaches foundational syntax directly (Tier 1: if/elif/else, for/while basics), AI companion handles complex patterns (Tier 2: nested structures, match-case), and students practice describing intent for AI to execute. No capstone project—focus is on progressive concept mastery through targeted exercises.

---

## Prerequisites

**Required Prior Knowledge** (from earlier chapters):

1. **Chapter 13: Introduction to Python**
   - Python syntax basics, variable assignment
   - Running Python programs via `uv run`, AI-assisted execution
   - Basic print() function usage

2. **Chapter 14: Data Types**
   - Understanding of core types: int, float, str, bool, None
   - Type hints and type checking (isinstance(), type())
   - Boolean values (True/False) as decision inputs

3. **Chapter 15: Operators, Keywords, and Variables**
   - **Comparison operators**: ==, !=, >, <, >=, <= (CRITICAL for conditionals)
   - **Logical operators**: and, or, not (CRITICAL for complex conditions)
   - Variable scope and assignment

4. **Chapter 16: Strings and Type Casting**
   - String manipulation for user input
   - Type casting (int(), float(), str()) for validation
   - Basic input() usage

**Assumed Skills** (from Part 1-3):
- AI collaboration mindset (Claude Code, Gemini CLI usage)
- Prompt engineering basics (asking clear questions)
- Reading and understanding error messages
- Testing code with different inputs

**What Students Should Be Able to Do Before Starting**:
- Write simple Python programs with variables and operators
- Use comparison operators to evaluate conditions (e.g., `age > 18` → True/False)
- Ask their AI companion to explain concepts and generate code
- Validate code output by testing with different inputs

---

## Learning Objectives (Aligned with Success Evals)

By the end of Chapter 17, students will be able to:

**LO-001**: **Write conditional logic** using `if`, `elif`, and `else` statements to make decisions based on multiple conditions (Aligns: EVAL-001, EVAL-004)

**LO-002**: **Apply comparison and logical operators** appropriately in conditional expressions to express complex decision criteria (Aligns: EVAL-002, EVAL-004)

**LO-003**: **Implement for loops** for definite iteration (known repetition count) using `range()` and iterable data structures (Aligns: EVAL-005)

**LO-004**: **Implement while loops** for indefinite iteration (condition-based repetition) with proper termination logic (Aligns: EVAL-005)

**LO-005**: **Control loop execution** using `break` (exit loop), `continue` (skip iteration), and `else` clauses (loop completion detection) (Aligns: EVAL-005, EVAL-007)

**LO-006**: **Use match-case pattern matching** (Python 3.10+) as a modern alternative to if/elif chains for structured decision-making (Aligns: EVAL-001)

**LO-007**: **Nest conditionals and loops** to solve multi-step problems requiring combined decision-making and repetition (Aligns: EVAL-003, EVAL-005)

**LO-008**: **Describe control flow intent clearly** so AI can generate correct conditional and loop structures from natural language specifications (Aligns: EVAL-006)

**LO-009**: **Validate control flow logic** by testing edge cases (boundary values, empty inputs, error conditions) and interpreting execution results (Aligns: EVAL-007)

**Proficiency Progression** (CEFR Mapping):
- Lessons 1-2: A2 (simple conditionals, basic loops)
- Lessons 3-4: A2-B1 (loop control, match-case)
- Lesson 5: B1 (nested structures, integration)

---

## Content Outline (5 Lessons)

### Lesson 1: Making Decisions with Conditionals (A2)
**Concepts** (max 7): `if` statement, `else` clause, `elif` chaining, comparison operators review (quick), logical operators review (quick), boolean expressions, indentation rules

**Learning Flow**:
1. Quick review: Comparison operators (Ch 15 callback - 1-2 examples)
2. Quick review: Logical operators (and/or/not - 1-2 examples)
3. Book teaches: `if` statement (single condition)
4. Book teaches: `if-else` (binary decision)
5. Book teaches: `if-elif-else` (multiple conditions)
6. AI Companion: Complex condition combinations
7. Practice: Decision-making exercises (grade calculator, age verification)

**Code Examples** (3-6 per lesson):
1. Simple if: age verification (Tier 1 - book teaches)
2. if-else: even/odd checker (Tier 1 - book teaches)
3. if-elif-else: grade classifier (Tier 1 - book teaches)
4. Complex conditions with and/or: discount eligibility (Tier 2 - AI companion)
5. Nested if: multi-criteria validation (Tier 2 - AI companion)

**Try With AI** (4 prompts - Bloom's progression):
1. Recall: "What's the difference between `if` and `elif`?"
2. Understand: "Explain how this if-elif-else chain evaluates: [code example]"
3. Apply: "Generate a conditional that checks if a number is positive, negative, or zero"
4. Analyze: "What edge cases should I test for this age verification logic?"

---

### Lesson 2: Pattern Matching with match-case (A2-B1)
**Concepts** (max 7): `match-case` syntax, pattern matching basics, literal patterns, wildcard pattern (`_`), when to use match vs if/elif, readability benefits, Python 3.10+ requirement

**Learning Flow**:
1. Review: if-elif-else chains (callback to Lesson 1)
2. Motivation: Why match-case exists (readability, intent)
3. Book teaches: match-case basic syntax
4. Book teaches: Literal patterns (matching specific values)
5. Book teaches: Wildcard pattern (default case)
6. AI Companion: Converting if/elif to match-case
7. Practice: Decision-making with pattern matching

**Code Examples** (3-6):
1. Simple match-case: HTTP status codes (Tier 1 - book teaches)
2. Match with wildcard: menu selection (Tier 1 - book teaches)
3. Converting if/elif to match: traffic light logic (Tier 2 - AI companion)
4. Multiple match cases: calculator operations (Tier 2 - AI companion)

**Try With AI** (4 prompts - Bloom's progression):
1. Recall: "What's the purpose of the wildcard pattern `_` in match-case?"
2. Understand: "Explain how this match-case evaluates: [code example]"
3. Apply: "Generate match-case for days of the week (1-7 → day name)"
4. Evaluate: "Compare readability: this if-elif chain vs match-case equivalent"

---

### Lesson 3: Repetition with Loops (A2-B1)
**Concepts** (max 7): `for` loop syntax, `range()` function (start, stop, step), iterating over sequences, `while` loop syntax, loop termination conditions, infinite loop dangers, when to use for vs while

**Learning Flow**:
1. Motivation: Why automate repetition?
2. Book teaches: for loop with range() (definite iteration)
3. Book teaches: range() variations (start, stop, step)
4. Book teaches: while loop (indefinite iteration)
5. Book teaches: Loop termination conditions
6. AI Companion: Choosing for vs while
7. Practice: Repetition exercises (countdown, sum of numbers)

**Code Examples** (3-6):
1. Basic for loop: print 1-10 (Tier 1 - book teaches)
2. for with range(start, stop, step): even numbers (Tier 1 - book teaches)
3. while loop: user input validation (Tier 1 - book teaches)
4. for vs while comparison: same task both ways (Tier 2 - AI companion)
5. Infinite loop demonstration + fix (Tier 2 - AI companion)

**Try With AI** (4 prompts - Bloom's progression):
1. Recall: "What's the difference between for and while loops?"
2. Understand: "Trace this for loop execution: [code with range()]"
3. Apply: "Generate a while loop that counts down from 10 to 1"
4. Analyze: "What happens if I forget to update the loop counter in while?"

---

### Lesson 4: Controlling Loops (A2-B1)
**Concepts** (max 7): `break` statement (exit loop), `continue` statement (skip iteration), `for...else` pattern, `while...else` pattern, loop control flow, when to use break/continue, loop completion detection

**Learning Flow**:
1. Review: Basic for/while loops (Lesson 3 callback)
2. Book teaches: break statement (early exit)
3. Book teaches: continue statement (skip iteration)
4. Book teaches: for...else clause (completion detection)
5. Book teaches: while...else clause
6. AI Companion: Complex loop control scenarios
7. Practice: Search, filter, validation exercises

**Code Examples** (3-6):
1. break: search for number in list (Tier 1 - book teaches)
2. continue: skip even numbers (Tier 1 - book teaches)
3. for...else: "not found" detection (Tier 1 - book teaches)
4. while...else: retry with limit (Tier 2 - AI companion)
5. Combined break/continue: input validation loop (Tier 2 - AI companion)

**Try With AI** (4 prompts - Bloom's progression):
1. Recall: "What's the difference between break and continue?"
2. Understand: "Trace this for...else execution: [code example]"
3. Apply: "Generate a while loop that retries 3 times before giving up"
4. Evaluate: "When is for...else more elegant than using a flag variable?"

---

### Lesson 5: Nested Control Structures (B1 - Integration)
**Concepts** (max 7): Nested if statements, nested loops, combining conditionals and loops, loop invariants, complexity management, indentation depth, when to refactor (NO new syntax introduced - all concepts from Lessons 1-4)

**Learning Flow**:
1. Review: All prior concepts (Lessons 1-4 integration)
2. Book teaches: Nested if statements (decision trees)
3. Book teaches: Nested loops (2D iteration)
4. Book teaches: Conditionals inside loops
5. Book teaches: Loops inside conditionals
6. AI Companion: Managing complexity (when to simplify)
7. Practice: Integration exercises (multiplication table, pattern printing, data validation)

**Code Examples** (3-6):
1. Nested if: multi-criteria eligibility (Tier 1 - book teaches)
2. Nested loops: multiplication table (Tier 1 - book teaches)
3. Conditional inside loop: filtering while iterating (Tier 1 - book teaches)
4. Loop inside conditional: process list only if condition met (Tier 2 - AI companion)
5. Complex nested structure: game logic (Tier 2 - AI companion)

**Try With AI** (4 prompts - Bloom's progression):
1. Recall: "Why is indentation critical in nested structures?"
2. Understand: "Trace this nested loop execution: [multiplication table code]"
3. Apply: "Generate nested loops to find all pairs (i, j) where i + j = 10"
4. Synthesize: "Describe a real-world problem that requires nested control flow, then ask AI to implement it"


---

## Acceptance Criteria (Quality Checklist - References Success Evals)

### Content Quality
- [ ] All lessons follow AI-Native Learning pattern
- [ ] CoLearning elements present throughout
- [ ] Conversational tone - NOT documentation style
- [ ] AI positioned as co-reasoning partner
- [ ] No forward references to Chapters 30+
- [ ] No formal specification writing terminology
- [ ] Flesch-Kincaid Grade Level: 7-9

### Pedagogical Effectiveness
- [ ] Graduated Teaching Pattern applied
- [ ] Cognitive load limits respected: Max 7 concepts per lesson
- [ ] CEFR proficiency progression: A2 to A2-B1 to B1
- [ ] Prerequisites clearly stated
- [ ] Learning objectives align with success evals
- [ ] Try With AI 4-prompt format in EVERY lesson
- [ ] Lesson closure: ONLY Try With AI section

### Code Quality
- [ ] All code examples tested on Python 3.14+
- [ ] Type hints mandatory on all functions
- [ ] Modern Python syntax
- [ ] No security issues
- [ ] Inline comments explaining logic
- [ ] Progressive complexity within lessons

### Success Eval Alignment
- [ ] EVAL-001 through EVAL-010 assessment methods specified
- [ ] All evals mapped to acceptance criteria

---

## Edge Cases & Error Scenarios

**Boundary Conditions**: Empty inputs, zero values, negative values, boundary values, infinite loops, off-by-one errors, type mismatches

**Common Errors**: IndentationError, SyntaxError, infinite loops, logic errors, type errors

**Error Literacy**: Each lesson includes Red Flags section with AI troubleshooting prompts

---

## Assumptions

**Technical**: Python 3.14+, Claude Code/Gemini CLI access, uv run capability, VSCode editor

**Pedagogical**: Chapters 13-16 completed, AI collaboration comfort, intermediate tier appropriate, practice-focused learning preferred, reading level 7-9

**Scope**: No list comprehensions, no exception handling details, no functions in examples, basic match-case patterns only

**Default Decisions**: 3-6 code examples per lesson, 5 lessons total, informal validation via print and AI dialogue

---

## Out of Scope

Functions, exception handling, list comprehensions, file I/O, OOP, decorators, asyncio, formal SDD methodology, advanced match-case, performance optimization

---

## Dependencies

**Internal**: Chapters 13-16 (Python basics, data types, operators, strings)

**External**: Python 3.14+, Claude Code/Gemini CLI, VSCode, terminal access

**Knowledge**: Boolean logic, arithmetic operations, variable scope, indentation, error reading

**No External Libraries Required**: Python standard library only

---

## Notes for Implementation

**For chapter-planner**: Apply skills-proficiency-mapper, validate cognitive load, integrate ai-collaborate-teaching throughout, ensure Try With AI is only closure

**For content-implementer**: CRITICAL - No forward references within chapter, conversational tone, AI prompts for all examples, type hints mandatory, error literacy examples

**For validation-auditor**: NEW - Pedagogical ordering compliance check, validate CoLearning elements, check lesson closure pattern, test all code on Python 3.14+

---

## Validation Readiness

**Ready for /sp.clarify when**: No NEEDS CLARIFICATION markers, learning objectives aligned, concept counts clear, code examples concrete

**Ready for /sp.plan when**: All clarifications resolved, lesson structure approved, pedagogical approach validated, scope boundaries confirmed

---

**Status**: Draft - Ready for human review and /sp.clarify quality gate
