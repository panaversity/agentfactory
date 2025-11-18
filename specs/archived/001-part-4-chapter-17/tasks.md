# Tasks: Chapter 17 - Control Flow and Loops

**Feature Branch**: `001-part-4-chapter-17`
**Input**: Design documents from `/specs/001-part-4-chapter-17/`
**Prerequisites**: plan.md (✓), spec.md (✓)

**Organization**: Tasks are grouped by lesson (treating each lesson as a "user story") to enable independent implementation and validation of each lesson.

**Tests**: No automated tests required for book content. Validation is manual (technical review, readability checks, code example testing).

## Format: `[ID] [P?] [Lesson] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Lesson]**: Which lesson this task belongs to (L1, L2, L3, L4, L5)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Chapter Infrastructure)

**Purpose**: Create chapter structure and shared resources

- [X] T001 Create chapter directory: `book-source/docs/04-Part-4-Python-Fundamentals/17-control-flow-loops/`
- [X] T002 Create README.md with chapter overview, learning outcomes, prerequisites, time estimate (4-5 hours), lesson TOC
- [X] T003 [P] Verify prerequisites: Chapters 13-16 status (Ch 15-16 currently "Planned" - may need quick review sections)
- [X] T004 [P] Set up Python 3.14 testing environment for code examples

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core resources and shared content that ALL lessons depend on

**⚠️ CRITICAL**: No lesson writing can begin until this phase is complete

- [X] T005 Document "Try With AI" format standard: 4 prompts per lesson (Recall → Understand → Apply → Analyze/Evaluate/Synthesize)
- [X] T006 Document CoLearning elements format: 💬 AI Colearning Prompt, 🎓 Instructor Commentary, 🚀 CoLearning Challenge, ✨ Teaching Tip
- [X] T007 Document lesson closure requirement: ONLY "Try With AI" section (no "Key Takeaways", "Summary", or "What's Next")
- [X] T008 Document Graduated Teaching Pattern labels: Tier 1 (Book Teaches), Tier 2 (AI Companion)
- [X] T009 Create shared code testing checklist: Python 3.14+, type hints, inline comments, copy-pasteable, expected output

**Checkpoint**: Foundation ready - lesson implementation can now begin in parallel

---

## Phase 3: Lesson 1 - Making Decisions with Conditionals (Priority: P1) 🎯 MVP

**Goal**: Teach if/elif/else statements, logical operators, and boolean expressions (A2 proficiency)

**Independent Test**:
- All 5 code examples run correctly on Python 3.14
- "Try With AI" section has exactly 4 prompts (Recall, Understand, Apply, Analyze)
- Lesson ends ONLY with "Try With AI" (no Key Takeaways)
- Reading level Grade 7-9 (Flesch-Kincaid check)

### Implementation for Lesson 1

- [X] T010 [L1] Create lesson file: `17-control-flow-loops/01-making-decisions-with-conditionals.md`
- [X] T011 [L1] Write YAML frontmatter: sidebar_position=1, title, description, reading_level, time_estimate=45-60min
- [X] T012 [L1] Write quick review section: Comparison operators (Ch 15 callback - 1-2 examples, max 5 min reading time)
- [X] T013 [L1] Write quick review section: Logical operators (and/or/not - 1-2 examples, max 5 min reading time)
- [X] T014 [L1] Write main content: if statement (single condition) with age verification example (Tier 1 - Book Teaches)
- [X] T015 [L1] Write main content: if-else (binary decision) with even/odd checker example (Tier 1 - Book Teaches)
- [X] T016 [L1] Write main content: if-elif-else (multiple conditions) with grade classifier example (Tier 1 - Book Teaches)
- [X] T017 [L1] Write AI Companion section: Complex conditions with and/or (discount eligibility example - Tier 2)
- [X] T018 [L1] Write AI Companion section: Nested if (multi-criteria validation example - Tier 2)
- [X] T019 [L1] Write Red Flags section: IndentationError, unreachable code, logical errors, type mismatches (with AI troubleshooting prompts)
- [X] T020 [L1] Write "Try With AI" section with 4 prompts: Recall (if vs elif), Understand (trace execution), Apply (generate code), Analyze (edge cases)
- [X] T021 [L1] Test all 5 code examples on Python 3.14 with type hints
- [X] T022 [L1] Validate lesson closure: ONLY "Try With AI" section (no other closing sections)
- [X] T023 [L1] Validate cognitive load: 7 concepts max (if, else, elif, comparison operators review, logical operators review, boolean expressions, indentation)
- [X] T024 [L1] Validate reading level: Grade 7-9 (Flesch-Kincaid)

**Checkpoint**: Lesson 1 should be complete, tested, and independently readable

---

## Phase 4: Lesson 2 - Pattern Matching with match-case (Priority: P2)

**Goal**: Teach match-case syntax, literal patterns, wildcard pattern (A2-B1 proficiency)

**Independent Test**:
- All 4 code examples run correctly on Python 3.10+
- Python version requirement clearly noted
- Readability comparison (if/elif vs match-case) included
- "Try With AI" section has exactly 4 prompts
- Lesson ends ONLY with "Try With AI"

### Implementation for Lesson 2

- [ ] T025 [P] [L2] Create lesson file: `17-control-flow-loops/02-pattern-matching-with-match-case.md`
- [ ] T026 [L2] Write YAML frontmatter: sidebar_position=2, title, description, reading_level, time_estimate=50-65min
- [ ] T027 [L2] Write motivation section: Why match-case exists (readability, intent clarity) with if/elif vs match-case comparison
- [ ] T028 [L2] Write main content: match-case basic syntax (Tier 1 - Book Teaches)
- [ ] T029 [L2] Write main content: Literal patterns with HTTP status codes example (Tier 1 - Book Teaches)
- [ ] T030 [L2] Write main content: Wildcard pattern `_` with menu selection example (Tier 1 - Book Teaches)
- [ ] T031 [L2] Write AI Companion section: Converting if/elif to match-case (traffic light logic - Tier 2)
- [ ] T032 [L2] Write AI Companion section: Multiple match cases (calculator operations - Tier 2)
- [ ] T033 [L2] Write Red Flags section: Python < 3.10 incompatibility, missing colon, unreachable patterns, missing default case
- [ ] T034 [L2] Write "Try With AI" section with 4 prompts: Recall (wildcard purpose), Understand (trace match-case), Apply (days of week), Evaluate (readability comparison)
- [ ] T035 [L2] Test all 4 code examples on Python 3.14 with type hints
- [ ] T036 [L2] Add Python 3.10+ requirement note prominently
- [ ] T037 [L2] Validate lesson closure: ONLY "Try With AI" section
- [ ] T038 [L2] Validate cognitive load: 5 concepts (match-case syntax, pattern matching, literal patterns, wildcard, when to use)
- [ ] T039 [L2] Validate reading level: Grade 7-9

**Checkpoint**: Lessons 1 AND 2 should both be independently complete and tested

---

## Phase 5: Lesson 3 - Repetition with Loops (Priority: P3)

**Goal**: Teach for loops, while loops, range() function, loop termination (A2-B1 proficiency)

**Independent Test**:
- All 5 code examples run correctly
- Infinite loop demonstration includes fix
- for vs while comparison clear
- "Try With AI" section has exactly 4 prompts
- Lesson ends ONLY with "Try With AI"

### Implementation for Lesson 3

- [X] T040 [P] [L3] Create lesson file: `17-control-flow-loops/03-repetition-with-loops.md`
- [ ] T041 [L3] Write YAML frontmatter: sidebar_position=3, title, description, reading_level, time_estimate=55-70min
- [ ] T042 [L3] Write motivation section: Why automate repetition? (real-world scenarios)
- [ ] T043 [L3] Write main content: for loop with range() (definite iteration) with print 1-10 example (Tier 1 - Book Teaches)
- [ ] T044 [L3] Write main content: range() variations (start, stop, step) with even numbers example (Tier 1 - Book Teaches)
- [ ] T045 [L3] Write main content: while loop (indefinite iteration) with user input validation example (Tier 1 - Book Teaches)
- [ ] T046 [L3] Write main content: Loop termination conditions (how to avoid infinite loops)
- [ ] T047 [L3] Write AI Companion section: for vs while comparison (same task both ways - Tier 2)
- [ ] T048 [L3] Write AI Companion section: Infinite loop demonstration + fix (Tier 2)
- [ ] T049 [L3] Write Red Flags section: Infinite loops, off-by-one errors, loop variable scope, step zero error
- [ ] T050 [L3] Write "Try With AI" section with 4 prompts: Recall (for vs while), Understand (trace for loop), Apply (countdown while), Analyze (infinite loop consequences)
- [ ] T051 [L3] Test all 5 code examples on Python 3.14 with type hints
- [ ] T052 [L3] Validate lesson closure: ONLY "Try With AI" section
- [ ] T053 [L3] Validate cognitive load: 7 concepts (for, range(), iterating sequences, while, termination conditions, infinite loop dangers, when to use for vs while)
- [ ] T054 [L3] Validate reading level: Grade 7-9

**Checkpoint**: Lessons 1, 2, AND 3 should all be independently complete and tested

---

## Phase 6: Lesson 4 - Controlling Loops (Priority: P4)

**Goal**: Teach break, continue, for...else, while...else patterns (A2-B1 proficiency)

**Independent Test**:
- All 5 code examples run correctly
- for...else and while...else patterns clearly explained
- "Try With AI" section has exactly 4 prompts
- Lesson ends ONLY with "Try With AI"

### Implementation for Lesson 4

- [X] T055 [P] [L4] Create lesson file: `17-control-flow-loops/04-controlling-loops.md`
- [ ] T056 [L4] Write YAML frontmatter: sidebar_position=4, title, description, reading_level, time_estimate=55-70min
- [ ] T057 [L4] Write motivation section: Early exit, skipping, retry scenarios (when loop control is needed)
- [ ] T058 [L4] Write main content: break statement (early exit) with search for number example (Tier 1 - Book Teaches)
- [ ] T059 [L4] Write main content: continue statement (skip iteration) with skip even numbers example (Tier 1 - Book Teaches)
- [ ] T060 [L4] Write main content: for...else clause (completion detection) with "not found" detection example (Tier 1 - Book Teaches)
- [ ] T061 [L4] Write main content: while...else clause with examples
- [ ] T062 [L4] Write AI Companion section: while...else retry with limit (Tier 2)
- [ ] T063 [L4] Write AI Companion section: Combined break/continue (input validation loop - Tier 2)
- [ ] T064 [L4] Write Red Flags section: Misunderstanding when else executes, continue outside loop, break unreachable, overuse of break
- [ ] T065 [L4] Write "Try With AI" section with 4 prompts: Recall (break vs continue), Understand (trace for...else), Apply (retry 3 times), Evaluate (for...else vs flag variable)
- [ ] T066 [L4] Test all 5 code examples on Python 3.14 with type hints
- [ ] T067 [L4] Validate lesson closure: ONLY "Try With AI" section
- [ ] T068 [L4] Validate cognitive load: 7 concepts (break, continue, for...else, while...else, loop control flow, when to use, completion detection)
- [ ] T069 [L4] Validate reading level: Grade 7-9

**Checkpoint**: Lessons 1-4 should all be independently complete and tested

---

## Phase 7: Lesson 5 - Nested Control Structures (Priority: P5)

**Goal**: Integrate all Lessons 1-4 concepts; teach nested if, nested loops, combining conditionals and loops (B1 proficiency)

**Independent Test**:
- All 5 code examples run correctly
- Integration of Lessons 1-4 concepts validated
- No new syntax introduced (all concepts are from previous lessons)
- "Try With AI" section has exactly 4 prompts (includes Synthesize level)
- Lesson ends ONLY with "Try With AI"

### Implementation for Lesson 5

- [X] T070 [P] [L5] Create lesson file: `17-control-flow-loops/05-nested-control-structures.md`
- [ ] T071 [L5] Write YAML frontmatter: sidebar_position=5, title, description, reading_level, time_estimate=60-75min
- [ ] T072 [L5] Write integration review section: Callback to Lessons 1-4 concepts (quick summary, max 8 min reading time)
- [ ] T073 [L5] Write main content: Nested if statements (decision trees) with multi-criteria eligibility example (Tier 1 - Book Teaches)
- [ ] T074 [L5] Write main content: Nested loops (2D iteration) with multiplication table example (Tier 1 - Book Teaches)
- [ ] T075 [L5] Write main content: Conditionals inside loops with filtering while iterating example (Tier 1 - Book Teaches)
- [ ] T076 [L5] Write main content: Loops inside conditionals with process list conditionally example (Tier 1 - Book Teaches)
- [ ] T077 [L5] Write AI Companion section: Complex nested structure (game logic - Tier 2)
- [ ] T078 [L5] Write AI Companion section: Managing complexity (when to simplify/refactor - preview of functions)
- [ ] T079 [L5] Write Red Flags section: Indentation errors, loop variable confusion, deep nesting reducing readability, infinite nested loops
- [ ] T080 [L5] Write "Try With AI" section with 4 prompts: Recall (indentation importance), Understand (trace nested loops), Apply (find pairs i+j=10), Synthesize (real-world problem + AI implementation)
- [ ] T081 [L5] Test all 5 code examples on Python 3.14 with type hints
- [ ] T082 [L5] Validate lesson closure: ONLY "Try With AI" section
- [ ] T083 [L5] Validate cognitive load: 6 concepts (all review - nested if, nested loops, conditionals in loops, loops in conditionals, complexity management, indentation depth)
- [ ] T084 [L5] Validate reading level: Grade 7-9
- [ ] T085 [L5] Validate NO new syntax introduced (all concepts from Lessons 1-4)

**Checkpoint**: All 5 lessons should now be independently complete, tested, and integrated

---

## Phase 8: Polish & Cross-Chapter Validation

**Purpose**: Improvements and validation that affect multiple lessons

- [ ] T086 [P] Validate chapter-wide consistency: CoLearning elements used throughout
- [ ] T087 [P] Validate all code examples have type hints (mandatory)
- [ ] T088 [P] Validate all lessons end ONLY with "Try With AI" (no other closing sections)
- [ ] T089 [P] Validate Graduated Teaching Pattern applied: Tier 1 (Book Teaches) and Tier 2 (AI Companion) labels present
- [ ] T090 [P] Validate no forward references to Chapters 30+ or Part 5 (formal SDD terminology)
- [ ] T091 [P] Validate CEFR proficiency progression: A2 (L1) → A2-B1 (L2-4) → B1 (L5)
- [ ] T092 [P] Validate cognitive load limits: L1 (7 concepts), L2 (5 concepts), L3-5 (6-7 concepts)
- [ ] T093 Validate all 25-30 code examples run on Python 3.14 (comprehensive test suite)
- [ ] T094 Update README.md with final lesson titles, time estimates, and learning outcomes
- [ ] T095 Cross-reference validation: All internal chapter references correct
- [ ] T096 [P] Flesch-Kincaid reading level check: All lessons Grade 7-9
- [ ] T097 [P] Run spell check and grammar review
- [ ] T098 Prepare chapter for validation-auditor validation

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all lesson writing
- **Lessons (Phases 3-7)**: All depend on Foundational phase completion
  - Lessons can proceed sequentially (L1 → L2 → L3 → L4 → L5) in priority order
  - OR Lessons 2-5 can start in parallel once Lesson 1 is complete (if multiple writers available)
- **Polish (Phase 8)**: Depends on all 5 lessons being complete

### Lesson Dependencies

- **Lesson 1 (L1)**: Can start after Foundational (Phase 2) - No dependencies on other lessons
- **Lesson 2 (L2)**: Should reference Lesson 1 concepts (if/elif/else for comparison) - Can start after L1 complete
- **Lesson 3 (L3)**: Independent of L2 - Can start after Foundational (Phase 2)
- **Lesson 4 (L4)**: Depends on Lesson 3 (basic for/while loops) - Can start after L3 complete
- **Lesson 5 (L5)**: Integrates ALL Lessons 1-4 - MUST wait for L1-4 complete

### Within Each Lesson

- YAML frontmatter before content
- Main content sections before AI Companion sections
- Red Flags after all code examples
- "Try With AI" section LAST (lesson closure)
- Code testing before lesson validation

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Lessons 2 and 3 can be written in parallel (after L1 complete, before L5 starts)
- All Polish tasks marked [P] can run in parallel (within Phase 8)

---

## Parallel Example: Lessons 2 and 3

```bash
# After Lesson 1 is complete, launch Lessons 2 and 3 together:
Task: "Write Lesson 2 (match-case) in 02-pattern-matching-with-match-case.md"
Task: "Write Lesson 3 (loops) in 03-repetition-with-loops.md"

# These are independent and can be written by different authors simultaneously
```

---

## Implementation Strategy

### MVP First (Lesson 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all lessons)
3. Complete Phase 3: Lesson 1
4. **STOP and VALIDATE**: Test Lesson 1 independently with validation-auditor
5. Get human approval before proceeding

### Sequential Delivery (Recommended for Solo Author)

1. Complete Setup + Foundational → Foundation ready
2. Write Lesson 1 → Test independently → Approve
3. Write Lesson 2 → Test independently → Approve
4. Write Lesson 3 → Test independently → Approve
5. Write Lesson 4 → Test independently → Approve
6. Write Lesson 5 → Test independently → Approve
7. Complete Polish phase → Chapter ready for publication

### Parallel Team Strategy (If Multiple Authors Available)

With multiple writers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Author A: Lesson 1
3. Once Lesson 1 approved:
   - Author A: Lesson 4 (depends on L3)
   - Author B: Lesson 2 (can start in parallel)
   - Author C: Lesson 3 (can start in parallel)
4. Once Lessons 1-4 approved:
   - Lead Author: Lesson 5 (integrates all concepts)
5. Team completes Polish together

---

## Policy Notes

**Lesson Closure Requirement**: Within Chapter 17, each lesson MUST end with a single final section titled "Try With AI" containing 4 prompts following Bloom's taxonomy progression (Recall → Understand → Apply → Analyze/Evaluate/Synthesize). No "Key Takeaways", "Summary", or "What's Next" sections are permitted.

**AI Tool Usage**: Before Part 5 (where Claude Code and Gemini CLI are taught), use ChatGPT web in "Try With AI" sections. After tool onboarding (Parts 1-3), instruct learners to use their preferred AI companion tool (e.g., Gemini CLI, Claude CLI), optionally providing both CLI and web variants.

**Type Hints**: All code examples MUST include type hints on variables and function parameters. This is non-negotiable per constitution (modern Python standards).

**Python Version**: All code examples MUST run on Python 3.14+. Lesson 2 requires Python 3.10+ for match-case (note this prominently).

**No Forward References**: Do NOT reference Chapters 30+ (Specification-Driven Development) or Part 5 concepts. Use AI-Native Learning terminology (describe intent, explore with AI, validate together) instead of formal SDD terminology (write spec, generate from spec).

---

## Total Task Count

- **Setup**: 4 tasks
- **Foundational**: 5 tasks
- **Lesson 1**: 15 tasks
- **Lesson 2**: 15 tasks
- **Lesson 3**: 15 tasks
- **Lesson 4**: 15 tasks
- **Lesson 5**: 16 tasks
- **Polish**: 13 tasks

**TOTAL**: 98 tasks

### Tasks by Lesson

- Lesson 1 (L1): 15 tasks
- Lesson 2 (L2): 15 tasks
- Lesson 3 (L3): 15 tasks
- Lesson 4 (L4): 15 tasks
- Lesson 5 (L5): 16 tasks

### Parallel Opportunities Identified

- 11 tasks marked [P] (can run in parallel within phases)
- Lessons 2 and 3 can be written in parallel (after L1 complete)

### Suggested MVP Scope

**Minimum Viable Product**: Lesson 1 only (15 implementation tasks + 4 setup + 5 foundational = 24 tasks)

This delivers foundational conditional logic (if/elif/else) and validates the teaching approach before committing to full chapter implementation.
