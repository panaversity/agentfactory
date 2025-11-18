# Tasks: Chapter 21 Exception Handling

**Feature**: Chapter 21: Exception Handling (Part 4, Python Fundamentals)
**Input**: Design documents from `/specs/015-part-4-chapter-21/`
**Prerequisites**: spec.md ✓, plan.md ✓
**Output**: 5 lessons in `book-source/docs/04-Part-4-Python-Fundamentals/21-exception-handling/`

**Organization**: Tasks are grouped by lesson (mapped to user stories) to enable independent lesson implementation and validation.

---

## Format: `- [ ] [ID] [P?] [Lesson] Description`

- **[P]**: Can run in parallel (different lessons, no dependencies)
- **[Lesson]**: Which lesson this task belongs to (L1, L2, L3, L4, L5)
- Include exact file paths in descriptions

**Path Convention**: All lessons under `book-source/docs/04-Part-4-Python-Fundamentals/21-exception-handling/`

---

## Phase 1: Setup (Chapter Infrastructure)

**Purpose**: Initialize chapter directory structure and shared resources

- [ ] T001 Create chapter directory: `book-source/docs/04-Part-4-Python-Fundamentals/21-exception-handling/`
- [ ] T002 Create chapter readme: `book-source/docs/04-Part-4-Python-Fundamentals/21-exception-handling/readme.md`
- [ ] T003 [P] Validate Python 3.14+ environment for code examples
- [ ] T004 [P] Set up code validation workflow (all examples must run)

**Checkpoint**: Chapter structure ready - lesson writing can begin

---

## Phase 2: Lesson 1 - Exception Fundamentals (A2 Level) 📘 Priority P1

**Goal**: Student understands what exceptions are, writes basic try/except, recognizes common error types

**Independent Test**: Student can write try/except catching 2+ different exception types with appropriate handling

**Learning Objectives** (from plan):
- Understand what exceptions are and why they occur
- Write basic try/except blocks
- Recognize ValueError, TypeError, ZeroDivisionError by name
- Read tracebacks to find error location

**Content Requirements** (from plan):
- 4 code examples (all with type hints, Python 3.14+)
- 3 progressive exercises
- 4 CoLearning elements (💬🎓🚀✨)
- 4-prompt "Try With AI" closure (NO additional summaries)

### Implementation for Lesson 1

- [ ] T005 [L1] Write lesson introduction explaining why exceptions matter in `01-exception-fundamentals.md`
- [ ] T006 [L1] Write section "What Are Exceptions?" (errors as objects with type/message/traceback)
- [ ] T007 [L1] Write section "Anatomy of a Traceback" (how to read error messages)
- [ ] T008 [L1] Write section "Basic try/except Structure" (attempt code, catch if error)
- [ ] T009 [L1] Write section "Common Exception Types" (ValueError, TypeError, ZeroDivisionError with examples)
- [ ] T010 [L1] Create Code Example 1: Uncaught exception crash → Same with try/except
- [ ] T011 [L1] Create Code Example 2: ValueError from `int(input())` with non-numeric input
- [ ] T012 [L1] Create Code Example 3: TypeError using wrong operation on type
- [ ] T013 [L1] Create Code Example 4: ZeroDivisionError with recovery
- [ ] T014 [L1] Create Exercise 1: Catch ValueError from string-to-int conversion
- [ ] T015 [L1] Create Exercise 2: Catch ZeroDivisionError; provide fallback
- [ ] T016 [L1] Create Exercise 3: Read traceback and identify error source
- [ ] T017 [L1] Add 💬 AI Colearning Prompt after try/except intro: "How does Python determine exception type?"
- [ ] T018 [L1] Add 🎓 Instructor Commentary after exceptions shown: "Semantics not syntax—understanding > memorization"
- [ ] T019 [L1] Add 🚀 CoLearning Challenge mid-lesson: "Describe your function's potential errors; ask AI to show exception handling"
- [ ] T020 [L1] Add ✨ Teaching Tip before exercises: "Use Claude Code to test your exception handling with various inputs"
- [ ] T021 [L1] Write "Try With AI" section with 4 prompts (Remember → Understand → Apply → Analyze)
- [ ] T022 [L1] Validate: All code examples run on Python 3.14+
- [ ] T023 [L1] Validate: Type hints used consistently
- [ ] T024 [L1] Validate: Lesson ends with "Try With AI" ONLY (no summaries/checklists after)
- [ ] T025 [L1] Validate: Reading level Grade 7-8, conversational tone (you, your, we)

**Checkpoint**: Lesson 1 complete and independently testable. Prerequisites established for Lesson 2.

---

## Phase 3: Lesson 2 - Except, Else, Finally (A2-B1 Level) 📘 Priority P2

**Goal**: Student writes multi-block exception handling; understands when else/finally execute

**Independent Test**: Student writes code with try/except/else/finally correctly predicting which blocks run

**Learning Objectives** (from plan):
- Write multiple except blocks for different error types
- Use else block (runs only if no exception)
- Use finally block (runs regardless, guaranteed execution)
- Predict control flow through all 4 blocks

**Content Requirements**:
- 4 code examples
- 3 exercises
- 4 CoLearning elements
- 4-prompt "Try With AI" closure

**Prerequisites**: Lesson 1 concepts (try/except, common exception types)

### Implementation for Lesson 2

- [ ] T026 [P] [L2] Write lesson introduction reviewing single try/except in `02-except-else-finally.md`
- [ ] T027 [L2] Write section "Multiple Except Blocks" (different errors, different handling)
- [ ] T028 [L2] Write section "The else Block" (runs only if NO exception occurred)
- [ ] T029 [L2] Write section "The finally Block" (runs regardless, guaranteed cleanup)
- [ ] T030 [L2] Write section "Order Matters" (except blocks evaluated top-down; finally always last)
- [ ] T031 [L2] Create Code Example 1: Single except → Multiple except blocks for different types
- [ ] T032 [L2] Create Code Example 2: try/except/else showing when else runs vs except
- [ ] T033 [L2] Create Code Example 3: try/except/finally showing guaranteed execution
- [ ] T034 [L2] Create Code Example 4: Complete four-block structure (try/except/else/finally)
- [ ] T035 [L2] Create Exercise 1: Multiple except blocks catching different errors
- [ ] T036 [L2] Create Exercise 2: try/except/else where else runs on success
- [ ] T037 [L2] Create Exercise 3: try/except/finally with cleanup code
- [ ] T038 [L2] Add 💬 Colearning Prompt: "Show me two exceptions; why catch them separately?"
- [ ] T039 [L2] Add 🎓 Instructor Commentary: "Semantics not syntax—when to use finally vs else"
- [ ] T040 [L2] Add 🚀 CoLearning Challenge: "Ask AI about file operations with finally block"
- [ ] T041 [L2] Add ✨ Teaching Tip: "Test different exception scenarios to see which blocks execute"
- [ ] T042 [L2] Write "Try With AI" section with 4 prompts (Remember → Understand → Apply → Analyze)
- [ ] T043 [L2] Validate: Uses only Lesson 1 + 2 concepts (no forward references)
- [ ] T044 [L2] Validate: All code runs on Python 3.14+, type hints used
- [ ] T045 [L2] Validate: Lesson ends with "Try With AI" ONLY

**Checkpoint**: Lesson 2 complete. Students understand multi-block exception handling.

---

## Phase 4: Lesson 3 - Raising & Custom Exceptions (B1 Level) 📘 Priority P2

**Goal**: Student writes functions that raise exceptions; creates custom exception classes

**Independent Test**: Student writes function raising custom exception + caller catching it with meaningful error recovery

**Learning Objectives** (from plan):
- Write functions that check preconditions and raise exceptions when violated
- Create custom exception classes inheriting from Exception
- Write error messages that explain what went wrong and why

**Content Requirements**:
- 4 code examples
- 3 exercises
- 4 CoLearning elements
- 4-prompt "Try With AI" closure

**Prerequisites**: Lessons 1-2 concepts (try/except/else/finally, exception types)

### Implementation for Lesson 3

- [ ] T046 [P] [L3] Write lesson introduction explaining when to raise exceptions in `03-raising-custom-exceptions.md`
- [ ] T047 [L3] Write section "The raise Statement" (explicitly signal error from function)
- [ ] T048 [L3] Write section "Custom Exception Classes" (inherit from Exception, domain-specific errors)
- [ ] T049 [L3] Write section "Meaningful Error Messages" (what went wrong, why, what to do)
- [ ] T050 [L3] Create Code Example 1: Function checking preconditions and raising ValueError
- [ ] T051 [L3] Create Code Example 2: Custom exception class (e.g., InvalidAgeError) definition
- [ ] T052 [L3] Create Code Example 3: Function raising custom exception with descriptive message
- [ ] T053 [L3] Create Code Example 4: Caller catching custom exception vs generic Exception
- [ ] T054 [L3] Create Exercise 1: Function validates positive integer, raises custom exception if negative
- [ ] T055 [L3] Create Exercise 2: Custom exception class for domain error (e.g., InvalidEmailError)
- [ ] T056 [L3] Create Exercise 3: Write error message that helps user fix problem
- [ ] T057 [L3] Add 💬 Colearning Prompt: "How do I design custom exceptions? When are they better than built-in types?"
- [ ] T058 [L3] Add 🎓 Instructor Commentary: "Custom exceptions communicate intent—code clarity over memorization"
- [ ] T059 [L3] Add 🚀 CoLearning Challenge: "Describe validation rule; ask AI to generate custom exception and raise logic"
- [ ] T060 [L3] Add ✨ Teaching Tip: "Ask Claude Code: 'What makes a good error message?' Test with real scenarios"
- [ ] T061 [L3] Write "Try With AI" section with 4 prompts (Apply → Analyze → Evaluate → Create)
- [ ] T062 [L3] Validate: Uses only Lessons 1-3 concepts (no Lesson 4+ concepts)
- [ ] T063 [L3] Validate: All code runs, type hints used, modern Python 3.14+ syntax
- [ ] T064 [L3] Validate: Lesson ends with "Try With AI" ONLY

**Checkpoint**: Lesson 3 complete. Students write defensive functions with custom exceptions.

---

## Phase 5: Lesson 4 - Error Handling Strategies (B1 Level) 📘 Priority P2

**Goal**: Student applies error handling strategies to realistic scenarios (retry, fallback, logging)

**Independent Test**: Student applies appropriate error handling strategy to given scenario and justifies choice

**Learning Objectives** (from plan):
- Apply retry logic for transient errors
- Implement fallback values for recoverable errors
- Use graceful degradation when feature unavailable
- Log errors for debugging without crashing program

**Content Requirements**:
- 4 code examples
- 3 exercises
- 4 CoLearning elements
- 4-prompt "Try With AI" closure

**Prerequisites**: Lessons 1-3 concepts (try/except, raising, custom exceptions)

### Implementation for Lesson 4

- [ ] T065 [P] [L4] Write lesson introduction explaining defensive programming patterns in `04-error-handling-strategies.md`
- [ ] T066 [L4] Write section "Retry Logic" (transient errors, exponential backoff pattern)
- [ ] T067 [L4] Write section "Fallback Values" (provide defaults when operation fails)
- [ ] T068 [L4] Write section "Graceful Degradation" (feature unavailable, core functionality continues)
- [ ] T069 [L4] Write section "Logging Errors" (record for debugging without crashing)
- [ ] T070 [L4] Create Code Example 1: Retry logic with max attempts for file operation
- [ ] T071 [L4] Create Code Example 2: Fallback value when API call fails
- [ ] T072 [L4] Create Code Example 3: Graceful degradation (optional feature fails, core continues)
- [ ] T073 [L4] Create Code Example 4: Logging errors with context for debugging
- [ ] T074 [L4] Create Exercise 1: Implement retry logic for network request
- [ ] T075 [L4] Create Exercise 2: Provide fallback when configuration file missing
- [ ] T076 [L4] Create Exercise 3: Choose appropriate strategy for given scenario and justify
- [ ] T077 [L4] Add 💬 Colearning Prompt: "What's the best error handling strategy for file I/O? For network operations?"
- [ ] T078 [L4] Add 🎓 Instructor Commentary: "Professional developers think strategically—not just catching errors, but recovering gracefully"
- [ ] T079 [L4] Add 🚀 CoLearning Challenge: "Describe real-world scenario; ask AI to suggest error handling strategy and explain trade-offs"
- [ ] T080 [L4] Add ✨ Teaching Tip: "Ask Claude Code: 'How do professionals handle network errors?' Learn patterns, not recipes"
- [ ] T081 [L4] Write "Try With AI" section with 4 prompts (Apply → Analyze → Design → Criticize)
- [ ] T082 [L4] Validate: Uses only Lessons 1-4 concepts (no Lesson 5 capstone patterns)
- [ ] T083 [L4] Validate: All code runs, type hints, Python 3.14+
- [ ] T084 [L4] Validate: Lesson ends with "Try With AI" ONLY

**Checkpoint**: Lesson 4 complete. Students apply error handling strategies to real scenarios. Ready for capstone integration.

---

## Phase 6: Lesson 5 - Capstone: Robust CSV Parser (B1 Level) 📘 Priority P3

**Goal**: Student builds complete file parser integrating all Lessons 1-4 concepts in realistic project

**Independent Test**: Parser handles 4+ error types gracefully, provides helpful feedback, reports summary

**Learning Objectives** (from plan):
- Integrate exception handling from Lessons 1-4 in realistic project
- Test error handling with intentional edge cases
- Validate robustness (no crashes, helpful user feedback)
- Demonstrate specification → validation → implementation cycle

**Content Requirements**:
- Capstone specification from spec.md
- Validation steps and expected outcomes
- 4 CoLearning elements (focused on testing and debugging)
- 4-prompt "Try With AI" closure
- 0 new concepts (integration only)

**Prerequisites**: Lessons 1-4 concepts (ALL exception handling patterns learned)

### Implementation for Lesson 5 (Capstone)

- [ ] T085 [P] [L5] Write capstone introduction explaining project goals in `05-capstone-csv-parser.md`
- [ ] T086 [L5] Write section "Project Specification" (what parser must do, from spec.md)
- [ ] T087 [L5] Write section "Planning Error Handling" (which exceptions to catch where)
- [ ] T088 [L5] Write section "Implementation Strategy" (build incrementally, test each error type)
- [ ] T089 [L5] Write section "CSV Parser Core Logic" (reading file, validating rows)
- [ ] T090 [L5] Write section "Handling FileNotFoundError" (tell user file location)
- [ ] T091 [L5] Write section "Handling ValueError" (log malformed row, continue processing)
- [ ] T092 [L5] Write section "Handling PermissionError" (tell user permission denied)
- [ ] T093 [L5] Write section "Generating Summary Report" (total rows, errors, successfully validated)
- [ ] T094 [L5] Create complete CSV parser code example integrating all Lessons 1-4 patterns
- [ ] T095 [L5] Create test CSV files (valid data, missing values, malformed data, etc.)
- [ ] T096 [L5] Write section "Testing Your Parser" (validate error handling works)
- [ ] T097 [L5] Write section "Debugging Common Issues" (what to check when parser fails)
- [ ] T098 [L5] Add 💬 Colearning Prompt: "What edge cases might my parser miss? How do I test for them?"
- [ ] T099 [L5] Add 🎓 Instructor Commentary: "Real projects integrate patterns—capstone shows you're thinking like a developer"
- [ ] T100 [L5] Add 🚀 CoLearning Challenge: "Ask AI to review your parser: 'What error scenarios did I miss?'"
- [ ] T101 [L5] Add ✨ Teaching Tip: "Use Claude Code to generate test cases: 'Create CSV files that test my error handling'"
- [ ] T102 [L5] Write "Try With AI" section with 4 prompts (Understand spec → Implement → Test → Debug issues)
- [ ] T103 [L5] Validate: Integrates ALL concepts from Lessons 1-4 (0 new concepts introduced)
- [ ] T104 [L5] Validate: Parser code runs on Python 3.14+, handles 4+ error types
- [ ] T105 [L5] Validate: Lesson ends with "Try With AI" ONLY
- [ ] T106 [L5] Validate: No forward references to Chapters 22+ (chapter scope only)

**Checkpoint**: Capstone complete. Students have built realistic project demonstrating all exception handling concepts.

---

## Phase 7: Chapter Polish & Validation

**Purpose**: Final quality checks, cross-lesson consistency, technical validation

- [ ] T107 Update chapter readme with lesson navigation and learning objectives
- [ ] T108 [P] Validate all 5 lessons use consistent tone (conversational, you/your/we)
- [ ] T109 [P] Validate all code examples run on Python 3.14+ (no syntax errors)
- [ ] T110 [P] Validate all type hints used consistently across all lessons
- [ ] T111 [P] Validate no forward references (Lessons 1-N only use concepts 1-N)
- [ ] T112 [P] Validate lesson closure pattern (all end with "Try With AI" ONLY)
- [ ] T113 [P] Validate CoLearning elements present in all lessons (💬🎓🚀✨)
- [ ] T114 [P] Validate CEFR progression (A2 → B1, no zigzag)
- [ ] T115 [P] Validate cognitive load (Lesson 1: 4 concepts, Lesson 2: 3, Lesson 3: 3, Lesson 4: 2, Lesson 5: 0)
- [ ] T116 [P] Validate reading level (Grade 7-8, Flesch-Kincaid 45-55)
- [ ] T117 Run validation-auditor subagent for complete chapter validation
- [ ] T118 Address critical issues from technical review (if any)
- [ ] T119 Update chapter-index.md status from "📋 Planned" to "✅ Implemented & Validated"

**Checkpoint**: Chapter 21 complete, validated, ready for publication

---

## Dependencies & Lesson Completion Order

**Strict Sequential Dependencies**:
```
Phase 1 (Setup) → Phase 2 (Lesson 1) → Phase 3 (Lesson 2) → Phase 4 (Lesson 3) → Phase 5 (Lesson 4) → Phase 6 (Lesson 5) → Phase 7 (Polish)
```

**Why Sequential**: Each lesson builds on prior concepts. Lesson boundaries validated. No parallel lesson writing.

**Within-Lesson Parallelization**: Tasks within same lesson marked [P] can be done in parallel (e.g., T010-T013 create different code examples simultaneously).

---

## Parallel Execution Examples

**Setup Phase (T001-T004)**: All can run in parallel (different setup tasks)

**Lesson 1 (T010-T013)**: Code examples can be written in parallel
**Lesson 1 (T014-T016)**: Exercises can be written in parallel
**Lesson 1 (T017-T020)**: CoLearning elements can be placed in parallel

**Lesson 2-4**: Same parallelization pattern (code examples, exercises, CoLearning)

**Lesson 5**: Most tasks sequential (capstone builds incrementally)

**Polish Phase (T108-T116)**: All validation tasks can run in parallel

---

## Implementation Strategy

**MVP Scope** (Minimum Viable Product):
- **Phase 1-2**: Lesson 1 (Exception Fundamentals) only
- **Why**: Establishes foundation; independently testable; delivers immediate value

**Incremental Delivery**:
1. **Sprint 1**: Lesson 1 (foundation)
2. **Sprint 2**: Lesson 2 (multi-block handling)
3. **Sprint 3**: Lesson 3 (raising/custom exceptions)
4. **Sprint 4**: Lesson 4 (strategies)
5. **Sprint 5**: Lesson 5 (capstone integration)
6. **Sprint 6**: Polish and validation

**Total Estimated Effort**: 119 tasks across 7 phases

---

## Task Summary

| Phase | Tasks | Parallelizable | Sequential | Focus |
|-------|-------|---------------|-----------|-------|
| 1: Setup | 4 | 2 | 2 | Chapter infrastructure |
| 2: Lesson 1 | 21 | 15 | 6 | Exception fundamentals (A2) |
| 3: Lesson 2 | 20 | 14 | 6 | Multi-block handling (A2-B1) |
| 4: Lesson 3 | 19 | 13 | 6 | Raising/custom exceptions (B1) |
| 5: Lesson 4 | 20 | 14 | 6 | Error handling strategies (B1) |
| 6: Lesson 5 | 22 | 4 | 18 | Capstone integration (B1) |
| 7: Polish | 13 | 12 | 1 | Validation and publication |
| **TOTAL** | **119** | **74** | **45** | **5 lessons + polish** |

**Parallel Opportunities**: 62% of tasks (74/119) can run in parallel within their phase
**Critical Path**: 45 sequential tasks determine minimum timeline

---

## Policy Notes for Lesson Authors

**Lesson Closure Pattern** (Constitution Mandate):
- Every lesson MUST end with "Try With AI" section containing exactly 4 prompts
- NO "Key Takeaways", "Summary", "What's Next", or "Completion Checklist" after "Try With AI"
- Prompt 4 provides cognitive closure through synthesis/reflection

**AI Tool References**:
- Part 1 (before tool onboarding): Use ChatGPT web in "Try With AI" sections
- Part 2+ (after Claude Code/Gemini CLI introduction): Instruct learners to use their preferred AI companion tool
- Optionally provide both CLI and web variants for flexibility

**No SDD Terminology in Part 4**:
- Use "describe intent" (NOT "write specification")
- Use "type hints and clear code" (NOT "formal specifications")
- References to AIDD from Part 1 are acceptable for context
- Formal SDD taught in Part 5 (Chapters 30+)

**Forward References**:
- Zero references to Chapters 22+ content
- Chapter 21 scope only (exception handling)
- Non-goals explicitly listed in spec.md (context managers deep dive, async exceptions, logging configuration deferred)

---

## Next Steps After Task Completion

1. **Run `/sp.implement`**: Invoke content-implementer subagent with this tasks.md as checklist
2. **Technical Review**: Run validation-auditor subagent on completed lessons
3. **User Review**: Present completed chapter for human feedback
4. **Iterate**: Address feedback, refine lessons
5. **Publish**: Merge to main branch, update chapter-index.md status
