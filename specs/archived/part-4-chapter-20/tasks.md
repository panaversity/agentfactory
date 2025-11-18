# Chapter 20: Modules and Functions — Implementation Tasks

**Chapter**: 20 - "Module and Functions"
**Part**: 4 (Python Fundamentals)
**Total Tasks**: 52 implementation tasks across 6 phases
**Status**: Ready for Implementation
**Target Complexity**: A2-B1 (Intermediate)
**Estimated Implementation Time**: 50–60 hours (full chapter with testing and validation)

---

## Overview

This document breaks down the creation of Chapter 20 into atomic, independently testable tasks organized by user story priority and lesson sequence. Each task is specific enough for an LLM (content-implementer subagent) to execute without additional context.

---

## Task Organization

**Phase 1: Setup & Infrastructure** (5 tasks)
- Create chapter directory structure
- Initialize lesson skeleton files
- Set up code examples directories
- Validate directory structure

**Phase 2: Foundational Content** (4 tasks)
- Create chapter readme.md with overview
- Create chapter intro section (shared across lessons)
- Establish code example style guide (Python 3.14+, type hints)
- Set up testing framework for code examples

**Phase 3: User Story 1 - Module Imports (P1)** (8 tasks)
- Lesson 1: Understanding Modules and Imports (full lesson with CoLearning elements and Try With AI)

**Phase 4: User Story 2 - Functions with Intent (P1)** (10 tasks)
- Lesson 2: Writing Functions with Intent (full lesson with type hints, docstrings)
- Lesson 3: Function Parameters and Returns (full lesson with advanced parameter patterns)

**Phase 5: User Story 3 & 4 - Scope & Capstone (P2)** (20 tasks)
- Lesson 4: Scope and Nested Functions (full lesson with scope exploration)
- Lesson 5: Calculator Utility Capstone (full capstone project with multi-module structure)

**Phase 6: Polish & Validation** (5 tasks)
- Cross-lesson consistency check
- Technical review preparation
- Final validation against specification
- Chapter index update
- Documentation review

---

## Task Checklist

### Phase 1: Setup & Infrastructure

- [ ] T001 Create chapter directory `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/ain/book-source/docs/04-Part-4-Python-Fundamentals/20-module-functions/`
- [ ] T002 Create lesson skeleton files (01-understanding-modules-imports.md, 02-writing-functions-intent.md, 03-function-parameters-returns.md, 04-scope-nested-functions.md, 05-calculator-utility-capstone.md)
- [ ] T003 Create code examples directory `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/ain/book-source/docs/04-Part-4-Python-Fundamentals/20-module-functions/examples/` with subdirectories: `lesson-1/`, `lesson-2/`, `lesson-3/`, `lesson-4/`, `lesson-5-capstone/`
- [ ] T004 Create chapter README.md with overview, prerequisites, learning objectives, and navigation
- [ ] T005 Validate directory structure against book conventions (naming, nesting, file permissions)

### Phase 2: Foundational Content

- [ ] T006 Create chapter introduction section (explaining Chapter 20's role in Part 4, why modules and functions matter, connection to AIDD)
- [ ] T007 Document code example style guide: Python 3.14+ syntax, mandatory type hints (`def func(x: int) -> int:`), docstrings for all functions, consistent formatting
- [ ] T008 Set up Python 3.14+ test environment for validating code examples (requirements.txt, test harness)
- [ ] T009 Create template for "Try With AI" section to ensure consistency across all 5 lessons (exactly 4 prompts, Bloom's progression, no sections after Try With AI)

### Phase 3: User Story 1 - Module Imports (P1) [US1]

**Story Goal**: Students understand module organization and can import + use built-in modules (math, random, os) with confidence.

**Independent Test Criteria**:
- Student can execute `import math` and use `math.sqrt()` correctly
- Student can explain difference between `import math`, `from math import sqrt`, `from math import sqrt as square_root`
- Student can import and use 3+ built-in modules without errors

- [ ] T010 [US1] Write Lesson 1 introduction: "What is a Module and Why Organize Code?" (5 min) in `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/ain/book-source/docs/04-Part-4-Python-Fundamentals/20-module-functions/01-understanding-modules-imports.md`
- [ ] T011 [US1] Write Lesson 1 foundational concept section: "Module Concept" explaining .py files, standard library, code reusability (8 min) with 2-3 sentence explanation + why it matters
- [ ] T012 [P] [US1] Create Code Example 1: Understanding Module Access (`import math` → namespace access) in `examples/lesson-1/example-1-module-access.py` with explanation
- [ ] T013 [P] [US1] Create Code Example 2: Direct Import (`from math import sqrt`) in `examples/lesson-1/example-2-direct-import.py` with explanation
- [ ] T014 [P] [US1] Create Code Example 3: Import with Alias (`as` keyword) in `examples/lesson-1/example-3-import-alias.py` with explanation
- [ ] T015 [P] [US1] Create Code Example 4: Using math module (math.floor, math.ceil, math.sqrt) in `examples/lesson-1/example-4-math-module.py` with explanation
- [ ] T016 [P] [US1] Create Code Example 5: Using random module (random.choice, random.randint, random.shuffle) in `examples/lesson-1/example-5-random-module.py` with explanation
- [ ] T017 [US1] Write Lesson 1 CoLearning elements: 💬 conceptual prompt, 🎓 commentary, 🚀 challenge, ✨ tip (must address module discovery and AI tool usage)
- [ ] T018 [US1] Write Lesson 1 "Try With AI" section: 4 progressive prompts (Remember → Understand → Apply → Analyze) with expected outcomes, NO additional sections after this

### Phase 4: User Story 2 - Functions with Intent (P1) [US2]

**Story Goal**: Students write functions with clear intent through type hints, docstrings, and can apply functions to real problems (calculations, data processing).

**Independent Test Criteria**:
- Student can write `def add(a: int, b: int) -> int:` with docstring
- Student can apply functions to 3+ real problems (calculate, process data, organize code)
- Student understands function parameters vs returns

- [ ] T019 [US2] Write Lesson 2 introduction: "Functions as Reusable Code Blocks" (5 min) emphasizing intent clarity through type hints
- [ ] T020 [US2] Write Lesson 2 foundational concept section: "Function Definition and Intent" explaining `def`, parameters, return, type hints, docstrings (10 min)
- [ ] T021 [P] [US2] Create Code Example 1 (Lesson 2): Simple function definition (`def add(a: int, b: int) -> int:`) in `examples/lesson-2/example-1-function-def.py` with docstring and explanation
- [ ] T022 [P] [US2] Create Code Example 2 (Lesson 2): Function with docstring (`"""Calculates...."""` format) in `examples/lesson-2/example-2-function-docstring.py` with explanation
- [ ] T023 [P] [US2] Create Code Example 3 (Lesson 2): Type hints in action (parameter type validation concept) in `examples/lesson-2/example-3-type-hints.py` with explanation
- [ ] T024 [P] [US2] Create Code Example 4 (Lesson 2): Calling functions and handling return values in `examples/lesson-2/example-4-function-calls.py` with explanation
- [ ] T025 [US2] Write Lesson 2 CoLearning elements: 💬 prompt (function specification), 🎓 commentary (semantics over syntax), 🚀 challenge (write a function), ✨ tip (AI code generation)
- [ ] T026 [US2] Write Lesson 2 "Try With AI" section: 4 prompts (Remember → Apply → Analyze → Synthesize) with expected outcomes
- [ ] T027 [US2] Write Lesson 3 introduction: "Function Parameters and Return Values" (5 min)
- [ ] T028 [US2] Write Lesson 3 foundational concept section: "Positional vs Default Parameters and Returns" (10 min)
- [ ] T029 [P] [US2] Create Code Example 1 (Lesson 3): Positional parameters in `examples/lesson-3/example-1-positional-params.py` with explanation
- [ ] T030 [P] [US2] Create Code Example 2 (Lesson 3): Default parameters (`def greet(name: str, greeting: str = "Hello"):`) in `examples/lesson-3/example-2-default-params.py` with explanation
- [ ] T031 [P] [US2] Create Code Example 3 (Lesson 3): Multiple return values and unpacking in `examples/lesson-3/example-3-multiple-returns.py` with explanation
- [ ] T032 [US2] Write Lesson 3 CoLearning elements: 💬 prompt, 🎓 commentary, 🚀 challenge, ✨ tip
- [ ] T033 [US2] Write Lesson 3 "Try With AI" section: 4 prompts with expected outcomes

### Phase 5: User Story 3 & 4 - Scope & Capstone (P2) [US3] [US4]

**Story Goal (US3)**: Students understand variable scope (local, global, enclosing) and can predict code behavior; nested functions prepared for Chapter 26+.

**Story Goal (US4)**: Students build integrated multi-module Calculator Utility applying all Chapter 20 concepts in cohesive, testable project.

**Independent Test Criteria (US3)**:
- Student can predict output of local/global variable code in 80%+ of scenarios
- Student understands when to use `global` keyword
- Student recognizes nested functions as foundation for decorators

**Independent Test Criteria (US4)**:
- Student builds Calculator Utility with 3 modules (math_operations.py, utils.py, main.py)
- All functions have type hints and docstrings
- Project runs without errors and produces expected output
- Student can test each function individually

- [ ] T034 [US3] Write Lesson 4 introduction: "Understanding Variable Scope" (5 min)
- [ ] T035 [US3] Write Lesson 4 foundational concept section: "Local, Global, and Enclosing Scope" (10 min)
- [ ] T036 [P] [US3] Create Code Example 1 (Lesson 4): Local vs global variables in `examples/lesson-4/example-1-local-global.py` with explanation
- [ ] T037 [P] [US3] Create Code Example 2 (Lesson 4): The `global` keyword for module-level modification in `examples/lesson-4/example-2-global-keyword.py` with explanation
- [ ] T038 [P] [US3] Create Code Example 3 (Lesson 4): Nested functions and closures (awareness for Chapter 26) in `examples/lesson-4/example-3-nested-functions.py` with explanation
- [ ] T039 [US3] Write Lesson 4 CoLearning elements: 💬 prompt (predict scope), 🎓 commentary (debugging skill), 🚀 challenge (scope puzzle), ✨ tip (AI as debugger)
- [ ] T040 [US3] Write Lesson 4 "Try With AI" section: 4 prompts with expected outcomes
- [ ] T041 [US4] Write Lesson 5 introduction: "Building a Multi-Module Project" (5 min) explaining capstone concept
- [ ] T042 [US4] Write Lesson 5 project overview: Calculator Utility structure (3 modules: math_operations.py, utils.py, main.py) with diagram or explanation
- [ ] T043 [P] [US4] Create Code Example 1 (Lesson 5 - Capstone): math_operations.py module with functions: `add(a, b)`, `subtract(a, b)`, `multiply(a, b)`, `divide(a, b)` — all with type hints and docstrings in `examples/lesson-5-capstone/math_operations.py`
- [ ] T044 [P] [US4] Create Code Example 2 (Lesson 5 - Capstone): utils.py module with functions: `validate_input(x: str) -> float`, `format_result(result: float) -> str` in `examples/lesson-5-capstone/utils.py`
- [ ] T045 [P] [US4] Create Code Example 3 (Lesson 5 - Capstone): main.py orchestrating math_operations and utils, accepting user input, calling functions, displaying results in `examples/lesson-5-capstone/main.py`
- [ ] T046 [P] [US4] Create Code Example 4 (Lesson 5 - Capstone): test_calculator.py validating each function (unit tests for add, subtract, etc.) in `examples/lesson-5-capstone/test_calculator.py`
- [ ] T047 [US4] Write Lesson 5 integration narrative: How modules work together, why separation of concerns matters, preparation for OOP (Chapter 24)
- [ ] T048 [US4] Write Lesson 5 CoLearning elements: 💬 prompt (AI as project architect), 🎓 commentary (orchestration over implementation), 🚀 challenge (extend calculator), ✨ tip (testing as verification)
- [ ] T049 [US4] Write Lesson 5 "Try With AI" section: 4 prompts (Remember capstone structure → Apply to testing → Analyze design → Synthesize learning) with expected outcomes

### Phase 6: Polish & Validation

- [ ] T050 [P] Cross-lesson consistency check: Verify all 5 lessons follow "Try With AI" closure pattern (4 prompts, Bloom's progression, NO additional sections after Try With AI)
- [ ] T051 [P] Validate code examples: Run all 30+ examples against Python 3.14+ to ensure correctness (no syntax errors, type hints valid, expected output matches)
- [ ] T052 [P] Create technical review checklist document including: pedagogical ordering validation, forward reference check, concept count per lesson, CEFR progression, CoLearning element presence
- [ ] T053 Update chapter-index.md status: Change Chapter 20 from "📋 Planned" to "✅ Implemented & Ready for Validation"
- [ ] T054 Create final validation report: Spec → Plan → Tasks alignment, success criteria coverage, student learning path verification

---

## Dependency Graph

```
Phase 1 (Setup): T001 → T002 → T003 → T004 → T005 ✓
                 ↓
Phase 2 (Foundational): T006, T007, T008, T009 (parallel) ✓
                 ↓
Phase 3 (US1 - Modules): T010 → [T012-T016 parallel] → T017 → T018 ✓
                 ↓
Phase 4 (US2 - Functions): T019 → [T021-T024 parallel] → T025 → T026 ✓
                           → T027 → [T029-T031 parallel] → T032 → T033 ✓
                 ↓
Phase 5 (US3/US4): T034 → [T036-T038 parallel] → T039 → T040 ✓
                   T041 → T042 → [T043-T046 parallel] → T047 → T048 → T049 ✓
                 ↓
Phase 6 (Polish): T050, T051, T052, T053, T054 (parallel except ordering) ✓
```

**Critical Path**: T001 → T002 → T003 → T004 → T005 → T006-T009 → T010-T018 → T019-T033 → T034-T049 → T050-T054
**Estimated Critical Path Time**: 50–60 hours

---

## Parallel Execution Strategy

**Maximum Parallelization** (for content-implementer subagent team approach):

1. **Phase 1-2 Sequential** (setup requires order)
2. **Phase 3 Sequential** (Lesson 1 foundational, code examples can run in parallel once lesson text is written)
3. **Phase 4 Parallel** (Lesson 2 + Lesson 3 can be written in parallel; code examples [T021-T024] and [T029-T031] parallel)
4. **Phase 5 Parallel** (Lesson 4 + Lesson 5 can be written in parallel; code examples in each lesson parallel)
5. **Phase 6 Parallel** (validation tasks independent)

**Recommended Team Split** (if using multiple content-implementer subagents):
- **Agent A**: Lesson 1 (T010-T018)
- **Agent B**: Lesson 2 (T019-T026)
- **Agent C**: Lesson 3 (T027-T033)
- **Agent D**: Lesson 4 (T034-T040)
- **Agent E**: Lesson 5 Capstone (T041-T049)
- **Agent F**: Validation & Polish (T050-T054)

---

## Success Criteria per Task Phase

**Phase 3 (US1) Success**:
- [ ] Lesson 1 teaches all 5 concepts without forward references
- [ ] 6 code examples all use Python 3.14+ type hints
- [ ] Try With AI section has exactly 4 prompts (Bloom's progression)
- [ ] CoLearning elements present (💬🎓🚀✨)
- [ ] Students can independently import and use 3+ built-in modules

**Phase 4 (US2) Success**:
- [ ] Lesson 2 teaches 6 concepts (function def, parameters, returns, type hints, docstrings, calling)
- [ ] Lesson 3 teaches 5 concepts (positional params, defaults, returns, unpacking, *args awareness)
- [ ] All functions use mandatory type hints and docstrings
- [ ] 8+ code examples across both lessons
- [ ] Try With AI sections properly formatted (4 prompts each, Bloom's progression)
- [ ] Students can write functions with clear intent on first attempt

**Phase 5 (US3/US4) Success**:
- [ ] Lesson 4 teaches scope (local, global, enclosing, nested) with CEFR B1-B2 complexity
- [ ] Lesson 5 Capstone integrates all previous concepts
- [ ] Calculator Utility has 3 modules, 8-10 functions total, full test coverage
- [ ] All code examples runnable and produce expected output
- [ ] Students build integrated project without errors

**Phase 6 Success**:
- [ ] All lessons follow consistency checklist
- [ ] All 30+ code examples validated for correctness
- [ ] No forward references within chapter
- [ ] Concept count per lesson within limits (5, 6, 5, 4, 7)
- [ ] Chapter ready for technical review

---

## Notes for Implementation

**Pedagogical Ordering Rules** (CRITICAL - enforce in every task):
1. Each lesson uses ONLY concepts from current + prior lessons (NO forward references)
2. Every function/keyword introduced with explanation before first use
3. Built-ins (len, type, isinstance) vs methods (.upper, .split) clearly distinguished
4. Lesson boundaries strictly enforced (L1 concepts not used until L1 is complete)

**AI-Native Learning** (CRITICAL - embed in all lessons):
- Every lesson demonstrates AI as intellectual partner, not code generator
- 💬 Conceptual prompts encourage exploration
- 🎓 Commentary emphasizes reasoning ("syntax cheap, semantics gold")
- 🚀 Challenges practice specification-driven thinking
- ✨ Tips build AI collaboration patterns

**Try With AI Format** (CRITICAL - exact compliance required):
- Exactly 4 prompts per lesson
- Progressive Bloom's levels: Remember → Understand → Apply → Analyze/Synthesize
- Each prompt with explicit "Expected outcome"
- NO "Key Takeaways", "Summary", or "What's Next" sections after Try With AI
- Try With AI is the ONLY closure point for each lesson

**Type Hints** (MANDATORY - all code examples):
- Every function: `def func(param: Type) -> ReturnType:`
- No ambiguous untyped functions
- Python 3.14+ modern syntax (list[int], dict[str, float], X | None)

---

## Acceptance & Validation Checklist

Before proceeding to content-implementer implementation, confirm:

- [ ] All 54 tasks are specific and actionable
- [ ] Each task has exact file path
- [ ] User stories (US1-US4) clearly mapped to task phases
- [ ] Independent test criteria defined for each story
- [ ] Dependency graph shows correct ordering
- [ ] Parallel opportunities identified
- [ ] Success criteria measurable and testable
- [ ] Estimated time realistic (50-60 hours)
- [ ] All pedagogical rules documented
- [ ] Try With AI format requirements clear
- [ ] Type hints and Python 3.14+ standards specified
- [ ] Capstone project structure fully detailed

---

**Status**: ✅ Ready for Lesson-Writer Implementation

This task list is comprehensive, specific, and ready for execution by the content-implementer subagent. All pedagogical decisions have been made; all constraints have been documented. The content-implementer has everything needed to create professional, specification-compliant lesson content for Chapter 20.
