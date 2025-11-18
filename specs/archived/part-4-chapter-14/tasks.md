# Tasks: Chapter 14 - Data Types

**Input**: Design documents from `/specs/part-4-chapter-14/`
**Prerequisites**: plan.md (required), spec.md (required)
**Chapter Type**: Educational Content (5 lessons)
**Target**: `book-source/docs/04-Part-4-Python-Fundamentals/14-data-types/`

**Organization**: Tasks are grouped by lesson to enable incremental chapter development. Each lesson maps to user stories from spec.md.

## Format: `[ID] [P?] [Lesson] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Lesson]**: Which lesson this task belongs to (e.g., L1, L2, L3, L4, L5)
- Include exact file paths in descriptions

## Path Conventions

**Book Content Structure**:
```
book-source/docs/04-Part-4-Python-Fundamentals/14-data-types/
├── readme.md              # Chapter index (auto-generated)
├── 01-variables-and-type-hints.md
├── 02-integers-and-floats.md
├── 03-strings-and-booleans.md
├── 04-collections-awareness.md
└── 05-type-explorer-capstone.md
```

---

## Phase 1: Setup (Chapter Structure)

**Purpose**: Initialize chapter directory structure and index

- [x] T001 Create chapter directory at `book-source/docs/04-Part-4-Python-Fundamentals/14-data-types/`
- [x] T002 Create chapter `readme.md` with frontmatter, title, and lesson links
- [x] T003 [P] Verify Python 3.14+ code examples environment (for testing code blocks)

---

## Phase 2: Lesson 1 - Variables and Type Hints (User Story 1: P1)

**Goal**: Students can create variables with type hints and understand how type hints describe intent

**Independent Test**: Student creates variables with type hints (int, float, str, bool) and uses print() to display values

**Lesson Mapping**:
- User Story 1: Variables with Type Hints (P1)
- Duration: 45 minutes
- Skills: Understanding Variables (A2), Writing Type Hints (A2), Describing Intent (B1)

### Implementation for Lesson 1

- [x] T004 [L1] Write lesson introduction in `book-source/docs/04-Part-4-Python-Fundamentals/14-data-types/01-variables-and-type-hints.md` (connection to Chapter 13 Hello World, why variables matter)
- [x] T005 [L1] Add "Concept: Variables Without Type Hints" section with Code Example 1 (5 lines: name, age, height, is_student)
- [x] T006 [L1] Add "Concept: Type Hints as Specifications" section with Code Example 2 (8 lines: same variables with type hints)
- [x] T007 [L1] Add "Using print() to Display Variables" section with Code Example 3 (6 lines: print variables from Ch 13 knowledge)
- [x] T008 [L1] Add "Try With AI" section with 4 prompts (concept exploration, syntax clarification, type variety, connection to AI-Native Learning)
- [x] T009 [L1] Test all code examples in Python 3.14+ environment
- [x] T010 [L1] Validate lesson follows output style (`.claude/output-styles/lesson.md`) and ends with "Try With AI" only

**Checkpoint**: ✅ Lesson 1 complete - students can create variables with type hints

---

## Phase 3: Lesson 2 - Understanding Data Types: Integers and Floats (User Story 2: P1)

**Goal**: Students understand the type concept, distinguish int vs float, use type() and isinstance()

**Independent Test**: Student explains "what is a type?" in own words, uses type() function, chooses int vs float appropriately

**Lesson Mapping**:
- User Story 2: Understanding Data Types through Numbers (P1)
- Duration: 50 minutes
- Skills: Data Type Concept (A2), Numeric Types (A2), Type Inspection (A2-B1)

### Implementation for Lesson 2

- [x] T011 [P] [L2] Write lesson introduction in `book-source/docs/04-Part-4-Python-Fundamentals/14-data-types/02-integers-and-floats.md` (what is a data type, why types matter)
- [x] T012 [L2] Add "Understanding the Type Concept" section (types as categories determining operations)
- [x] T013 [L2] Add "Numeric Types: int and float" section with Code Example 1 (6 lines: type() inspector)
- [x] T014 [L2] Add "When to Use int vs float" section with Code Example 2 (10 lines: counting vs measuring examples)
- [x] T015 [L2] Add "Type Validation with isinstance()" section with Code Example 3 (6 lines: isinstance() checker)
- [x] T016 [L2] Add "Try With AI" section with 4 prompts (type concept, int vs float, type() function, TypeError troubleshooting)
- [x] T017 [L2] Test all code examples in Python 3.14+ environment
- [x] T018 [L2] Validate lesson follows output style and ends with "Try With AI" only

**Checkpoint**: Lesson 2 complete - students understand type concept and numeric types

---

## Phase 4: Lesson 3 - Strings and Booleans: Text and Truth (User Story 3: P2)

**Goal**: Students understand str (text basics), bool (True/False), None, and type conversion

**Independent Test**: Student creates strings with type hints, understands truthy/falsy values, converts between types safely

**Lesson Mapping**:
- User Story 3: Strings and Booleans Understanding (P2)
- Duration: 50 minutes
- Skills: String Basics (A2), Boolean Logic (A2-B1), Type Conversion (B1)

### Implementation for Lesson 3

- [x] T019 [P] [L3] Write lesson introduction in `book-source/docs/04-Part-4-Python-Fundamentals/14-data-types/03-strings-and-booleans.md` (text and truth values)
- [x] T020 [L3] Add "String Basics" section with Code Example 1 (8 lines: creating strings, simple concatenation - defer methods to Ch 16)
- [x] T021 [L3] Add "Boolean Logic: True and False" section with Code Example 2 (10 lines: bool values, truthy/falsy evaluation)
- [x] T022 [L3] Add "Type Conversion" section with Code Example 3 (12 lines: str(), int(), float(), bool() conversion safely)
- [x] T023 [L3] Add "The None Type" subsection (None as "no value", None in boolean context)
- [x] T024 [L3] Add "Try With AI" section with 4 prompts (truthy/falsy, conversion failures, None usage, safe conversion)
- [x] T025 [L3] Test all code examples in Python 3.14+ environment
- [x] T026 [L3] Validate lesson follows output style and ends with "Try With AI" only

**Checkpoint**: Lesson 3 complete - students understand strings, booleans, None, and type conversion

---

## Phase 5: Lesson 4 - Introduction to Collections: Overview Only (User Story 4: P3)

**Goal**: Students have awareness of list, tuple, dict, set (conceptual only - deep dive in Ch 18-19)

**Independent Test**: Student creates basic list, tuple, dict, set with type hints and knows where comprehensive coverage is

**Lesson Mapping**:
- User Story 4: Collection Awareness (P3)
- Duration: 40 minutes
- Skills: Collection Awareness (A1-A2)

### Implementation for Lesson 4

- [x] T027 [P] [L4] Write lesson introduction in `book-source/docs/04-Part-4-Python-Fundamentals/14-data-types/04-collections-awareness.md` (why collections exist, defer notice to Ch 18-19)
- [x] T028 [L4] Add "Why Collections Exist" section (grouping related data - conceptual introduction)
- [x] T029 [L4] Add "Collection Types Overview" section with Code Example 1 (12 lines: basic list, tuple, dict, set creation)
- [x] T030 [L4] Add "Type Hints for Collections" section with Code Example 2 (8 lines: list[int], dict[str, int] syntax)
- [x] T031 [L4] Add "When to Use Each Collection" section with Code Example 3 (10 lines: conceptual examples - no methods)
- [x] T032 [L4] Add forward reference notice to Chapters 18-19 for comprehensive coverage
- [x] T033 [L4] Add "Try With AI" section with 4 prompts (collection differences, type hints syntax, where to learn methods, choosing collection type)
- [x] T034 [L4] Test all code examples in Python 3.14+ environment
- [x] T035 [L4] Validate lesson follows output style and ends with "Try With AI" only

**Checkpoint**: Lesson 4 complete - students have collection awareness (not mastery)

---

## Phase 6: Lesson 5 - Building a Type Explorer: Hands-On Practice (User Story 5: P1)

**Goal**: Students build interactive type explorer applying core type concepts (int, float, str, bool, None)

**Independent Test**: Student completes working type explorer with type hints, isinstance() validation, type conversion, AI-guided extensions

**Lesson Mapping**:
- User Story 5: Building Type Explorer with Core Types (P1 - Capstone)
- Duration: 50 minutes
- Skills: Integration (B1), Type Validation (B1), AI-Guided Improvement (B1)

### Implementation for Lesson 5

- [x] T036 [P] [L5] Write lesson introduction in `book-source/docs/04-Part-4-Python-Fundamentals/14-data-types/05-type-explorer-capstone.md` (capstone project integrating all concepts)
- [x] T037 [L5] Add "Project Overview" section (what type explorer does, core types only - not collections)
- [x] T038 [L5] Add "Building the Type Explorer" section with complete code example (~70 lines: type explorer program with type hints, isinstance(), type(), user interaction)
- [x] T039 [L5] Add "Code Walkthrough" subsection (explain key parts: type hints, isinstance() validation, type() inspection, conversion handling)
- [x] T040 [L5] Add "Running Your Type Explorer" section (how to execute, expected output examples)
- [x] T041 [L5] Add "Try With AI" section with 4 prompts (code review, improvement ideas, error handling, reflection on learning)
- [x] T042 [L5] Test type explorer code in Python 3.14+ environment
- [x] T043 [L5] Validate lesson follows output style and ends with "Try With AI" only

**Checkpoint**: Lesson 5 complete - students have working type explorer demonstrating all core concepts

---

## Phase 7: Chapter Integration & Quality Assurance

**Purpose**: Ensure chapter cohesion, technical accuracy, and constitution alignment

- [x] T044 [P] Update chapter `readme.md` with final lesson descriptions and learning objectives
- [x] T045 Validate all Python code examples work in Python 3.14+ (run all code blocks)
- [x] T046 [P] Run type checking on all code examples with mypy/pyright (type hints validation)
- [x] T047 Check chapter progression: L1 (variables) → L2 (types) → L3 (str/bool) → L4 (collections awareness) → L5 (capstone)
- [x] T048 Verify no forward references to Chapters 15-29 (operators, control flow, functions, classes)
- [x] T049 [P] Validate all lessons end with "Try With AI" section only (no "Key Takeaways" or "What's Next")
- [x] T050 [P] Check reading level (Grade 7-8 target) with automated readability check
- [x] T051 Validate AI-Native Learning pattern applied throughout (describe intent → explore with AI → validate → learn from errors)
- [x] T052 Cross-reference with spec.md success criteria (all 5 user stories addressed, all acceptance scenarios covered)

**Checkpoint**: Chapter 14 ready for technical validation

---

## Phase 8: Polish & Documentation

**Purpose**: Final polish and metadata

- [x] T053 [P] Add chapter frontmatter (YAML) to all lesson files (title, description, authors, date, status, part, sidebar_position)
- [x] T054 [P] Verify Docusaurus build succeeds with new chapter
- [x] T055 Update Part 4 index to include Chapter 14
- [x] T056 [P] Add navigation links (next/previous chapter references)
- [x] T057 Run content-evaluation-framework skill for quality assessment (optional)
- [x] T058 Document any pedagogical decisions in chapter notes

**Checkpoint**: Chapter 14 complete and publication-ready

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Lesson 1 (Phase 2)**: Depends on Setup completion
- **Lesson 2 (Phase 3)**: Can start in parallel with Lesson 1 (different files)
- **Lesson 3 (Phase 4)**: Can start in parallel with Lessons 1-2 (different files)
- **Lesson 4 (Phase 5)**: Can start in parallel with Lessons 1-3 (different files)
- **Lesson 5 (Phase 6)**: Depends on Lessons 1-4 conceptual completion (integrates all prior content)
- **Integration (Phase 7)**: Depends on all lessons complete
- **Polish (Phase 8)**: Depends on Integration complete

### Lesson Dependencies

- **Lesson 1 (Variables)**: No dependencies - foundation lesson
- **Lesson 2 (Types Concept)**: Conceptually builds on L1 but can be written in parallel
- **Lesson 3 (Str/Bool)**: Conceptually builds on L2 but can be written in parallel
- **Lesson 4 (Collections)**: Conceptually builds on L1-3 but can be written in parallel
- **Lesson 5 (Capstone)**: Must integrate L1-4 content - write last

### User Story Mapping

- **User Story 1 (Variables with Type Hints - P1)**: Lesson 1
- **User Story 2 (Data Types through Numbers - P1)**: Lesson 2
- **User Story 3 (Strings and Booleans - P2)**: Lesson 3
- **User Story 4 (Collection Awareness - P3)**: Lesson 4
- **User Story 5 (Type Explorer Capstone - P1)**: Lesson 5

### Within Each Lesson

1. Introduction and context
2. Concept sections with code examples
3. "Try With AI" section (MUST be final section)
4. Test code examples in Python 3.14+
5. Validate output style compliance

### Parallel Opportunities

- **Phase 1 (Setup)**: All tasks marked [P] can run in parallel
- **Phase 2-5 (Lessons 1-4)**: Can be written in parallel by different authors (different files)
- **Phase 6 (Lesson 5)**: Sequential - must wait for L1-4 conceptual completion
- **Phase 7 (Integration)**: Tasks marked [P] can run in parallel
- **Phase 8 (Polish)**: Tasks marked [P] can run in parallel

---

## Parallel Example: Lessons 1-4

```bash
# Launch all lessons 1-4 together (different authors/sessions):
Task: "Write Lesson 1: Variables and Type Hints in 01-variables-and-type-hints.md"
Task: "Write Lesson 2: Integers and Floats in 02-integers-and-floats.md"
Task: "Write Lesson 3: Strings and Booleans in 03-strings-and-booleans.md"
Task: "Write Lesson 4: Collections Awareness in 04-collections-awareness.md"

# Then sequentially:
Task: "Write Lesson 5: Type Explorer Capstone in 05-type-explorer-capstone.md" (integrates all prior lessons)
```

---

## Implementation Strategy

### MVP First (Lessons 1-2 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Lesson 1 (Variables)
3. Complete Phase 3: Lesson 2 (Types Concept)
4. **STOP and VALIDATE**: Test code examples, review content quality
5. Get feedback before proceeding

### Incremental Delivery

1. Complete Setup → Chapter structure ready
2. Add Lesson 1 → Test independently → Review
3. Add Lesson 2 → Test independently → Review
4. Add Lesson 3 → Test independently → Review
5. Add Lesson 4 → Test independently → Review
6. Add Lesson 5 (Capstone) → Test integration → Review
7. Run Integration & Polish phases → Chapter complete

### Parallel Team Strategy

With multiple lesson writers:

1. Team completes Setup together
2. Once structure is ready:
   - Writer A: Lesson 1 (Variables)
   - Writer B: Lesson 2 (Integers/Floats)
   - Writer C: Lesson 3 (Strings/Booleans)
   - Writer D: Lesson 4 (Collections)
3. After Lessons 1-4 complete:
   - Lead writer: Lesson 5 (Capstone - integrates all)
4. Team reviews together in Integration phase

---

## Critical Lesson Structure Requirements

**IMPORTANT**: All lessons MUST follow this structure:

### Required Lesson Elements

1. **Frontmatter** (YAML metadata)
2. **Introduction** (connection to prior chapter/lesson)
3. **Concept Sections** (with code examples)
4. **"Try With AI" Section** (MUST be final section)

### Forbidden Lesson Elements

- ❌ **"Key Takeaways"** section
- ❌ **"What's Next"** section
- ❌ **"Summary"** section (unless part of introduction)
- ❌ **"Exercises"** section (use "Try With AI" instead)

### Lesson Closure Policy

**Per constitution and output styles**: Lessons in this chapter MUST end with a single final section titled **"Try With AI"**. No other closure elements permitted.

**AI Tool Selection**:
- Before AI tools taught (Part 1): Use ChatGPT web
- After tool onboarding (Part 4+): Learner's preferred AI companion (Claude Code, Gemini CLI, ChatGPT)
- Optionally provide CLI and web variants in prompts

---

## Quality Gates

### Lesson-Level Quality Gates

Before marking any lesson complete, verify:

- [ ] All code examples tested in Python 3.14+
- [ ] Type hints validated with mypy/pyright
- [ ] Lesson ends with "Try With AI" only (no other closure)
- [ ] Reading level Grade 7-8 maintained
- [ ] No forward references to Chapters 15-29
- [ ] AI-Native Learning pattern applied (describe intent → explore → validate → learn from errors)
- [ ] Follows output style (`.claude/output-styles/lesson.md`)

### Chapter-Level Quality Gates

Before marking chapter complete, verify:

- [ ] All 5 user stories from spec.md addressed
- [ ] All acceptance scenarios covered
- [ ] Success evals achievable (comprehension, skill acquisition, type exploration, error recovery, capstone)
- [ ] Chapter progression logical (variables → types → str/bool → collections → capstone)
- [ ] No cognitive overload (max 7 concepts per lesson for A2-B1 tier)
- [ ] Chapter boundaries respected (defer operators to Ch 15, strings to Ch 16, collections to Ch 18-19)

---

## Notes

- [P] tasks = different files, no dependencies - can run in parallel
- [L#] label maps task to specific lesson for traceability
- User stories map to lessons (not traditional development stories)
- Code examples are educational, not production code
- All Python code must use type hints (Python 3.14+ modern syntax)
- Tests are NOT needed (educational content, not software feature)
- Commit after each lesson completion
- Stop at any checkpoint to validate lesson quality
- Avoid: vague tasks, inconsistent terminology, cognitive overload (max 7 concepts/lesson)

---

## Task Summary

**Total Tasks**: 58
- **Phase 1 (Setup)**: 3 tasks
- **Phase 2 (Lesson 1)**: 7 tasks
- **Phase 3 (Lesson 2)**: 8 tasks
- **Phase 4 (Lesson 3)**: 8 tasks
- **Phase 5 (Lesson 4)**: 9 tasks
- **Phase 6 (Lesson 5)**: 8 tasks
- **Phase 7 (Integration)**: 9 tasks
- **Phase 8 (Polish)**: 6 tasks

**Parallel Opportunities**:
- Phase 1: 1 task (T003)
- Lessons 1-4: Can write in parallel (4 parallel tracks)
- Phase 7: 5 tasks (T044, T046, T049, T050, T051)
- Phase 8: 4 tasks (T053, T054, T056, T057)

**MVP Scope**: Lessons 1-2 (Variables + Type Concept) = 15 tasks

**Independent Tests per Lesson**:
- L1: Student creates variables with type hints and uses print()
- L2: Student explains type concept, uses type() and isinstance()
- L3: Student creates strings/bools, converts types safely
- L4: Student creates basic collections with type hints
- L5: Student builds working type explorer with all core concepts
