# Tasks: Chapter 18 - Lists, Tuples, and Dictionary

**Input**: Design documents from `/specs/001-part-4-chapter-18/`
**Prerequisites**: plan.md (11 lessons), spec.md (4 user stories with priorities)

**Tests**: Not requested in specification - focus on content creation and validation

**Organization**: Tasks organized by user story to enable independent lesson implementation

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different lesson files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- Chapter content: `book-source/docs/04-Part-4-Python-Fundamentals/18-lists-tuples-dictionary/`
- Lesson files: `01-lesson-1.md`, `02-lesson-2.md`, etc.
- Supporting files: `README.md`, `_category_.json`

---

## Phase 1: Setup (Chapter Infrastructure)

**Purpose**: Initialize chapter structure and metadata

- [ ] T001 Create chapter directory at `book-source/docs/04-Part-4-Python-Fundamentals/18-lists-tuples-dictionary/`
- [ ] T002 [P] Create `_category_.json` with chapter metadata (title, position 18, collapsed: false)
- [ ] T003 [P] Create `README.md` with chapter overview, 11 lessons list, prerequisites (Ch 1-17)

**Checkpoint**: Chapter structure ready for lesson content

---

## Phase 2: Foundational (Prerequisites for All User Stories)

**Purpose**: No blocking foundational tasks - lessons can be written independently

**⚠️ NOTE**: This chapter has no shared dependencies. Lessons reference each other but can be written in parallel after setup.

**Checkpoint**: Ready to begin lesson implementation

---

## Phase 3: User Story 1 - Understanding Collection Fundamentals (Priority: P1) 🎯 MVP

**Goal**: Students understand when to use lists vs tuples vs dictionaries and can make informed data structure choices

**Independent Test**: Student can explain three structures' differences (mutability, use cases, performance) and choose correct structure for 3 scenarios with 80%+ accuracy

**Maps to Lessons**: Lesson 1 (Introduction to Collections), Lesson 10 (Choosing the Right Structure)

### Implementation for User Story 1

- [ ] T004 [P] [US1] Write Lesson 1: Introduction to Collections in `01-lesson-1.md`
  - **Content**: 5 concepts (collections, sequences vs mappings, mutability, type hints, use-case matching)
  - **CEFR**: A1 (Foundation)
  - **Elements**: 💬 AI Colearning (Why immutable tuple?), 🎓 Commentary (intent via type hints), 🚀 Challenge (compare structures), Try With AI (4 prompts: Remember → Analyze)
  - **Code**: EX-001 (lists vs tuples vs dicts side-by-side with type hints)
  - **Validation**: Max 5 concepts (A1 limit), no forward references, "Try With AI" only closure

- [ ] T005 [US1] Write Lesson 10: Choosing the Right Structure in `10-lesson-10.md`
  - **Content**: 7 concepts (decision matrix, performance awareness, memory patterns, mutability needs, lookup patterns, architectural thinking, anti-patterns)
  - **CEFR**: B1 (Intermediate synthesis)
  - **Prerequisites**: Lessons 1-9 complete (integrates all prior knowledge)
  - **Elements**: 💬 AI Colearning (When to use each?), 🎓 Commentary (architectural thinking), 🚀 Challenge (design data structures), ✨ Tip (ask AI for tradeoffs), Try With AI (4 prompts: Understand → Evaluate)
  - **Code**: Decision flowchart, performance comparison table, real-world scenario mappings
  - **Validation**: Max 7 concepts (B1 limit), synthesizes (doesn't introduce new structures)

**Checkpoint**: User Story 1 complete - students can identify and choose correct data structures

---

## Phase 4: User Story 2 - Manipulating Lists and Dicts with Type Hints (Priority: P2)

**Goal**: Students create and manipulate lists and dictionaries using modern Python 3.14+ syntax with type hints

**Independent Test**: Student writes type-hinted code creating `list[str]`, adding/removing items, creating `dict[str, int]`, accessing values with `get()` - code runs without errors

**Maps to Lessons**: Lessons 2-4 (Lists), Lessons 7-8 (Dictionaries)

### Implementation for User Story 2

- [ ] T006 [P] [US2] Write Lesson 2: Lists Part 1 - Creation and Basic Operations in `02-lesson-2.md`
  - **Content**: 6 concepts (literals, type hints list[T], positive/negative indexing, slicing, len(), homogeneous vs heterogeneous)
  - **CEFR**: A2 (Basic)
  - **Elements**: 💬 AI Colearning (Why 0-based indexing?), 🎓 Commentary (don't memorize, understand), 🚀 Challenge (5 slice notations), Try With AI (4 prompts)
  - **Code**: EX-002 (list creation/indexing), EX-003 (slicing patterns), EX-004 (aliasing vs copying)
  - **Validation**: Max 6 concepts (within A2 limit of 7)

- [ ] T007 [P] [US2] Write Lesson 3: Lists Part 2 - Mutability and Modification in `03-lesson-3.md`
  - **Content**: 7 concepts (append, extend, insert, remove, pop, clear, method vs function)
  - **CEFR**: A2-B1 (Transitional)
  - **Elements**: 💬 AI Colearning (append vs extend difference), 🎓 Commentary (understand intent, AI handles syntax), 🚀 Challenge (shopping cart simulation), Try With AI (4 prompts)
  - **Code**: EX-005 (mutation methods with shopping_cart example), EX-006 (common errors)
  - **Validation**: Max 7 concepts (AT A2-B1 limit)

- [ ] T008 [P] [US2] Write Lesson 4: Lists Part 3 - Sorting, Reversing, Advanced Methods in `04-lesson-4.md`
  - **Content**: 7 concepts (sort(), sorted(), reverse(), [::-1], count(), index(), copy())
  - **CEFR**: B1 (Intermediate)
  - **Elements**: 💬 AI Colearning (sort vs sorted), 🎓 Commentary (in-place returns None pattern), 🚀 Challenge (sort/reverse/count sequence), Try With AI (4 prompts)
  - **Code**: EX-007 (sort vs sorted), EX-008 (count/index/reverse), EX-009 (aliasing and .copy())
  - **Validation**: Max 7 concepts (conservative within B1 limit of 10)

- [ ] T009 [P] [US2] Write Lesson 7: Dictionaries Part 1 - Key-Value Mappings in `07-lesson-7.md`
  - **Content**: 6 concepts (dict literals, type hints dict[K,V], bracket notation, KeyError, .get() with defaults, unique keys)
  - **CEFR**: A2 (Basic)
  - **Elements**: 💬 AI Colearning (why .get() over brackets?), 🎓 Commentary (dicts for meaningful lookup), 🚀 Challenge (student record system), Try With AI (4 prompts)
  - **Code**: EX-010 (dict creation/access with union types str | int)
  - **Validation**: Max 6 concepts (within A2 limit of 7)

- [ ] T010 [P] [US2] Write Lesson 8: Dictionaries Part 2 - CRUD Operations in `08-lesson-8.md`
  - **Content**: 7 concepts (adding keys, updating values, deleting keys with del/pop, .clear(), .keys()/.values()/.items() intro, key existence checking, pop semantics)
  - **CEFR**: A2-B1 (Transitional)
  - **Elements**: 💬 AI Colearning (pop vs del difference), 🎓 Commentary (CRUD operations), 🚀 Challenge (inventory management), Try With AI (4 prompts)
  - **Code**: EX-011 (CRUD operations on inventory dict)
  - **Validation**: Max 7 concepts (AT A2-B1 limit)

**Checkpoint**: User Story 2 complete - students can manipulate lists and dicts with type hints

---

## Phase 5: User Story 3 - Writing Comprehensions for Data Transformation (Priority: P3)

**Goal**: Students write list and dictionary comprehensions to transform data concisely and read professional Python code

**Independent Test**: Student converts 5-line for loop to list comprehension and writes dict comprehension to transform key-value pairs - both run correctly

**Maps to Lessons**: Lesson 5 (List Comprehensions), Lesson 9 (Dict Comprehensions & Iteration)

### Implementation for User Story 3

- [ ] T011 [P] [US3] Write Lesson 5: List Comprehensions - Transforming Data in `05-lesson-5.md`
  - **Content**: 6 concepts (comprehension syntax, expression evaluation, iteration variable, if filtering, readability considerations, nested comprehensions brief intro)
  - **CEFR**: B1 (Intermediate)
  - **Prerequisites**: Chapter 17 (for loops) - CRITICAL prerequisite
  - **Elements**: 💬 AI Colearning (rewrite loop as comprehension), 🎓 Commentary (clarity trumps cleverness), 🚀 Challenge (convert 8-line loop), Try With AI (4 prompts)
  - **Code**: EX-006 (loop vs comprehension side-by-side), EX-007 (filtering with if), EX-008 (readability examples)
  - **Validation**: Max 6 concepts (within B1 limit), nested comprehensions explicitly deferred

- [ ] T012 [P] [US3] Write Lesson 9: Dictionaries Part 3 - Iteration and Comprehensions in `09-lesson-9.md`
  - **Content**: 7 concepts (iterating .keys()/.values()/.items(), dict comprehension syntax, transforming dicts, filtering dict comprehensions, nested dicts intro, unpacking in comprehensions, data transformation patterns)
  - **CEFR**: B1 (Intermediate)
  - **Prerequisites**: Lesson 5 (comprehension syntax), Lesson 8 (dict CRUD)
  - **Elements**: 💬 AI Colearning (keys vs values vs items when?), 🎓 Commentary (dict comprehensions for transformation), 🚀 Challenge (temperature conversion), Try With AI (4 prompts)
  - **Code**: EX-012 (iterating three ways), EX-013 (dict comprehension Celsius→Fahrenheit), EX-014 (word frequency counter)
  - **Validation**: Max 7 concepts (conservative within B1 limit)

**Checkpoint**: User Story 3 complete - students can write comprehensions for data transformation

---

## Phase 6: User Story 4 - Building Data Processing Pipeline Capstone (Priority: P1) 🎯 INTEGRATION

**Goal**: Students build complete Data Processing Pipeline application using lists, tuples, and dictionaries together in realistic project

**Independent Test**: Student completes capstone that ingests CSV-like data, processes with lists/dicts/comprehensions, outputs results - all three structures used appropriately

**Maps to Lessons**: Lesson 6 (Tuples - prerequisite), Lesson 11 (Capstone - integration)

### Implementation for User Story 4

- [ ] T013 [P] [US4] Write Lesson 6: Tuples - Immutable Sequences in `06-lesson-6.md`
  - **Content**: 7 concepts (tuple literals, single-element syntax, immutability guarantee, type hints tuple[T,...], indexing/slicing, unpacking, hashable dict keys, tuple methods count/index)
  - **CEFR**: A2-B1 (Transitional)
  - **Elements**: 💬 AI Colearning (why can tuples be dict keys?), 🎓 Commentary (immutability is promise/safety), 🚀 Challenge (game map coordinates), Try With AI (4 prompts)
  - **Code**: EX-009 (tuple creation/immutability), EX-010 (unpacking), EX-011 (tuples as dict keys)
  - **Validation**: Max 7 concepts (AT A2-B1 limit)

- [ ] T014 [US4] Write Lesson 11: Capstone - Data Processing Pipeline in `11-lesson-11.md`
  - **Content**: 0 new concepts (integration only - combines all prior lessons)
  - **CEFR**: B1 (Intermediate integration)
  - **Prerequisites**: Lessons 1-10 complete (synthesizes all three structures)
  - **Elements**: 💬 AI Colearning (design data structure), 🎓 Commentary (professional integration patterns), 🚀 Challenge (extend pipeline), ✨ Tip (debugging with AI), Try With AI (4 prompts: Apply → Create)
  - **Code**: EX-015 (60-80 lines: CSV ingestion → list[dict] → filter with comprehensions → aggregate with dicts → formatted output)
  - **Features**: Ingest (parse CSV-like string), Process (filter/transform), Aggregate (statistics/grouping), Output (human-readable)
  - **Validation**: No new concepts, demonstrates realistic integration, step-by-step guidance

**Checkpoint**: User Story 4 complete - students have built complete application demonstrating mastery

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Chapter-wide quality improvements

- [ ] T015 [P] Validate all lessons follow "Try With AI" only closure (no summaries/checklists after)
- [ ] T016 [P] Validate CoLearning elements (💬🎓🚀✨) appear throughout lessons (not just at end)
- [ ] T017 [P] Validate max 7 concepts per lesson (cognitive load compliance)
- [ ] T018 [P] Validate Python 3.14+ type hints: `list[T]`, `dict[K,V]`, `tuple[T,...]`, `X | None`
- [ ] T019 [P] Validate no forward references to Ch 20+ (functions, exceptions, file I/O, OOP)
- [ ] T020 [P] Validate reading level Grade 7-8 using automated tools (Flesch-Kincaid)
- [ ] T021 [P] Test all code examples on Python 3.14+ (ensure they run)
- [ ] T022 [P] Validate skills proficiency metadata in plan.md (40 unique skills, 5 coherence tests)
- [ ] T023 Run validation-auditor validation on complete chapter
- [ ] T024 Address critical issues from validation-auditor (if any)
- [ ] T025 Update chapter-index.md status: 📋 Planned → ✅ Implemented & Validated

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: None (no blocking dependencies)
- **User Stories (Phase 3-6)**: Depend on Setup completion
  - US1 (Lessons 1, 10): Can start after Setup
  - US2 (Lessons 2-4, 7-8): Can start after Setup (independent of US1)
  - US3 (Lessons 5, 9): Can start after Setup (Lesson 5 requires Ch 17 for loops knowledge)
  - US4 (Lessons 6, 11): Lesson 11 depends on Lessons 1-10 completion
- **Polish (Phase 7)**: Depends on all 11 lessons complete

### User Story Dependencies

- **US1 (P1)**: Independent - can implement Lessons 1, 10 immediately after Setup
- **US2 (P2)**: Independent - can implement Lessons 2-4, 7-8 in parallel with US1
- **US3 (P3)**: Lesson 5 independent, Lesson 9 depends on Lesson 8 (dict CRUD prerequisite)
- **US4 (P1)**: Lesson 6 independent, Lesson 11 depends on Lessons 1-10 (capstone integration)

### Within Each Lesson

- Each lesson can be written independently (different files)
- Lesson 11 (Capstone) MUST be written last (requires understanding all prior lessons)
- All lessons follow same structure: YAML frontmatter → Content → "Try With AI" closure

### Parallel Opportunities

- **Phase 1**: All Setup tasks (T001-T003) can run in parallel
- **Phase 3-6**: All lesson tasks marked [P] can run in parallel EXCEPT:
  - Lesson 9 (T012) after Lesson 8 (T010) - dict CRUD prerequisite
  - Lesson 11 (T014) after Lessons 1-10 - capstone integration
- **Phase 7**: All validation tasks (T015-T022) can run in parallel before technical review

---

## Parallel Example: User Story 2 (Lists & Dicts Manipulation)

```bash
# Launch all User Story 2 lessons together (independent files):
Task T006: "Write Lesson 2: Lists Part 1 - Creation and Basic Operations in 02-lesson-2.md"
Task T007: "Write Lesson 3: Lists Part 2 - Mutability and Modification in 03-lesson-3.md"
Task T008: "Write Lesson 4: Lists Part 3 - Sorting, Reversing, Advanced Methods in 04-lesson-4.md"
Task T009: "Write Lesson 7: Dictionaries Part 1 - Key-Value Mappings in 07-lesson-7.md"
Task T010: "Write Lesson 8: Dictionaries Part 2 - CRUD Operations in 08-lesson-8.md"

# All 5 lessons can be written in parallel by different agents/team members
```

---

## Implementation Strategy

### MVP First (User Stories 1 + 4 Only)

1. Complete Phase 1: Setup (T001-T003)
2. Complete Phase 2: Foundational (none - skip)
3. Complete Phase 3: User Story 1 (T004-T005) - Understanding fundamentals
4. Complete Phase 6: User Story 4 Lesson 6 (T013) - Tuples prerequisite
5. **PAUSE**: At this point, students can understand collections and tuples
6. Complete Phase 4: User Story 2 (T006-T010) - Lists and dicts manipulation
7. Complete Phase 5: User Story 3 (T011-T012) - Comprehensions
8. Complete Phase 6: User Story 4 Lesson 11 (T014) - Capstone integration
9. **STOP and VALIDATE**: Test complete chapter independently
10. Run Phase 7: Polish & validation

### Incremental Delivery (By User Story Priority)

1. Setup → Foundation ready
2. Add US1 (Lessons 1, 10) → Test independently → Foundational understanding validated
3. Add US2 (Lessons 2-4, 7-8) → Test independently → Manipulation skills validated
4. Add US3 (Lessons 5, 9) → Test independently → Transformation skills validated
5. Add US4 (Lessons 6, 11) → Test independently → Integration validated → Chapter complete!

### Parallel Team Strategy

With multiple content-implementer agents or team members:

1. Team completes Setup together (T001-T003)
2. Once Setup done:
   - Agent A: Lessons 1, 10 (US1)
   - Agent B: Lessons 2-4 (US2 Lists)
   - Agent C: Lessons 7-8 (US2 Dicts)
   - Agent D: Lessons 5, 6 (US3 & US4)
   - Agent E: Lesson 9 (US3 - after Agent C completes Lesson 8)
3. All agents complete → Agent F writes Lesson 11 (Capstone - requires all prior lessons)
4. Team runs validation together (Phase 7)

---

## Notes

### General
- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each lesson should be independently writable (different .md file)
- Commit after each lesson completion
- Stop at any checkpoint to validate story independently

### Chapter-Specific
- **Cognitive Load**: Strict 7-concept limit per lesson (intermediate A2-B1 tier)
- **Proficiency Progression**: A1 → A2 → A2-B1 → B1 across 11 lessons (non-regressing)
- **Lesson Closure**: ONLY "Try With AI" section (4 prompts, Bloom's progression) - NO summaries, NO "What's Next", NO checklists
- **CoLearning Elements**: Must appear THROUGHOUT lessons (💬 AI Colearning, 🎓 Instructor Commentary, 🚀 CoLearning Challenge, ✨ Teaching Tip)
- **Type Hints**: Modern Python 3.14+ syntax (`list[T]`, `dict[K,V]`, `tuple[T,...]`, `X | None`) - NO legacy `typing.List`
- **AI Partnership**: Position AI as co-reasoning partner (not autocomplete) - students describe intent, AI explores, students validate
- **Constitutional Alignment**: ADR-0008 (11-lesson structure), ADR-0009 (CEFR proficiency metadata), Principle 12 (cognitive load), Principle 13 (graduated teaching)

### Avoid
- Vague tasks ("write good content")
- Same file conflicts (all lessons are separate files)
- Forward references to Ch 20+ (functions, exceptions, file I/O, OOP)
- Cross-story dependencies that break lesson independence
- Adding summaries or checklists after "Try With AI" section (violates closure rules)
