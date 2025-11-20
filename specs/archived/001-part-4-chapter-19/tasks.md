---
description: "Task list for Chapter 19: Set, Frozen Set, and GC implementation"
---

# Tasks: Chapter 19 - Set, Frozen Set, and GC

**Input**: Design documents from `/specs/001-part-4-chapter-19/`
**Prerequisites**: plan.md (complete), spec.md (complete), ADR-0008 (6-lesson pattern)

**Tests**: No automated tests for educational content. Validation via validation-auditor and factual-verifier agents.

**Organization**: Tasks are grouped by learning journey (user story equivalent) to enable independent lesson implementation and validation.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which learning journey this task belongs to (e.g., LJ1, LJ2, LJ3)
- Include exact file paths in descriptions

## Path Conventions

- **Chapter directory**: `book-source/docs/04-Part-4-Python-Fundamentals/19-set-frozenset-gc/`
- **Lesson files**: `01-set-basics.md`, `02-set-operations.md`, etc.
- **Specification files**: `specs/001-part-4-chapter-19/`

---

## Phase 1: Setup (Chapter Infrastructure)

**Purpose**: Create chapter directory structure and initial files

- [ ] T001 Create chapter directory `book-source/docs/04-Part-4-Python-Fundamentals/19-set-frozenset-gc/`
- [ ] T002 Create chapter intro file `book-source/docs/04-Part-4-Python-Fundamentals/19-set-frozenset-gc/intro.md` with YAML frontmatter, overview, prerequisites, learning objectives
- [ ] T003 Create chapter README `book-source/docs/04-Part-4-Python-Fundamentals/19-set-frozenset-gc/README.md` linking to lessons and capstone

---

## Phase 2: Foundational (Chapter-Level Requirements)

**Purpose**: Complete chapter-level content that all lessons reference

**⚠️ CRITICAL**: No lesson writing can begin until this phase is complete

- [ ] T004 Document chapter-specific CoLearning pedagogy approach in chapter intro (💬🎓🚀✨ elements)
- [ ] T005 Define chapter-wide type hint standards (Python 3.14+, `set[T]`, `frozenset[T]`)
- [ ] T006 Create placeholder lesson files (01-set-basics.md through 06-memory-profiler-capstone.md) with YAML frontmatter

**Checkpoint**: Foundation ready - lesson implementation can now begin

---

## Phase 3: Learning Journey 1 - Set Fundamentals Mastery (Priority: P1) 🎯 MVP

**Goal**: Lesson 1 teaches set creation, uniqueness, hashability, and basic operations (A2 level, 7 concepts, 50 min)

**Independent Test**: Student can create sets with type hints, explain uniqueness + hashability, use add/remove/discard methods

### Implementation for Learning Journey 1 (Lesson 1)

- [ ] T007 [LJ1] Write Lesson 1 introduction (5 min) in `book-source/docs/04-Part-4-Python-Fundamentals/19-set-frozenset-gc/01-set-basics.md`
- [ ] T008 [LJ1] Write "What Makes Sets Different" section (uniqueness, unordered, hash-based storage) in `01-set-basics.md`
- [ ] T009 [LJ1] Write "Creating Sets" section (literal syntax, constructor, type hints, empty set pitfall) in `01-set-basics.md`
- [ ] T010 [LJ1] Write "The Hashability Requirement" section (immutable elements needed) in `01-set-basics.md`
- [ ] T011 [LJ1] Add Code Example 1: "Creating Sets with Type Hints" (`set[int]`, `set[str]`, `set()` constructor) in `01-set-basics.md`
- [ ] T012 [LJ1] Add Code Example 2: "Understanding Uniqueness" (email deduplication, automatic elimination) in `01-set-basics.md`
- [ ] T013 [LJ1] Add Code Example 3: "Modifying Sets" (`.add()`, `.remove()`, `.discard()` with error handling) in `01-set-basics.md`
- [ ] T014 [LJ1] Add Code Example 4: "The Hashability Requirement" (hashable vs unhashable types, try/except demos) in `01-set-basics.md`
- [ ] T015 [LJ1] Add 4 practice exercises (create sets, deduplication, error handling, tuples in sets) in `01-set-basics.md`
- [ ] T016 [LJ1] Write "Try With AI" section with 4 prompts (concept exploration, syntax clarification, hashability deep dive, error handling connection) in `01-set-basics.md`
- [ ] T017 [LJ1] Add 💬 AI Colearning Prompt, 🎓 Instructor Commentary, 🚀 CoLearning Challenge, ✨ Teaching Tip elements throughout `01-set-basics.md`
- [ ] T018 [LJ1] Validate YAML frontmatter (title, sidebar_position: 1, complexity: A2, concepts: 7, skills metadata with CEFR/Bloom's/DigComp levels) in `01-set-basics.md`
- [ ] T019 [LJ1] Validate cognitive load (max 7 concepts for A2 level) in `01-set-basics.md`
- [ ] T020 [LJ1] Validate reading level (Grade 7-8 target) using Flesch-Kincaid Grade Level calculator for `01-set-basics.md`

**Checkpoint**: Lesson 1 complete and independently testable. Student can create and modify sets.

---

## Phase 4: Learning Journey 2 - Mathematical Set Operations (Priority: P1)

**Goal**: Lesson 2 teaches union, intersection, difference, symmetric difference, and set comprehensions (A2-B1 level, 7 concepts, 50 min)

**Independent Test**: Student can perform all 4 set operations and write set comprehensions for practical problems

### Implementation for Learning Journey 2 (Lesson 2)

- [ ] T021 [LJ2] Write Lesson 2 introduction (set operations context) in `book-source/docs/04-Part-4-Python-Fundamentals/19-set-frozenset-gc/02-set-operations.md`
- [ ] T022 [LJ2] Write "Union — Combining Sets" section (`|` operator and `.union()`) in `02-set-operations.md`
- [ ] T023 [LJ2] Write "Intersection — Finding Common Elements" section (`&` operator and `.intersection()`) in `02-set-operations.md`
- [ ] T024 [LJ2] Write "Difference — Excluding Elements" section (`-` operator and `.difference()`) in `02-set-operations.md`
- [ ] T025 [LJ2] Write "Symmetric Difference" section (`^` operator for "either but not both") in `02-set-operations.md`
- [ ] T026 [LJ2] Add Code Example 1: "Union Operations" (team member combining, multiple sets) in `02-set-operations.md`
- [ ] T027 [LJ2] Add Code Example 2: "Intersection Operations" (bilingual developers, commutativity) in `02-set-operations.md`
- [ ] T028 [LJ2] Add Code Example 3: "Difference Operations" (student enrollment, non-commutativity) in `02-set-operations.md`
- [ ] T029 [LJ2] Add Code Example 4: "Symmetric Difference" (shift schedules, XOR relationship) in `02-set-operations.md`
- [ ] T030 [LJ2] Add Code Example 5: "Set Comprehensions with Filtering" (even numbers, squares of odd, long words) in `02-set-operations.md`
- [ ] T031 [LJ2] Add Code Example 6: "Real-World Problem — Finding Common Interests" (user matching scenario) in `02-set-operations.md`
- [ ] T032 [LJ2] Add 4 practice exercises (intersection, union + difference chaining, set comprehension, inventory comparison) in `02-set-operations.md`
- [ ] T033 [LJ2] Write "Try With AI" section with 4 prompts (Venn diagram, operator vs method, symmetric difference, comprehension challenge) in `02-set-operations.md`
- [ ] T034 [LJ2] Add CoLearning elements (💬🎓🚀✨) throughout `02-set-operations.md`
- [ ] T035 [LJ2] Validate YAML frontmatter (title, sidebar_position: 2, complexity: A2-B1, concepts: 7, skills metadata with CEFR/Bloom's/DigComp levels) in `02-set-operations.md`
- [ ] T036 [LJ2] Validate cognitive load (max 7 concepts for A2-B1 level) in `02-set-operations.md`
- [ ] T037 [LJ2] Validate reading level (Grade 7-8 target) using Flesch-Kincaid Grade Level calculator for `02-set-operations.md`

**Checkpoint**: Lesson 2 complete. Student can use all 4 set operations and write comprehensions.

---

## Phase 5: Learning Journey 3 - Set Internals and Performance (Priority: P2)

**Goal**: Lesson 3 teaches hash functions, O(1) lookup, hash tables, and performance analysis (B1 level, 9 concepts, 50 min)

**Independent Test**: Student explains why set lookups are O(1) and when sets outperform lists

### Implementation for Learning Journey 3 (Lesson 3)

- [ ] T038 [LJ3] Write Lesson 3 introduction (why internals matter) in `book-source/docs/04-Part-4-Python-Fundamentals/19-set-frozenset-gc/03-set-internals-hashing.md`
- [ ] T039 [LJ3] Write "What Are Hash Functions?" section (intuitive explanation, `hash()` function) in `03-set-internals-hashing.md`
- [ ] T040 [LJ3] Write "Why Immutability Matters" section (hash stability, what happens if objects change) in `03-set-internals-hashing.md`
- [ ] T041 [LJ3] Write "Performance Comparison" section (O(1) vs O(n) lookup, demonstrate with timing) in `03-set-internals-hashing.md`
- [ ] T042 [LJ3] Write "Hash Tables Conceptually" section (high-level internal storage) in `03-set-internals-hashing.md`
- [ ] T043 [LJ3] Add Code Example 1: "Hash Functions and Immutable Objects" (`hash()` calls, hashable vs unhashable) in `03-set-internals-hashing.md`
- [ ] T044 [LJ3] Add Code Example 2: "Why Immutability is Required" (pseudo-code explanation, dict key connection) in `03-set-internals-hashing.md`
- [ ] T045 [LJ3] Add Code Example 3: "Performance Comparison — O(1) vs. O(n)" (timing code, 1M elements) in `03-set-internals-hashing.md`
- [ ] T046 [LJ3] Add Code Example 4: "Hash Collisions (Conceptual)" (collision handling, rehashing awareness) in `03-set-internals-hashing.md`
- [ ] T047 [LJ3] Add Code Example 5: "Real-World Performance Decision" (user ID lookup scenario) in `03-set-internals-hashing.md`
- [ ] T048 [LJ3] Add 4 practice exercises (call hash(), verify tuple hashability, time comparison, design decision) in `03-set-internals-hashing.md`
- [ ] T049 [LJ3] Write "Try With AI" section with 4 prompts (hash function exploration, immutability deep dive, performance analysis, design decision) in `03-set-internals-hashing.md`
- [ ] T050 [LJ3] Add CoLearning elements (💬🎓🚀✨) throughout `03-set-internals-hashing.md`
- [ ] T051 [LJ3] Validate YAML frontmatter (title, sidebar_position: 3, complexity: B1, concepts: 9, skills metadata with CEFR/Bloom's/DigComp levels) in `03-set-internals-hashing.md`
- [ ] T052 [LJ3] Validate cognitive load (max 10 concepts for B1 level) in `03-set-internals-hashing.md`
- [ ] T053 [LJ3] Validate reading level (Grade 7-8 target) using Flesch-Kincaid Grade Level calculator for `03-set-internals-hashing.md`

**Checkpoint**: Lesson 3 complete. Student understands hashing, O(1) performance, and hash tables conceptually.

---

## Phase 6: Learning Journey 4 - Frozensets for Immutability (Priority: P2)

**Goal**: Lesson 4 teaches frozensets as immutable set variants, dict keys, and set nesting (A2-B1 level, 7 concepts, 40 min)

**Independent Test**: Student creates dictionaries with frozenset keys and nests frozensets in sets

### Implementation for Learning Journey 4 (Lesson 4)

- [ ] T054 [LJ4] Write Lesson 4 introduction (why immutable sets matter) in `book-source/docs/04-Part-4-Python-Fundamentals/19-set-frozenset-gc/04-frozensets.md`
- [ ] T055 [LJ4] Write "Frozenset as Immutable Set" section (creation, properties, operations) in `04-frozensets.md`
- [ ] T056 [LJ4] Write "Frozensets are Hashable" section (can use as dict keys, set members) in `04-frozensets.md`
- [ ] T057 [LJ4] Add Code Example 1: "Creating and Using Frozensets" (creation, immutability, operations) in `04-frozensets.md`
- [ ] T058 [LJ4] Add Code Example 2: "Frozensets as Dictionary Keys" (user groups, permissions scenario) in `04-frozensets.md`
- [ ] T059 [LJ4] Add Code Example 3: "Nesting Frozensets in Sets" (groups of groups, union of members) in `04-frozensets.md`
- [ ] T060 [LJ4] Add Code Example 4: "Set vs. Frozenset Comparison" (decision matrix, trade-offs) in `04-frozensets.md`
- [ ] T061 [LJ4] Add 4 practice exercises (create frozenset, dict keys, nested structure, conversion) in `04-frozensets.md`
- [ ] T062 [LJ4] Write "Try With AI" section with 4 prompts (immutability exploration, dict keys use case, nesting practice, real-world scenario) in `04-frozensets.md`
- [ ] T063 [LJ4] Add CoLearning elements (💬🎓🚀✨) throughout `04-frozensets.md`
- [ ] T064 [LJ4] Validate YAML frontmatter (title, sidebar_position: 4, complexity: A2-B1, concepts: 7, skills metadata with CEFR/Bloom's/DigComp levels) in `04-frozensets.md`
- [ ] T065 [LJ4] Validate cognitive load (max 7 concepts for A2-B1 level) in `04-frozensets.md`
- [ ] T066 [LJ4] Validate reading level (Grade 7-8 target) using Flesch-Kincaid Grade Level calculator for `04-frozensets.md`

**Checkpoint**: Lesson 4 complete. Student can use frozensets as dict keys and nested in sets.

---

## Phase 7: Learning Journey 5 - Garbage Collection Fundamentals (Priority: P2)

**Goal**: Lesson 5 teaches reference counting, memory management, `gc` module, and circular references (B1 level, 9 concepts, 50 min)

**Independent Test**: Student uses `gc.get_objects()` to analyze memory and explains reference counting

### Implementation for Learning Journey 5 (Lesson 5)

- [ ] T067 [LJ5] Write Lesson 5 introduction (why memory management matters) in `book-source/docs/04-Part-4-Python-Fundamentals/19-set-frozenset-gc/05-garbage-collection.md`
- [ ] T068 [LJ5] Write "Reference Counting" section (how Python tracks usage, immediate deletion) in `05-garbage-collection.md`
- [ ] T069 [LJ5] Write "Circular References" section (when refcount fails, cycle detector solution) in `05-garbage-collection.md`
- [ ] T070 [LJ5] Write "The `gc` Module" section (manual collection, memory profiling, analysis) in `05-garbage-collection.md`
- [ ] T071 [LJ5] Add Code Example 1: "Reference Counting Basics" (`sys.getrefcount()`, create/delete references) in `05-garbage-collection.md`
- [ ] T072 [LJ5] Add Code Example 2: "Circular References (Problem and Solution)" (Node class, A→B→A cycle, `gc.collect()`) in `05-garbage-collection.md`
- [ ] T073 [LJ5] Add Code Example 3: "The `gc` Module for Memory Analysis" (`gc.get_objects()`, manual collection, stats) in `05-garbage-collection.md`
- [ ] T074 [LJ5] Add Code Example 4: "Reference Counting in Practice" (function arguments, automatic cleanup) in `05-garbage-collection.md`
- [ ] T075 [LJ5] Add Code Example 5: "Generational Garbage Collection" (thresholds, manual triggers, tuning awareness) in `05-garbage-collection.md`
- [ ] T076 [LJ5] Add 4 practice exercises (track refcount, create circular ref, use gc.get_objects(), analyze memory) in `05-garbage-collection.md`
- [ ] T077 [LJ5] Write "Try With AI" section with 4 prompts (reference counting explanation, circular refs deep dive, memory profiling challenge, performance implication) in `05-garbage-collection.md`
- [ ] T078 [LJ5] Add CoLearning elements (💬🎓🚀✨) throughout `05-garbage-collection.md`
- [ ] T079 [LJ5] Validate YAML frontmatter (title, sidebar_position: 5, complexity: B1, concepts: 9, skills metadata with CEFR/Bloom's/DigComp levels) in `05-garbage-collection.md`
- [ ] T080 [LJ5] Validate cognitive load (max 10 concepts for B1 level) in `05-garbage-collection.md`
- [ ] T081 [LJ5] Validate reading level (Grade 7-8 target) using Flesch-Kincaid Grade Level calculator for `05-garbage-collection.md`

**Checkpoint**: Lesson 5 complete. Student understands reference counting, GC, and memory profiling basics.

---

## Phase 8: Learning Journey 6 - Memory Profiler Capstone Integration (Priority: P3)

**Goal**: Lesson 6 guides students to build a Memory Profiler tool integrating all chapter concepts (B1-B2 level, 9 concepts, 60 min)

**Independent Test**: Student submits working Memory Profiler that tracks object creation/deletion and displays statistics

### Implementation for Learning Journey 6 (Lesson 6 - Capstone)

- [ ] T082 [LJ6] Write Lesson 6 introduction (capstone goal, integration overview) in `book-source/docs/04-Part-4-Python-Fundamentals/19-set-frozenset-gc/06-memory-profiler-capstone.md`
- [ ] T083 [LJ6] Write "Tool Specification" section (requirements, input/output, constraints) in `06-memory-profiler-capstone.md`
- [ ] T084 [LJ6] Write "Implementation Guide" section (design approach, class structure) in `06-memory-profiler-capstone.md`
- [ ] T085 [LJ6] Add Code Example 1: "Memory Profiler Implementation" (MemoryProfiler class with sets for tracking) in `06-memory-profiler-capstone.md`
- [ ] T086 [LJ6] Add Code Example 2: "Advanced Tracking with Frozensets" (AdvancedMemoryProfiler with categorization) in `06-memory-profiler-capstone.md`
- [ ] T087 [LJ6] Add Code Example 3: "Testing with Edge Cases" (circular references, large graphs) in `06-memory-profiler-capstone.md`
- [ ] T088 [LJ6] Write "Practice / Implementation Approach" section (4 phases: design, implement, test, reflect) in `06-memory-profiler-capstone.md`
- [ ] T089 [LJ6] Write "Try With AI" section with 4 prompts (specification refinement, implementation partner, testing strategy, integration reflection) in `06-memory-profiler-capstone.md`
- [ ] T090 [LJ6] Add CoLearning elements (💬🎓🚀✨) throughout `06-memory-profiler-capstone.md`
- [ ] T091 [LJ6] Validate YAML frontmatter (title, sidebar_position: 6, complexity: B1-B2, concepts: 9, skills metadata with CEFR/Bloom's/DigComp levels) in `06-memory-profiler-capstone.md`
- [ ] T092 [LJ6] Validate cognitive load (max 10 concepts for B1-B2 level) in `06-memory-profiler-capstone.md`
- [ ] T093 [LJ6] Validate reading level (Grade 7-8 target) using Flesch-Kincaid Grade Level calculator for `06-memory-profiler-capstone.md`

**Checkpoint**: All learning journeys complete. Student has working Memory Profiler demonstrating mastery.

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Chapter-level improvements and final validation

- [ ] T094 [P] Update chapter intro.md with lesson summaries and navigation
- [ ] T095 [P] Update chapter README.md with capstone showcase
- [ ] T096 [P] Add chapter-level "What's Next" section pointing to Chapter 20 (Modules and Functions)
- [ ] T097 [P] Cross-reference validation: Ensure Lessons 1-6 reference each other appropriately
- [ ] T098 [P] Constitutional compliance check: Validate all 17 principles applied (AI-Native Learning, Graduated Teaching Pattern, CoLearning)
- [ ] T099 Run validation-auditor validation on all 6 lessons
- [ ] T100 Run factual-verifier on complete chapter for publication readiness
- [ ] T101 Update chapter-index.md to mark Chapter 19 as "Implemented & Validated"

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all learning journeys
- **Learning Journeys (Phase 3-8)**: All depend on Foundational phase completion
  - LJ1 and LJ2 (both P1) can proceed in parallel after Foundation
  - LJ3, LJ4, LJ5 (all P2) can proceed in parallel after LJ1/LJ2
  - LJ6 (P3) depends on LJ1-LJ5 completion (capstone integrates all concepts)
- **Polish (Phase 9)**: Depends on all learning journeys being complete

### Learning Journey Dependencies

- **LJ1 (P1) - Set Fundamentals**: Can start after Foundational - No dependencies on other journeys
- **LJ2 (P1) - Set Operations**: Can start after Foundational - Builds on LJ1 but can be drafted in parallel
- **LJ3 (P2) - Set Internals**: Depends on LJ1 (must understand sets before internals)
- **LJ4 (P2) - Frozensets**: Depends on LJ1 (must understand sets before frozensets)
- **LJ5 (P2) - Garbage Collection**: Depends on LJ1 (sets used in examples)
- **LJ6 (P3) - Capstone**: Depends on ALL prior journeys (integrates all concepts)

### Within Each Learning Journey

- Introduction before concept sections
- Concept sections before code examples
- Code examples before practice exercises
- Practice exercises before "Try With AI" section
- CoLearning elements interspersed throughout
- Validation tasks at end of each lesson

### Parallel Opportunities

- All Setup tasks (T001-T003) can run in parallel
- All Foundational tasks (T004-T006) can run in parallel within Phase 2
- LJ1 and LJ2 (Lessons 1-2) can be drafted in parallel after Foundation
- LJ3, LJ4, LJ5 (Lessons 3-5) can be drafted in parallel after LJ1/LJ2 complete
- Within a lesson: Code examples can be written in parallel before integration
- Polish tasks marked [P] can run in parallel

---

## Parallel Example: Learning Journey 1 (Lesson 1)

```bash
# After writing introduction and concept sections, launch code examples in parallel:
Task: "Add Code Example 1: Creating Sets with Type Hints in 01-set-basics.md"
Task: "Add Code Example 2: Understanding Uniqueness in 01-set-basics.md"
Task: "Add Code Example 3: Modifying Sets in 01-set-basics.md"
Task: "Add Code Example 4: Hashability Requirement in 01-set-basics.md"

# After code examples, complete practice and validation sequentially
```

---

## Implementation Strategy

### MVP First (Learning Journey 1 Only - Lesson 1)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all lessons)
3. Complete Phase 3: Learning Journey 1 (Lesson 1: Set Basics)
4. **STOP and VALIDATE**: Run validation-auditor on Lesson 1
5. If validated, proceed to LJ2; if issues, revise Lesson 1

### Incremental Delivery

1. Complete Setup + Foundational → Chapter structure ready
2. Add LJ1 (Lesson 1) → Validate → Publish (Students can start!)
3. Add LJ2 (Lesson 2) → Validate → Publish (Set operations available)
4. Add LJ3-LJ5 (Lessons 3-5) → Validate → Publish (Intermediate concepts)
5. Add LJ6 (Lesson 6 Capstone) → Validate → Publish (Complete chapter!)
6. Each lesson adds value without breaking previous lessons

### Parallel Team Strategy

With multiple content creators:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Writer A: Learning Journey 1 (Lesson 1)
   - Writer B: Learning Journey 2 (Lesson 2)
3. After LJ1/LJ2 complete:
   - Writer A: Learning Journey 3 (Lesson 3)
   - Writer B: Learning Journey 4 (Lesson 4)
   - Writer C: Learning Journey 5 (Lesson 5)
4. After LJ3-LJ5 complete:
   - Writer A: Learning Journey 6 (Lesson 6 - Capstone)
5. Team validates together

---

## Notes

### Content Policy for Lesson Authors

**CRITICAL - Lesson Closure Standard**:
- Within this chapter, each lesson MUST end with a single final section titled "Try With AI"
- NO "Key Takeaways" sections (violates Constitution Rule 6)
- NO "What's Next" sections (unless chapter-level in intro.md)
- NO separate conclusion sections beyond "Try With AI"

**AI Tool Selection**:
- Before AI tools are taught (Part 1): Use ChatGPT web in "Try With AI" section
- After tool onboarding (Parts 2+): Instruct learners to use their preferred AI companion (Gemini CLI, Claude Code, ChatGPT web)
- Optionally provide CLI and web variants for prompts

### Format Requirements

- [P] tasks = different files, no dependencies, can run in parallel
- [LJ#] label maps task to specific learning journey for traceability
- Each learning journey should produce an independently completable and testable lesson
- Validate cognitive load, reading level, and constitutional compliance for each lesson
- Commit after each lesson or logical group of tasks
- Stop at any checkpoint to validate lesson independently
- Avoid: vague tasks, same file conflicts, cross-journey dependencies that break independence

### Validation Gates

- After each lesson: Technical-reviewer for code quality and pedagogical effectiveness
- After all lessons: Proof-validator for publication readiness
- Chapter-level: Constitutional compliance check (all 17 principles)
- Reading level: Grade 7-8 (Flesch-Kincaid 7.0-8.0)
- Cognitive load: A2 max 7 concepts, B1 max 10 concepts per lesson

---

**Total Tasks**: 101
- Setup: 3 tasks
- Foundational: 3 tasks
- Learning Journey 1 (Lesson 1): 14 tasks
- Learning Journey 2 (Lesson 2): 17 tasks
- Learning Journey 3 (Lesson 3): 16 tasks
- Learning Journey 4 (Lesson 4): 13 tasks
- Learning Journey 5 (Lesson 5): 15 tasks
- Learning Journey 6 (Lesson 6 - Capstone): 12 tasks
- Polish: 8 tasks

**Parallel Opportunities**: 25+ tasks marked [P] can run in parallel within their phases

**MVP Scope**: Complete Phase 1-3 (Learning Journey 1 / Lesson 1) for minimal viable chapter (students can start learning sets)

**Full Chapter Completion**: All phases (1-9) for complete, production-ready Chapter 19 with 6 lessons and capstone project
