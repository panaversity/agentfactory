---
description: "Implementation tasks for Chapter 26: Meta Classes and Data Classes"
---

# Tasks: Chapter 26 - Meta Classes and Data Classes

**Input**: Design documents from `/specs/001-part-4-chapter-26/`
**Prerequisites**: plan.md (required), spec.md (required)

**Organization**: Tasks are organized by lesson to enable sequential implementation with review checkpoints.

**Chapter Context**: Advanced Python chapter teaching metaclasses and dataclasses with equal 50/50 depth. No capstone project—conceptual mastery with focused examples.

## Format: `[ID] [P?] [Lesson] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Lesson]**: Which lesson this task belongs to (L1, L2, L3, L4, L5)
- Include exact file paths in descriptions

## Path Conventions

All lessons are in: `docs/part-4-python-fundamentals/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Chapter initialization and structure setup

- [ ] T001 Create chapter directory structure at docs/part-4-python-fundamentals/chapter-26-metaclasses-dataclasses/
- [ ] T002 [P] Create chapter intro.md file following chapter introduction template
- [ ] T003 [P] Validate prerequisites are met (Chapters 24-25 OOP content exists)
- [ ] T004 [P] Setup Python 3.14+ code validation environment for all examples

**Checkpoint**: Chapter structure ready - lesson implementation can begin

---

## Phase 2: Lesson 1 - Understanding Metaclasses (B1 Proficiency)

**Lesson Goal**: Demystify metaclasses by revealing how Python creates classes using `type`

**Duration**: 45-60 minutes
**Concepts**: 10 (type mechanism, metaclass syntax, when to use)
**Code Examples**: 4
**Try With AI**: 4 prompts (Recall → Understand → Apply → Analyze)

**Independent Test**: Student can explain `type()` as class factory and trace metaclass execution

### Content Development for Lesson 1

- [ ] T005 [L1] Create lesson-1-understanding-metaclasses.mdx with frontmatter (title, description, CEFR: B1)
- [ ] T006 [L1] Write introduction section explaining metaclass concept ("classes that create classes")
- [ ] T007 [L1] Develop Section 1: "What Are Metaclasses?" (concepts 1-3: metaclass definition, type metaclass, type() factory)
- [ ] T008 [L1] Develop Section 2: "How Python Creates Classes" (concepts 4-5: class creation flow, __new__ and __init__)
- [ ] T009 [L1] Develop Section 3: "Custom Metaclass Syntax" (concepts 6-8: metaclass= syntax, MRO, method resolution)
- [ ] T010 [L1] Develop Section 4: "When to Use Metaclasses" (concepts 9-10: appropriate use cases, when NOT to use)

### Code Examples for Lesson 1

- [ ] T011 [P] [L1] Create Example 1: type() as class factory in lesson-1-understanding-metaclasses.mdx
- [ ] T012 [P] [L1] Create Example 2: Basic custom metaclass (logging class creation)
- [ ] T013 [P] [L1] Create Example 3: Attribute validation metaclass (enforce required attributes)
- [ ] T014 [P] [L1] Create Example 4: Metaclass vs decorator comparison

### AI-Native Learning Integration for Lesson 1

- [ ] T015 [L1] Write "Try With AI" section with 4 prompts following Bloom's progression
  - Prompt 1 (Recall): "Explain how Python creates a class when I write `class Foo: pass`"
  - Prompt 2 (Understand): "What's the difference between a metaclass and a regular class?"
  - Prompt 3 (Apply): "Create a metaclass that enforces all methods must have docstrings"
  - Prompt 4 (Analyze): "When would I use a metaclass vs a class decorator? Give examples."

### Validation for Lesson 1

- [ ] T016 [L1] Test all 4 code examples run on Python 3.14+ without errors
- [ ] T017 [L1] Verify type hints present on 100% of code examples
- [ ] T018 [L1] Validate cognitive load: exactly 10 concepts, no more
- [ ] T019 [L1] Check reading level with Flesch-Kincaid (target: Grade 10-11)
- [ ] T020 [L1] Verify lesson ends with "Try With AI" section ONLY (no Key Takeaways or What's Next)

**Checkpoint**: Lesson 1 complete and validated - ready for human review

---

## Phase 3: Lesson 2 - Practical Metaclass Patterns (B1-B2 Proficiency)

**Lesson Goal**: Apply metaclasses to solve real problems (validation, registration, framework design)

**Duration**: 50-65 minutes
**Concepts**: 10 (validation, registration, singleton, framework patterns)
**Code Examples**: 6
**Try With AI**: 4 prompts (Recall → Understand → Apply → Evaluate)

**Independent Test**: Student can implement validation or registration pattern with metaclasses

### Content Development for Lesson 2

- [ ] T021 [L2] Create lesson-2-practical-metaclass-patterns.mdx with frontmatter (CEFR: B1-B2)
- [ ] T022 [L2] Write introduction section connecting to Lesson 1 concepts
- [ ] T023 [L2] Develop Section 1: "Validation Pattern" (concepts 1-2: attribute validation, structure enforcement)
- [ ] T024 [L2] Develop Section 2: "Registration Pattern" (concepts 3-4: auto-registration, plugin systems)
- [ ] T025 [L2] Develop Section 3: "Singleton and Abstract Enforcement" (concepts 5-6: singleton via metaclass, abstract methods)
- [ ] T026 [L2] Develop Section 4: "Framework Patterns" (concepts 7-9: Django ORM, SQLAlchemy, when NOT to use)
- [ ] T027 [L2] Develop Section 5: "Alternatives to Metaclasses" (concept 10: __init_subclass__, decorators)

### Code Examples for Lesson 2

- [ ] T028 [P] [L2] Create Example 1: Validation metaclass (enforce required attributes) in lesson-2-practical-metaclass-patterns.mdx
- [ ] T029 [P] [L2] Create Example 2: Registration metaclass (auto-register plugins)
- [ ] T030 [P] [L2] Create Example 3: Singleton metaclass (single instance enforcement)
- [ ] T031 [P] [L2] Create Example 4: Abstract method enforcement metaclass
- [ ] T032 [P] [L2] Create Example 5: Simplified Django-like model metaclass (field collection)
- [ ] T033 [P] [L2] Create Example 6: Alternative approach using __init_subclass__

### AI-Native Learning Integration for Lesson 2

- [ ] T034 [L2] Write "Try With AI" section with 4 prompts
  - Prompt 1 (Recall): "Explain the registration pattern with a metaclass"
  - Prompt 2 (Understand): "How does Django's Model metaclass automatically find fields?"
  - Prompt 3 (Apply): "Create a metaclass that logs every method call on a class"
  - Prompt 4 (Evaluate): "Should I use a metaclass or decorator for validation? Why?"

### Validation for Lesson 2

- [ ] T035 [L2] Test all 6 code examples run on Python 3.14+ without errors
- [ ] T036 [L2] Verify type hints present on 100% of code examples
- [ ] T037 [L2] Validate cognitive load: exactly 10 concepts, no more
- [ ] T038 [L2] Verify Django/SQLAlchemy references are accurate (framework awareness)
- [ ] T039 [L2] Verify lesson ends with "Try With AI" section ONLY

**Checkpoint**: Lesson 2 complete and validated - metaclass coverage complete (50%)

---

## Phase 4: Lesson 3 - Introduction to Dataclasses (B1 Proficiency)

**Lesson Goal**: Master dataclasses for clean, type-safe data models without boilerplate

**Duration**: 45-60 minutes
**Concepts**: 10 (decorator basics, type hints, auto-generated methods)
**Code Examples**: 5
**Try With AI**: 4 prompts (Recall → Understand → Apply → Analyze)

**Independent Test**: Student can create dataclass with type hints, defaults, frozen/unfrozen states

### Content Development for Lesson 3

- [ ] T040 [L3] Create lesson-3-introduction-to-dataclasses.mdx with frontmatter (CEFR: B1)
- [ ] T041 [L3] Write introduction section explaining dataclass purpose (eliminate boilerplate)
- [ ] T042 [L3] Develop Section 1: "What Are Dataclasses?" (concepts 1-3: decorator-based containers, type hints required, auto-generation)
- [ ] T043 [L3] Develop Section 2: "Auto-Generated Methods" (concepts 4-5: __init__, __repr__, __eq__)
- [ ] T044 [L3] Develop Section 3: "Basic Features" (concepts 6-8: defaults, frozen, order)
- [ ] T045 [L3] Develop Section 4: "When to Use Dataclasses" (concepts 9-10: API models, config, comparison with NamedTuple/traditional)

### Code Examples for Lesson 3

- [ ] T046 [P] [L3] Create Example 1: Basic dataclass with auto-generated methods in lesson-3-introduction-to-dataclasses.mdx
- [ ] T047 [P] [L3] Create Example 2: Dataclass with default values and optional fields
- [ ] T048 [P] [L3] Create Example 3: Frozen dataclass (immutable data)
- [ ] T049 [P] [L3] Create Example 4: Ordered dataclass (sortable instances)
- [ ] T050 [P] [L3] Create Example 5: Dataclass vs traditional class comparison (show boilerplate saved)

### AI-Native Learning Integration for Lesson 3

- [ ] T051 [L3] Write "Try With AI" section with 4 prompts
  - Prompt 1 (Recall): "What methods does @dataclass auto-generate?"
  - Prompt 2 (Understand): "Why must dataclass fields have type hints?"
  - Prompt 3 (Apply): "Create a dataclass for a User with name, email, age, and optional bio"
  - Prompt 4 (Analyze): "When should I use frozen=True for a dataclass?"

### Validation for Lesson 3

- [ ] T052 [L3] Test all 5 code examples run on Python 3.14+ without errors
- [ ] T053 [L3] Verify type hints present on 100% of dataclass fields
- [ ] T054 [L3] Validate cognitive load: exactly 10 concepts, no more
- [ ] T055 [L3] Check reading level with Flesch-Kincaid (target: Grade 10-11)
- [ ] T056 [L3] Verify lesson ends with "Try With AI" section ONLY

**Checkpoint**: Lesson 3 complete and validated - dataclass introduction complete

---

## Phase 5: Lesson 4 - Advanced Dataclass Features (B1-B2 Proficiency)

**Lesson Goal**: Use advanced dataclass features for production-quality data models

**Duration**: 50-65 minutes
**Concepts**: 10 (field customization, metadata, post_init, validation)
**Code Examples**: 6
**Try With AI**: 4 prompts (Recall → Understand → Apply → Create)

**Independent Test**: Student can use field(), __post_init__(), InitVar, and JSON serialization

### Content Development for Lesson 4

- [ ] T057 [L4] Create lesson-4-advanced-dataclass-features.mdx with frontmatter (CEFR: B1-B2)
- [ ] T058 [L4] Write introduction section building on Lesson 3 basics
- [ ] T059 [L4] Develop Section 1: "Field Customization" (concepts 1-3: field() function, default_factory, init/repr parameters)
- [ ] T060 [L4] Develop Section 2: "Field Metadata" (concept 4: attaching arbitrary data to fields)
- [ ] T061 [L4] Develop Section 3: "Post-Init Processing" (concepts 5-7: __post_init__(), InitVar, validation)
- [ ] T062 [L4] Develop Section 4: "Serialization and Real-World Patterns" (concepts 8-10: JSON conversion, API models, combining features)

### Code Examples for Lesson 4

- [ ] T063 [P] [L4] Create Example 1: Using default_factory for mutable defaults in lesson-4-advanced-dataclass-features.mdx
- [ ] T064 [P] [L4] Create Example 2: Field with metadata (for serialization hints)
- [ ] T065 [P] [L4] Create Example 3: __post_init__() for validation and computed fields
- [ ] T066 [P] [L4] Create Example 4: InitVar for post-init processing without storage
- [ ] T067 [P] [L4] Create Example 5: Dataclass with JSON serialization (to_dict/from_dict)
- [ ] T068 [P] [L4] Create Example 6: Real-world API model (combining all features)

### AI-Native Learning Integration for Lesson 4

- [ ] T069 [L4] Write "Try With AI" section with 4 prompts
  - Prompt 1 (Recall): "What's the difference between default and default_factory?"
  - Prompt 2 (Understand): "How does __post_init__() work and when do I use it?"
  - Prompt 3 (Apply): "Create a Product dataclass with price validation in __post_init__()"
  - Prompt 4 (Create): "Design an API response dataclass with nested objects and optional fields"

### Validation for Lesson 4

- [ ] T070 [L4] Test all 6 code examples run on Python 3.14+ without errors
- [ ] T071 [L4] Verify type hints present on 100% of dataclass fields
- [ ] T072 [L4] Validate cognitive load: exactly 10 concepts, no more
- [ ] T073 [L4] Verify API modeling examples are realistic and practical
- [ ] T074 [L4] Verify lesson ends with "Try With AI" section ONLY

**Checkpoint**: Lesson 4 complete and validated - dataclass coverage complete (50%)

---

## Phase 6: Lesson 5 - Choosing the Right Tool (B2 Proficiency)

**Lesson Goal**: Synthesize knowledge to make informed architectural decisions

**Duration**: 40-50 minutes
**Concepts**: 8 (problem domains, decision matrix, performance, tradeoffs)
**Code Examples**: 4
**Try With AI**: 4 prompts (Recall → Understand → Evaluate → Synthesize)

**Independent Test**: Student can analyze problem and choose between metaclass, dataclass, traditional class

### Content Development for Lesson 5

- [ ] T075 [L5] Create lesson-5-choosing-the-right-tool.mdx with frontmatter (CEFR: B2)
- [ ] T076 [L5] Write introduction section synthesizing Lessons 1-4
- [ ] T077 [L5] Develop Section 1: "Problem Domains" (concepts 1-3: metaclass use cases, dataclass use cases, traditional class use cases)
- [ ] T078 [L5] Develop Section 2: "Decision Matrix" (concept 4: when to use which approach)
- [ ] T079 [L5] Develop Section 3: "Framework Insights" (concept 5: why frameworks chose specific approaches)
- [ ] T080 [L5] Develop Section 4: "Tradeoffs and Best Practices" (concepts 6-8: performance, readability, maintainability)

### Code Examples for Lesson 5

- [ ] T081 [P] [L5] Create Example 1: Same problem solved 3 ways (metaclass, dataclass, traditional) in lesson-5-choosing-the-right-tool.mdx
- [ ] T082 [P] [L5] Create Example 2: Framework-like design using metaclass (registration + validation)
- [ ] T083 [P] [L5] Create Example 3: API layer using dataclasses (request/response models)
- [ ] T084 [P] [L5] Create Example 4: Hybrid approach (dataclass with metaclass for advanced control)

### AI-Native Learning Integration for Lesson 5

- [ ] T085 [L5] Write "Try With AI" section with 4 prompts
  - Prompt 1 (Recall): "Summarize when to use metaclasses vs dataclasses"
  - Prompt 2 (Understand): "Why are dataclasses preferred for most data modeling?"
  - Prompt 3 (Evaluate): "Analyze this code: should it use a metaclass, dataclass, or neither?"
  - Prompt 4 (Synthesize): "Design a system with both metaclasses and dataclasses working together"

### Validation for Lesson 5

- [ ] T086 [L5] Test all 4 code examples run on Python 3.14+ without errors
- [ ] T087 [L5] Verify type hints present on 100% of code examples
- [ ] T088 [L5] Validate cognitive load: exactly 8 concepts (synthesis lesson)
- [ ] T089 [L5] Verify decision matrix is clear and actionable
- [ ] T090 [L5] Verify lesson ends with "Try With AI" section ONLY

**Checkpoint**: Lesson 5 complete and validated - chapter synthesis complete

---

## Phase 7: Chapter Integration and Polish

**Purpose**: Cross-cutting concerns and final chapter validation

- [ ] T091 Update chapter intro.md with lesson navigation links
- [ ] T092 Verify 50/50 balance: 10 metaclass examples (L1-2), 11 dataclass examples (L3-4), 4 synthesis (L5)
- [ ] T093 Cross-reference validation: all prerequisite chapter links work
- [ ] T094 [P] Create chapter-level summary connecting all 5 lessons
- [ ] T095 [P] Verify no forward references to Chapter 27+ or Part 5 SDD terminology
- [ ] T096 [P] Validate Part 4 appropriate language ("describe intent" not "write specifications")
- [ ] T097 Run Flesch-Kincaid readability test on all lessons (target: Grade 10-11)
- [ ] T098 Verify all 20 "Try With AI" prompts follow Bloom's progression
- [ ] T099 Test complete chapter flow (45-60-45-60-40 minute lesson sequence = 3.5-4.5 hours)
- [ ] T100 Final constitutional compliance check against all acceptance criteria from spec.md

**Checkpoint**: Chapter 26 complete and ready for technical review

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Lesson 1 (Phase 2)**: Depends on Setup completion
- **Lesson 2 (Phase 3)**: Depends on Lesson 1 completion (builds on metaclass foundation)
- **Lesson 3 (Phase 4)**: Depends on Setup completion (independent of metaclass lessons)
- **Lesson 4 (Phase 5)**: Depends on Lesson 3 completion (builds on dataclass basics)
- **Lesson 5 (Phase 6)**: Depends on Lessons 1-4 completion (synthesis requires all prior lessons)
- **Polish (Phase 7)**: Depends on all lessons being complete

### Lesson Dependencies

- **Lesson 1**: Standalone (introduces metaclass fundamentals)
- **Lesson 2**: Requires Lesson 1 (applies metaclass patterns)
- **Lesson 3**: Standalone (introduces dataclass fundamentals)
- **Lesson 4**: Requires Lesson 3 (applies advanced dataclass features)
- **Lesson 5**: Requires Lessons 1-4 (compares and synthesizes both topics)

### Within Each Lesson

- Content development before code examples
- All code examples can be created in parallel [P]
- Code examples before AI-Native Learning section
- AI-Native Learning section before validation
- All validation tasks can run in parallel
- Lesson complete before moving to next lesson

### Parallel Opportunities

**Phase 1 (Setup)**:
- T002, T003, T004 can run in parallel

**Phase 2 (Lesson 1)**:
- T011, T012, T013, T014 (all code examples) can run in parallel

**Phase 3 (Lesson 2)**:
- T028-T033 (all 6 code examples) can run in parallel

**Phase 4 (Lesson 3)**:
- T046-T050 (all 5 code examples) can run in parallel

**Phase 5 (Lesson 4)**:
- T063-T068 (all 6 code examples) can run in parallel

**Phase 6 (Lesson 5)**:
- T081-T084 (all 4 code examples) can run in parallel

**Phase 7 (Polish)**:
- T094, T095, T096 can run in parallel

---

## Implementation Strategy

### Sequential Lesson Development (Recommended)

1. Complete Phase 1: Setup
2. Complete Phase 2: Lesson 1 → **VALIDATE** → Human review checkpoint
3. Complete Phase 3: Lesson 2 → **VALIDATE** → Human review checkpoint (metaclass coverage complete)
4. Complete Phase 4: Lesson 3 → **VALIDATE** → Human review checkpoint
5. Complete Phase 5: Lesson 4 → **VALIDATE** → Human review checkpoint (dataclass coverage complete)
6. Complete Phase 6: Lesson 5 → **VALIDATE** → Human review checkpoint (synthesis complete)
7. Complete Phase 7: Polish → Final chapter validation

### Parallel Development Strategy (Advanced)

With multiple developers:

1. Team completes Phase 1: Setup together
2. Once Setup done:
   - **Developer A**: Lesson 1 → Lesson 2 (metaclass track)
   - **Developer B**: Lesson 3 → Lesson 4 (dataclass track)
   - **Developer C**: Chapter polish and cross-references
3. All complete before Lesson 5 (requires both tracks)
4. Team reviews Lesson 5 together (synthesis requires all context)

### Quality Gates

**After Each Lesson**:
- All code examples tested on Python 3.14+
- Type hints validated (100% coverage)
- Cognitive load validated (≤10 concepts)
- Reading level checked (Grade 10-11)
- Lesson ends with "Try With AI" ONLY

**After Chapter Complete**:
- 50/50 balance verified (metaclass vs dataclass depth)
- Total duration 3.5-4.5 hours
- No forward references
- Part 4 appropriate language
- Constitutional compliance

---

## Policy Note for Lesson Authors

**Lesson Closure Pattern** (Constitutional Requirement):
- Each lesson MUST end with a single final section titled **"Try With AI"**
- NO "Key Takeaways" section
- NO "What's Next" section
- NO "Summary" section after Try With AI

**AI Tool Instructions**:
- Before AI tools are taught (Parts 1-3): Use ChatGPT web in "Try With AI" sections
- After tool onboarding (Part 4+): Instruct learners to use their preferred AI companion tool
- Optionally provide both CLI and web variants for flexibility

**For Chapter 26 (Part 4)**:
Learners have been introduced to AI tools. "Try With AI" sections should say:
> "Use your preferred AI companion (Claude CLI, ChatGPT, Gemini, etc.) to explore these prompts."

---

## Notes

- Total tasks: 100 (comprehensive chapter implementation)
- Estimated effort: ~45-55 story points (5 lessons × ~9-11 SP each)
- [P] tasks = different files, no dependencies - can run in parallel
- [Lesson] label maps task to specific lesson for traceability
- Each lesson should be independently reviewable and testable
- Stop at each checkpoint to validate lesson quality before proceeding
- Commit after each lesson or logical group
- Avoid: vague tasks, cognitive overload, missing validation steps
