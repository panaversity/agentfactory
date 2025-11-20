# Feature Specification: Chapter 19 - Set, Frozen Set, and GC

**Feature Branch**: `001-part-4-chapter-19`
**Created**: 2025-11-08
**Status**: Draft
**Chapter**: 19 (Part 4: Python Fundamentals)
**Complexity Tier**: Intermediate (A2-B1)
**Input**: Chapter 19: "Set, Frozen Set, and GC" - Balanced approach with 6 lessons (Sets fundamentals, operations, internals + Frozensets + GC + Memory Profiler capstone)

---

## Success Evals *(FIRST - Defined BEFORE Implementation)*

### Measurable, Business-Goal-Aligned Success Criteria

**SC-001: Set Conceptual Mastery**
- **Metric**: 75%+ of students can explain set vs list tradeoffs in their own words (post-chapter quiz)
- **Business Goal**: Students understand when to choose appropriate data structures, reducing code inefficiency
- **Assessment**: Written explanation scored on: uniqueness property, O(1) lookup, unordered nature, immutability requirement

**SC-002: Practical Application - Type-Hinted Set Operations**
- **Metric**: 80%+ of students can write type-hinted functions using sets correctly (code exercise)
- **Business Goal**: Students apply modern Python standards in real code
- **Assessment**: Code review checking: type hints (`set[T]`), correct operations, working examples

**SC-003: Memory Profiling Competency**
- **Metric**: 70%+ of students successfully build and use the Memory Profiler capstone tool (practical project)
- **Business Goal**: Students understand Python memory management for optimization
- **Assessment**: Working tool that profiles object creation/deletion, demonstrates reference counting

**SC-004: Reading Level Accessibility**
- **Metric**: Chapter content maintains Grade 7-8 reading level (automated check)
- **Business Goal**: Content accessible to intermediate learners (A2-B1 proficiency)
- **Assessment**: Flesch-Kincaid readability score 7.0-8.0

**SC-005: Engagement and Completion**
- **Metric**: 85%+ lesson completion rate across all 6 lessons
- **Business Goal**: Content holds student attention and maintains learning momentum
- **Assessment**: Tracking data from LMS/chapter analytics

**SC-006: AI Partnership Skill Development**
- **Metric**: Students use AI tools for exploration in 90%+ of "Try With AI" exercises
- **Business Goal**: Build AI-native learning habits, not rote memorization
- **Assessment**: Self-reported AI usage + observation of collaborative problem-solving patterns

---

## Topic Summary

Python's set and frozenset data types enable efficient membership testing, duplicate elimination, and mathematical set operations. Sets provide O(1) average-time lookup through hash-based storage, making them essential for performance-critical tasks like deduplication and uniqueness checking. Understanding sets requires grasping Python's hashing mechanism—why only immutable objects can be set members and how hash collisions are managed internally.

Garbage collection (GC) is Python's automatic memory management system using reference counting and cycle detection. While Python handles memory automatically, understanding GC helps developers write more efficient code, avoid memory leaks with circular references, and profile memory usage in production applications. The `gc` module provides tools to analyze and optimize memory patterns.

This chapter builds on prior knowledge of lists, tuples, and dictionaries (Chapter 18) and type systems (Chapter 14), introducing intermediate concepts through AI-Native Learning: students describe intent with type hints, explore concepts with AI co-teachers, validate understanding through testing, and learn from errors through collaborative debugging. The capstone Memory Profiler project integrates all concepts into a practical tool for analyzing Python program memory usage.

**Why This Matters**:
- **Performance**: Sets offer O(1) lookup vs O(n) for lists—critical at scale
- **Data Integrity**: Automatic duplicate elimination prevents data errors
- **Memory Efficiency**: Understanding GC enables optimization and leak prevention
- **Professional Development**: Hash-based data structures are interview staples and production essentials

**Connection to Prior Chapters**:
- Chapter 14 (Data Types): Builds on type system understanding, adds collection types
- Chapter 18 (Lists, Tuples, Dictionary): Completes Python's core collection types
- Chapters 1-11 (AIDD): Applies AI-Native Learning methodology learned in Part 1

---

## Prerequisites *(mandatory)*

**Explicit Prerequisites**:

1. **Chapter 14: Data Types** - Understanding of Python's type system, type hints, and basic type operations
2. **Chapter 18: Lists, Tuples, Dictionary** - Familiarity with mutable/immutable collection types and tradeoffs
3. **Chapter 13: Introduction to Python** - Python syntax basics, variables, basic iteration
4. **All Chapters 1-11** - AIDD principles, AI partnership, validation-first thinking
5. **Development Environment** - Python 3.14+ installed, Claude Code or Gemini CLI configured

**Assumed Knowledge**:
- Basic terminal/command line usage
- Text editor or IDE setup
- Type hints syntax (`list[int]`, `dict[str, str]`)
- Iteration with `for` loops
- Basic function definition and calling

**Not Required** (will be taught in this chapter):
- Hash functions or hashing mechanics
- Memory management concepts
- Advanced type theory
- Set theory mathematics (will introduce basics as needed)

---

## Learning Journeys & Testing *(mandatory)*

### Learning Journey 1 - Set Fundamentals Mastery (Priority: P1)

**Scenario**: A student learning Python needs to efficiently check if values exist in a collection and automatically eliminate duplicates from user input.

**Learning Path**:
1. Student understands what sets are (unordered, unique, mutable)
2. Student creates sets with type hints: `unique_ids: set[int] = {1, 2, 3}`
3. Student performs basic operations: `add()`, `remove()`, `discard()`
4. Student iterates over sets and understands order unpredictability
5. Student explains why sets require immutable elements (hashability)

**Why this priority**: Foundational understanding enables all subsequent lessons. Without grasping uniqueness + hashability, later concepts (operations, internals, frozensets) lack context.

**Independent Test**: Student can write a function that accepts a list and returns a set of unique values with proper type hints. Test delivers value: working deduplication tool.

**Acceptance Scenarios**:

1. **Given** a Python REPL session, **When** student creates `emails: set[str] = {"alice@example.com", "bob@example.com"}`, **Then** they can explain why duplicates are impossible
2. **Given** attempt to add `[1, 2, 3]` to a set, **When** TypeError occurs, **Then** student explains "lists are mutable, sets need hashable immutable elements"
3. **Given** a set `{3, 1, 2}`, **When** printed multiple times, **Then** student understands order may vary (unordered property)
4. **Given** AI partnership, **When** student asks "Why use a set instead of a list?", **Then** AI explains O(1) lookup vs O(n) iteration
5. **Given** code exercise, **When** student writes type-hinted function removing duplicates, **Then** function works and uses modern syntax

---

### Learning Journey 2 - Mathematical Set Operations (Priority: P1)

**Scenario**: A student building data analysis tools needs to find common elements between datasets, unique elements, or combine datasets without duplicates.

**Learning Path**:
1. Student performs union (`|` operator or `.union()`)
2. Student performs intersection (`&` operator or `.intersection()`)
3. Student performs difference (`-` operator or `.difference()`)
4. Student performs symmetric difference (`^` operator or `.symmetric_difference()`)
5. Student uses set comprehensions for filtered sets
6. Student applies set operations to real-world problems (data deduplication, filtering, comparison)

**Why this priority**: Set operations are the practical application of set theory. These enable real-world tasks and differentiate sets from lists/tuples.

**Independent Test**: Student writes a function comparing two user lists to find "common interests" (intersection) and "unique interests" (symmetric difference). Delivers value: working comparison tool.

**Acceptance Scenarios**:

1. **Given** `users_A = {1, 2, 3}` and `users_B = {3, 4, 5}`, **When** student computes `users_A | users_B`, **Then** result is `{1, 2, 3, 4, 5}` (union)
2. **Given** same sets, **When** student computes `users_A & users_B`, **Then** result is `{3}` (intersection)
3. **Given** same sets, **When** student computes `users_A - users_B`, **Then** result is `{1, 2}` (difference)
4. **Given** same sets, **When** student computes `users_A ^ users_B`, **Then** result is `{1, 2, 4, 5}` (symmetric difference)
5. **Given** AI collaboration, **When** student asks "Generate a set comprehension filtering even numbers", **Then** AI provides `{x for x in range(10) if x % 2 == 0}` and explains syntax

---

### Learning Journey 3 - Set Internals and Performance (Priority: P2)

**Scenario**: A student optimizing code needs to understand why set lookups are O(1) and when sets outperform lists for membership testing.

**Learning Path**:
1. Student understands hash functions convert values to integers
2. Student grasps hash-based internal storage (hash table)
3. Student explains why immutability is required (hash values must be stable)
4. Student understands hash collisions and rehashing
5. Student analyzes performance: O(1) lookup vs O(n) list iteration
6. Student uses AI to explore internal mechanisms beyond scope

**Why this priority**: Understanding internals builds professional competence. While not required for basic usage, this knowledge enables optimization and debugging.

**Independent Test**: Student explains (in writing or to AI) why `{1, 2, 3} in my_set` is faster than iterating a list. Test delivers conceptual understanding applicable to production code.

**Acceptance Scenarios**:

1. **Given** concept of hashing, **When** student explains why `hash("hello")` produces an integer, **Then** they understand hashing converts objects to lookup keys
2. **Given** error "TypeError: unhashable type: 'list'", **When** student encounters it, **Then** they explain "lists are mutable, hash values would change"
3. **Given** performance comparison, **When** student tests `x in my_set` vs `x in my_list` with 10,000 elements, **Then** they observe set is dramatically faster
4. **Given** AI partnership, **When** student asks "How does Python store sets internally?", **Then** AI explains hash table structure without overwhelming detail
5. **Given** rehashing concept, **When** student adds many elements to a set, **Then** they understand Python resizes the hash table automatically

---

### Learning Journey 4 - Frozensets for Immutability (Priority: P2)

**Scenario**: A student needs to use sets as dictionary keys or nest sets within sets, requiring immutable set variants.

**Learning Path**:
1. Student understands frozenset as immutable set
2. Student creates frozensets: `frozen: frozenset[int] = frozenset([1, 2, 3])`
3. Student uses frozensets as dictionary keys
4. Student nests frozensets in sets
5. Student chooses between set and frozenset based on requirements

**Why this priority**: Frozensets are specialized. Students must grasp sets first, then learn immutable variants for specific use cases.

**Independent Test**: Student creates a dictionary mapping frozensets to values (e.g., `{frozenset([1, 2]): "group_A"}`). Test delivers understanding of hashability for complex keys.

**Acceptance Scenarios**:

1. **Given** immutability requirement, **When** student attempts `my_frozen.add(4)`, **Then** AttributeError occurs and student explains "frozensets are immutable"
2. **Given** dictionary key use case, **When** student creates `{frozenset([1, 2]): "value"}`, **Then** it works (frozensets are hashable)
3. **Given** attempt to use regular set as dictionary key, **When** TypeError occurs, **Then** student explains "only immutable types can be dict keys"
4. **Given** AI collaboration, **When** student asks "When should I use frozenset vs set?", **Then** AI explains immutability tradeoffs
5. **Given** comparison task, **When** student lists differences between set and frozenset, **Then** they identify mutability, hashability, use cases

---

### Learning Journey 5 - Garbage Collection Fundamentals (Priority: P2)

**Scenario**: A student writing long-running programs needs to understand Python's automatic memory management to avoid leaks and optimize performance.

**Learning Path**:
1. Student understands reference counting (primary GC mechanism)
2. Student explores automatic memory management (objects deleted when refcount reaches 0)
3. Student uses `gc` module to analyze memory
4. Student understands cycle detection for circular references
5. Student profiles memory usage in Python programs
6. Student knows when GC runs and how to trigger manually

**Why this priority**: GC is automatic—students don't need to manage it for basic programs. Understanding enables optimization and debugging in production.

**Independent Test**: Student writes a script using `gc.get_objects()` to count objects in memory before/after creating/deleting variables. Test delivers hands-on GC exploration.

**Acceptance Scenarios**:

1. **Given** reference counting concept, **When** student creates `x = [1, 2, 3]` then `del x`, **Then** they explain "refcount drops to 0, memory freed"
2. **Given** circular reference `a.next = b` and `b.prev = a`, **When** student deletes both, **Then** they understand cycle detector finds and collects them
3. **Given** `gc` module, **When** student runs `gc.collect()`, **Then** they manually trigger garbage collection
4. **Given** memory profiling, **When** student uses `gc.get_objects()`, **Then** they see all tracked objects
5. **Given** AI partnership, **When** student asks "How do I profile memory in Python?", **Then** AI introduces `gc`, `sys.getsizeof()`, and profiling tools

---

### Learning Journey 6 - Memory Profiler Capstone Integration (Priority: P3)

**Scenario**: A student completing the chapter builds a working Memory Profiler tool integrating sets, frozensets, and GC concepts.

**Learning Path**:
1. Student designs tool specification (with AI collaboration)
2. Student implements object tracking using sets
3. Student profiles memory with `gc` module
4. Student displays results showing reference counts and memory usage
5. Student tests and validates tool functionality
6. Student reflects on integration of all chapter concepts

**Why this priority**: Capstone validates mastery. While valuable, it's the culmination—requires P1/P2 foundations first.

**Independent Test**: Student submits working Memory Profiler tool that tracks object creation/deletion and displays memory statistics. Delivers portfolio-worthy project.

**Acceptance Scenarios**:

1. **Given** tool specification, **When** student describes requirements to AI, **Then** AI helps design class structure
2. **Given** implementation, **When** student tracks objects in a set, **Then** tool correctly identifies created/deleted objects
3. **Given** memory profiling, **When** student uses `gc` module, **Then** tool displays current object counts
4. **Given** testing, **When** student creates/deletes variables, **Then** tool tracks changes accurately
5. **Given** reflection, **When** student explains tool design, **Then** they articulate how sets, frozensets, and GC integrate
6. **Given** validation, **When** student tests edge cases (circular references, many objects), **Then** tool handles them correctly

---

### Edge Cases

**Set Edge Cases**:
- What happens when students try to add mutable objects (lists, dicts) to sets? (TypeError: unhashable type)
- How does the system handle empty sets created with `set()` vs `{}`? (Distinguish empty set from empty dict)
- What occurs when set operations are performed on sets of different types? (Works, but type hints guide correctness)
- How are hash collisions managed internally? (Python's hash table handles collisions transparently)

**Frozenset Edge Cases**:
- What happens when students try to modify a frozenset? (AttributeError: no add/remove methods)
- How does the system handle frozenset of frozensets? (Works—frozensets are hashable)
- What occurs when comparing frozenset to set with same elements? (Equal for value comparison)

**GC Edge Cases**:
- What happens with circular references in Python? (Cycle detector finds and collects them)
- How does the system handle large object graphs? (GC automatically manages, may be slow for huge graphs)
- What occurs when students manually disable GC? (Memory leaks possible with cycles)

---

## Requirements *(mandatory)*

### Functional Requirements

**Lesson 1: Set Basics**

- **FR-001**: Students MUST be able to create sets using literal syntax `{1, 2, 3}` and constructor `set([1, 2, 3])` with proper type hints
- **FR-002**: Students MUST understand and explain uniqueness property (duplicates automatically eliminated)
- **FR-003**: Students MUST understand and explain unordered nature (iteration order unpredictable)
- **FR-004**: Students MUST be able to add and remove elements using `.add()`, `.remove()`, `.discard()`
- **FR-005**: Students MUST understand why mutable objects cannot be set members (hashability requirement)
- **FR-006**: Chapter MUST teach via AI-Native Learning: describe intent → explore → validate → learn from errors

**Lesson 2: Set Operations**

- **FR-007**: Students MUST perform union using `|` operator and `.union()` method
- **FR-008**: Students MUST perform intersection using `&` operator and `.intersection()` method
- **FR-009**: Students MUST perform difference using `-` operator and `.difference()` method
- **FR-010**: Students MUST perform symmetric difference using `^` operator and `.symmetric_difference()` method
- **FR-011**: Students MUST write set comprehensions with filtering conditions
- **FR-012**: Students MUST apply set operations to practical problems (deduplication, comparison, filtering)

**Lesson 3: Set Internals**

- **FR-013**: Students MUST understand hash functions convert objects to integers
- **FR-014**: Students MUST explain why immutable objects are required (stable hash values)
- **FR-015**: Students MUST grasp O(1) average-time lookup performance
- **FR-016**: Students MUST understand hash collisions conceptually (without implementation details)
- **FR-017**: Students MUST understand rehashing when sets grow
- **FR-018**: Students MUST explore internal mechanisms with AI assistance

**Lesson 4: Frozensets**

- **FR-019**: Students MUST create frozensets using `frozenset()` constructor with type hints
- **FR-020**: Students MUST understand immutability (no add/remove after creation)
- **FR-021**: Students MUST use frozensets as dictionary keys
- **FR-022**: Students MUST nest frozensets within sets
- **FR-023**: Students MUST choose between set and frozenset based on requirements

**Lesson 5: Garbage Collection**

- **FR-024**: Students MUST understand reference counting as primary GC mechanism
- **FR-025**: Students MUST explain automatic memory management (objects deleted when refcount = 0)
- **FR-026**: Students MUST use `gc` module to analyze memory
- **FR-027**: Students MUST understand cycle detection for circular references
- **FR-028**: Students MUST profile basic memory usage in scripts
- **FR-029**: Students MUST know when GC runs and how to trigger manually

**Lesson 6: Memory Profiler Capstone**

- **FR-030**: Students MUST design Memory Profiler tool with AI collaboration
- **FR-031**: Students MUST implement object tracking using sets
- **FR-032**: Students MUST integrate `gc` module for memory profiling
- **FR-033**: Students MUST display results showing object counts and memory usage
- **FR-034**: Students MUST test and validate tool functionality
- **FR-035**: Students MUST reflect on integration of chapter concepts

**Constitutional Compliance**

- **FR-036**: ALL lessons MUST end with "Try With AI" section (4 prompts, Bloom's progression)
- **FR-037**: ALL lessons MUST include CoLearning elements throughout (💬🎓🚀✨)
- **FR-038**: ALL code examples MUST use Python 3.14+ with mandatory type hints
- **FR-039**: Chapter MUST maintain conversational tone (not documentation style)
- **FR-040**: Chapter MUST include AI troubleshooting prompts for common errors
- **FR-041**: Lessons MUST progress: manual practice (L1-2) → AI companion (L3+) → AI orchestration (capstone)
- **FR-042**: Chapter MUST maintain A2-B1 complexity (max 7 concepts/lesson, 3-4 options introduced)

### Key Concepts *(educational entities)*

**Set Concepts**:
- **Set**: Unordered collection of unique hashable elements; mutable; supports add/remove; O(1) lookup
- **Uniqueness**: Automatic duplicate elimination; fundamental property distinguishing sets from lists
- **Hashability**: Requirement that elements be immutable (strings, numbers, tuples, frozensets); enables hash-based storage
- **Unordered**: No guaranteed iteration order; optimized for membership testing not sequencing

**Frozenset Concepts**:
- **Frozenset**: Immutable variant of set; hashable; can be dict keys or set members; no add/remove methods
- **Immutability**: Fixed after creation; enables use in contexts requiring hashable types

**Operations Concepts**:
- **Union** (`|`): Combines elements from multiple sets; mathematical ∪
- **Intersection** (`&`): Elements common to all sets; mathematical ∩
- **Difference** (`-`): Elements in first set but not second; mathematical −
- **Symmetric Difference** (`^`): Elements in either set but not both; mathematical △

**Performance Concepts**:
- **Hash Table**: Internal storage mechanism; enables O(1) average lookup
- **O(1) Lookup**: Constant-time membership testing; drastically faster than O(n) list iteration
- **Rehashing**: Automatic resizing when set grows; maintains performance

**Memory Concepts**:
- **Reference Counting**: Primary GC mechanism; tracks object usage
- **Garbage Collection**: Automatic memory management; frees unused objects
- **Cycle Detection**: Finds and collects circular references
- **gc Module**: Python standard library for memory profiling and manual GC control

---

## Success Criteria - Implementation Checklist *(mandatory)*

### Content Quality Validation

- [ ] **No implementation leakage**: No mention of CPython internals, specific hash algorithms, or optimization tricks beyond O(1) concept
- [ ] **Business value clear**: Every concept answers "why does this matter for building software?"
- [ ] **Accessible language**: Grade 7-8 reading level maintained (check with automated tool)
- [ ] **All mandatory sections completed**: Evals, summary, prerequisites, learning objectives, content outline, examples, acceptance criteria

### Requirement Completeness

- [ ] **No unresolved clarifications**: All `[NEEDS CLARIFICATION]` markers removed or resolved
- [ ] **Requirements testable**: Every FR can be validated through code exercise, quiz, or capstone submission
- [ ] **Success criteria measurable**: Each SC has specific metric (percentage, count, time)
- [ ] **Success criteria technology-agnostic**: No "Python does X" in success criteria; use "Students can explain X"
- [ ] **Acceptance scenarios defined**: Each learning journey has Given/When/Then scenarios
- [ ] **Edge cases identified**: Set, frozenset, and GC edge cases documented
- [ ] **Scope clearly bounded**: Chapter 19 content vs Chapter 20+ content distinguished
- [ ] **Dependencies identified**: Prerequisites explicitly listed

### Educational Feature Readiness

- [ ] **All 6 lessons outlined**: Set Basics, Set Operations, Set Internals, Frozensets, GC, Capstone
- [ ] **Learning objectives aligned with evals**: Each SC maps to specific learning journey(s)
- [ ] **Cognitive load within limits**: Max 7 concepts per lesson (intermediate A2-B1 tier)
- [ ] **CoLearning elements specified**: 💬🎓🚀✨ elements planned for each lesson
- [ ] **Try With AI format defined**: 4 prompts per lesson, Bloom's progression
- [ ] **Graduated teaching pattern applied**: Tier 1 (foundational) → Tier 2 (complex) → Tier 3 (orchestration)
- [ ] **Type hints mandatory**: All code examples use Python 3.14+ modern syntax
- [ ] **No forward references**: No Chapter 30+ SDD terminology; use "AI-Native Learning" framing

### Code Examples Strategy

- [ ] **15-20 examples planned**: Distributed across 6 lessons
- [ ] **Examples have specifications**: Each example's purpose and pedagogical value documented
- [ ] **Complexity progression**: Simple → applied → integrated
- [ ] **AI prompts defined**: Prompts that generate each example specified
- [ ] **Validation steps included**: How students test understanding after each example

---

## Notes and Assumptions

### Assumptions Made

1. **Python Version**: Assumed Python 3.14+ is installed and accessible (per prerequisites)
2. **AI Tools**: Assumed students have Claude Code or Gemini CLI configured (per Part 2)
3. **Prior Knowledge**: Assumed solid understanding of Chapter 18 (lists, tuples, dicts) and Chapter 14 (type hints)
4. **Reading Level**: Target Grade 7-8 (accessible to intermediate learners)
5. **Time Investment**: Estimated 4-5 hours to complete all 6 lessons (typical Part 4 chapter)
6. **Mathematical Background**: No formal set theory required (will introduce basics as needed)
7. **Performance Testing**: Assumed students can run timing benchmarks (basic `time` module usage)
8. **Memory Constraints**: Assumed student machines can handle 1M element sets for performance demos

### Reference Materials Usage

**Context Material**: `context/13_chap12_to_29_specs/Lesson_07_The_Set,_Frozenset_&_GC.md`

**Extracted Concepts (reference only)**:
- Set characteristics (unordered, unique, mutable)
- Set methods examples (union, intersection, difference)
- Hashing mechanics and hash table internals
- Frozenset immutability and use cases
- GC reference counting basics
- Internal storage explanations

**NOT Extracted (replaced with AI-Native Learning approach)**:
- Tutorial-style structure → Converted to CoLearning pedagogy
- Documentation tone → Converted to conversational style
- Traditional teaching → Converted to Graduated Teaching Pattern
- Missing CoLearning elements → Added 💬🎓🚀✨ throughout

### Design Decisions

1. **6 Lessons (not 5 or 7)**: Balanced approach splitting sets into 3 lessons (basics, operations, internals), 1 for frozensets, 1 for GC, 1 capstone integration
2. **Memory Profiler Capstone**: Chosen over simpler exercises to provide portfolio-worthy project demonstrating all concepts
3. **Graduated Teaching Pattern**: Lessons 1-2 manual practice, 3-5 AI companion, 6 AI orchestration (follows constitutional principles)
4. **Type Hints Mandatory**: Enforces modern Python standards and "describe intent" methodology
5. **No Custom Hash Functions**: Out of scope for intermediate tier; requires OOP knowledge (Chapter 24+)
6. **Conceptual GC Coverage**: Focuses on understanding, not tuning/optimization (appropriate for A2-B1)

---

## Next Steps

**After Specification Approval**:

1. **Run `/sp.clarify`** - Identify underspecified areas and ask targeted clarification questions (if needed)
2. **Run `/sp.plan`** - Create detailed lesson-by-lesson plan with CEFR proficiency mapping
3. **Run `/sp.adr`** - Document architectural decisions (if significant choices made during planning)
4. **Run `/sp.tasks`** - Generate actionable task checklist with acceptance criteria
5. **Run `/sp.analyze`** - Validate cross-artifact consistency (spec ↔ plan ↔ tasks)
6. **Implement with content-implementer** - Create actual lesson content with CoLearning pedagogy
7. **Validate with validation-auditor** - Verify code quality, pedagogical effectiveness, constitutional alignment

**Specification Status**: ✅ READY FOR REVIEW

This specification defines Chapter 19: "Set, Frozen Set, and GC" with:
- Evals-first success criteria (measurable, business-goal-aligned)
- 6 comprehensive learning journeys (independently testable)
- 42 functional requirements (testable and unambiguous)
- Constitutional compliance validated (AI-Native Learning, Graduated Teaching Pattern, CoLearning pedagogy)
- Complexity tier confirmed (A2-B1 intermediate, appropriate for Part 4)

Ready to proceed to planning phase (`/sp.plan`) after human approval.
