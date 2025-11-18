# ADR-0008: 11-Lesson Collections Structure for Intermediate A2-B1 Learners

> **Scope**: Pedagogical architecture for teaching three collection types (lists, tuples, dictionaries) to intermediate A2-B1 learners with cognitive load constraints. Extends ADR-0006 pattern from beginner (5 concepts/lesson) to intermediate (7 concepts/lesson) tier.

- **Status:** Accepted
- **Date:** 2025-11-08
- **Feature:** Chapter 18 - Lists, Tuples, and Dictionary
- **Context:** Part 4 Python Fundamentals (Chapters 12-29) - Intermediate-tier content (A2-B1 CEFR)

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: ✅ YES - Establishes intermediate cognitive load pattern for Part 4 chapters 17-23; affects chapter structure decisions
     2) Alternatives: ✅ YES - Considered 8-lesson, 10-lesson, and 12-lesson structures with different tradeoffs
     3) Scope: ✅ YES - Cross-cutting pedagogical concern affecting all Part 4 intermediate chapters (17-23)
-->

## Decision

**Adopt 11-lesson chapter structure for Chapter 18 collections, organized by data structure type with progressive complexity:**

### Lesson Distribution:
- **Lessons 1-5**: Lists (Foundation → Creation → Mutation → Advanced → Comprehensions)
  - Lesson 1: Collections Foundation (5 concepts, A1)
  - Lesson 2: Lists Part 1 - Creation & Basic Operations (6 concepts, A2)
  - Lesson 3: Lists Part 2 - Mutability & Modification (7 concepts, A2-B1)
  - Lesson 4: Lists Part 3 - Sorting & Advanced Methods (7 concepts, B1)
  - Lesson 5: List Comprehensions (6 concepts, B1)

- **Lesson 6**: Tuples (Immutability & Use Cases)
  - 7 concepts, A2-B1 (immutability, unpacking, hashable keys)

- **Lessons 7-9**: Dictionaries (Creation → CRUD → Iteration)
  - Lesson 7: Dicts Part 1 - Key-Value Mappings (6 concepts, A2)
  - Lesson 8: Dicts Part 2 - CRUD Operations (7 concepts, A2-B1)
  - Lesson 9: Dicts Part 3 - Iteration & Comprehensions (7 concepts, B1)

- **Lesson 10**: Choosing the Right Structure (7 concepts, B1)
  - Decision matrix, performance awareness, architectural thinking

- **Lesson 11**: Capstone - Data Processing Pipeline (0 new concepts, B1)
  - Integration only, no new concepts introduced

### Cognitive Load Enforcement:
- **Maximum 7 concepts per lesson** (intermediate A2-B1 limit)
- **Proficiency Progression**: A1 → A2 → A2-B1 → B1 (non-regressing across lessons)
- **Total Duration**: 44 hours (11 lessons × 4 hours each)

### Design Rationale:
1. **Lists get 5 lessons** (most complex, highest usage in real code)
2. **Tuples get 1 lesson** (simpler, focused on immutability concept)
3. **Dicts get 3 lessons** (moderate complexity, key-value semantics)
4. **Synthesis lesson** (choosing structures, architectural thinking)
5. **Capstone lesson** (integration, no new concepts)

## Consequences

### Positive

1. **Respects Intermediate Cognitive Load Science**: 7-concept limit aligns with A2-B1 working memory capacity (extends beginner 5-concept limit from ADR-0006)

2. **Depth-First Learning**: 5 lessons on lists ensures mastery before moving to tuples/dicts; prevents surface-level understanding

3. **Natural Progression**: Foundation → Basic → Mutability → Advanced → Transformation (comprehensions) follows cognitive scaffolding principles

4. **Prevents Conceptual Confusion**: Separating lists (mutable) from tuples (immutable) from dicts (key-value) reduces cognitive interference

5. **Prepares for Real Development**: Data Processing Pipeline capstone demonstrates realistic integration of all three structures

6. **Proficiency Mapping**: CEFR levels (A1/A2/A2-B1/B1) map cleanly to lessons; cognitive load validation straightforward

7. **Enables Skills Metadata**: Each lesson can be tagged with specific proficiency levels for institutional integration (see ADR-0009)

8. **Capstone Integration**: Lesson 11 reinforces all structures in cohesive project without introducing new concepts (integration not addition)

9. **Manageable Implementation**: content-implementer can focus on one structure aspect per lesson; easier to review and validate than monolithic collections lesson

10. **Constitutional Alignment**: Follows Principle 12 (Cognitive Load Management) and Principle 13 (Graduated Teaching Pattern)

### Negative

1. **Higher Chapter Scope**: 11 lessons = 44 hours (vs typical 8 lessons = 32 hours); risk of student drop-off (mitigated by cognitive load limits)

2. **Implementation Complexity**: content-implementer must maintain coherence across 11 lessons; requires careful cross-referencing

3. **Repetition Risk**: "Try With AI" sections must vary across 11 lessons to avoid monotony

4. **Comprehension Coverage**: Splitting lists into 5 lessons means comprehensions (Lesson 5) come late; students can't use comprehensions in earlier list lessons

5. **Tuples Underrepresented**: Only 1 lesson on tuples (vs 5 for lists, 3 for dicts) may give impression tuples are less important (actually reflects real-world usage patterns)

6. **Delayed Integration**: Capstone is Lesson 11; students don't see all-three-structures integration until end of chapter

## Alternatives Considered

### Alternative A: 8-Lesson Structure (More Compact)
**Structure**:
- Lesson 1: Collections Foundation
- Lessons 2-3: Lists (Basic + Advanced)
- Lesson 4: Tuples
- Lessons 5-6: Dicts (Basic + Advanced)
- Lesson 7: Comprehensions (Lists + Dicts)
- Lesson 8: Capstone

**Why Rejected**:
- Lessons 2-3 would contain 10+ concepts each (violates cognitive load limit)
- List comprehensions and dict comprehensions in same lesson = 8+ concepts (overload)
- Less depth, more breadth (contradicts Principle 12)

### Alternative B: 10-Lesson Structure (Remove Foundation)
**Structure**:
- Skip Lesson 1 (Collections Foundation)
- Start directly with Lists Part 1

**Why Rejected**:
- A2-B1 students need conceptual framing (Why do these three structures exist?)
- Missing foundation = students don't understand when to choose list vs tuple vs dict
- Violates Principle 13 (Graduated Teaching: Book teaches foundations)

### Alternative C: 12-Lesson Structure (Separate Comprehensions)
**Structure**:
- Add Lesson 6: List Comprehensions
- Add Lesson 10: Dict Comprehensions (separate from dict iteration)

**Why Rejected**:
- 12 lessons = 48 hours (too long, EVAL-007 completion rate risk)
- Comprehensions are related (better to teach together in Lessons 5 and 9)
- Diminishing returns: 12th lesson doesn't add significant value

### Alternative D: Merge Tuples into Lists (10 Lessons)
**Structure**:
- Lessons 1-5: Lists (including tuples as "immutable lists")
- Lessons 6-8: Dicts
- Lesson 9: Choosing Structure
- Lesson 10: Capstone

**Why Rejected**:
- Tuples are conceptually distinct (immutability, hashable keys, unpacking)
- Merging creates cognitive confusion (when is immutability important?)
- Violates pedagogical principle: Teach concepts distinctly before comparing

## References

- Feature Spec: `/specs/001-part-4-chapter-18/spec.md`
- Implementation Plan: `/specs/001-part-4-chapter-18/plan.md`
- Related ADRs:
  - ADR-0006: 5-Lesson Operator Separation Cognitive Load Pattern (beginner A1-A2 tier)
  - ADR-0009: CEFR Proficiency-Based Skills Metadata for Institutional Integration (enables proficiency tracking)
- Constitution: `.specify/memory/constitution.md` (Principle 12: Cognitive Load Management, Principle 13: Graduated Teaching Pattern)
- Research Foundation:
  - Miller's Law (7±2 items in working memory)
  - Cognitive Load Theory (Sweller, 1988)
  - CEFR proficiency levels (40+ years of language learning research)
