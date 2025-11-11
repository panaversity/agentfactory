# ADR-0008: Part 4 Chapter Architecture Pattern - 6-Lesson Concept-to-Integration Sequence

> **Scope**: Establishes the pedagogical architecture pattern for all Part 4 (Python Fundamentals) chapters 12-29. Documents lesson sequencing, complexity progression, and capstone integration as a cohesive pattern.

- **Status:** Accepted
- **Date:** 2025-11-08
- **Feature:** 001-part-4-chapter-19 (Sets, Frozensets, GC)
- **Context:** Part 4 teaches Python fundamentals to intermediate learners (CEFR A2-B1). After completing Chapter 14 (Data Types) with 5 lessons, we needed to determine the optimal chapter architecture for subsequent technical chapters. Chapter 19 (Sets, Frozensets, GC) serves as the reference implementation establishing this pattern.

<!-- Significance checklist (ALL must be true to justify this ADR)
     ✓ 1) Impact: Long-term consequence - defines architecture for 15+ chapters (Chapters 13-29)
     ✓ 2) Alternatives: Multiple viable options considered (3-lesson, 4-lesson, 5-lesson, defer-system-concepts)
     ✓ 3) Scope: Cross-cutting concern - affects all Part 4 content creation, student experience, completion rates
-->

## Decision

**Adopt a 6-lesson pedagogical architecture for Part 4 technical chapters following this sequence:**

1. **Lesson 1: Basics** — Core concept introduction, syntax, type hints, fundamental operations (A2 level, max 7 concepts)
2. **Lesson 2: Operations/Methods** — Applying the concept to solve problems, method exploration (A2-B1 level, max 7 concepts)
3. **Lesson 3: Internals/Performance** — Understanding "how it works," performance characteristics, design tradeoffs (B1 level, max 10 concepts)
4. **Lesson 4: Variants/Special Cases** — Related concepts, edge cases, when to use alternatives (B1 level, max 7-9 concepts)
5. **Lesson 5: System Integration** — How the concept integrates with Python's runtime, memory, or broader ecosystem (B1 level, max 9 concepts)
6. **Lesson 6: Capstone Integration** — Hands-on project synthesizing all chapter concepts into working tool (B1-B2 level, max 9 concepts)

**Key Architectural Principles:**
- Graduated complexity: A2 (beginner) → B1 (intermediate) → B2 (advanced applications)
- Cognitive load limits enforced: A2 max 7 concepts, B1 max 10 concepts per lesson
- Capstone project mandatory: Students build production-quality tool using all chapter concepts
- System concepts taught early: Don't defer "advanced" topics (e.g., GC, performance) to later parts if they aid understanding
- Total chapter time: 4.5-5 hours (balanced between depth and completion rate)

**Chapter 19 Reference Implementation:**
- Lesson 1: Set Basics (50 min, A2, 7 concepts)
- Lesson 2: Set Operations (50 min, A2-B1, 7 concepts)
- Lesson 3: Set Internals & Hashing (50 min, B1, 9 concepts)
- Lesson 4: Frozensets (40 min, A2-B1, 7 concepts)
- Lesson 5: Garbage Collection (50 min, B1, 9 concepts)
- Lesson 6: Memory Profiler Capstone (60 min, B1-B2, 9 concepts)

## Consequences

### Positive

- **Cognitive Load Optimized**: Granular lessons (6 vs. 3-4) distribute concepts, preventing overload. A2 lessons max 7 concepts; B1 lessons max 10 concepts.
- **CEFR Proficiency Alignment**: Progressive complexity (A2→B1→B2) matches international language learning standards, enabling competency-based assessment and institutional accreditation.
- **Real-World Synthesis**: Capstone projects (Lesson 6) produce portfolio-worthy tools (Memory Profiler, Type Validator, etc.), demonstrating mastery beyond quiz scores.
- **Reusable Pattern**: Template applies to all Part 4 chapters (13-29), accelerating content creation with consistent student experience.
- **Early System Thinking**: Teaching internals/performance (Lesson 3) and system integration (Lesson 5) builds foundational understanding, not just "how to use" but "how it works."
- **Assessment Validity**: 6-lesson structure provides multiple checkpoints (Try With AI prompts per lesson) + capstone for summative assessment.
- **Accessibility**: Grade 7-8 reading level maintained across lessons; scaffolding reduces barrier to entry for aspiring developers.

### Negative

- **Longer Completion Time**: 6 lessons × 45-60 min = 4.5-5 hours per chapter vs. 2-3 hours for 3-lesson chapters. Part 4 (18 chapters) = 81-90 hours total (students may perceive as "slow").
- **Capstone Support Overhead**: Lesson 6 projects require more instructor scaffolding, debugging support, and AI partnership guidance than standalone exercises.
- **Scope Creep Risk**: Capstone projects can expand beyond 60 minutes if students attempt production-quality features (e.g., GUI for Memory Profiler). Mitigation: Clear specifications with MVP scope.
- **System Concepts Difficulty**: Teaching GC, hashing, performance in Part 4 (intermediate tier) may confuse students expecting "just syntax." Requires careful scaffolding and AI companion support.
- **AI Dependency**: Pattern assumes students have access to AI tools (ChatGPT, Claude Code, Gemini CLI) per Constitution. Without AI, capstone projects become significantly harder.
- **Consistency Burden**: Deviating from 6-lesson pattern for any chapter creates student confusion ("Why is Chapter X different?"). Must maintain pattern or document exceptions.

## Alternatives Considered

### Alternative A: 3-Lesson Dense Architecture

**Structure**: (1) Basics & Operations, (2) Advanced Features & Internals, (3) Practice Exercises

**Tradeoffs:**
- ✓ Faster completion: 2-3 hours per chapter (Part 4 = 36-54 hours total)
- ✓ Fewer lesson boundaries, smoother narrative flow
- ✗ Cognitive overload: 15+ concepts per lesson exceeds A2-B1 limits
- ✗ Steeper learning curve, higher dropout risk
- ✗ No dedicated capstone synthesis

**Why Rejected**: Violates Constitution Principle 12 (Cognitive Load Consciousness). Students at A2-B1 proficiency cannot process 15+ new concepts in one lesson without scaffolding.

### Alternative B: 4-Lesson Balanced Architecture

**Structure**: (1) Basics, (2) Operations, (3) Advanced Features, (4) Practice

**Tradeoffs:**
- ✓ Moderate completion time: 3-4 hours per chapter
- ✓ Balanced cognitive load: 10-12 concepts per lesson
- ✗ No dedicated internals lesson (performance thinking deferred)
- ✗ No system integration lesson (GC, memory management skipped)
- ✗ Practice lesson not integrated capstone (scattered exercises)

**Why Rejected**: Misses opportunity to teach "how Python works" (internals, performance, system integration). Students learn syntax but not system thinking.

### Alternative C: 5-Lesson No-Capstone Architecture

**Structure**: (1) Basics, (2) Operations, (3) Internals, (4) Variants, (5) System Integration

**Tradeoffs:**
- ✓ Comparable time: 4 hours per chapter
- ✓ Covers internals and system concepts
- ✗ No synthesis project (scattered exercises instead)
- ✗ No portfolio-worthy artifact
- ✗ Students don't integrate concepts into working tool

**Why Rejected**: Violates Constitution Principle 10 (Real-World Projects). Scattered exercises don't demonstrate mastery through integrated application.

### Alternative D: Defer System Concepts

**Structure**: Move GC to Chapter 29 (CPython/GIL), Frozensets to advanced topics, focus only on "user-facing" features in Part 4

**Tradeoffs:**
- ✓ Simpler intermediate content (fewer "hard" concepts)
- ✓ Students learn collections usage without system complexity
- ✗ Missed opportunity for early system thinking
- ✗ Students learn "what" but not "why" or "how"
- ✗ Performance misunderstandings (e.g., using lists for lookups instead of sets)

**Why Rejected**: Students benefit from early exposure to system concepts. Understanding GC, hashing, and performance helps them make better design decisions even as beginners. Deferring to Chapter 29 means 13-28 are taught without foundational system knowledge.

## References

- **Feature Spec**: `/specs/001-part-4-chapter-19/spec.md`
- **Implementation Plan**: `/specs/001-part-4-chapter-19/plan.md`
- **Related ADRs**:
  - ADR-0006: 5-Lesson Operator Separation Cognitive Load Pattern (Chapter 14 Data Types pattern; Chapter 19 extends to 6 lessons)
  - ADR-0001: Two-Climax Pedagogical Structure for Chapter 32 (Part 5 uses different pattern for conceptual chapters)
- **Constitution**:
  - Principle 10: Real-World Projects (capstone requirement)
  - Principle 12: Cognitive Load Consciousness (concept limits per lesson)
  - Principle 13: Graduated Teaching Pattern (scaffolding: Book → AI Companion → AI Orchestration)
- **Research Foundation**:
  - CEFR (Common European Framework of Reference for Languages) - 40+ years of proficiency research
  - Bloom's Taxonomy (1956/2001) - Cognitive complexity levels
  - Cognitive Load Theory (Sweller, 1988) - Working memory limits
