# ADR-0013: Nine-Lesson Structure for Context Engineering Chapter

> **Scope**: Document pedagogical architecture decision for Chapter 11 lesson count and phase structure.

- **Status:** Accepted
- **Date:** 2025-01-18
- **Feature:** 001-011-chapter-11-context-engineering-rewrite
- **Context:** Chapter 11 Context Engineering rewrite requires determining optimal lesson count to balance concept coverage (7 major concepts) with B1 tier cognitive load limits (7-10 concepts per lesson) while applying 4-stage teaching framework (Stage 1→2→3→4) and hands-on discovery modality.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: YES - Affects entire chapter structure, content-implementer execution, student learning experience
     2) Alternatives: YES - Could use 6-lesson (compressed), 12-lesson (granular), or variable structure
     3) Scope: YES - Cross-cutting decision affecting all lessons, pedagogical phases, cognitive load management
-->

## Decision

**Nine-lesson pedagogical structure** organized in five phases:
- **Foundation Phase (Lessons 1-2)**: Stage 1 manual foundation - Context windows, degradation symptoms, manual tracking
- **Application Phase (Lessons 3-5)**: Stage 2 AI collaboration - Progressive loading, compression, isolation with Three Roles demonstrated
- **Integration Phase (Lessons 6-7)**: Stage 3 intelligence design - Memory files, tool selection, reusable skills creation
- **Validation Phase (Lesson 8)**: Stage 2 application - Hands-on debugging/optimization scenarios
- **Mastery Phase (Lesson 9)**: Stage 4 spec-driven capstone - Complete tool specification WITHOUT implementation

**Concept Distribution**: 6-9 concepts per lesson (within B1 tier limit of 10)

## Consequences

### Positive

- **Cognitive load managed**: Each lesson stays within 6-9 concepts, well below B1 tier limit of 10
- **Stage progression explicit**: All 4 stages represented (Stage 1 foundation, Stage 2 collaboration, Stage 3 intelligence, Stage 4 capstone)
- **Three Roles coverage**: 3 dedicated lessons (3, 4, 5) demonstrate complete Three Roles cycle
- **Hands-on discovery enabled**: 9 lessons provide sufficient time for experiment→observe→learn sequences without rushed delivery
- **Reusable intelligence creation**: Stage 3 lessons (6-7) create 3 reusable skills (memory-file-architecture, tool-selection-framework, progressive-loading-strategy)
- **Capstone orchestration**: Dedicated Lesson 9 for spec-driven integration without implementation pressure

### Negative

- **Chapter length**: 9 lessons may feel longer than typical 6-7 lesson chapters, potential student perception of "too much content"
- **Maintenance burden**: More lessons = more content to maintain, update when tools evolve (context window specs change)
- **Pacing risk**: If lessons are poorly executed, 9 lessons could become repetitive rather than progressive
- **Assessment complexity**: 9 independent tests (one per lesson) increases grading overhead for instructors

## Alternatives Considered

**Alternative A: Six-Lesson Compressed Structure**
- Structure: 2 foundation + 2 application + 1 integration + 1 capstone
- **Pros**: Shorter chapter, faster completion, less maintenance
- **Cons**: Violates cognitive load limits (12-14 concepts per lesson to fit all content), insufficient time for hands-on discovery, Three Roles compressed into single lesson (inadequate demonstration)
- **Why rejected**: Cannot maintain B1 tier cognitive load limits with 6 lessons

**Alternative B: Twelve-Lesson Granular Structure**
- Structure: Each concept gets dedicated lesson (context windows, degradation, foundation loading, current loading, on-demand loading, compression, isolation, memory files, tool selection, debugging, validation, capstone)
- **Pros**: Maximum concept isolation, easy cognitive load management
- **Cons**: Excessive fragmentation, lessons become too narrow (3-4 concepts each = underutilized learning time), students lose sight of integrated workflows, chapter feels artificially padded
- **Why rejected**: Over-segmentation reduces pedagogical coherence, students need to see concepts integrated not atomized

**Alternative C: Variable Lesson Structure (6-9 lessons based on student feedback)**
- Structure: Start with 6 lessons, add optional "deep dive" lessons (7-9) for advanced students
- **Pros**: Adaptive to student needs, core path shorter
- **Cons**: Fragmented learning path (which students skip which lessons?), Three Roles might end up in "optional" content (violates requirements), spec mandates fixed structure for acceptance tests
- **Why rejected**: Specification requires all 4 stages and Three Roles demonstrations, cannot make these optional

## References

- Feature Spec: `specs/001-011-chapter-11-context-engineering-rewrite/spec.md`
- Implementation Plan: `specs/001-011-chapter-11-context-engineering-rewrite/plan.md` (lines 38-49: Pedagogical Phases)
- Related ADRs: ADR-0008 (11-lesson collections structure for A2-B1 learners provides precedent)
- Constitutional Principle: Principle 2 (Progressive Complexity) - B1 tier cognitive load limits
- Spec Justification: Lines 446-447 "Estimated Lesson Count: 9 lessons" with concept density analysis
