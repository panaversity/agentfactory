# ADR-0014: Three Roles Framework Integration (Lessons 3-5)

> **Scope**: Document pedagogical decision to demonstrate Three Roles (AI as Teacher/Student/Co-Worker) explicitly in Lessons 3, 4, 5 with session transcripts and iteration cycles.

- **Status:** Accepted
- **Date:** 2025-01-18
- **Feature:** 001-011-chapter-11-context-engineering-rewrite
- **Context:** Chapter 11 spec requires demonstrating Three Roles framework (AI as Teacher, AI as Student, AI as Co-Worker) in 3+ lessons to prevent convergence toward "AI as passive tool" anti-pattern. Constitution Section IIa mandates bidirectional learning in Stage 2 (AI Collaboration).

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: YES - Core pedagogical methodology affecting how AI collaboration is taught
     2) Alternatives: YES - Could demonstrate in single lesson, or omit explicit role annotations
     3) Scope: YES - Affects content structure for 3 lessons, student learning objectives, assessment criteria
-->

## Decision

**Dedicate Lessons 3, 4, 5 (Application Phase)** to explicit Three Roles demonstrations:

**Lesson 3 (Progressive Loading)**:
- **AI as Teacher**: AI suggests Foundation→Current→On-Demand pattern student didn't know
- **AI as Student**: Student refines AI's generic suggestion with project-specific constraints (custom auth module)
- **AI as Co-Worker**: 3+ iteration rounds converging on optimal loading strategy for THIS codebase

**Lesson 4 (Context Compression)**:
- **AI as Teacher**: AI explains checkpoint structure (decisions + progress + next steps)
- **AI as Student**: Student corrects AI's vague checkpoint with specific architectural decisions and concrete scope
- **AI as Co-Worker**: Iterate on token budget (50 → 1000 → 500 tokens convergence)

**Lesson 5 (Context Isolation)**:
- **AI as Teacher**: AI explains task similarity scoring and pollution risk
- **AI as Student**: Student teaches AI domain boundaries ("endpoints in same module, same session")
- **AI as Co-Worker**: Iterate on isolation criteria (directory-based → framework-based → domain-based convergence)

**Evidence Format**: Session transcripts with explicit role annotations, 3+ iteration rounds documented per lesson

## Consequences

### Positive

- **Three Roles internalized**: Students experience bidirectional learning (not one-way AI→student teaching)
- **Anti-convergence reinforced**: Prevents "AI as passive tool" pattern that dominates generic AI tutorials
- **Convergence demonstrated**: Students see how iteration improves solutions beyond initial AI/human proposals
- **Compliance with Test-003**: Acceptance test requires 3+ lessons demonstrating all three roles (explicitly satisfied)
- **CoLearning methodology validated**: Aligns with constitutional requirement for Stage 2 collaboration
- **Transferable pattern**: Students apply Three Roles in other chapters/projects (not context-engineering-specific)

### Negative

- **Content creation overhead**: Requires authentic session transcripts showing all three roles (cannot fabricate)
- **Transcript length**: 3+ iteration cycles per lesson increase lesson length (may push cognitive load boundaries)
- **Role annotation maintenance**: If AI tools evolve (Claude Code → different interface), transcripts need updating
- **Assessment complexity**: Grading Three Roles demonstrations requires subjective judgment (did student truly teach AI?)
- **Student confusion risk**: Explicit role labeling ("AI as Teacher") might feel artificial, students prefer natural flow

## Alternatives Considered

**Alternative A: Single Consolidated Lesson (Lesson 3 only)**
- Demonstrate all three roles in one comprehensive lesson
- **Pros**: Reduced content overhead, concentrated demonstration
- **Cons**: Violates Test-003 requirement (3+ lessons), insufficient practice time, students don't internalize pattern across contexts
- **Why rejected**: Spec explicitly requires 3+ lessons demonstrating Three Roles

**Alternative B: Implicit Three Roles (No Explicit Annotations)**
- Show AI collaboration naturally without labeling "AI as Teacher/Student/Co-Worker"
- **Pros**: More natural reading flow, less didactic tone
- **Cons**: Students might miss the pedagogical pattern, cannot measure Three Roles compliance objectively, violates spec requirement for "explicitly demonstrated"
- **Why rejected**: Spec Test-003 requires explicit evidence, implicit demonstrations fail measurability criterion

**Alternative C: Three Roles Across All 9 Lessons**
- Demonstrate Three Roles in every lesson (not just 3-5)
- **Pros**: Maximum reinforcement, pervasive pattern
- **Cons**: Stage 1 lessons (1-2) are manual-only (no AI), Stage 4 capstone (9) is spec-writing (different collaboration mode), excessive repetition creates fatigue
- **Why rejected**: Three Roles apply specifically to Stage 2 (AI Collaboration), not appropriate for manual foundation or spec-driven capstone

## References

- Feature Spec: `specs/001-011-chapter-11-context-engineering-rewrite/spec.md` (Test-003, FR-003)
- Implementation Plan: `specs/001-011-chapter-11-context-engineering-rewrite/plan.md` (Lessons 3, 4, 5 detailed plans with role annotations)
- Constitution: `.specify/memory/constitution.md` Section IIa (Layer 2: AI Collaboration - Three Roles Framework)
- Related ADRs: None directly, but aligns with Constitution v6.0.0 reasoning-activated methodology
- Success Criteria: SC-007 (85%+ submissions demonstrate all three roles)
