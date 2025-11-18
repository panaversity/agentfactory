# ADR-0015: Hands-On Discovery Pedagogy (Anti-Convergence from Chapter 10)

> **Scope**: Document pedagogical modality decision for Chapter 11 teaching approach to prevent convergence toward monotonous lecture-style delivery.

- **Status:** Accepted
- **Date:** 2025-01-18
- **Feature:** 001-011-chapter-11-context-engineering-rewrite
- **Context:** Constitution Principle 6 (Anti-Convergence) requires varying teaching modalities across consecutive chapters. Chapter 10 used Direct Teaching (explain → demonstrate → practice). Chapter 11 must use different modality to prevent pedagogical homogenization and student fatigue.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: YES - Affects how all 9 lessons are structured, content flow, student engagement
     2) Alternatives: YES - Could use case-study-based, project-based, or maintain direct teaching
     3) Scope: YES - Cross-cutting decision affecting content creation workflow, learning sequences, assessment design
-->

## Decision

**Hands-On Discovery Modality** applied across all 9 lessons:

**Pattern**: Experiment → Observe → Learn (students discover patterns through practice BEFORE receiving explanatory frameworks)

**Implementation Examples**:
- **Lesson 1**: Manual token estimation exercise → Observe actual vs estimated counts → Learn token counting frameworks
- **Lesson 2**: Transcript analysis marking degradation symptoms → Overlay degradation metrics → Name symptoms and introduce mitigation frameworks
- **Lessons 3-5**: Progressive loading practice → Track context utilization and AI quality → Formalize as Foundation→Current→On-Demand pattern
- **Lesson 8**: Session diagnosis scenarios → Apply learned patterns → Validate remediation effectiveness

**NOT Used**: Direct Teaching (explain → demonstrate → practice) as in Chapter 10

## Consequences

### Positive

- **Anti-convergence achieved**: Different modality from Chapter 10 prevents teaching pattern homogenization (Test-014 compliance)
- **Deeper retention**: Discovery learning produces stronger mental models than passive information transfer
- **Intrinsic motivation**: Students experience "aha moments" through experimentation, not told facts upfront
- **Transfer learning**: Discovery-based patterns transfer better to novel contexts (students learn HOW to discover, not just WHAT was discovered)
- **Alignment with B1 tier**: Intermediate students have prerequisite knowledge (Chapter 10) enabling independent exploration
- **Reduces cognitive overload**: Concepts introduced when needed (just-in-time) rather than front-loaded theory dump

### Negative

- **Content creation overhead**: Requires designing authentic experiments, scaffolding discovery sequences, providing observation tools (harder than writing explanations)
- **Time intensive**: Discovery sequences take longer than direct teaching (9 lessons accommodate this, but chapter is longer)
- **Risk of incorrect discoveries**: Students might discover wrong patterns without proper scaffolding (requires careful exercise design)
- **Assessment complexity**: Must validate discovery process, not just final understanding (Did student discover pattern or memorize it?)
- **Instructor facilitation burden**: If used in classroom setting, requires skilled facilitation to guide discovery without revealing answers prematurely

## Alternatives Considered

**Alternative A: Direct Teaching (Same as Chapter 10)**
- Explain concepts upfront → Demonstrate with examples → Practice with exercises
- **Pros**: Faster content delivery, predictable structure, easier to create
- **Cons**: Violates Constitution Principle 6 (Anti-Convergence), creates monotonous lecture-style experience, Test-014 failure
- **Why rejected**: Explicit spec requirement for anti-convergence from Chapter 10's modality

**Alternative B: Case-Study-Based Learning**
- Present real-world context engineering failures → Analyze what went wrong → Extract principles
- **Pros**: Authentic scenarios, problem-solving focus, different from Chapter 10
- **Cons**: Passive analysis (not hands-on), students observe rather than experience, less engagement than discovery
- **Why rejected**: Less active learning than hands-on discovery, students need to FEEL degradation symptoms not just read about them

**Alternative C: Project-Based Learning (Build Context-Aware Tool)**
- Students implement full context management system throughout chapter
- **Pros**: Maximum hands-on engagement, integrated capstone, authentic product
- **Cons**: Violates spec constraint (capstone is spec-only, NO implementation per user requirement), B1 tier students lack implementation skills for production tool
- **Why rejected**: Spec explicitly prohibits implementation ("no programming" in capstone), project-based would require coding throughout

**Alternative D: Hybrid (Discovery + Direct Teaching)**
- Alternate between discovery lessons (1, 3, 5, 8) and direct teaching lessons (2, 4, 6, 7, 9)
- **Pros**: Balances depth (discovery) with efficiency (direct teaching), varied pace
- **Cons**: Inconsistent student experience, doesn't fully commit to anti-convergence principle, students confused by modality switching mid-chapter
- **Why rejected**: Spec requires chapter-level anti-convergence, not lesson-level variation

## References

- Feature Spec: `specs/001-011-chapter-11-context-engineering-rewrite/spec.md` (FR-009, Test-005, Test-014)
- Implementation Plan: `specs/001-011-chapter-11-context-engineering-rewrite/plan.md` (Discovery Sequence sections in each lesson)
- Constitution: `.specify/memory/constitution.md` Principle 6 (Anti-Convergence Variation)
- User Clarification: Teaching modality selected as "Hands-on discovery" during Phase 1 clarification questions
- Chapter 10 Reference: `book-source/docs/03-Markdown-Prompt-Context-Engineering/10-prompt-engineering-for-aidd/` (Direct Teaching modality)
