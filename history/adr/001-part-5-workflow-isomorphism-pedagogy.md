# ADR-001: Part 5 Workflow Isomorphism Pedagogical Pattern

> **Scope**: Document pedagogical architecture decision for Part 5 (Spec-Kit Plus Methodology). This decision clusters lesson structure, project complexity, and workflow sequence to create authentic learning experience.

- **Status:** Accepted
- **Date:** 2025-11-05
- **Feature:** Part 5 Spec-Kit Plus Methodology (Chapters 30-32)
- **Context:** Part 5 teaches professional Spec-Kit Plus workflow. Must choose between theory-heavy approach vs. hands-on practice; simple exercises vs. sustained capstone; fragmented topics vs. integrated workflow.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: YES - Determines how thousands of learners experience Spec-Kit Plus methodology; affects all Part 5 chapters
     2) Alternatives: YES - Multiple pedagogical sequences evaluated (see below)
     3) Scope: YES - Cross-cutting concern affecting lesson structure, assessment strategy, project design across 4 chapters
-->

## Decision

**Adopt Workflow Isomorphism Pattern**: Structure all Spec-Kit Plus lessons to mirror actual workflow phases in sequence:

1. **Installation & Setup** → 2. **Constitution Phase** → 3. **Specify Phase** → 4. **Clarify Phase** (refinement) → 5. **Plan Phase** → 6. **Tasks Phase** → 7. **Implement Phase** → 8. **Capstone Integration**

**Project Complexity**: Use calculator capstone project (5 operations: add, subtract, multiply, divide, power) as sustained learning artifact. Goldilocks complexity: not too simple (justifies professional workflow) nor too complex (avoids overwhelming learners).

**Delivery Strategy**:
- Each lesson = one actual workflow phase (not theory about multiple phases)
- Students build ONE complete project end-to-end (not multiple disconnected exercises)
- Generate authentic artifacts: Constitution, Spec, Plan, Tasks, ADRs (2-3), PHRs (8-10)
- Tier 2→3 transition (Intermediate to Advanced: A2→B1→B2 proficiency)

## Consequences

### Positive

- **Authentic Learning**: Students practice REAL workflow, not theoretical simulation
- **Cognitive Coherence**: Lesson structure matches actual process; no mental translation needed
- **Cumulative Achievement**: Single sustained project builds confidence and mastery
- **Natural Artifact Creation**: Decisions genuinely need ADRs; AI interactions naturally produce PHRs
- **Foundation for Team Workflows**: Chapter 32 can build on established workflow for parallelization patterns
- **Domain Familiarity**: Calculator operations universally understood; no domain learning curve
- **Professional Readiness**: Graduates understand complete SDD cycle from Constitution to Validation

### Negative

- **Rigid Sequence**: Students must complete phases in order; can't skip ahead to "interesting parts"
- **Single Project Limitation**: One calculator example may not cover all workflow variations
- **Complexity Constraint**: Must keep calculator simple enough for 13-15 hour chapter scope
- **Pacing Challenge**: Slower learners may struggle with Phase 5-7 complexity ramp (A2→B1→B2)
- **Repetition for Advanced Learners**: Professional developers may find Phases 1-3 too basic

## Alternatives Considered

### Alternative A: Theory-First Approach
**Structure**: Lessons teach concepts (specifications, planning, tasks) with small disconnected exercises per topic.

**Why Rejected**:
- Theory-heavy; students don't experience workflow cohesion
- Fragmented learning; no sustained practice building complete projects
- ADRs/PHRs feel artificial (no genuine decisions to document)
- Doesn't prepare for Chapter 32 (team workflows)

### Alternative B: Multiple Small Projects
**Structure**: 3-4 small projects (todo list, calculator, quiz system); each demonstrates one workflow phase.

**Why Rejected**:
- Fragmented learning; students don't see cascade effect (spec quality → code quality)
- Setup overhead for each project (Constitution, context)
- No cumulative achievement; feels like disconnected exercises
- Harder to demonstrate PHR accumulation (each project = fresh start)

### Alternative C: Enterprise LMS (Large Project)
**Structure**: Complete Learning Management System with authentication, courses, assignments, grading as capstone.

**Why Rejected**:
- Too complex for 13-15 hour scope; violates graduated progression
- Domain complexity adds cognitive load (LMS business logic)
- Professional developers might expect production features beyond scope
- Maintenance burden (keeping example current with real LMS patterns)

### Alternative D: Workflow Tutorial (No Hands-On)
**Structure**: Walk through example project showing workflow steps; students observe, don't build.

**Why Rejected**:
- Passive learning; no hands-on practice
- Students don't generate artifacts (no Constitution, Spec, Plan, Tasks)
- Can't assess mastery (no deliverables to evaluate)
- Doesn't build confidence ("I watched someone else do it")

## References

- Feature Spec: `specs/010-chapter-31-redesign/spec.md`
- Implementation Plan: `specs/010-chapter-31-redesign/plan.md`
- Related ADRs: ADR-002 (Progressive Proficiency Scaffolding), ADR-003 (Human Control & Checkpoint Pattern)
- Chapter Index: `specs/book/chapter-index.md` (Part 5: Chapters 30-32)
- Evaluator Evidence: PHR-0007 (`history/prompts/010-chapter-31-redesign/0007-revise-chapter-31-plan-human-control-checkpoints.plan.prompt.md`)

## Why This Won

1. **Goldilocks Complexity**: Calculator justifies professional SDD workflow without overwhelming learners
2. **Workflow Isomorphism**: Lesson structure = actual process; no abstraction gap
3. **Cumulative Achievement**: Single sustained project builds confidence and mastery
4. **Natural ADR/PHR Creation**: Grading calculator design decisions genuinely need documentation
5. **Chapter 32 Foundation**: Multi-phase workflow enables parallelization lesson (coming next)
6. **Domain Familiarity**: Everyone understands calculator operations; no domain learning curve
7. **International Standards Alignment**: CEFR proficiency levels enable institutional accreditation

## Decision History

- **2025-11-05**: ADR created documenting pedagogical architecture for Part 5
- **2025-11-03**: Initial specification for Chapter 31 redesign created
- **2025-10-31**: Constitution v3.0.0 established Spec-Kit Plus as core methodology
