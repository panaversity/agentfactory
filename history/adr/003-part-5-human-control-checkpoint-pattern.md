# ADR-003: Part 5 Human Control & Checkpoint Pattern

> **Scope**: Document human-in-control paradigm and checkpoint-driven execution pattern for Part 5 Spec-Kit Plus Methodology. This decision clusters human/AI role definition, checkpoint implementation, and validation-first safety.

- **Status:** Accepted
- **Date:** 2025-11-05
- **Feature:** Part 5 Spec-Kit Plus Methodology (Chapters 30-32)
- **Context:** AI tools can run autonomously for extended workflows, generating large amounts of code without human checkpoints. Must choose between autonomous execution vs. human-guided phases; batch validation vs. incremental review; "magic button" vs. collaborative workflow.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: YES - Determines human/AI collaboration model; affects safety, validation strategy, professional practice alignment for all Part 5
     2) Alternatives: YES - Autonomous AI vs. checkpoint-driven vs. manual-only approaches evaluated
     3) Scope: YES - Cross-cutting concern affecting workflow design, command usage, validation teaching across Part 5
-->

## Decision

**Adopt Human-in-Control with Checkpoint-Driven Execution**: Emphasize human as architect and validator; AI orchestrator as collaborative executor. Implement checkpoint pattern throughout all Spec-Kit Plus workflows:

**Core Paradigm**:
- **Human Role**: YOU are in control; YOU define intent, make decisions, validate outputs, guide workflow
- **AI Orchestrator Role**: Your collaborator; understands context, delegates to specialized subagents per YOUR command
- **Spec-Kit Plus Role**: Opinionated toolkit providing workflow structure; AI tools execute it
- **Critical**: AI is collaborative partner, NOT autonomous agent that runs independently

**Checkpoint Pattern Implementation**:

**Phase Completion Checkpoints** (`/sp.tasks`, `/sp.implement`):
- Agent completes set of tasks (e.g., Phase 1: add, subtract operations)
- Agent returns to YOU for review
- YOU review code, run tests, verify against spec
- YOU commit changes to source control
- YOU decide: continue to next phase OR adjust spec and regenerate

**Command Invocation**:
- Request: "Implement Phase 1 tasks, then pause for my review"
- Agent generates → Returns to YOU → YOU validate → YOU commit → YOU request Phase 2
- NOT: "Implement all tasks" → AI runs autonomously → Returns hours later → Massive code drop

**Atomic Tasks with Human Validation**:
- Tasks are 1-2 hour completions
- Each task has clear acceptance criteria (pass/fail)
- Human validates each task against criteria before proceeding

**Prevents**: Long autonomous loops where AI generates extensive code without human checkpoints.

## Consequences

### Positive

- **Professional Safety**: Mirrors real-world practice; never deploy unvalidated AI code
- **Validation-First Culture**: Builds validation as core skill (Constitution Principle 15)
- **Incremental Confidence**: Small validations build trust; students understand each step
- **Catch Errors Early**: Checkpoint after Phase 1 catches spec ambiguities before Phase 2-3 amplify them
- **Learning Transparency**: Students see AI reasoning phase-by-phase, not black-box batch output
- **Alignment with AIDD**: Intent (YOU) → Generation (AI) → Validation (YOU) → Decision (YOU: accept/refine/reject)
- **Prevents Overwhelming**: Small code drops easier to review than massive batches
- **Source Control Integration**: Natural git workflow (commit after each phase validation)

### Negative

- **Slower Workflow**: Multiple checkpoints take more time than autonomous batch execution
- **Context Switching**: Human must return after each phase; interrupts focus
- **Beginner Friction**: New learners may want "magic button" (autonomous AI); must learn discipline
- **Command Verbosity**: Must explicitly request checkpoints in commands (extra cognitive load)
- **Patience Required**: Students must resist urge to skip validation for speed
- **Coordination Overhead**: Team workflows require explicit checkpoint synchronization

## Alternatives Considered

### Alternative A: Autonomous AI Execution
**Structure**: `/sp.implement` runs all tasks end-to-end without checkpoints; returns complete codebase.

**Why Rejected**:
- Black-box generation; student doesn't understand AI reasoning per phase
- Massive code drop overwhelming to validate (100s of lines)
- Errors cascade (Phase 1 spec ambiguity breaks Phase 2-3)
- Doesn't teach validation-first safety (Constitution Principle 15)
- Misaligns with professional practice (never deploy unvalidated AI code)
- Students learn "magic button" mindset (not collaborative partnership)

### Alternative B: Manual-Only Workflow (No AI)
**Structure**: Students write all specifications, plans, tasks, code manually; no AI assistance.

**Why Rejected**:
- Doesn't teach AI-native development (book's core value proposition)
- Time-intensive; students spend hours on boilerplate
- Misses AI collaboration skills (critical for modern developers)
- Doesn't demonstrate cascade effect (AI-generated code quality from spec quality)
- Can't practice validation of AI outputs (no AI outputs to validate)

### Alternative C: Validation-Only Checkpoints (No Phase Checkpoints)
**Structure**: AI generates all code; human validates at END only (not per phase).

**Why Rejected**:
- Late error detection; Phase 1 spec ambiguities already propagated to Phase 2-3
- Massive validation burden (entire codebase at once)
- Doesn't teach incremental validation (professional practice)
- Missed learning opportunities (students don't see phase-by-phase reasoning)
- Harder to trace failures (which phase introduced the bug?)

### Alternative D: Pair Programming with AI (Continuous Collaboration)
**Structure**: Human writes spec → AI generates line-by-line with human approval per line.

**Why Rejected**:
- Too granular; excessive interruptions (approval every few seconds)
- Doesn't teach phase-based thinking (Constitution, Spec, Plan, Tasks structure)
- Time-prohibitive (40+ hour chapter for 15-hour scope)
- Misses workflow automation benefits (SDD is meant to accelerate work)

## References

- Feature Spec: `specs/010-chapter-31-redesign/spec.md` (Lines 13-21: Core Design Philosophy)
- Implementation Plan: `specs/010-chapter-31-redesign/plan.md`
  - Lines 981-1012: Lesson 6 (/sp.tasks checkpoint pattern)
  - Lines 1127-1162: Lesson 7 (/sp.implement checkpoint pattern)
- Related ADRs: ADR-001 (Workflow Isomorphism), ADR-002 (Progressive Proficiency)
- Constitution: `.specify/memory/constitution.md` (Principle 15: Validation-Before-Trust)
- PHR Evidence: `history/prompts/010-chapter-31-redesign/0007-revise-chapter-31-plan-human-control-checkpoints.plan.prompt.md`

## Why This Won

1. **Professional Safety**: Mirrors real-world practice; validation is non-negotiable
2. **Validation-First Culture**: Builds core skill per Constitution Principle 15
3. **Incremental Confidence**: Small validations build trust; students understand each step
4. **Early Error Detection**: Checkpoint after Phase 1 catches spec ambiguities early
5. **Learning Transparency**: Students see AI reasoning phase-by-phase, not black-box
6. **AIDD Alignment**: Intent → Generation → Validation → Decision loop explicit
7. **Source Control Integration**: Natural git workflow (commit after each validated phase)
8. **Prevents Overwhelming**: Small code drops easier to review than massive batches

## Implementation Examples

### Lesson 6: `/sp.tasks` with Checkpoints

**Student Command**:
```
Generate tasks for Phase 1 (core operations: add, subtract), then pause for my review.
```

**AI Response**:
```
✅ Phase 1 Tasks Generated:
- Task 1.1: Define calculator data structure
- Task 1.2: Implement add() with type hints
- Task 1.3: Implement subtract() with type hints
- Task 1.4: Write tests for add() and subtract()
- Task 1.5: Validate input edge cases

CHECKPOINT: Review these tasks. When ready, request Phase 2 (multiply, divide, power).
```

**Student Reviews → Commits → Continues**:
```
Tasks look good. Generate Phase 2 tasks (multiply, divide, power).
```

### Lesson 7: `/sp.implement` with Checkpoints

**Student Command**:
```
Implement Phase 1 tasks (add, subtract operations), then pause for my review.
```

**AI Response**:
```python
# Generated: calculator.py (Phase 1)
def add(a: float, b: float) -> float:
    """Add two numbers."""
    return a + b

def subtract(a: float, b: float) -> float:
    """Subtract b from a."""
    return a - b

# Generated: test_calculator.py (Phase 1)
def test_add():
    assert add(2, 3) == 5
    assert add(-1, 1) == 0

def test_subtract():
    assert subtract(5, 3) == 2
    assert subtract(0, 0) == 0
```

**CHECKPOINT**: Run tests. Verify against spec acceptance criteria. Commit Phase 1 code.

**Student Reviews → Tests Pass → Commits → Continues**:
```
Phase 1 validated. Continue with Phase 2 (multiply, divide, power).
```

## Teaching Emphasis (All Part 5 Lessons)

**Explicit Messaging**:
- "YOU are in control; AI orchestrator is your collaborator, not autonomous agent"
- "Checkpoints prevent runaway automation; keep YOU in control"
- "Review → Commit → Continue pattern is professional practice"
- "Atomic tasks enable validation; small increments build confidence"

**Safety Framing**:
- "Never deploy code you haven't read and understood"
- "AI is powerful tool but requires human validation"
- "Your job: verify AI understood your intent at each phase"

## Decision History

- **2025-11-05**: ADR created documenting human-in-control pattern for Part 5
- **2025-11-05**: User explicitly required checkpoint pattern emphasis in plan revision (PHR-0007)
- **2025-10-31**: Constitution v3.0.0 established Principle 15 (Validation-Before-Trust)
