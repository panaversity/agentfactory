# Implementation Tasks: Chapter 31 (SIMPLIFIED)

**Version**: 2.0 (Simplified) | **Feature**: 10-chapter-31-redesign | **Date**: 2025-11-05

---

## Overview

Chapter 31 implementation organized into **8 major milestones** (one per lesson) + **2 support phases** (setup + validation).

**Key Change from Old Tasks**: Focused on milestones, not micro-tasks. Lesson-writer subagent handles detailed breakdown.

**Total Estimated Time**: 16-20 hours

---

## Phase 1: Setup & Foundation (2-3 hours)

### Milestone 1.1: Environment Ready
- [ ] Create chapter directory: `book-source/docs/05-Spec-Driven-Development/31-spec-kit-plus-hands-on/`
- [ ] Verify branch: `claude/what-skill-011CUpWbAZvCt9xZdMJj3MEu`
- [ ] Create chapter README with overview + 8-lesson structure

**Acceptance Criteria**:
- ✓ Directory exists with correct path
- ✓ Branch is active
- ✓ README references Constitution v3.0.1 principles

---

## Phase 2: Lesson Implementation (12-15 hours)

### Milestone 2.1: Lesson 1 — Installation & Setup (1.5-2 hrs)

**Invoke**: content-implementer subagent with simplified plan context

**Deliverables**:
- `01-installation-and-setup.md` complete
- Installation verification steps
- Tool configuration guide (Claude Code OR Gemini CLI)
- "Try With AI" activity

**Acceptance Criteria**:
- ✓ Clear installation instructions for both AI tools
- ✓ Horizontal + Vertical Intelligence explained (5 concepts max, A2 level)
- ✓ Working project structure verification
- ✓ Proficiency: A2 | Bloom's: Remember + Understand

---

### Milestone 2.2: Lesson 2 — Constitution Phase (1.5-2 hrs)

**Invoke**: content-implementer subagent

**Deliverables**:
- `02-constitution-phase.md` complete
- Constitution template for calculator
- Best practice workflow (Constitution → Commit → Features)
- "Try With AI" activity

**Acceptance Criteria**:
- ✓ Constitution vs Specs distinction clear
- ✓ One-time creation explained
- ✓ Example Constitution with 7 standards (quality, testing, error-handling, etc.)
- ✓ Proficiency: A2 | Bloom's: Understand + Apply

---

### Milestone 2.3: Lesson 3 — Specify Phase (2-2.5 hrs)

**Invoke**: content-implementer subagent

**Deliverables**:
- `03-specify-phase.md` complete
- **Evals as Collaboration** section (critical!)
- Complete calculator specification example
- "Try With AI" activity

**Acceptance Criteria**:
- ✓ **Evals taught as pre-spec human-AI discussion** (NOT formal phase)
- ✓ Example: "I want calculator" → AI asks clarifying questions → discussion → THEN `/sp.specify`
- ✓ SMART criteria framework applied
- ✓ 5 operations specified (add, subtract, multiply, divide, power)
- ✓ Edge cases identified (division by zero, type preservation, negative exponents)
- ✓ Proficiency: A2→B1 | Bloom's: Understand + Apply

**Critical Teaching Point**: "Evals = talk with AI about success BEFORE formalizing spec"

---

### Milestone 2.4: Lesson 4 — Clarify Phase (1.5-2 hrs)

**Invoke**: content-implementer subagent

**Deliverables**:
- `04-clarify-phase.md` complete
- `/sp.clarify` workflow demonstration
- Iteration examples (spec v1 → v2 → v3)
- "Try With AI" activity

**Acceptance Criteria**:
- ✓ `/sp.clarify` command usage clear
- ✓ Gap analysis demonstrated (ambiguous terms, missing assumptions, untestable criteria)
- ✓ Iterative refinement cycle shown
- ✓ Cascade improvement explained (better spec → better plan)
- ✓ Proficiency: B1 | Bloom's: Apply + Analyze

---

### Milestone 2.5: Lesson 5 — Plan Phase + ADRs (2-2.5 hrs)

**Invoke**: content-implementer subagent

**Deliverables**:
- `05-plan-phase.md` complete
- `/sp.plan` workflow demonstration
- ADR creation with `/sp.adr <title>`
- 2-3 example ADRs (error handling, type system, operation patterns)
- "Try With AI" activity

**Acceptance Criteria**:
- ✓ `/sp.plan` generates architecture from spec
- ✓ Plan structure explained (phases, dependencies, milestones)
- ✓ ADR creation taught (when/why to document decisions)
- ✓ Example ADRs for calculator (error handling strategy, type preservation approach)
- ✓ Proficiency: B1 | Bloom's: Apply + Analyze

---

### Milestone 2.6: Lesson 6 — Tasks Phase (1.5-2 hrs)

**Invoke**: content-implementer subagent

**Deliverables**:
- `06-tasks-phase.md` complete
- `/sp.tasks` workflow demonstration
- **Checkpoint pattern teaching** (critical!)
- Task dependency visualization
- "Try With AI" activity

**Acceptance Criteria**:
- ✓ `/sp.tasks` decomposes plan into atomic tasks
- ✓ **Checkpoint pattern explained**: Agent completes Phase 1 → Human reviews/commits → Agent continues
- ✓ **Human control emphasized**: YOU decide when to proceed; AI doesn't run autonomously
- ✓ Atomic task definition (1-2 hour, clear acceptance criteria)
- ✓ Dependency graph example
- ✓ Lineage traceability (spec → plan → task)
- ✓ Proficiency: B1 | Bloom's: Apply + Analyze

**Critical Teaching Point**: "You control workflow through checkpoints; AI waits for your review"

---

### Milestone 2.7: Lesson 7 — Implement + Validate (2.5-3 hrs)

**Invoke**: content-implementer subagent

**Deliverables**:
- `07-implement-validate-phase.md` complete
- `/sp.implement` workflow demonstration
- **PHR auto-creation explanation** (critical!)
- Validation protocol (6 phases)
- "Try With AI" activity

**Acceptance Criteria**:
- ✓ `/sp.implement` orchestrates code generation
- ✓ **Checkpoint-driven implementation**: Implement Phase 1 → pause → review → continue
- ✓ **PHR auto-creation explained**: System creates PHR for each `/sp.implement` session (8-10 typical)
- ✓ Validation checklist (read → understand → test → verify → review)
- ✓ Security review (hardcoded secrets, input validation)
- ✓ AIDD loop with human control (Intent → Generation → Validation → Decision)
- ✓ Working calculator code with all 5 operations
- ✓ Passing tests (80%+ coverage)
- ✓ Proficiency: B1→B2 | Bloom's: Apply + Analyze + Evaluate

**Critical Teaching Point**: "PHRs are automatic; you find them in `history/prompts/` later"

---

### Milestone 2.8: Lesson 8 — Capstone Integration (3-4 hrs)

**Invoke**: content-implementer subagent

**Deliverables**:
- `08-capstone-project.md` complete
- Two project options (temperature converter, unit converter)
- Complete workflow checklist
- Reflection template
- Assessment rubric

**Acceptance Criteria**:
- ✓ Project options clearly specified
- ✓ Complete workflow requirements (Constitution → Spec → Plan → Tasks → Code)
- ✓ Deliverables list (all artifacts required)
- ✓ Reflection prompts (spec quality → code quality cascade)
- ✓ Proficiency: B2 | Bloom's: Create + Evaluate

---

## Phase 3: Reference Updates (1-2 hours)

### Milestone 3.1: Fix Chapter References in Part 5

**Files to Update** (user provided list):
1. `32-real-world-spec-kit-workflows/10-capstone-part-3-reflect-on-scale.md` (lines 342, 391-393)
2. `32-real-world-spec-kit-workflows/08-capstone-part-1-decompose-your-spec.md` (lines 27, 35, 43)
3. `32-real-world-spec-kit-workflows/07-write-your-professional-commitment.md` (line 227)
4. `32-real-world-spec-kit-workflows/04-see-how-specs-flow-through-everything.md` (lines 95, 297)

**Changes**:
- Replace "Chapter 25" → "Chapter 30"
- Replace "Chapter 26" → "Chapter 31"
- Replace "Chapter 27" → "Chapter 32"

**Acceptance Criteria**:
- ✓ All 5 files updated
- ✓ Cross-references accurate
- ✓ No broken links

---

## Phase 4: Validation & Polish (2-3 hours)

### Milestone 4.1: Technical Review

**Invoke**: validation-auditor subagent

**Checks**:
- ✓ All `/sp.*` commands accurate (no hallucinations)
- ✓ Code examples work (tested with Python 3.13+)
- ✓ Installation instructions valid
- ✓ Calculator project completable

---

### Milestone 4.2: Constitutional Alignment Review

**Manual Check** (use content-evaluation-framework skill):
- ✓ Evals-first philosophy (#1) applied
- ✓ Specification-first methodology (#2) centered
- ✓ Validation-before-trust (#15) emphasized
- ✓ Graduated complexity (A2→B1→B2) maintained
- ✓ Cognitive load limits followed (5/7/10 concepts)
- ✓ Human-in-control framing throughout

---

### Milestone 4.3: Cross-Chapter Coherence

**Checks**:
- ✓ Chapter 30 handoff clear (4 tools → Spec-Kit Plus choice)
- ✓ Chapter 32 preparation (multi-component foundation)
- ✓ No forward references without explanation
- ✓ Consistent terminology (Spec-Kit Plus, not variations)

---

### Milestone 4.4: Publication Ready

- [ ] Docusaurus build test (no errors)
- [ ] All lessons have proper frontmatter
- [ ] Links verified (internal + external)
- [ ] Images/diagrams present and loading
- [ ] "Try With AI" activities in all 8 lessons

**Acceptance Criteria**:
- ✓ `npm run build` succeeds
- ✓ All links resolve
- ✓ Chapter accessible on local preview

---

## Phase 5: Deployment

### Milestone 5.1: Merge to Main

- [ ] Create PR with summary of changes
- [ ] PR description references:
  - Constitution v3.0.1 alignment
  - Evals-first clarification
  - Human control emphasis
  - Simplified plan/tasks
- [ ] Merge when approved

### Milestone 5.2: Live Verification

- [ ] Verify Chapter 31 live on https://ai-native.panaversity.org
- [ ] Test navigation from Chapter 30 → 31 → 32
- [ ] Verify chapter-index.md reflects Part 5 = Chapters 30-33

---

## Success Metrics

**By completion, all of these must be true**:

1. ✓ 8 lessons published with constitutional alignment
2. ✓ Calculator capstone project fully specified
3. ✓ Evals taught as pre-spec collaboration (not formal phase)
4. ✓ Human control emphasized (checkpoint pattern throughout)
5. ✓ PHR auto-creation explained (not manual command)
6. ✓ ADR creation taught in Lesson 5
7. ✓ All Chapter 32 references updated (25-27 → 30-32)
8. ✓ Workflow isomorphism achieved (lessons mirror phases)
9. ✓ Natural progression from Chapter 30 clear
10. ✓ Technical review passed (no hallucinated commands)

---

## Key Differences from Old Tasks

**What's Simplified**:
- ❌ 52 micro-tasks → 15 major milestones
- ❌ Detailed file-by-file breakdown → high-level deliverables
- ❌ Granular acceptance criteria per task → milestone-level validation

**What's Retained**:
- ✓ Phase structure (Setup → Lessons → Validation → Deployment)
- ✓ Critical teaching points flagged
- ✓ Constitutional alignment checkpoints
- ✓ Clear acceptance criteria

**What's Added**:
- ✓ Evals clarification (pre-spec collaboration)
- ✓ Checkpoint pattern emphasis
- ✓ PHR auto-creation teaching
- ✓ Reference update tasks (user-provided list)

---

**Next Step**: Run `/sp.implement` with content-implementer subagent for each lesson milestone.
