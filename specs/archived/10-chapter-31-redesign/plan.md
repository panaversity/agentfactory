# Chapter 31 Implementation Plan: Spec-Kit Plus Hands-On (SIMPLIFIED)

**Version**: 2.0 (Simplified) | **Date**: 2025-11-05 | **Feature**: 10-chapter-31-redesign

---

## I. Executive Summary

**What We're Building**: Chapter 31 teaches practical Spec-Kit Plus workflow through hands-on calculator project.

**Key Transitions**:
- From Chapter 30's "why SDD matters" → Chapter 31's "how to do SDD"
- From manual spec-writing → tool-assisted workflow with `/sp.*` commands
- From understanding tools → practicing chosen tool (Spec-Kit Plus)

**Core Pedagogical Approach**:
- **Human in Control**: Students drive workflow; AI orchestrator is collaborator
- **Evals as Collaboration**: Initial human-AI discussion about success BEFORE formal `/sp.specify`
- **Workflow Isomorphism**: Each lesson = one actual workflow phase
- **Checkpoint-Driven**: Agent works → human reviews → human decides next step
- **Calculator Capstone**: Complete 5-operation calculator (add, subtract, multiply, divide, power)

**Target Outcome**: Students complete full Spec-Kit Plus workflow end-to-end with working validated code.

---

## II. Learning Progression

### Complexity Tiers
- **Lessons 1-2**: A2 (Setup + Constitution) - Procedural, max 5 concepts
- **Lessons 3-4**: A2→B1 (Specify + Clarify) - Basic application, max 5-7 concepts
- **Lessons 5-6**: B1 (Plan + Tasks) - Independent application, max 7 concepts
- **Lesson 7**: B1→B2 (Implement + Validate) - Analysis + evaluation, max 10 concepts
- **Lesson 8**: B2 (Capstone) - Creative synthesis

### Total Duration: **13-15 hours**

---

## III. 8-Lesson Structure (Strategic Overview)

### **Lesson 1: Installation & Setup** (1.5 hrs | A2)

**Goal**: Working Spec-Kit Plus installation with AI tool configured

**Key Concepts** (5):
1. Spec-Kit Plus = independent framework (works with Claude Code or Gemini CLI)
2. Horizontal Intelligence = ADRs + PHRs capture organizational knowledge
3. Vertical Intelligence = Human → Orchestrator → Subagents
4. AI tool options (Claude Code vs Gemini CLI)
5. Project structure (`.specify/`, `specs/`, `history/`)

**Deliverable**: Installed framework + configured AI tool + initialized calculator project + verified commands work

**Skills**: Tool Configuration (A2), Architecture Awareness (A2)

---

### **Lesson 2: Constitution Phase** (1.5 hrs | A2)

**Goal**: Create project Constitution defining calculator quality standards

**Key Concepts** (5):
1. Constitution = global rules (applies to ALL features)
2. One-time creation (not per feature)
3. Constitution vs Specs distinction
4. Cascade starting point (Constitution → Specs → Plans → Code)
5. Best practice: Constitution → Commit → Features

**Deliverable**: Complete `constitution.md` with quality/testing/error-handling standards

**Skills**: Constitution Creation (A2), Global Rules vs Feature Specs (A2)

---

### **Lesson 3: Specify Phase** (2 hrs | A2→B1)

**Goal**: Write complete calculator specification

**Key Concepts** (5):
1. **Evals as Pre-Spec Collaboration**: Discuss success criteria with AI BEFORE formal spec
   - Example: "Calculator should handle edge cases" → AI asks: "Which edge cases?" → Discuss: division by zero, negative exponents, type preservation
   - This conversation informs the specification, not a separate phase
2. Specification components (Overview, Scope, Requirements, Acceptance Criteria, Constraints)
3. SMART criteria (Specific, Measurable, Achievable, Relevant, Time-bound)
4. User stories vs acceptance criteria
5. Edge case identification

**Deliverable**: Complete `specs/calculator/spec.md` with all 5 operations specified

**Skills**: Specification Writing (B1), Requirements Clarity (B1), Evals Collaboration (A2)

**Note on Evals**: This is where students learn evals = initial ideation discussion with AI, not formal phase. Example:
- Student: "I want a calculator"
- AI: "What operations? What edge cases matter? What's success?"
- Student: Discusses and clarifies intent
- Then formally captures in `/sp.specify`

---

### **Lesson 4: Clarify Phase** (1.5 hrs | B1)

**Goal**: Use `/sp.clarify` to refine specification

**Key Concepts** (7):
1. `/sp.clarify` = AI analyzes spec for gaps/ambiguities
2. Iterative refinement cycle
3. Gap categories (missing assumptions, ambiguous terms, incomplete requirements)
4. Cascade improvement (better spec → better plan)
5. Version control for specs
6. AI as partner (surfaces gaps you might miss)
7. Human decides which feedback to act on

**Deliverable**: Refined `spec.md` with ambiguities resolved

**Skills**: Specification Refinement (B1), AI Feedback Interpretation (B1)

---

### **Lesson 5: Plan Phase + ADRs** (2 hrs | B1)

**Goal**: Generate implementation plan and document architectural decisions

**Key Concepts** (7):
1. `/sp.plan` generates HOW from WHAT (spec)
2. Plan structure (phases, dependencies, milestones)
3. Cascade quality (clear spec → clear plan)
4. ADRs = document significant architectural decisions
5. ADR creation with `/sp.adr <title>`
6. When to create ADRs (long-term impact, multiple alternatives, future questioning)
7. Dependencies and sequencing

**Deliverable**: `plan.md` + 2-3 ADRs (e.g., error handling strategy, type system choices)

**Skills**: Plan Generation (B1), Dependency Analysis (B1), ADR Documentation (B1)

---

### **Lesson 6: Tasks Phase** (1.5 hrs | B1)

**Goal**: Decompose plan into atomic work units

**Key Concepts** (7):
1. `/sp.tasks` breaks phases into atomic tasks
2. Atomic task = 1-2 hour completion, clear acceptance criteria
3. **Checkpoint pattern**: Agent completes Phase 1 → Human reviews → Human commits → Agent continues
4. **Human-guided phases**: YOU control progression
5. Task dependencies and sequencing
6. Acceptance criteria per task
7. Lineage traceability (spec requirement → plan phase → task)

**Deliverable**: `tasks.md` with atomic units + dependency graph

**Skills**: Task Decomposition (B1), Lineage Traceability (B1), Checkpoint Management (B1)

**Critical Teaching Point**: Students learn they control workflow; agent doesn't run autonomously

---

### **Lesson 7: Implement + Validate** (2.5 hrs | B1→B2)

**Goal**: AI-driven code generation + systematic validation

**Key Concepts** (10):
1. `/sp.implement` orchestrates code generation
2. **Checkpoint-driven implementation**: Implement Phase 1 → pause → review → continue
3. **Human-guided implementation**: YOU review/test/commit each phase
4. Generated artifacts (code, tests, docs)
5. Validation checklist (read → understand → test → verify → review)
6. Acceptance criteria validation (all must pass)
7. Code comprehension (never accept code you don't understand)
8. **PHR auto-creation**: System automatically creates PHR for `/sp.implement` session
9. Security review (hardcoded secrets, input validation)
10. AIDD loop with human control (Intent → Generation → Validation → Decision)

**Deliverable**: Working calculator code + passing tests + 8-10 auto-created PHRs + validation report

**Skills**: Implementation Orchestration (B1), Validation Mastery (B2), PHR Understanding (A2)

**Critical Teaching Point**: PHRs are auto-created; students just need to know where to find them

---

### **Lesson 8: Capstone Integration** (3-4 hrs | B2)

**Goal**: Complete end-to-end workflow independently

**Project Options**:
- **Simple**: Temperature Converter (Celsius ↔ Fahrenheit ↔ Kelvin)
- **Complex**: Unit Converter System (length, weight, temperature)

**Deliverables**:
- Constitution → Spec → Plan → Tasks → Code → Tests
- 2-3 ADRs + 8-10 auto-created PHRs
- Validation report
- Reflection document (spec quality → code quality cascade)

**Skills**: Complete SDD Workflow (B2), Design Decisions (B2), Independent Application (B2)

---

## IV. Critical Success Criteria (8 Criteria)

1. **Natural Progression from Chapter 30**: Lesson 1 recaps 4 tools; explains why Spec-Kit Plus chosen
2. **Workflow Isomorphism**: Lesson structure mirrors actual workflow phases
3. **Calculator Capstone**: 5 operations (add, subtract, multiply, divide, power) with edge cases
4. **Evals as Collaboration**: Taught as pre-spec human-AI discussion (not formal phase)
5. **Cascade Effect Proven**: Clear spec → clear plan → clear tasks → working code (demonstrated)
6. **Complete Artifact Creation**: Constitution + Spec + Plan + ADRs + PHRs + Code + Tests
7. **Human Control Emphasis**: Students understand THEY drive workflow; AI is collaborator
8. **Checkpoint Pattern Mastery**: Students learn to review → commit → continue (not autonomous loops)

---

## V. Constitutional Alignment (v3.0.1)

### Key Principles Applied:
- ✅ **Evals-First** (Philosophy #1): Taught as pre-spec collaboration discussion
- ✅ **Specification-First** (Philosophy #2): Entire chapter centers on spec-driven workflow
- ✅ **Validation-Before-Trust** (Principle #15): Lesson 7 emphasizes validation as core skill
- ✅ **Planning-First** (Principle #14): Spec → Plan → Tasks → Code sequence enforced
- ✅ **Graduated Complexity** (Principle #12): A2 → B1 → B2 progression with cognitive load limits
- ✅ **Human-in-Control** (Principle #16): Checkpoint pattern throughout; human decides progression

### Cognitive Load Management:
- A2 lessons: Max 5 concepts
- B1 lessons: Max 7 concepts
- B2 lessons: Max 10 concepts
- All validated against CEFR/Bloom's/DigComp standards

---

## VI. Risks & Mitigations

| Risk | Mitigation |
|------|-----------|
| **Students skip manual thinking (L1-3)** | Explicitly contrast manual vs tool-assisted; show tools amplify thinking |
| **Evals misunderstood as formal phase** | Teach as "chat with AI before /sp.specify"; show in action in Lesson 3 |
| **Spec-Kit Plus commands break** | Document version used; emphasize principles over commands |
| **Validation seems tedious** | Frame as non-negotiable professional skill; validation protocol structured |
| **Students blame AI for failures** | Lesson 7 teaches: "If code fails, ask: AI error or spec ambiguity?" |
| **Checkpoint pattern unclear** | Visual diagrams showing: Command → Work → Checkpoint → Review → Decision |

---

## VII. Integration Points

### From Chapter 30:
- Recap 4 tools (Kiro, Spec-Kit, Spec-Kit Plus, Tessel) in Lesson 1
- Reference Chapter 30's calculator exercise (manual spec-writing)
- Transition: Understanding → Practice

### To Chapter 32:
- Chapter 32 teaches team parallelization patterns
- Calculator project (single-component) prepares for multi-component projects
- Workflow mastery enables team coordination

---

## VIII. Implementation Phases

### Phase 1: Content Creation (content-implementer subagent)
- [ ] Lesson 1: Installation & Setup
- [ ] Lesson 2: Constitution Phase
- [ ] Lesson 3: Specify Phase (with evals collaboration)
- [ ] Lesson 4: Clarify Phase
- [ ] Lesson 5: Plan + ADR Phase
- [ ] Lesson 6: Tasks + Checkpoint Phase
- [ ] Lesson 7: Implement + Validate + PHR Phase
- [ ] Lesson 8: Capstone Integration

### Phase 2: Validation
- [ ] Technical review (commands accurate, code works)
- [ ] Pedagogical review (progression clear, assessments aligned)
- [ ] Constitutional alignment check (all principles verified)

### Phase 3: Publication
- [ ] Merge to main
- [ ] Update cross-references
- [ ] Live on site

---

## IX. Key Differences from Old Plan

**What's Simplified**:
- ❌ Removed: Detailed content outlines (save for lesson implementation)
- ❌ Removed: Exact prompts and activities (content-implementer handles)
- ❌ Removed: Full rubrics per lesson (summarized as "Skills Taught")
- ❌ Removed: Extensive examples in plan (examples go in actual lessons)

**What's Retained**:
- ✅ Strategic structure (8 lessons with clear goals)
- ✅ Learning progression (CEFR/Bloom's levels)
- ✅ Key concepts per lesson (5-10 concepts)
- ✅ Deliverables per lesson
- ✅ Risk analysis and mitigation
- ✅ Constitutional alignment validation

**What's Clarified**:
- ✅ **Evals = pre-spec collaboration** (not formal phase)
- ✅ **Human control** emphasized throughout
- ✅ **Checkpoint pattern** explicitly taught
- ✅ **PHR auto-creation** (not manual command)

---

**This plan is ready for:**
1. Tasks generation (`/sp.tasks`)
2. Lesson content creation (content-implementer subagent)
3. Constitutional validation

**Plan Focus**: Strategic decisions, not tactical execution. Lesson-writer handles detailed content.
