# Chapter 35 Implementation Plan: Building Reliable Agents with Google ADK

**Status**: Ready for Content Implementation
**Created**: 2025-12-26
**Specification**: `/specs/046-chapter-35-google-adk/spec.md`
**Constitution**: `.specify/memory/constitution.md` (v7.0.0)

---

## I. Executive Summary

### Chapter Vision

Chapter 35 teaches the discipline of building **production-grade AI agents** — agents that work reliably in enterprise environments, not just in demos.

**Core Philosophy**: "The difference between a demo agent and a production agent is not features — it's reliability engineering."

This chapter is organized around **Three Pillars of Reliable Agents**:

1. **Testable** — Evaluation-first development with `adk eval`
2. **Predictable** — Workflow agents (Sequential, Parallel, Loop) for deterministic orchestration
3. **Safe** — Guardrails and callbacks as core architecture, not afterthoughts

### Target Audience

- **Proficiency Level**: B1 (Intermediate)
- **Prerequisites**: Chapter 33 (Agent Taxonomy), Chapter 34 (OpenAI SDK)
- **Digital FTE Outcome**: Produce deployment-ready, sellable TaskManager agent by Lesson 8 (capstone)

---

## II. Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: google-adk, google-genai (Gemini), pytest
**Storage**: Firestore (production), InMemory (development)
**Testing**: pytest + AgentEvaluator (adk eval)
**Target Platform**: Local development → Vertex AI Agent Engine (production)
**Project Type**: Educational content (lessons 1-8) + runnable examples
**Constraints**: B1 cognitive load (7-10 concepts per lesson max), evaluation-first non-negotiable

---

## III. Constitution Alignment

### Applicable Principles

| Principle | Application |
|-----------|-----------|
| **Specification Primacy** | Write eval specs BEFORE agent code (Lesson 1 essential) |
| **Progressive Complexity** | B1 tier: 7-10 concepts per lesson distributed across 8 lessons |
| **Factual Accuracy** | All ADK APIs verified against SKILL.md + official docs |
| **Coherent Progression** | Foundation (L1) → Application (L2-3) → Integration (L4) → Mastery (capstone) |
| **Intelligence Accumulation** | 6 skills created (Lessons 2-7), composed in Lesson 8 capstone |
| **Anti-Convergence** | Vary teaching modalities (Socratic, hands-on, spec-first, error analysis) |
| **Minimal Content** | Focus on three pillars, exclude comprehensive feature coverage |

### Layer Progression Validation

- [x] **L1 (Manual)**: Lesson 1 (reliability mindset, manual eval case writing, no AI)
- [x] **L2 (AI Collab)**: Lessons 2-5 (Three Roles framework, invisible to students)
- [x] **L3 (Intelligence)**: Lessons 6-7 (skill creation, reusable patterns)
- [x] **L4 (Spec-Driven)**: Lesson 8 (capstone, spec-first, skill composition)

---

## IV. Lesson Architecture Overview

### Concept Density Analysis

**Core Concepts**: 16 total, distributed across 8 lessons

| Pillar | Lesson | Concepts | Load |
|--------|--------|----------|------|
| **Testable** | 1: Mindset | 2 | Low |
|  | 2: Eval-Driven | 3 new | Medium |
| **Predictable** | 3: Sequential/Parallel | 2 new | Medium |
|  | 4: LoopAgent | 1 new | Low |
| **Safe** | 5: Callbacks | 2 new | Medium |
| **Infrastructure** | 6: State Mgmt | 3 new | Medium |
|  | 7: Deployment | 2 new | Medium |
| **Synthesis** | 8: Capstone | 0 new | Integration |

**Justification**: 8 lessons (not arbitrary 9) because concept density naturally divides three pillars, B1 proficiency allows 7-10 concepts per lesson, capstone integrates without new core concepts.

---

## V. Detailed Lesson Structure

### Lesson 1: The Reliability Mindset

**Layer**: 1 (Manual)
**Duration**: 45 min | **Cognitive Load**: 2 concepts
**Teaching Modality**: Socratic Dialogue

**Concepts**:
1. Evaluation-first mindset (tests before code)
2. JSON eval case anatomy (tool_uses specification)

**Core Activity**: Students write 3 eval cases for TaskManager manually (no AI).

**Skills Created**: None (Layer 1 focuses on foundation).

**Assessment**: Students produce valid JSON eval cases testing add_task, list_tasks, complete_task.

---

### Lesson 2: Evaluation-Driven Development

**Layer**: 2 (AI Collaboration)
**Duration**: 60 min | **Cognitive Load**: 3 new + review = 5 total
**Teaching Modality**: Hands-On Discovery (error analysis)

**Concepts**:
1. `adk eval` CLI (running tests, interpreting results)
2. Iterative improvement (failing tests → agent refinement → passing tests)
3. pytest integration (AgentEvaluator, CI/CD automation)

**Three Roles** (INVISIBLE to students):
- AI as Teacher: Suggests tool improvements
- AI as Student: Adapts to user constraints
- AI as Co-Worker: Iterates on agent instruction

**Core Activity**: Build TaskManager agent iteratively, run evals, fix failures, pass all tests.

**Skills Created**:
- `evaluation-driven-development` (Persona: reliability engineer, Framework: test-first workflow)

**Assessment**: Agent code passes 100% of eval cases via `adk eval`.

---

### Lesson 3: Predictable Pipelines

**Layer**: 2 (AI Collaboration)
**Duration**: 50 min | **Cognitive Load**: 2 new concepts
**Teaching Modality**: Specification-First (design workflow in spec, then code)

**Concepts**:
1. SequentialAgent (deterministic order: stage 1 → stage 2 → stage 3)
2. ParallelAgent (concurrent execution with merged results)

**Three Roles** (INVISIBLE):
- AI as Teacher: Explains sequential vs parallel tradeoffs
- AI as Student: Learns workflow requirements
- AI as Co-Worker: Designs appropriate agent type

**Core Activity**: Design researcher → writer → editor pipeline, compare to LLM routing.

**Skills Created**:
- `workflow-agent-selection` (Persona: systems architect, Framework: determinism vs flexibility)

**Assessment**: Students articulate when to use workflow agents vs LLM routing.

---

### Lesson 4: Iterative Quality

**Layer**: 2 (AI Collaboration)
**Duration**: 45 min | **Cognitive Load**: 1 new concept
**Teaching Modality**: Error Analysis (infinite loop → fix via iteration)

**Concepts**:
1. LoopAgent (iterative refinement with exit_loop and max_iterations safety)

**Three Roles** (INVISIBLE):
- AI as Teacher: Explains convergence patterns and safety
- AI as Student: Learns quality criteria specification
- AI as Co-Worker: Iterates on exit conditions

**Core Activity**: Build LoopAgent with quality feedback loop, test convergence behavior.

**Skills Created**:
- `iterative-refinement` (Persona: quality engineer, Framework: fail-safe convergence)

**Assessment**: Agent converges with max_iterations safety, quality criteria met.

---

### Lesson 5: Safety as Architecture

**Layer**: 2 (AI Collaboration)
**Duration**: 55 min | **Cognitive Load**: 2 new concepts
**Teaching Modality**: Hands-On Security (implement attack, then defense)

**Concepts**:
1. `before_model_callback` (input validation/blocking layer)
2. `before_tool_callback` (tool execution restriction layer)
3. Layered defense patterns (defense in depth principle)

**Three Roles** (INVISIBLE):
- AI as Teacher: Explains attack surfaces and defense strategies
- AI as Student: Learns security requirements
- AI as Co-Worker: Refines defense layers iteratively

**Core Activity**: Show attack scenario, implement callbacks to block behavior, verify prevention.

**Skills Created**:
- `agent-safety-architecture` (Persona: security engineer, Framework: defense in depth)

**Assessment**: Students prevent 2+ categories of dangerous behavior architecturally.

---

### Lesson 6: Production State

**Layer**: 3 (Intelligence Design)
**Duration**: 50 min | **Cognitive Load**: 3 infrastructure concepts
**Teaching Modality**: Comparative Analysis (InMemory vs Firestore vs VertexAI)

**Concepts**:
1. SessionService abstraction (pluggable persistence layer)
2. ToolContext (tool access to session state)
3. McpToolset (standard protocol for tool ecosystems)

**Layer 3 Transition**: Students design reusable patterns for state management (no new AI collaboration examples, focus on architectural thinking).

**Core Activity**: Switch from InMemory (dev) to Firestore (production), observe state persistence.

**Skills Created**:
- `production-state-management` (Persona: infrastructure engineer, Framework: abstraction enables flexibility)

**Assessment**: Tools read/write session state, state persists across sessions with Firestore.

---

### Lesson 7: Deployment & Verification

**Layer**: 3 (Intelligence Design)
**Duration**: 55 min | **Cognitive Load**: Deployment architecture + verification
**Teaching Modality**: Step-by-Step Walkthrough (fixed deployment steps with checkpoints)

**Concepts**:
1. Vertex AI Agent Engine deployment (`adk deploy agent_engine`)
2. Deployment verification (eval cases pass remotely, matches local behavior)

**Core Activity**: Deploy TaskManager to Vertex AI, run evals remotely, verify matches local.

**Skills Created**:
- `agent-deployment-verification` (Persona: DevOps engineer, Framework: production readiness)

**Assessment**: Remote eval cases pass, behavior matches local tests.

---

### Lesson 8: Capstone — Multi-Agent Reliable TaskManager

**Layer**: 4 (Spec-Driven Integration)
**Duration**: 120 min (2-hour intensive capstone)
**Cognitive Load**: 0 new concepts (integration/synthesis)
**Teaching Modality**: Specification-First Project

**Structure**:
1. **Phase 1 (30 min)**: Write spec.md FIRST (intent, constraints, success criteria)
2. **Phase 2 (20 min)**: Identify 3+ skills from Lessons 2-7, plan composition
3. **Phase 3 (40 min)**: Implement multi-agent TaskManager using composed skills
4. **Phase 4 (20 min)**: Deploy to Vertex AI, verify evals pass remotely
5. **Phase 5 (10 min)**: Reflect on reliability patterns, discuss monetization as Digital FTE

**Digital FTE Outcome**: Produces deployment-ready, sellable TaskManager agent (specification-driven, all three pillars applied, production-deployable).

**Skills Composed**:
- evaluation-driven-development (Lesson 2)
- workflow-agent-selection (Lesson 3)
- iterative-refinement (Lesson 4)
- agent-safety-architecture (Lesson 5)
- production-state-management (Lesson 6)
- agent-deployment-verification (Lesson 7)

**Assessment**:
- Specification written first (25%)
- Three pillars applied (50%)
- 3+ skills composed (15%)
- Deployed & verified on Vertex AI (10%)

---

## VI. Pedagogical Arc

```
Foundation          Application       Integration    Mastery
─────────           ───────────       ───────────    ───────
Lesson 1: Mindset
  └─ Manual eval    Lesson 2: Eval-Driven
                      └─ AI collab (L2)    Lesson 3: Predictable
                          └─ Sequential    Lesson 4: Iterative
                                          └─ LoopAgent    Lesson 5: Safety
                                                          └─ Callbacks
                                                              Lesson 6: Infrastructure
                                                                └─ Skills (L3)
                                                                    Lesson 7: Deployment
                                                                        └─ Verification
                                                                            Lesson 8: CAPSTONE
                                                                                └─ Digital FTE
```

---

## VII. Three Roles Framework (Invisible Implementation)

**Critical**: Students EXPERIENCE Three Roles but NEVER see framework labels or meta-commentary.

### Implementation Strategy

| Component | Visible to Students | Implementation |
|-----------|-------------------|-----------------|
| Framework labels | ✗ Hidden | Use action prompts, not role names |
| Three Roles | ✓ Experienced | Through iterative collaboration patterns |
| Scaffolding | ✗ Hidden | Transparent pedagogy (no "What to notice") |
| Learning | ✓ Achieved | Through self-reflection + prompt interactions |

### Lesson 2 Example (Proper Implementation)

**CORRECT** (action prompts + self-reflection):
```markdown
## Try With AI: Building by Making Tests Pass

**Part 1: Ask AI**
"My agent isn't calling the right tools. Suggest improvements."

**Part 2: Evaluate**
Review AI's suggestions. Ask yourself:
- Does this improve my agent?
- What assumptions did AI make?
- What constraints did I miss in my instruction?

**Part 3: Refine**
Tell AI: "Actually, we need [constraint]. How should I adjust?"

**Part 4: Iterate**
Run evals again. Do more pass?
```

**WRONG** (meta-commentary):
```markdown
## Three Roles in Action

**Role 1: AI as Teacher**
AI suggests tool improvements (what you didn't know)

**Role 2: AI as Student**
You teach AI your constraints (AI adapts)

**Role 3: AI as Co-Worker**
Convergence through iteration
```

---

## VIII. Skills Library (Layer 3 Outcomes)

Each lesson 2-7 creates ONE reusable skill (Persona + Questions + Principles pattern):

| Lesson | Skill Name | Persona | Key Decision Questions |
|--------|-----------|---------|------------------------|
| 2 | evaluation-driven-development | Reliability engineer | Is behavior testable? Do tests pass? What's failing? |
| 3 | workflow-agent-selection | Systems architect | Is order critical? Can steps run concurrently? Dependencies? |
| 4 | iterative-refinement | Quality engineer | What's the quality metric? When is "good enough"? Prevent infinite loops? |
| 5 | agent-safety-architecture | Security engineer | What attack surfaces? If one layer fails? Architecturally prevented? |
| 6 | production-state-management | Infrastructure engineer | What must persist? Storage tier? Tool access pattern? |
| 7 | agent-deployment-verification | DevOps engineer | Local behavior preserved? Evals passing? Verification checklist? |

**Lesson 8 composes all 6 skills** for capstone.

---

## IX. Cognitive Load Distribution

| Lesson | New Concepts | Total in Lesson | B1 Limit | Status |
|--------|----------|-----------------|----------|--------|
| 1 | 2 | 2 | 10 | ✓ Under |
| 2 | 3 | 5 | 10 | ✓ Under |
| 3 | 2 | 7 | 10 | ✓ Under |
| 4 | 1 | 8 | 10 | ✓ Under |
| 5 | 2 | 10 | 10 | ✓ At limit |
| 6 | 3 | 13 | - | ⚠ Layer 3 (higher autonomy) |
| 7 | 2 | 15 | - | ⚠ Integration lesson |
| 8 | 0 | All | - | ✓ Synthesis (no new) |

**Note on Lessons 6-7**: Slightly higher concept count because they're Layer 3 (intelligence design) and Layer 4 (capstone) where students have higher autonomy. Infrastructure and deployment are naturally grouped concepts.

---

## X. Teaching Modality Variation (Anti-Convergence)

To avoid defaulting to lecture-style, lessons use distinct modalities:

| Lesson | Modality | Why |
|--------|----------|-----|
| 1 | Socratic Dialogue | Mindset shift requires questioning, not telling |
| 2 | Hands-On Discovery | Build by failing, then fixing |
| 3 | Specification-First | Design workflow before implementing code |
| 4 | Error Analysis | Show infinite loop problem, fix via iteration |
| 5 | Hands-On Security | Implement attack, design defense, verify block |
| 6 | Comparative Analysis | InMemory vs Firestore vs VertexAI comparison |
| 7 | Step-by-Step Walkthrough | Deployment has fixed sequential steps |
| 8 | Capstone Project | Integrated synthesis of all approaches |

**No two consecutive lessons repeat modality.** Prevents convergence on familiar patterns.

---

## XI. Assessment Strategy

### Formative Assessments (During Lessons)

| Lesson | Assessment | Success Criteria |
|--------|-----------|------------------|
| 1 | Write 3 eval cases manually | Valid JSON, test meaningful behaviors |
| 2 | Make all eval cases pass | 100% pass rate via `adk eval` |
| 3 | Design workflow agent | Correct agent type, tested deterministically |
| 4 | Build LoopAgent | Converges with max_iterations safety |
| 5 | Implement callbacks | Dangerous behaviors architecturally prevented |
| 6 | Configure SessionService | State persists with Firestore |
| 7 | Deploy to Vertex AI | Remote evals match local evals |

### Summative Assessment (Capstone)

**Rubric** (Lesson 8):
- Specification Quality (25%): Clear intent, constraints, success criteria
- Three Pillars Applied (50%): Testable (eval cases), Predictable (workflow), Safe (callbacks)
- Skills Composition (15%): Uses 3+ skills from prior lessons
- Deployment & Verification (10%): Runs on Vertex AI, evals pass remotely

---

## XII. Content Quality Standards

### Code Examples

All code examples must be:
- [ ] Executed and tested (include pytest output)
- [ ] Verified against current ADK APIs (from SKILL.md reference)
- [ ] Aligned with TaskManager domain (consistency with Chapter 34)
- [ ] Production-appropriate (not toy examples)

### Try With AI Sections

Each lesson includes 3 prompts that:
- [ ] Are copyable in code blocks
- [ ] Include "What you're learning:" explanation
- [ ] Don't expose Three Roles framework
- [ ] Activate different reasoning modes (teach, learn, collaborate)

### Safety Notes

- [ ] Integrated INTO "Try With AI" sections (1-2 contextual sentences)
- [ ] NOT standalone sections at lesson end
- [ ] Reference guardrails where appropriate

---

## XIII. Implementation Handoff

### For Content-Implementer

1. **Reference ADK Skill**: All code must align with `/skills/building-with-google-adk/SKILL.md`
2. **TaskManager Consistency**: Use same domain (add_task, list_tasks, etc.) as Chapter 34
3. **Three Roles Invisibility**: Use action prompts, NEVER meta-commentary
4. **Evaluation-First**: Every lesson includes Try With AI; write specs/evals before code
5. **Skills Composition**: Lessons 2-7 each create ONE reusable skill
6. **Quality Standards**: All code tested, all claims fact-checked, all APIs verified

### For Educational-Validator

1. Verify learning objectives match Bloom's for B1 proficiency
2. Check cognitive load (new concepts per lesson)
3. Validate Three Roles is invisible (no meta-commentary)
4. Confirm assessments map to spec requirements
5. Check layer progression (L1 → L2 → L3 → L4)

### For Technical-Reviewer

1. Execute all code examples (test logs required)
2. Verify ADK APIs against current docs + SKILL.md
3. Fact-check all claims (sources documented)
4. Test TaskManager examples across all lessons
5. Test Lesson 7 deployment against live Vertex AI

---

## XIV. References

**Context Sources**:
- Specification: `/specs/046-chapter-35-google-adk/spec.md`
- Constitution: `.specify/memory/constitution.md` (v7.0.0)
- ADK Skill: `.claude/skills/building-with-google-adk/SKILL.md`
- Chapter 34: OpenAI SDK (for TaskManager pattern baseline)

**Fact-Checking Protocol**: All ADK APIs verified against official Google ADK documentation + SKILL.md reference + test execution.

---

**Status: READY FOR CONTENT IMPLEMENTATION**

This plan provides clear pedagogical direction, cognitive load management, and quality standards. Proceed to content implementation with confidence.

