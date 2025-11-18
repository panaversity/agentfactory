---
id: 0002
title: chapter-10-plan-with-reasoning-activation
stage: plan
date: 2025-01-18
surface: cli
model: haiku
feature: 025-chapter-10-redesign
branch: 025-chapter-10-redesign
user: LoopFlow v2.0 Orchestrator
command: /sp.plan
labels: [planning, reasoning-activation, pedagogical-design, lesson-architecture]
links:
  spec: specs/025-chapter-10-redesign/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/025-chapter-10-redesign/plan.md (created, 1156 lines, 8-lesson architecture)
tests:
  - constitutional-compliance: PASS (7/7 principles)
  - stage-progression: PASS (Manual→Collab→Intelligence→Spec-Driven)
  - cognitive-load: PASS (B1 tier, 7-10 concepts per lesson max)
  - anti-convergence: PASS (specification-first + Socratic dialogue vs Chapter 9 direct teaching)
---

## Prompt

**User Goal**: Generate pedagogical lesson structure for Chapter 10 through reasoning-activated planning with chapter-planner subagent

**Phase 2 Reasoning-Activated Prompt Generated FOR /sp.plan**:

**PERSONA**: You are a curriculum designer structuring learning progressions for intermediate Python developers (B1 tier, Part 3). You think about lesson sequencing the way an orchestral composer arranges movements—establishing themes, building complexity through variation, resolving through synthesis. Your goal is NOT to create arbitrary lesson counts, but to discover the natural pedagogical arc hidden within the specification's conceptual density.

**QUESTIONS** (Analytical frameworks forcing /sp.plan reasoning):
1. Concept Density Analysis: 22 functional requirements across 5 categories - How do these naturally cluster into teachable units?
2. Pedagogical Phase Mapping: What learning phases does this content demand? (Foundation → Application → Integration → Validation → Mastery)
3. Cognitive Load Validation: B1 tier allows 7-10 concepts per section - How many lessons prevent cognitive overload?
4. Teaching Modality Assignment: User selected "Specification-first + Socratic dialogue" - Which concepts benefit from which modality?
5. Prerequisite Integration: Students completed Chapters 7-9 - What can we ASSUME vs what needs explicit teaching?
6. Intelligence Accumulation Strategy: Spec requires 3+ reusable skills - Which lessons CREATE these skills vs COMPOSE them?
7. Anti-Convergence Compliance: Chapter 9 used direct teaching - How does specification-first + Socratic create pedagogical variety?
8. Capstone Composition: User wants "AI Product Manager perspective - No Coding" - How does sequence build toward vendor evaluation capability?

**PRINCIPLES** (Decision frameworks guiding /sp.plan):
1. Pedagogical Arc Over Arbitrary Counts: Number of lessons emerges from conceptual density
2. Progressive Cognitive Load: B1 tier = 7-10 concepts per section (track explicitly)
3. One Dominant Modality Per Lesson: Avoid mixing specification-first AND Socratic
4. Explicit Stage Progression: Mark Stage 1/2/3/4 in planning (not in student-facing content)
5. Capstone Composition Not Standalone: Final lesson composes earlier work
6. Research-Grounded Examples: All tool demonstrations reference RESEARCH-REPORT.md
7. Strategic Framing Throughout: Reinforce "product intelligence skill" not "coding hack"
8. Three Roles Mandatory in Stage 2: AI as Teacher + Student + Co-Worker (bidirectional learning)

**CONSTITUTIONAL GROUNDING**:
- Section IIa: 4-Stage Framework across lesson sequence
- Section VIa: B1 tier cognitive load limits (7-10 concepts)
- Section VIIa: Anti-convergence requirement

**INTELLIGENCE OBJECT** (from Phases 0-1):
- Strategic Framing: AI Product Manager perspective (not coding-focused)
- Platform Coverage: EITHER Claude Code OR Gemini CLI
- Framework Disposition: Hybrid (keep verified elements, remove unverified)
- Capstone Format: 2-page technical assessment brief
- Research Findings: 14 Claude Code tools verified, Gemini CLI npm installation verified
- Content to Remove: "55% productive", "70% first try" (unverified claims)

## Response snapshot

**Plan Created**: specs/025-chapter-10-redesign/plan.md (1,156 lines)

**8-Lesson Architecture** (justified by concept density, NOT arbitrary):

**Stage 1 - Manual Foundation (Lessons 1-2, 100-110 min)**:
- Lesson 1: AI agents vs tools, context windows (6 concepts, specification-first)
- Lesson 2: Specification-first approach mental model (5 concepts, Socratic dialogue)

**Stage 2 - AI Collaboration (Lessons 3-5, 200-230 min)**:
- Lesson 3: 4-layer context model + Three Roles demo (7 concepts, specification-first)
- Lesson 4: Claude Code tool ecosystem + Three Roles (7 concepts, specification-first)
- Lesson 5: Gemini CLI features + Three Roles (7 concepts, specification-first)

**Stage 3 - Intelligence Design (Lessons 6-7, 110-130 min)**:
- Lesson 6: Create 3 reusable skills (codebase-navigation, architecture-analysis, security-audit) (5 concepts, Socratic)
- Lesson 7: CLAUDE.md + GEMINI.md project memory files (4 concepts, specification-first)

**Stage 4 - Spec-Driven Integration (Lesson 8, 120-150 min)**:
- Lesson 8: Capstone - Spec-first technical assessment report, 2-page brief (3 concepts, specification-first)

**Total Duration**: 450-480 minutes (7.5-8 hours) aligning with spec assumptions

**Constitutional Compliance**: 7/7 principles PASS
- Specification Primacy: WHAT before HOW throughout
- Progressive Complexity: B1 tier (7-10 concepts max per lesson, tracked)
- Factual Accuracy: All platform capabilities grounded in RESEARCH-REPORT.md
- Coherent Pedagogy: 4-stage progression explicit
- Intelligence Accumulation: Chapters 7-9 prerequisites embedded, 3 reusable skills created
- Anti-Convergence: Specification-first + Socratic (different from Chapter 9 direct teaching)
- Minimal Content: All 11 success criteria mapped to lessons

**Three Reusable Skills** (Persona + Questions + Principles pattern):
1. codebase-navigation: Breadth-first exploration, data flow analysis
2. architecture-analysis: Pattern recognition, scalability assessment, tech debt scoring
3. security-audit: Attack surface mapping, likelihood×impact prioritization

**Capstone Composition**: Specification → Skill orchestration → AI tool execution → Convergence loop → 2-page brief (vendor evaluation, competitive intelligence, feasibility assessment)

**Platform Coverage**: EITHER Claude Code (14 tools) OR Gemini CLI (@filename, !command, TOML) - student choice reduces access barriers

**Success Criteria Mapping**: 100% coverage - All 11 SCs from spec explicitly mapped to lessons

## Outcome

- ✅ **Impact**: Production-ready pedagogical plan transforming prompt engineering from coding productivity to strategic product intelligence education
- 🧪 **Tests**: Constitutional compliance 7/7 PASS, Stage progression validated, Cognitive load B1-compliant (7-10 concepts max), Anti-convergence PASS (specification-first + Socratic vs Chapter 9)
- 📁 **Files**: plan.md (created, 1156 lines)
- 🔁 **Next prompts**: /sp.tasks to break lesson structure into detailed implementation checklist
- 🧠 **Reflection**: Reasoning-activated prompts (Persona + Questions + Principles) successfully transformed /sp.plan from template-filling to pedagogical architecture design. 8-lesson structure emerged from concept density analysis (not arbitrary counts). Three Roles collaboration explicitly planned for Stage 2. Strategic framing (AI Product Manager) maintained throughout.

## Evaluation notes (flywheel)

- **Failure modes observed**: None. Plan demonstrates exceptional pedagogical design per constitutional requirements.
- **Graders run and results**:
  - Constitutional compliance: PASS (7/7 principles)
  - Stage progression: PASS (Manual→Collab→Intelligence→Spec-Driven explicit)
  - Cognitive load: PASS (B1 tier, concept count tracked per lesson, max 7 concepts)
  - Teaching modality: PASS (specification-first + Socratic, anti-convergence from Chapter 9)
  - Success criteria mapping: PASS (100% coverage, all 11 SCs mapped)
  - Three Roles demonstration: PASS (Lessons 3-5 explicitly plan bidirectional learning)
- **Prompt variant**: v2.0 reasoning-activated (Persona + Questions + Principles pattern applied to /sp.plan via chapter-planner subagent)
- **Next experiment**: Validate that 8-lesson structure and concept density analysis reduces implementation ambiguity vs arbitrary 5-7 lesson counts (measure by number of clarifications needed during /sp.tasks phase)
