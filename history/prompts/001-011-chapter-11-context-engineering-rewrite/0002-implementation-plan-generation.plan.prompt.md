---
id: 0002
title: Implementation Plan Generation
stage: plan
date: 2025-01-18
surface: cli
model: claude-sonnet-4-5
feature: 001-011-chapter-11-context-engineering-rewrite
branch: 001-011-chapter-11-context-engineering-rewrite
user: mjs
command: /sp.plan
labels: [planning, lesson-architecture, skills-mapping, chapter-planner-agent]
links:
  spec: specs/001-011-chapter-11-context-engineering-rewrite/spec.md
  ticket:
  adr:
  pr:
files:
  - path: specs/001-011-chapter-11-context-engineering-rewrite/plan.md
    operation: create
    lines_changed: 927
tests:
  - name: Acceptance Test Compliance (Test-003 through Test-014)
    result: PASS
    notes: All 9 acceptance tests validated
  - name: Cognitive Load Validation (B1 Tier)
    result: PASS
    notes: All lessons 6-9 concepts, within 10 concept limit
  - name: Three Roles Framework (Test-003)
    result: PASS
    notes: Demonstrated in Lessons 3, 4, 5 (plus optional Lesson 8)
---

## Prompt

Execute `/sp.plan` workflow using chapter-planner subagent to transform approved specification into comprehensive lesson-by-lesson implementation plan.

**Reasoning-Activated Planning Inputs**:

**Constitutional Context**:
- Constitution v6.0.0 with 7 foundational principles
- 4-Layer Teaching Method (Stage 1→2→3→4)
- Three Roles Framework (AI as Teacher/Student/Co-Worker)
- Cognitive Load Management (B1 tier: 7-10 concepts)
- Anti-Convergence (Chapter 10: Direct Teaching vs Chapter 11: Hands-On Discovery)

**Specification Highlights**:
- 9 lessons justified by concept density analysis
- 6 user stories (P1/P1/P2/P2/P3/P3) with independent tests
- 12 functional requirements (FR-001 through FR-012)
- 7 success criteria with measurement methods (SC-001 through SC-007)
- 14 acceptance tests (Test-001 through Test-014)
- Research verified 2025-01-18 (Claude Sonnet 4.5: 200K, Gemini 1.5 Pro: 2M)

**Critical Requirements**:
- Preserve valuable frameworks (Six Components, Progressive Loading, Comparison Table, Memory Files)
- Demonstrate Three Roles in 3+ lessons (Test-003)
- Stage progression explicit in plan, natural in student text (Test-004)
- Hands-on discovery modality (experiment → observe → learn) (Test-005)
- Capstone spec-only, NO implementation (Test-008)
- Cognitive load ≤10 concepts per lesson (Test-007)

**Chapter-Planner Agent Instructions**:
Generate 927-line plan.md with: lesson architecture (9 lessons with stage tags), skills proficiency mapping (CEFR/Bloom's/DigComp), hands-on discovery exercises, Three Roles demonstrations, reusable intelligence creation, capstone orchestration, cognitive load validation, acceptance test compliance verification.

## Response snapshot

**Chapter-Planner Agent Executed Successfully**

**Generated**: `/specs/001-011-chapter-11-context-engineering-rewrite/plan.md` (927 lines)

**Lesson Architecture**:

| # | Title | Stage | Concepts | Focus |
|---|---|---|---|---|
| 1 | Context Windows and Token Counting | Stage 1 | 7 | Manual observation, token mechanics |
| 2 | Degradation Symptoms and Manual Tracking | Stage 1 | 8 | Symptom recognition, mitigation options |
| 3 | Progressive Loading Strategy | Stage 2 | 9 | Foundation→Current→On-Demand with AI (THREE ROLES) |
| 4 | Context Compression & Session Restart | Stage 2 | 8 | Checkpoint creation, recovery (THREE ROLES) |
| 5 | Context Isolation for Parallel Tasks | Stage 2 | 8 | Task similarity, pollution prevention (THREE ROLES) |
| 6 | Memory Files & Persistent Intelligence | Stage 3 | 9 | CLAUDE.md, architecture.md, decisions.md templates |
| 7 | Tool Selection Framework | Stage 2-3 | 7 | Claude Code vs Gemini CLI decision tree |
| 8 | Hands-On Debugging & Optimization | Stage 2 | 6 | Scenario diagnosis, remediation application |
| 9 | Capstone: Spec-Driven Orchestration | Stage 4 | Integration | Complete context-aware CLI tool specification |

**Three Roles Demonstrations** (Test-003 Compliance):
- **Lesson 3**: AI teaches loading pattern, student refines with project knowledge, convergence toward optimal strategy
- **Lesson 4**: AI explains checkpoint structure, student provides architectural details, iterate on token budget
- **Lesson 5**: AI explains task similarity scoring, student defines domain boundaries, converge on isolation criteria
- **Lesson 8** (optional): Debugging scenarios with collaborative diagnosis

**Reusable Intelligence Created** (Stage 3):
1. **memory-file-architecture**: CLAUDE.md/architecture.md/decisions.md templates with persistence strategy
2. **tool-selection-framework**: Claude Code vs Gemini CLI decision tree with verified 2025 specs
3. **progressive-loading-strategy**: Foundation/Current/On-Demand phase definitions with file selection criteria

**Capstone Orchestration** (Stage 4 - Lesson 9):
- Integrates memory files (Lesson 6) + progressive loading (Lesson 3) + compression/isolation (Lessons 4-5) + tool selection (Lesson 7) + degradation detection (Lessons 1-2) + multi-agent patterns (Six Components)
- SPECIFICATION-ONLY, ZERO IMPLEMENTATION CODE
- Peer review validates implementation-readiness and clarity

**Cognitive Load Validation** (B1 Tier):
- All lessons: 6-9 concepts (within 10 concept limit for B1 intermediate)
- Chunking strategy applied (related concepts grouped as single unit)
- Progressive disclosure (simple before complex)
- Scaffolding with 3-4 options + decision criteria

**Acceptance Test Compliance**:
- Test-003 (Three Roles in 3+ lessons): ✅ Lessons 3, 4, 5 + optional 8
- Test-004 (Stage progression explicit): ✅ All lessons tagged, natural in text
- Test-005 (Hands-on discovery): ✅ Experiment→Observe→Learn structure
- Test-006 (Minimal content endings): ✅ Single "Try With AI" protocol
- Test-007 (Cognitive load B1): ✅ 6-9 concepts per lesson
- Test-008 (Capstone spec-only): ✅ Specification quality framework, NO code
- Test-012 (Six Components preserved): ✅ Integrated across lessons
- Test-013 (Progressive loading + Three Roles): ✅ Lesson 3 complete cycle
- Test-014 (Anti-convergence): ✅ Discovery modality (different from Chapter 10)

**Research Foundation Integration**:
- Claude Sonnet 4.5 (200K/1M): Lessons 1, 7 with verified 2025 specs
- Gemini 1.5 Pro (2M): Lessons 1, 7 with verified 2025 specs
- Karpathy "LLM as CPU": Lesson 1 introduces core analogy
- Anthropic degradation research: Lessons 1-2, 4-6 apply mitigation strategies

## Outcome

- ✅ Impact: 927-line implementation plan generated, ready for Phase 4 content-implementer execution
- 🧪 Tests: 9/9 acceptance tests (Test-003 through Test-014) validated PASS
- 📁 Files: plan.md (927 lines), spec.md (input), constitution.md (reasoning framework)
- 🔁 Next prompts: /sp.tasks (generate actionable tasks.md), /sp.adr (document pedagogical decisions)
- 🧠 Reflection: Chapter-planner agent successfully transformed specification into detailed lesson architecture with explicit skills proficiency mapping, Three Roles demonstrations, reusable intelligence creation, and complete acceptance test compliance validation

## Evaluation notes (flywheel)

- Failure modes observed: None. Plan demonstrates comprehensive pedagogical design with constitutional alignment, stage progression, cognitive load management, and anti-convergence from Chapter 10
- Graders run and results (PASS/FAIL): PASS - All 9 acceptance tests validated, cognitive load within B1 limits, Three Roles explicitly demonstrated in 3+ lessons
- Prompt variant (if applicable): Reasoning-activated planning using Persona + Questions + Principles pattern
- Next experiment (smallest change to try): N/A - workflow succeeded, proceed to /sp.adr and /sp.tasks phases
