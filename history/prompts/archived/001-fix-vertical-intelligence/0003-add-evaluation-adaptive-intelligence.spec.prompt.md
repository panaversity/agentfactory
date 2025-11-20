---
id: 0003
title: add-evaluation-adaptive-intelligence
stage: spec
date: 2025-11-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-fix-vertical-intelligence
branch: 001-fix-vertical-intelligence
user: mjs
command: iterative-spec-enhancement
labels: ["evaluation", "adaptive-intelligence", "scalability", "user-feedback", "sub-agent-quality", "two-phase"]
links:
  spec: specs/001-fix-vertical-intelligence/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-fix-vertical-intelligence/spec.md (expanded from 331 to 490 lines)
tests:
  - Added 13 new functional requirements (FR-019 through FR-031)
  - Added 4 new success criteria (SC-008 through SC-011)
  - Validated two-phase approach against external feedback
---

## Prompt

User provided external feedback analysis highlighting critical blind spots:

"If we don't know what good looks like we can not measure the effectiveness at all. So let's do both A and C."

External analysis identified we were "solving the wrong problem" by tunneling on internal structure fixes while missing:
1. No evaluation system for sub-agent quality (how good is chapter-planner? content-implementer?)
2. No user/learner feedback loop (ignoring actual book readers)
3. No dynamic adaptation mechanism (one-time fix, no future-proofing)
4. No scalability planning (fixes 14 chapters, no plan for remaining 41)
5. Missing alignment with AI-first paradigm (Garry Tan: imagination + prompts + evals over manual work)

## Response snapshot

Implemented comprehensive spec expansion combining immediate fix with adaptive intelligence infrastructure.

**Major Additions**:

**13 New Functional Requirements** (Phase 2 - Adaptive Intelligence):

*Sub-Agent Evaluation Framework (FR-019 to FR-022)*:
- FR-019: Benchmark suite with 50 test prompts per agent (normal, edge, incomplete, ambiguous cases)
- FR-020: Quality thresholds (>95% accuracy, >98% consistency, >90% completeness)
- FR-021: Cost metrics (tokens, time, API costs per operation)
- FR-022: Failure analysis (error patterns, retry strategies)

*User Feedback Integration (FR-023 to FR-025)*:
- FR-023: "Try With AI" completion tracking (started, completed, time, success indicators)
- FR-024: Community input channels (GitHub templates, surveys, analytics dashboard)
- FR-025: A/B testing framework (test pedagogical variations, measure outcomes, auto-promote winners)

*Adaptive Intelligence System (FR-026 to FR-028)*:
- FR-026: Weekly drift-detection agent (scans misalignments, flags violations, generates alerts)
- FR-027: Auto-migration system (generates scripts for v3→v4, impact analysis, dry-run validation)
- FR-028: AI tool evolution monitoring (tracks Claude/Gemini updates, tests against benchmarks, recommends upgrades)

*Scalability Infrastructure (FR-029 to FR-031)*:
- FR-029: Multi-model orchestration (model selector, load balancing, fallback strategies, cost optimization)
- FR-030: Load simulation (10 chapters in parallel, resource measurement, parallelization opportunities)
- FR-031: Cost control (budget alerts, per-chapter reporting, optimization recommendations, ROI analysis)

**4 New Success Criteria** (Phase 2 targets):
- SC-008: Sub-agents pass benchmarks (50 prompts, >95% accuracy, >98% consistency, >90% completeness, cost tracking)
- SC-009: User feedback operational (>80% sections instrumented, channels established, 50+ responses/month)
- SC-010: Adaptive intelligence prevents drift (weekly detection, auto-migration works, zero undetected drift over 3 months)
- SC-011: Scalability demonstrated (10 chapters parallel, cost controls, multi-model routing, can generate 41 remaining chapters)

**Two-Phase Structure**:
- **Phase 1 (2 weeks)**: Emergency fix - synchronize layers, unblock authors (FR-001 to FR-018, SC-001 to SC-007)
- **Phase 2 (4 weeks)**: Adaptive intelligence - build eval systems, feedback loops, drift prevention, scalability (FR-019 to FR-031, SC-008 to SC-011)

**Updated Notes Section** addresses each external feedback point:
1. "Can't measure without knowing good" → Quantitative thresholds defined (>95%, >98%, >90%)
2. "No sub-agent evaluation" → Benchmark suite, failure analysis, cost metrics
3. "No user feedback" → Completion tracking, GitHub/surveys, A/B testing, 50+ responses/month target
4. "No adaptation" → Weekly drift detection, auto-migration, tool monitoring, zero drift over 3 months
5. "No scalability" → Multi-model orchestration, load simulation, cost controls, 41-chapter capability

## Outcome

- ✅ Impact: Transformed spec from static one-time fix to adaptive, eval-driven vertical intelligence system that can evolve with AI and measure its own effectiveness
- 🧪 Tests: 4 new quantitative success criteria with measurable targets (>95% accuracy, 50+ user responses/month, zero drift, 10x parallel generation)
- 📁 Files: spec.md expanded by 159 lines (331→490), adding 13 FRs, 4 SCs, 2-phase structure, external feedback analysis
- 🔁 Next prompts: `/sp.plan` to generate two-phase implementation plan with Phase 1 (immediate) and Phase 2 (adaptive) task breakdowns
- 🧠 Reflection: User's insight "if we don't know what good looks like, we can't measure effectiveness" was the turning point - forced shift from structure-fixing to building systems that define and maintain quality over time

## Evaluation notes (flywheel)

- Failure modes observed: Initial spec tunneled on internal fixes, ignored broader evaluation/adaptation needs. External feedback prevented shipping incomplete solution.
- Graders run and results (PASS/FAIL): PASS - Spec now addresses all 5 gaps identified in external analysis with measurable requirements
- Prompt variant (if applicable): Option A+C combined (add all FRs immediately + structure as two phases) - prevents losing sight of vision while fixing urgently
- Next experiment (smallest change to try): During Phase 1 implementation, track which validation tasks take longest to inform Phase 2 automation priorities (e.g., if manual terminology checking is slow, prioritize drift-detection agent)
