---
id: 0001
title: "Chapter 12 Lightning Python Stack - Specification Created"
stage: spec
date: 2025-01-15T19:30:00Z
surface: slash-command
model: claude-sonnet-4-5-20250929
feature: 001-chapter-12-lightning-python-stack
branch: 001-chapter-12-lightning-python-stack
user: orchestrator
command: /sp.specify
labels: [spec-creation, chapter-revision, evals-first, toolchain-integration]
links:
  spec: specs/001-chapter-12-lightning-python-stack/spec.md
  ticket: https://github.com/panaversity/ai-native-software-development/issues/138
  adr: null
  pr: null
files:
  - path: specs/001-chapter-12-lightning-python-stack/spec.md
    status: created
    lines: 424
  - path: specs/001-chapter-12-lightning-python-stack/checklists/requirements.md
    status: created
    lines: 95
tests: []
---

## Prompt

Chapter 12 revision request from GitHub issue #138:

**User Intent**: Expand Chapter 12 from "uv package manager only" to comprehensive Lightning Python Stack (uv + Zed + Ruff + Pyright).

**Revision Strategy** (User Confirmed via AskUserQuestion):
- EXPAND existing 6 uv lessons (keep lessons 1-6 as-is)
- ADD 4-6 new lessons for Zed/Ruff/Pyright integration
- Tool-by-tool teaching sequence: uv (1-3) → Zed (4-5) → Ruff (6-7) → Pyright (8) → Integration (9-10)
- Awareness-level coverage of pytest/pre-commit/MkDocs (show configs, don't teach in depth)

**New Chapter Title**: "The Lightning Python Stack — uv, Zed, Ruff & Pyright"

**Constitutional Principles to Apply**:
- Principle 1: AI-First Teaching
- Principle 3: Modern Python Standards (3.13+ with type hints)
- Principle 4: Test-First Mindset (Ruff + Pyright as quality gates)
- Principle 13: Graduated Teaching (direct → AI companion → AI orchestration)

**AI Usage Strategy Derived**:
- Tier 1 (Direct): `uv init`, `uv add`, `ruff format`, `pyright --version`, basic Zed setup
- Tier 2 (AI Companion): Complex pyproject.toml, Ruff rule selection, Pyright strict setup, Zed settings.json
- Tier 3 (AI Orchestration): Complete toolchain for 10+ projects, CI/CD automation

## Response snapshot

Created comprehensive specification using evals-first methodology:

**Success Criteria Defined First** (12 measurable outcomes):
- SC-001 to SC-004: Reader comprehension & skill acquisition (80%+, 75%+, 85%+, 70%+ targets)
- SC-005 to SC-006: AI partnership effectiveness (direct vs. AI usage understanding)
- SC-007 to SC-009: Content quality (B1 cognitive load, realistic durations, verified facts)
- SC-010 to SC-011: Integration success (smooth transition, tool progression understanding)
- SC-012: Awareness-level coverage (pytest/pre-commit/MkDocs known, depth deferred)

**User Stories Prioritized** (P1/P2/P3):
- P1: Professional Python setup for first-time learners (CORE value proposition)
- P2: AI-assisted configuration mastery (demonstrates Graduated Teaching)
- P2: Understanding tool-by-tool progression (strategic thinking, not cargo cult)
- P3: Cross-platform setup success (inclusivity)
- P3: Awareness-level exposure to advanced tools (scope boundaries)

**Functional Requirements** (27 FRs):
- FR-001 to FR-002: Content preservation (lessons 1-6 unchanged)
- FR-003 to FR-005: New content addition (lessons 7-12, tool-by-tool sequence)
- FR-006 to FR-008: Zed IDE coverage
- FR-009 to FR-011: Ruff coverage
- FR-012 to FR-014: Pyright coverage
- FR-015 to FR-017: Integration coverage
- FR-018 to FR-020: AI-first teaching
- FR-021 to FR-023: Awareness-level content
- FR-024 to FR-027: Constitutional compliance

**Scope Boundaries**:
- In Scope: Core 4 tools (uv, Zed, Ruff, Pyright), integration, AI workflows, cross-platform, awareness mentions
- Out of Scope: Deep pytest/pre-commit/MkDocs, alternative IDEs, advanced features, Docker, CI/CD
- Deferred: Phase 0.5 verification, lesson numbering, quiz expansion, readme rewrite

**Risk Mitigation**:
- Tool version volatility → version pins, update triggers
- Cognitive load overflow → tool-by-tool progression, graduated teaching
- Configuration complexity → AI-driven configuration, working templates
- Hallucinated configuration → Phase 0.5 verification against official docs

## Outcome

- ✅ Impact: Comprehensive spec created (424 lines) with evals-first approach, ready for Phase 0.5 (Deep Research)
- 🧪 Tests: Validation checklist passed (all 15 quality criteria met)
- 📁 Files: spec.md (424 lines), checklists/requirements.md (95 lines)
- 🔁 Next prompts: Phase 0.5 /sp.verify (Context7: Zed/Ruff/Pyright docs), then Phase 1 /sp.clarify (if gaps remain), Phase 2 /sp.plan
- 🧠 Reflection: Evals-first worked well—12 success criteria defined BEFORE requirements. User clarifications (3 questions) resolved strategy early. Phase 0.5 trigger identified due to tool-specific claims requiring verification.

## Evaluation notes (flywheel)

- Failure modes observed: None. Spec passed all 15 validation checklist items on first iteration.
- Graders run and results (PASS/FAIL): Specification Quality Checklist: PASS (all items green)
- Prompt variant (if applicable): Standard /sp.loopflow → /sp.specify with intelligence object context
- Next experiment (smallest change to try): Validate Phase 0.5 effectiveness—does Context7 verification catch hallucinated configs that manual review misses?
