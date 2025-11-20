---
id: 004
title: Constitution v3.0 — spectrum, spec-first alignment
stage: constitution
date: 2025-10-31
surface: agent
model: gpt-5
feature: none
branch: main
user: mjs
command: constitution-update
labels: ["v3.0","spectrum","spec-first","validation-first","bilingual","deployment"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
  - CLAUDE.md
  - .claude/agents/chapter-planner.md
  - .claude/agents/content-implementer.md
  - .claude/agents/validation-auditor.md
  - .claude/agents/factual-verifier.md
  - .claude/skills/code-example-generator/SKILL.md
  - .claude/skills/ai-augmented-teaching/SKILL.md
  - .claude/skills/learning-objectives/SKILL.md
  - .claude/skills/assessment-builder/SKILL.md
  - .claude/skills/technical-clarity/SKILL.md
  - .claude/skills/book-scaffolding/SKILL.md
  - .claude/skills/concept-scaffolding/SKILL.md
  - .claude/skills/exercise-designer/SKILL.md
tests:
  - None (governance/content updates)
---

PROMPT_TEXT:

Update the constitution to v3.0, clarify the AI development spectrum (Assisted → Driven → Native), and align operational docs (CLAUDE.md, agents, skills) to spec-first and validation-first. Remove or generalize references to skills not present; rename AI teaching skill to ai-collaborate-learning. Add minimal acceptance checks to consistently-used skills.

RESPONSE_TEXT:

- Constitution v3.0: Added “AI Development Spectrum (Assisted → Driven → Native)” under Section I. Clarified emphasis on Driven and Native; preserved planning-first, validation-before-trust, bilingual, deployment principles. Added repo-availability note for skills in II.B.
- CLAUDE.md: Generalized skills references to those present; listed current core skills; ensured subagents/flows enforce spec-first and validation steps.
- Agents: chapter-planner (spec-first, acceptance criteria, tier, closure); content-implementer (spec→prompt→code→validation, SpecRef, closure); validation-auditor (spectrum note, sequence check); factual-verifier (must-pass gates: acceptance, closure, sources/tests).
- Skills improvements: learning-objectives (Bloom tags, evidence mapping), assessment-builder (objective coverage matrix, validation pack), technical-clarity (gatekeeping auto-fix, spec alignment, audience band), book-scaffolding (tier tags, dependency index), concept-scaffolding (per-step verifiability, load budget), code-example-generator (test snippet, security lint, failure-mode), exercise-designer (difficulty bands, hints, rubric), ai-collaborate-learning (spectrum tag, verification prompts).

OUTCOME:

Constitution and operational artifacts aligned to v3.0 spec-first/validation-first workflow with explicit spectrum framing; repository references constrained to existing skills; agents and skills now include lightweight acceptance checks. Lints clean.

