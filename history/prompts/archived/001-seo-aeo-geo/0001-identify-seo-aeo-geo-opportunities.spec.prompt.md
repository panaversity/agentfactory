id: 0001
title: Identify SEO AEO GEO opportunities
stage: spec
date: 2025-11-04
surface: agent
model: unknown
feature: 001-seo-aeo-geo
branch: 001-seo-aeo-geo
user: mjs
command: /sp.specify
labels: ["seo","aeo","geo","spec"]
links:
spec: /Users/mjs/Documents/code/panaversity-official/tutorgpt-build/bbb/specs/001-seo-aeo-geo/spec.md
ticket: null
adr: null
pr: null
files:

- specs/001-seo-aeo-geo/spec.md
- specs/001-seo-aeo-geo/checklists/requirements.md
  tests:
- none

---

## Prompt

Follow instructions in sp.specify.prompt.md.
As AI Native Book built using Vertical Intelligence Layer let's identifiy the SEO, AEO and GRO opportunites (See <attachments> above for file contents. You may not need to search or read the file again.)

## Response snapshot

Created feature branch 001-seo-aeo-geo and populated spec with user stories, edge cases, functional requirements, entities, assumptions, and measurable outcomes; added three [NEEDS CLARIFICATION] markers (GRO vs GEO, provenance signing, KPI thresholds) and created the quality checklist with current pass/fail notes.

## Outcome

- ✅ Impact: Spec ready for planning with clear scope and KPIs pending; groundwork for SEO/AEO/GEO established
- 🧪 Tests: Initial lint/validation to be implemented in planning; monthly evaluator defined conceptually
- 📁 Files: Spec and checklist written under specs/001-seo-aeo-geo/
- 🔁 Next prompts: /sp.clarify to resolve the 3 questions; then /sp.plan
- 🧠 Reflection: Keep content technology‑agnostic; enforce answerability and provenance early

## Evaluation notes (flywheel)

- Failure modes observed: Potential ambiguity around GRO term; provenance policy undecided; KPI realism depends on baseline
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Run a tiny AEO eval with 10 questions to baseline current coverage
