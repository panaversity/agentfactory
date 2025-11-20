id: 0007
title: Evidence-first content policy
stage: explainer
date: 2025-11-04
surface: agent
model: unknown
feature: 001-seo-aeo-geo
branch: 001-seo-aeo-geo
user: mjs
command: /explainer
labels: ["policy","content","evidence-first"]
links:
spec: /Users/mjs/Documents/code/panaversity-official/tutorgpt-build/bbb/specs/001-seo-aeo-geo/spec.md
ticket: null
adr: null
pr: null
files:

- specs/001-seo-aeo-geo/plan.md
- specs/001-seo-aeo-geo/tasks.md
  tests:
- none

---

## Prompt

no we dont edit content unless procevn

## Response snapshot

Updated plan constraints and tasks to enforce evidence-first policy: no direct edits to book-source/docs unless supported by evaluator or lint findings. US1 tasks now render JSON-LD via derived metadata, produce findings reports, and avoid doc edits; authoring examples moved to writer docs.

## Outcome

- ✅ Impact: Prevents premature content changes; keeps work measurable and reversible
- 🧪 Tests: Evaluator/lint will justify any future edits; none executed yet
- 📁 Files: Plan and tasks updated; no content edited
- 🔁 Next prompts: Proceed with US1 implementation under evidence-first constraints
- 🧠 Reflection: This aligns with validation-first principle and business goals

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Run baseline lint and structured-data validation to generate findings
