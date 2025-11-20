id: 0006
title: Authoring ground rules for docs
stage: explainer
date: 2025-11-04
surface: agent
model: unknown
feature: 001-seo-aeo-geo
branch: 001-seo-aeo-geo
user: mjs
command: /explainer
labels: ["authoring","ground-rules","seo","aeo","geo"]
links:
spec: /Users/mjs/Documents/code/panaversity-official/tutorgpt-build/bbb/specs/001-seo-aeo-geo/spec.md
ticket: null
adr: null
pr: null
files:

- docs/WRITER-AUTHORING-GROUND-RULES.md
  tests:
- none

---

## Prompt

Are you planning any updates in content itself in docs/ What are the groundles princples for this if any

## Response snapshot

Created a ground rules document for docs authoring focused on answerability, canonical hygiene, provenance, frontmatter policy, JSON‑LD triggers, readability, links/anchors, freshness, and safety. Added an authoring checklist and ready-to-paste snippets for Q&A, HowTo, and Claims & Evidence.

## Outcome

- ✅ Impact: Writers have concrete rules to align with SEO/AEO/GEO goals before editing content
- 🧪 Tests: Lint and validators will enforce; no tests in this step
- 📁 Files: New ground rules doc created under docs/
- 🔁 Next prompts: Optional — update WRITER-HANDOFF.md to link to these rules; proceed with US1 content updates
- 🧠 Reflection: Keep rules concise and visible where writers start

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Add a short checklist linter to flag missing TL;DR or Q&A/HowTo blocks
