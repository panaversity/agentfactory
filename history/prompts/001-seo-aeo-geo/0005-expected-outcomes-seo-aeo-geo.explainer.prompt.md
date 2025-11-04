id: 0005
title: Expected outcomes SEO AEO GEO
stage: explainer
date: 2025-11-04
surface: agent
model: unknown
feature: 001-seo-aeo-geo
branch: 001-seo-aeo-geo
user: mjs
command: /explainer
labels: ["expectations","seo","aeo","geo"]
links:
spec: /Users/mjs/Documents/code/panaversity-official/tutorgpt-build/bbb/specs/001-seo-aeo-geo/spec.md
ticket: null
adr: null
pr: null
files:

- specs/001-seo-aeo-geo/spec.md
- specs/001-seo-aeo-geo/plan.md
- specs/001-seo-aeo-geo/tasks.md
  tests:
- none

---

## Prompt

What are the expectatiosn once its done (See <attachments> above for file contents. You may not need to search or read the file again.)

## Response snapshot

Outlined concrete expectations on completion: deliverables (robots.txt, sitemap config, JSON‑LD on top pages, retrieval feed, channel registry, evaluator, analytics), measurable outcomes (SC‑001..SC‑005), operating cadence (monthly evaluator, weekly freshness), and realistic ranking expectations (no guarantees, improved inclusion/citations over 2–6 weeks).

## Outcome

- ✅ Impact: Team aligned on success metrics and artifacts; implementation can be judged objectively
- 🧪 Tests: Evaluator and lints to verify answerability and feed; to be executed in implementation
- 📁 Files: Referenced spec/plan/tasks only; no code added in this step
- 🔁 Next prompts: Proceed to US1 implementation per tasks.md
- 🧠 Reflection: Set expectations for indexing lag and channel variance early

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Run JSON‑LD validator on one page and ship page‑1 feed
