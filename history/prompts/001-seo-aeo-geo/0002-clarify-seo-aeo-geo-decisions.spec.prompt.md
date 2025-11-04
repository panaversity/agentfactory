id: 0002
title: Clarify SEO/AEO/GEO decisions
stage: spec
date: 2025-11-04
surface: agent
model: unknown
feature: 001-seo-aeo-geo
branch: 001-seo-aeo-geo
user: mjs
command: /sp.clarify
labels: ["seo","aeo","geo","clarify"]
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

I mean SEO/GEO to map, monitor, and continuously improve AI search rankings across ChatGPT, Perplexity, Google AIO, etc.

2. Not sure

3. Not weaure

AEO means answer engine optimization

We need to do all Optimizations to rank everywhere

## Response snapshot

Resolved clarifications: GRO→GEO; provenance policy defaults to lightweight signed metadata (evaluate C2PA later); keep proposed 60‑day KPI thresholds. Added FR‑011 for channel registry and monitoring across major AI assistants/search engines. Updated spec and checklist accordingly.

## Outcome

- ✅ Impact: Spec unblocked for planning; evaluation and monitoring aligned to AI search/assistant channels
- 🧪 Tests: Evaluation framework defined conceptually (monthly); to be implemented in plan
- 📁 Files: Spec and checklist updated; no code changes yet
- 🔁 Next prompts: /sp.plan for 001-seo-aeo-geo
- 🧠 Reflection: Keep goals ambitious but revise monthly as baselines emerge

## Evaluation notes (flywheel)

- Failure modes observed: None; remaining work is planning/implementation
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Draft seed query sets per channel for the first monthly eval
