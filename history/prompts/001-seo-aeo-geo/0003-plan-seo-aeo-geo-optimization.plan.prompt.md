id: 0003
title: Plan SEO/AEO/GEO optimization
stage: plan
date: 2025-11-04
surface: agent
model: unknown
feature: 001-seo-aeo-geo
branch: 001-seo-aeo-geo
user: mjs
command: /sp.plan
labels: ["plan","seo","aeo","geo"]
links:
spec: /Users/mjs/Documents/code/panaversity-official/tutorgpt-build/bbb/specs/001-seo-aeo-geo/spec.md
ticket: null
adr: null
pr: null
files:

- specs/001-seo-aeo-geo/plan.md
- specs/001-seo-aeo-geo/research.md
- specs/001-seo-aeo-geo/data-model.md
- specs/001-seo-aeo-geo/contracts/retrieval-feed.openapi.yaml
- specs/001-seo-aeo-geo/quickstart.md
  tests:
- none

---

## Prompt

Follow instructions in sp.plan.prompt.md. Yeah and what about robots.ts and sitemap and all other required thingks for search engine optimization.

## Response snapshot

Initialized planning via setup-plan; populated plan.md with summary, technical context including robots.txt and sitemap strategy, constitution check, and repository structure. Created research.md (decisions/rationales), data-model.md, retrieval-feed OpenAPI stub, quickstart.md. Updated agent context.

## Outcome

- ✅ Impact: Planning artifacts ready; clear path for robots/sitemap and AEO/GEO evaluator
- 🧪 Tests: To be defined in tasks as content lint + monthly evaluator harness
- 📁 Files: Plan, research, data model, contracts, quickstart created
- 🔁 Next prompts: /sp.tasks to produce actionable tasks and acceptance checks
- 🧠 Reflection: Keep framework-agnostic; avoid Next.js-specific robots.ts — use static robots.txt

## Evaluation notes (flywheel)

- Failure modes observed: Script warns about multiple '001' features; non-blocking
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Prototype a single feed page and validate schema with 5 KnowledgeUnits
