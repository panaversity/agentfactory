id: 0004
title: Generate tasks for SEO/AEO/GEO
stage: tasks
date: 2025-11-04
surface: agent
model: unknown
feature: 001-seo-aeo-geo
branch: 001-seo-aeo-geo
user: mjs
command: /sp.tasks
labels: ["tasks","seo","aeo","geo"]
links:
spec: /Users/mjs/Documents/code/panaversity-official/tutorgpt-build/bbb/specs/001-seo-aeo-geo/spec.md
ticket: null
adr: null
pr: null
files:

- specs/001-seo-aeo-geo/tasks.md
  tests:
- none

---

## Prompt

Follow instructions in sp.tasks.prompt.md. Context for task generation: none.

## Response snapshot

Generated tasks.md organized by phases and user stories with strict checklist format. Included robots.txt, sitemap config, JSON‑LD injection, retrieval feed scaffolding, analytics and evaluator tasks, plus polish/cross‑cutting work.

## Outcome

- ✅ Impact: Tasks immediately actionable by LLM agents; clear file paths and dependencies
- 🧪 Tests: Evaluator and lints planned; not executed yet
- 📁 Files: One new tasks.md created
- 🔁 Next prompts: Begin implementation or refine tasks if you want test‑first additions
- 🧠 Reflection: Kept framework‑agnostic; used static robots.txt not Next.js robots.ts

## Evaluation notes (flywheel)

- Failure modes observed: Prereq script warns about multiple '001' features; non‑blocking
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Create a tiny validator script to check JSON‑LD presence per page
