```prompt
---
id: 0018
title: Align tasks/quickstart; add SSR JSON-LD checker
stage: green
date: 2025-11-04
surface: agent
model: github-copilot
feature: 001-seo-aeo-geo
branch: 001-seo-aeo-geo
user: user
command: Update tasks/quickstart for Article-only SSR; add check-jsonld script and report templates
labels: [seo, aeo, validation, tooling]
links:
  spec: specs/001-seo-aeo-geo/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-seo-aeo-geo/tasks.md
  - specs/001-seo-aeo-geo/quickstart.md
  - book-source/scripts/check-jsonld-ssr.mjs
  - book-source/package.json
  - tests/unit/answerability.lint.md
  - tests/integration/structured-data-validation-report.md
tests:
  - none (tooling; run after site build)
---

## Prompt

Okay review do pending work or update spec, plan tasks if something have beocme irrelevant. We need data driven results (See <attachments> above for file contents. You may not need to search or read the file again.)

## Response snapshot

- Updated tasks to remove FAQ/HowTo generation, focus on SSR Article JSON-LD, and add a JSON-LD presence checker task.
- Revised quickstart to reflect Article-only SSR validation and usage of the checker.
- Added a Node script (check-jsonld-ssr.mjs) to scan built HTML for the SSR Article JSON-LD.
- Created report templates for unit/integration validation results.

## Outcome

- ✅ Impact: Clear, evidence-first workflow with zero runtime DOM scans and a concrete post-build checker.
- 🧪 Tests: To be executed after building the site (book-source); populate reports with results.
- 📁 Files: Tasks and quickstart updated; script and templates added.
- 🔁 Next prompts: Build site, run check:jsonld, fill reports; optionally plan LearningResource build-time plugin.
- 🧠 Reflection: Tooling enables data-driven verification without content edits or runtime cost.

## Evaluation notes (flywheel)

- Risks: None significant; script reads static build output only.
- Next experiment: Implement LearningResource plugin and compare validator outcomes vs Article-only.
```
