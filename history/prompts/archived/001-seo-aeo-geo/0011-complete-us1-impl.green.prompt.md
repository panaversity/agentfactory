```prompt
id: 0011
title: Complete US1 implementation (injector + feed)
stage: green
date: 2025-11-04
surface: agent
model: unknown
feature: 001-seo-aeo-geo
branch: 001-seo-aeo-geo
user: mjs
command: Implement JSON-LD injector and retrieval feed page; update tasks
labels: ["seo","aeo","geo","implement"]
links:
spec: /Users/mjs/Documents/code/panaversity-official/tutorgpt-build/bbb/specs/001-seo-aeo-geo/spec.md
ticket: null
adr: null
pr: null
files:

- book-source/src/theme/Root.tsx
- book-source/static/api/knowledge-feed/page-1.json
- specs/001-seo-aeo-geo/tasks.md
- tests/unit/answerability.lint.md
  tests:
- none

---

## Prompt

okay complete implementation

## Response snapshot

- Added Root.tsx injector that emits Article JSON-LD on all pages and conditionally emits FAQPage/HowTo based on existing Q&A/HowTo blocks (DOM heuristics only). Wrapped with existing AnalyticsTracker. Dedupe by script id and feature-flag capable via window.__JSONLD_ENABLED__.
- Created retrieval feed at /api/knowledge-feed/page-1.json with 50 KnowledgeUnits (stable ids ku-0001..ku-0050), aligned to the data model.
- Added report-only answerability lint template at tests/unit/answerability.lint.md.
- Marked tasks T006, T007, T009, T010, T011, T013, T014 as completed; left T012/T015 for validator run and report.

## Outcome

- ✅ Impact: Pages emit machine-readable signals without content edits; assistants can consume a starter feed; tasks reflect progress.
- 🧪 Tests: Validator/lint to be run next on 10 pages; injector has built-in dedupe and conservative heuristics.
- 📁 Files: Injector TSX added; feed JSON created; task checklist updated; lint template added.
- 🔁 Next prompts: Run validator on 10 target pages; produce findings and optional YAML suggestions report; submit sitemap if desired.
- 🧠 Reflection: Keep FAQ/HowTo emission conservative; upgrade detection to component-driven in future for precision.
```
