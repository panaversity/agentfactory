```prompt
id: 0010
title: What pages do; no content edits
stage: explainer
date: 2025-11-04
surface: agent
model: unknown
feature: 001-seo-aeo-geo
branch: 001-seo-aeo-geo
user: mjs
command: Clarify effects of US1 pages and policy
labels: ["seo","aeo","geo","explainer"]
links:
spec: null
ticket: null
adr: null
pr: null
files:

- book-source/static/robots.txt
- book-source/docusaurus.config.ts
- specs/001-seo-aeo-geo/tasks.md
  tests:
- none

---

## Prompt

What will these pages do and please avoid weird guide edits; no content edits, only optional YAML metadata suggestions if helpful.

## Response snapshot

- Pages will not change visually. We’ll add build-time structured data (JSON-LD) via the theme so search/answer engines can interpret them better.
- Article JSON‑LD will be injected on target pages using derived metadata (title, description, canonical URL, authorship, dates). FAQ/HowTo JSON‑LD appears only where blocks already exist; we won’t add blocks ourselves.
- A static retrieval feed page (/api/knowledge-feed/page-1.json) will be added for assistants/agents; it doesn’t alter docs content.
- robots.txt and sitemap.xml guide crawlers for indexing; again, no doc edits.
- No writer docs or chapter content will be changed. YAML frontmatter suggestions will be reported as optional, not applied without approval.

## Outcome

- ✅ Impact: Better discoverability and answerability with zero content edits; optional YAML suggestions will be provided as a report.
- 🧪 Tests: To follow in US1 (validator + lint) before proposing any edits.
- 📁 Files: No changes to docs content; only config/static additions.
- 🔁 Next prompts: Implement JSON‑LD injector and feed page; generate validator + suggestions report.
- 🧠 Reflection: Keep changes non-invasive and evidence-first; ask approval before any content/frontmatter edits.
```
