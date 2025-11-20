```prompt
id: 0009
title: What will it do
stage: explainer
date: 2025-11-04
surface: agent
model: unknown
feature: 001-seo-aeo-geo
branch: 001-seo-aeo-geo
user: mjs
command: Explain effects of robots/sitemap/tasks edits
labels: ["seo","aeo","geo","explain"]
links:
spec: null
ticket: null
adr: null
pr: null
files:

- book-source/static/robots.txt
- book-source/docusaurus.config.ts
- specs/001-seo-aeo-geo/tasks.md
- specs/001-seo-aeo-geo/research-addendum.md
  tests:
- none

---

## Prompt

what will it do

## Response snapshot

- robots.txt will instruct crawlers to index the site (allow all), skip tag listings (/tags/), and discover the canonical sitemap at https://ai-native.panaversity.org/sitemap.xml.
- Docusaurus will generate sitemap.xml on build with weekly changefreq, medium priority, and no tag pages, improving index coverage and reducing noise.
- No content changes were made; authoring remains evidence‑first. The research addendum adds seed channels/queries for evaluation only.
- Next visible effects will come after US1 JSON‑LD injection and the retrieval feed: pages will emit Article (and FAQ/HowTo where present), and a feed page will be available under /api/knowledge-feed/page-1.json.

## Outcome

- ✅ Impact: Crawlers get clear directives and a discoverable sitemap; groundwork set for answerability and assistant ranking.
- 🧪 Tests: None yet; validator and lint to run in upcoming US1 tasks.
- 📁 Files: robots.txt created, sitemap config added, tasks updated, research addendum added.
- 🔁 Next prompts: Implement JSON‑LD (Root.tsx, MDXComponents.tsx) and create retrieval feed page; run validators and report.
- 🧠 Reflection: Keep robots sitemap static and simple now; consider dynamic sitemap URL in robots.txt later if multi-env needs arise.

## Evaluation notes (flywheel)

- Failure modes observed: Editor may show Node type warnings in docusaurus.config.ts; does not affect static build.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment: Validate sitemap presence in build output and fetch robots/sitemap via deployed URL.
```
