```prompt
---
id: 0016
title: Remove FAQ/HowTo scans from Root.tsx
stage: refactor
date: 2025-11-04
surface: agent
model: github-copilot
feature: 001-seo-aeo-geo
branch: 001-seo-aeo-geo
user: user
command: Remove FAQ and HowTo DOM scans; keep SSR Article JSON-LD only
labels: [performance, seo, aeo, jsonld]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - book-source/src/theme/Root.tsx
  - history/prompts/001-seo-aeo-geo/0016-remove-faq-howto-scans.refactor.prompt.md
tests:
  - none
---

## Prompt

We can inface remove HotTo and FAQ scans a book schema don;t have them anyway (See <attachments> above for file contents. You may not need to search or read the file again.)

## Response snapshot

Removed all FAQ and HowTo DOM scans and helpers from Root.tsx. Kept SSR Article JSON-LD injection via Head so structured data remains visible to crawlers without JS. No content edits.

## Outcome

- ✅ Impact: Zero runtime DOM scanning; simpler, faster pages; consistent Article JSON-LD only.
- 🧪 Tests: To be validated via rich results/structured data validator on selected pages.
- 📁 Files: Root.tsx updated to remove heuristics; Article JSON-LD SSR intact.
- 🔁 Next prompts: If richer educational schema is desired, propose build-time plugin for LearningResource using lesson frontmatter.
- 🧠 Reflection: Aligns with book schema direction while preserving answerability via Article metadata.

## Evaluation notes (flywheel)

- Risks: None significant; loss of optional FAQ/HowTo markup is intentional per direction.
- Next experiment: Prototype build-time JSON-LD emission for LearningResource (zero runtime).
```
