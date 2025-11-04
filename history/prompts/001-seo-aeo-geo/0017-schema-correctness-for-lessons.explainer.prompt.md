```prompt
---
id: 0017
title: Schema correctness for lessons (Article vs LearningResource)
stage: explainer
date: 2025-11-04
surface: agent
model: github-copilot
feature: 001-seo-aeo-geo
branch: 001-seo-aeo-geo
user: user
command: Evaluate if making each lesson an Article is correct and propose alternatives
labels: [seo, aeo, schema, jsonld]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - book-source/src/theme/Root.tsx
  - history/prompts/001-seo-aeo-geo/0017-schema-correctness-for-lessons.explainer.prompt.md
tests:
  - none
---

## Prompt

Is your schema really correct as you make each book chapter lesson an article (See <attachments> above for file contents. You may not need to search or read the file again.)

## Response snapshot

Short answer: Article is an acceptable baseline for docs pages and is broadly recognized, but for educational content the more semantically correct type is LearningResource (optionally alongside Article). Best long-term: add a build-time JSON-LD emitter mapping frontmatter (duration_minutes → timeRequired, CEFR → educationalLevel, skills → educationalAlignment) and optionally model Book/Chapter relationships with isPartOf/hasPart. Keeping SSR Article now preserves compatibility and zero runtime cost; we can augment with a dual-type ["Article","LearningResource"] later.

## Outcome

- ✅ Impact: Clarity on baseline vs. ideal schema; plan to evolve to LearningResource without content edits.
- 🧪 Tests: Validate with structured-data testing tools once implemented.
- 📁 Files: No code changes; Root.tsx continues to emit SSR Article JSON-LD.
- 🔁 Next prompts: Approve adding a small build-time plugin plan for LearningResource mapping.
- 🧠 Reflection: Dual-typing maintains compatibility while improving semantics; build-time approach avoids client work.

## Evaluation notes (flywheel)

- Risks: Over-modeling without SERP benefit; mitigated by dual-type and zero-runtime approach.
- Next experiment: Prototype a plugin on 1-2 lessons and verify validator output.
```
