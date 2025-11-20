```prompt
---
id: 0015
title: Optimize JSON-LD injection performance
stage: refactor
date: 2025-11-04
surface: agent
model: github-copilot
feature: 001-seo-aeo-geo
branch: 001-seo-aeo-geo
user: user
command: optimize Root.tsx JSON-LD injector (defer DOM work, add article-only mode)
labels: [performance, seo, aeo, jsonld]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - book-source/src/theme/Root.tsx
  - history/prompts/001-seo-aeo-geo/0015-optimize-jsonld-injection-performance.refactor.prompt.md
tests:
  - none
---

## Prompt

Won;t your Root.tsx slow down mist of the things you can check out lesson structure as well (See <attachments> above for file contents. You may not need to search or read the file again.)

## Response snapshot

Implemented low-risk performance optimizations: (1) moved FAQ/HowTo DOM scans to requestIdleCallback with a timeout fallback, (2) added a global mode flag with default "article-only" to skip heuristics unless explicitly enabled, and (3) restricted scans to /docs/ routes; kept Article JSON‑LD injection lightweight. Added optional dev timing logs via localStorage jsonld:debug=1.

## Outcome

- ✅ Impact: Reduced main-thread work on navigation and content render; heuristics disabled by default; easier rollback and measurement.
- 🧪 Tests: No build executed here; validation to follow via structured-data validator; optional perf spot-check with debug logs.
- 📁 Files: Patched Root.tsx with idle scheduling and mode gating.
- 🔁 Next prompts: Run validator on target pages and capture timings with jsonld:debug=1; consider build-time injection for long-term.
- 🧠 Reflection: Heuristic extraction is useful but should be opt-in; future move to build-time/plugin would eliminate client parsing entirely.

## Evaluation notes (flywheel)

- Failure modes observed: Potential lack of /docs/ basePath alignment on custom deployments (guarded by simple regex; revisit if baseUrl differs)
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Toggle window.__JSONLD_MODE__ = "auto" on a single page and measure idle-task timing and CLS/TTI impact.
```
