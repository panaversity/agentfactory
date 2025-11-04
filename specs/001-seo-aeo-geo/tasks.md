# Tasks — SEO/AEO/GEO Opportunities (AI‑Native Book)

Branch: 001-seo-aeo-geo  
Spec: specs/001-seo-aeo-geo/spec.md  
Plan: specs/001-seo-aeo-geo/plan.md

## Dependencies (story order)

1. US1 → Make content answerable and discoverable (P1)
2. US2 → Measure assistant and agent impact (P2)
3. US3 → Author answerable content blocks (P3) — can run in parallel after foundational

Parallel opportunities are marked with [P]. Each user story is independently testable per its acceptance criteria.

---

## Phase 1 — Setup

- [x] T001 Ensure branch and feature directory exist (specs/001-seo-aeo-geo)
- [x] T002 Create static robots file at book-source/static/robots.txt (allow public docs, disallow drafts, include Sitemap directive)
- [x] T003 Verify sitemap generation in book-source/docusaurus.config.ts (enable sitemap plugin with siteUrl and changefreq/priority)
- [ ] T004 [P] Add canonical URL, authorship, license, last_updated frontmatter policy to content authoring guide (docs/WRITER-HANDOFF.md)
- [x] T005 Define channel registry seed (ChatGPT, Perplexity, Google AIO) at specs/001-seo-aeo-geo/research.md (append section)

## Phase 2 — Foundational

- [x] T006 Implement SSR JSON‑LD injection for Article schema only (book-source/src/theme/Root.tsx)
- [x] T007 Remove FAQ/HowTo DOM scans (align with book schema) (book-source/src/theme/Root.tsx)
- [ ] T008 Add JSON‑LD SSR presence check script and docs (book-source/scripts/check-jsonld-ssr.mjs)
- [x] T009 [P] Scaffold static retrieval feed directory (book-source/static/api/knowledge-feed/)
- [ ] T010 Define KnowledgeUnit JSON structure and sample (book-source/static/api/knowledge-feed/page-1.json)

---

## Phase 3 — User Story 1 (P1): Make content answerable and discoverable (evidence-first — no content edits)

Goal: Ship answerable pages with valid structured data and a basic retrieval feed page.

Independent test: Validate JSON‑LD renders on 10 target pages using derived metadata; fetch page‑1 of feed; confirm required fields and stable IDs; generate answerability findings report (no content edits).

- [x] T011 [US1] Ensure JSON‑LD Article renders server-side for 10 target pages (no edits to docs content) (book-source/src/theme/Root.tsx)
- [ ] T012 [P] [US1] Run JSON‑LD presence checker + validator on 10 target pages; produce findings report (tests/unit/answerability.lint.md)
- [ ] T013 [US1] N/A — FAQ/HowTo generation removed per schema direction
- [ ] T014 [P] [US1] Produce page-1 retrieval feed with ≥50 KnowledgeUnits (book-source/static/api/knowledge-feed/page-1.json)
- [ ] T015 [US1] Validate structured data (script and/or validator) and record results (specify steps in specs/001-seo-aeo-geo/quickstart.md)

---

## Phase 4 — User Story 2 (P2): Measure assistant and agent impact

Goal: Capture assistant citations, referrals, and feed usage; run first evaluator.

Independent test: Execute seed queries per channel and record citations/referrals; run evaluator and export a report.

- [ ] T016 [US2] Add analytics capture points for assistant citations/referrals (src/utils/analytics.ts)
- [ ] T017 [P] [US2] Log retrieval feed access with page and count (server logs or static host analytics) — document approach (specs/001-seo-aeo-geo/quickstart.md)
- [ ] T018 [US2] Create monthly evaluator harness (tests/integration/aeo-geo-evaluator.md) with 20 seed queries across channels
- [ ] T019 [P] [US2] Document evaluator runbook and targets (specs/001-seo-aeo-geo/quickstart.md)
- [ ] T020 [US2] Generate first monthly report and store snapshot (tests/integration/aeo-geo-evaluator-report-YYYY-MM.md)

---

## Phase 5 — User Story 3 (P3): Author answerable content blocks

Goal: Make answerable blocks easy to author and lint.

Independent test: Lint enhancements flag malformed blocks; CI gate prevents publish on critical issues. Authoring examples live in writer docs, not chapter content.

- [ ] T021 [US3] Add authoring snippets/examples for Q&A and HowTo blocks (docs/WRITER-AUTHORING-GROUND-RULES.md)
- [ ] T022 [P] [US3] Enhance lint to catch malformed blocks and missing definitions (tests/unit/answerability.lint.md)
- [ ] T023 [US3] Add CI gate to block publish on critical answerability failures (document pipeline integration in specs/001-seo-aeo-geo/quickstart.md)

---

## Final Phase — Polish & Cross‑Cutting

- [ ] T024 Review canonical conflicts and fragment anchors for determinism (book-source/docs/\*)
- [ ] T025 Add visible attribution and license footer pattern (book-source/src/theme/Footer/)
- [ ] T026 [P] Submit sitemaps to major engines and document submission in quickstart (specs/001-seo-aeo-geo/quickstart.md)
- [ ] T027 [P] Schedule monthly evaluator and analytics review cadence (history/guides/seo-aeo-geo-ops.md)

## Parallel execution examples

- In Phase 2: T007 and T009 can proceed in parallel (separate files).
- In Phase 3: T012 and T014 can proceed in parallel (content vs static feed JSON).
- In Phase 4: T017 and T019 can proceed in parallel (logging approach vs runbook).

## MVP scope

- Complete Phase 3 (US1) with 10 pages answerable and validated JSON‑LD (Article only). Retrieval feed may be deferred; if deferred, document rationale and leave README placeholder.

## Format validation

- All tasks use strict checklist format: `- [ ] T### [P] [US#] Description with file path`
- User story phases include [US#] labels; setup/foundational/polish phases do not.
