# Implementation Plan: SEO, AEO, GEO Opportunities (AI‑Native Book)

**Branch**: `001-seo-aeo-geo` | **Date**: 2025-11-04 | **Spec**: specs/001-seo-aeo-geo/spec.md
**Input**: Feature specification from `/specs/001-seo-aeo-geo/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Primary goal: Make the book’s content answerable, retrievable, and rankable across SEO/AEO/GEO channels, and measure performance to iterate.

Technical approach (high level):

- Add and validate structured data (Article + FAQPage/HowTo where applicable) with canonical, authorship, license, last_updated.
- Publish a retrieval‑ready feed of atomic KnowledgeUnits (paged JSON) with stable IDs, Q&A, claims/evidence.
- Maintain a channel registry and seed query sets for ChatGPT, Perplexity, Google AIO, etc., to map and monitor rankings.
- Capture analytics for assistant citations, assistant‑attributed referrals, retrieval usage, and run a monthly evaluator.
- Ensure crawl/index controls: robots directives (robots.txt) and sitemaps correctly expose public content and feed endpoints.

## Technical Context

<!--
  ACTION REQUIRED: Replace the content in this section with the technical details
  for the project. The structure here is presented in advisory capacity to guide
  the iteration process.
-->

**Language/Version**: Markdown/TypeScript site (static generation)  
**Primary Dependencies**: Static site generator with plugin support; JSON‑LD injection; sitemap generation  
**Storage**: N/A (static assets)  
**Testing**: Content linting, structured‑data validation, evaluator harness (monthly)  
**Target Platform**: Static website; bots/assistants consuming HTML/feeds  
**Project Type**: Web documentation site  
**Performance Goals**: 95% of feed fetches in <1s; pages pass Core Web Vitals; structured data valid  
**Constraints**: No vendor lock; avoid framework‑specific code in spec/plan; provenance minimal at first; no direct edits to `book-source/docs/` content unless supported by evaluator/lint evidence (evidence‑first content policy)  
**Scale/Scope**: ≥500 KnowledgeUnits; top 50 pages fully optimized

Robots and Sitemaps (channel exposure):

- Robots directives: Provide crawl permissions and exclusions via platform‑appropriate mechanism; for this repo’s site, include a static `robots.txt` under the public/static directory.
- Sitemap: Ensure automatic sitemap generation is enabled and includes canonical URLs and, where feasible, feed endpoints; submit to major engines.

## Constitution Check

_GATE: Must pass before Phase 0 research. Re-check after Phase 1 design._

Gates derived from constitution v3.0.1:

- Evals‑First: Monthly evaluator defined before implementation (PASS — spec includes SC‑004 and evaluator requirement FR‑007)
- Spec‑First: Implementation deferred; plan remains tech‑agnostic (PASS)
- Validation‑First Safety: Lint and validator gates block publish (PASS — FR‑010)
- Bilingual/Production standards: Not applicable to this feature’s code; content/process oriented (PASS)

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

<!--
  ACTION REQUIRED: Replace the placeholder tree below with the concrete layout
  for this feature. Delete unused options and expand the chosen structure with
  real paths (e.g., apps/admin, packages/something). The delivered plan must
  not include Option labels.
-->

```text
specs/001-seo-aeo-geo/
├── plan.md
├── research.md
├── data-model.md
├── quickstart.md
└── contracts/

book-source/
├── docusaurus.config.ts              # site config (sitemap config lives here)
├── static/
│   └── robots.txt                    # robots directives (to be added/verified)
└── docs/                             # content with JSON‑LD injection at build

tests/
├── integration/                      # evaluator harness output snapshots
└── unit/                             # content lints for answerability
```

**Structure Decision**: Use existing documentation site structure; add static robots.txt and ensure sitemap generation is configured in site config. Contracts and evaluator artifacts live under the feature spec directory for now (tech‑agnostic).

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation                  | Why Needed         | Simpler Alternative Rejected Because |
| -------------------------- | ------------------ | ------------------------------------ |
| [e.g., 4th project]        | [current need]     | [why 3 projects insufficient]        |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient]  |
