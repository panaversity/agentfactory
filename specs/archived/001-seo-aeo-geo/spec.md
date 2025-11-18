# Feature Specification: SEO, AEO, GEO Opportunities (AI‑Native Book)

**Feature Branch**: `001-seo-aeo-geo`  
**Created**: 2025-11-04  
**Status**: Draft  
**Input**: User description: "As AI Native Book built using Vertical Intelligence Layer let's identifiy the SEO, AEO and GRO opportunites"

## User Scenarios & Testing _(mandatory)_

<!--
  IMPORTANT: User stories should be PRIORITIZED as user journeys ordered by importance.
  Each user story/journey must be INDEPENDENTLY TESTABLE - meaning if you implement just ONE of them,
  you should still have a viable MVP (Minimum Viable Product) that delivers value.

  Assign priorities (P1, P2, P3, etc.) to each story, where P1 is the most critical.
  Think of each story as a standalone slice of functionality that can be:
  - Developed independently
  - Tested independently
  - Deployed independently
  - Demonstrated to users independently
-->

### User Story 1 - Make content answerable and discoverable (Priority: P1)

As the Book Architect, I can structure the top 10 priority chapters into atomic, machine‑readable knowledge units (titles, summaries, Q&A, HowTo steps, claims/evidence, canonical URLs, license, authorship) so assistants and agents can reliably cite and use our content (AEO/GEO) while preserving SEO.

**Why this priority**: Direct impact on reach and learning outcomes; creates the substrate for AEO/GEO while improving traditional SEO.

**Independent Test**: Validate 10 target pages with a structured‑data checker and a retrieval‑feed checker; run a small AEO/GEO eval set to confirm assistants answer from our content and cite us.

**Acceptance Scenarios**:

1. **Given** a target chapter URL, **When** a structured‑data validator runs, **Then** it finds valid JSON‑LD (Article and/or FAQPage/HowTo) with canonical_url, authorship, license, and last_updated.
2. **Given** the retrieval feed, **When** an agent fetches knowledge units by ID, **Then** it receives stable IDs, Q&A pairs, and claims/evidence fields for the chapter.

---

### User Story 2 - Measure assistant and agent impact (Priority: P2)

As the Analytics Owner, I can track assistant citations, attributed assistant referrals, retrieval‑feed usage, and monthly eval accuracy so we can iterate toward targets.

**Why this priority**: Without measurement, we cannot optimize AEO/GEO; analytics informs our backlog.

**Independent Test**: Trigger test queries to target assistants; capture citation/referral events; run monthly evaluator and export a report.

**Acceptance Scenarios**:

1. **Given** an assistant query set, **When** we execute the evaluation, **Then** the report shows accuracy, coverage, and citation counts attributed to our corpus.
2. **Given** the retrieval endpoint, **When** agents fetch pages of knowledge units, **Then** usage is logged and surfaced in a dashboard.

---

### User Story 3 - Author answerable content blocks (Priority: P3)

As a Content Editor, I can add concise Q&A blocks, HowTo steps, and definitions per section, with optional counterexamples, so content is directly answerable for assistants.

**Why this priority**: Improves selection probability for answer engines and simplifies retrieval chunking.

**Independent Test**: Lint a chapter for presence/quality of Q&A/HowTo blocks; preview resulting JSON‑LD.

**Acceptance Scenarios**:

1. **Given** a chapter in markdown, **When** the content linter runs, **Then** it flags missing or malformed Q&A/HowTo sections and suggests fixes.
2. **Given** valid blocks, **When** the site builds, **Then** JSON‑LD includes FAQPage/HowTo entries aligned with on‑page content.

---

[Add more user stories as needed, each with an assigned priority]

### Edge Cases

- Chapters not suitable for FAQ/HowTo (pure narrative) — ensure Article schema still complete without forcing FAQ.
- Conflicting or duplicate canonical URLs — must fail validation and block publish until fixed.
- Stale last_updated dates — treat as low freshness; included in AEO ranking; add editor reminder.
- Private/paid content — exclude from public feed while keeping internal eval coverage.
- Retrieval rate limiting — agents must be rate‑limited and receive retry headers.

## Requirements _(mandatory)_

<!--
  ACTION REQUIRED: The content in this section represents placeholders.
  Fill them out with the right functional requirements.
-->

### Functional Requirements

- **FR-001**: System MUST embed valid structured data (JSON‑LD) on each chapter page using appropriate schema types (Article and, where applicable, FAQPage and HowTo) with canonical_url, authorship, license, and last_updated.
- **FR-002**: System MUST expose a retrieval‑ready feed of atomic knowledge units (paged JSON) with stable IDs, titles, summaries, Q&A pairs, claims/evidence, canonical_url, authors[], last_updated, and license.
- **FR-003**: System MUST support authoring and build‑time generation of Q&A and HowTo blocks directly from markdown sections.
- **FR-004**: System MUST publish sitemaps that include site pages and retrieval feed endpoints; deep links to fragment anchors MUST be deterministic.
- **FR-005**: System MUST include provenance metadata and visible attribution on every page; adopt lightweight signed metadata initially, with a milestone to evaluate C2PA asset signing for public assets.
- **FR-006**: System MUST capture analytics for assistant citations, attributed assistant referrals, retrieval endpoint usage, and monthly evaluator results.
- **FR-007**: System MUST provide a monthly AEO/GEO evaluator that executes a curated question set against our corpus and reports accuracy, coverage, and citation rate.
- **FR-008**: System MUST reconcile terminology: treat "GRO" as GEO (Generative Engine Optimization) and use SEO/AEO/GEO consistently across the project.
- **FR-009**: System MUST define initial KPI targets per channel (SEO/AEO/GEO) and surface them in an analytics view; adopt the proposed 60‑day baselines (≥50 assistant citations/week; ≥5% assistant‑attributed referrals among organic sessions) and revisit monthly.

- **FR-011**: System MUST maintain a channel registry and query sets for major AI assistants/search engines (e.g., ChatGPT, Perplexity, Google AIO) to map, monitor, and continuously improve rankings in a platform‑safe way.
- **FR-010**: System MUST lint chapters for answerability (presence/quality of Q&A/HowTo blocks, summaries, definitions) and block publish on critical failures.

### Key Entities _(include if feature involves data)_

- **KnowledgeUnit**: id, title, summary, questions[], answers[], claims[], evidence[], canonical_url, last_updated, license, authors[]
- **RetrievalFeedPage**: page_number, page_size, items[] (KnowledgeUnit), next_page
- **CitationEvent**: id, source_assistant, query, destination_url, timestamp, channel (SEO|AEO|GEO), attributed

## Assumptions & Dependencies

- Book is built with a static site generator that allows injecting JSON‑LD and generating sitemaps.
- Analytics can record assistant citations and referrals via server logs or client events without naming specific vendors.
- Legal review approves chosen license and provenance policy.

## Success Criteria _(mandatory)_

<!--
  ACTION REQUIRED: Define measurable success criteria.
  These must be technology-agnostic and measurable.
-->

### Measurable Outcomes

- **SC-001**: 90% of the top 50 pages validate with complete structured data (Article and, where applicable, FAQPage/HowTo) with no errors.
- **SC-002**: Retrieval feed is accessible and complete: at least 500 KnowledgeUnits with stable IDs; pagination works end‑to‑end; assistants can fetch answers in under 1 second for 95% of test queries.
- **SC-003**: Within 60 days of launch, achieve ≥50 assistant citations/week and ≥5% assistant‑attributed referrals among organic sessions.
- **SC-004**: Monthly AEO/GEO evaluation achieves ≥80% answer accuracy and ≥85% coverage using only our corpus.
- **SC-005**: 0 duplicate/conflicting canonical URLs; 100% of pages show visible authorship and license.
