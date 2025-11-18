---
id: 0008
title: Setup robots, sitemap, seeds
stage: green
date: 2025-11-04
surface: agent
model: github-copilot
feature: 001-seo-aeo-geo
branch: 001-seo-aeo-geo
user: user
command: Implement setup tasks (robots.txt, sitemap config) and seed registry
labels: [seo, aeo, geo, setup]
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
  - none yet
---

## Prompt

<conversation-summary>
<analysis>
[Chronological Review: Walk through conversation phases: initial request → exploration → implementation → debugging → current state]
- Phase 1: Concepts and Direction
  - User asked what AEO, SEO, and GEO are and how they look in 2026 for an AI‑native context. We defined SEO (search), AEO (answer engines), and GEO (generative engines), and outlined a 2026 playbook aligned to AI‑native distribution (answerability, provenance, retrieval feeds, evaluation).
- Phase 2: Specification Initialization
  - Followed the specify workflow to create a new feature “SEO/AEO/GEO opportunities.” Created a dedicated branch and spec scaffold; populated the spec with user stories, edge cases, functional requirements, entities, assumptions, and measurable success criteria. Created a quality checklist.
- Phase 3: Clarifications
  - User clarified that “GRO” should be considered “GEO,” wants SEO/GEO to map/monitor/continuously improve AI search rankings across ChatGPT, Perplexity, Google AIO, etc., and confirmed AEO = Answer Engine Optimization. We resolved three spec clarifications (GRO→GEO; provenance policy; KPI thresholds) and added a channel registry FR.
- Phase 4: Planning
  - Executed planning: generated plan.md, research.md, data-model.md, contracts (OpenAPI for retrieval feed), and quickstart.md. Added robots.txt and sitemap strategy. Updated agent context. Logged PHR.
- Phase 5: Tasks
  - Generated tasks.md organized by phases and user stories (strict checklist format). Covered robots.txt, sitemap config, JSON‑LD injection, retrieval feed scaffolding, analytics, evaluator, and polish work. Logged PHR.
- Phase 6: Expectations and Authoring Policy
  - Summarized expected deliverables and measurable outcomes. Created a ground rules doc for authors to ensure answerability (without auto-editing chapters). The user mandated “no content edits unless proven.” We updated the plan constraints and tasks to enforce an evidence‑first policy (render JSON‑LD via derived metadata, generate reports, no direct doc edits unless evaluator/lint justifies).
- Phase 7: Implementation Prep
  - Began the implement workflow by running prerequisites, checking checklists, and reading tasks/plan and key repo files (gitignore, Docusaurus config) to prepare US1 implementation under the evidence‑first policy.

[Intent Mapping: List each explicit user request with message context]

- “What are these AEO, SEO and GEO…share how it will look like in 2026”: Definition and future outlook.
- “Follow instructions in sp.specify…identify SEO, AEO and GRO opportunities”: Create a feature spec and scaffolding.
- “Yeah and what about robots.ts and sitemap…?”: Include robots/sitemap implementation strategy in the plan.
- “Follow instructions in sp.tasks…”: Generate tasks.md structured by user stories with strict checklist format.
- “What are the expectations once it’s done?”: Provide concrete deliverables and measurable outcomes.
- “Are you planning any updates in content…What are the ground rules?”: Provide authoring ground rules and clarify approach to content updates.
- “no we dont edit content unless procevn”: Enforce an evidence‑first policy (no content edits without evaluator/lint justification).
- “Follow instructions in sp.implement…”: Start implementation workflow with prerequisites, checks, and context loading.

[Technical Inventory: Catalog all technologies, patterns, and decisions mentioned]

- Site/runtime: Docusaurus static site (TypeScript config), static assets under book-source/.
- SEO controls: Static robots.txt (book-source/static/robots.txt), sitemap configuration in book-source/docusaurus.config.ts.
- Structured data: JSON‑LD for Article universally; FAQPage/HowTo when Q&A/steps exist; injected via Root.tsx and MDXComponents.tsx.
- Retrieval for GEO: Paged JSON feed under book-source/static/api/knowledge-feed/ with stable IDs and metadata.
- Analytics: Capture assistant citations/referrals and feed usage; evaluator harness for AEO/GEO accuracy/coverage.
- Provenance: Start with lightweight signed metadata; evaluate C2PA later.
- Policy: Evidence‑first content edits; channel registry for AI search/assistants; KPIs for 60‑day ramp.
- Spec/plan framework artifacts: spec.md, plan.md, research.md, data-model.md, contracts/, quickstart.md, tasks.md, checklists, PHRs.

[Code Archaeology: Document every file, function, and code change discussed]

- Created/Updated:
  - Feature branch: 001-seo-aeo-geo
  - specs/001-seo-aeo-geo/spec.md: Concrete spec (US1/US2/US3; FR‑001..011; SC‑001..005).
  - specs/001-seo-aeo-geo/checklists/requirements.md: Checklist marked complete (clarifications resolved).
  - specs/001-seo-aeo-geo/plan.md: Summary, technical context (robots/sitemap, JSON‑LD), constitution check, structure; later updated constraints to “no direct docs edits unless evidence.”
  - specs/001-seo-aeo-geo/research.md: Decisions (GRO→GEO, provenance policy, KPI targets, channel registry, robots/sitemaps).
  - specs/001-seo-aeo-geo/data-model.md: Entities (KnowledgeUnit, RetrievalFeedPage, CitationEvent).
  - specs/001-seo-aeo-geo/contracts/retrieval-feed.openapi.yaml: Feed endpoint schema.
  - specs/001-seo-aeo-geo/quickstart.md: Operational steps to validate and iterate.
  - specs/001-seo-aeo-geo/tasks.md: Tasks (strict checklist), later revised to evidence‑first (US1 avoids content edits; report/findings only).
  - docs/WRITER-AUTHORING-GROUND-RULES.md: Authoring ground rules and snippets.
  - Multiple PHR files under history/prompts/001-seo-aeo-geo/: 0001 (spec), 0002 (clarify), 0003 (plan), 0004 (tasks), 0005 (expectations), 0006 (authoring rules), 0007 (evidence‑first policy).
- Referenced (read-only):
  - .gitignore (existing)
  - book-source/docusaurus.config.ts (existing site config)

[Progress Assessment: What's done vs. pending with specific status]

- Completed:
  - Feature spec and checklist; clarifications applied (GRO→GEO; provenance; KPIs; FR‑011 channel registry).
  - Plan, research, data model, contracts, quickstart; agent context updated.
  - Tasks.md generated; later adjusted to evidence‑first (no content edits without justification).
  - Authoring ground rules doc added; expectations documented.
  - PHRs capturing each stage.
- Pending Implementation (US1 MVP):
  - Add robots.txt; verify sitemap config and submission.
  - Implement JSON‑LD injection scaffolding (Root.tsx, MDXComponents.tsx) without editing docs content.
  - Produce first retrieval feed page (static JSON).
  - Run answerability lint and structured‑data validation to produce findings (no edits).
- Later Phases:
  - Analytics capture, evaluator harness, monthly report, CI gates, polish tasks.

[Context Validation: Ensure all critical information for continuation is captured]

- Current branch: 001-seo-aeo-geo.
- Evidence‑first policy is in place; tasks and plan reflect it.
- Site paths and config files are known; no code changes made yet in Root.tsx, MDXComponents.tsx, or static robots.txt.
- Multiple “001‑\*” spec folders exist; scripts warn but proceed using explicit FEATURE_DIR.

[Recent Commands Analysis: Last agent commands executed, tool results (truncated if long), immediate pre-summarization state]

- Ran prerequisites for implement:
  - Command: check prerequisites with tasks required
  - Result: Warning “Multiple spec directories found with prefix '001' … Please ensure only one spec directory exists per numeric prefix.” Despite warning, JSON returned:
    - FEATURE_DIR: …/specs/001-seo-aeo-geo
    - AVAILABLE_DOCS: ["research.md","data-model.md","contracts/","quickstart.md","tasks.md"]
- Read requirements checklist:
  - File shows all items checked [x]; clarifications resolved.
- Repo inspection:
  - Listed repo root; .gitignore contents verified (Node/Docusaurus ignores).
  - Read book-source/docusaurus.config.ts (confirmed siteUrl config and existing plugin layout; sitemap plugin not explicitly configured—flagged for tasks T003).
- Immediate state before summarization:
  - We were preparing to execute implementation per tasks: verifying setup, confirming checklist pass, and reviewing site configuration for robots/sitemap/JSON‑LD injection in support of US1 under evidence‑first constraints.

</analysis>

<summary>
[omitted here for brevity in PHR; full text preserved in conversation history]
</summary>
</conversation-summary>

## Response snapshot

- Created book-source/static/robots.txt with Allow/Disallow and Sitemap.
- Enabled sitemap generation in book-source/docusaurus.config.ts with weekly changefreq and priority.
- Added research addendum for channel registry and seed queries.
- Updated specs/001-seo-aeo-geo/tasks.md to mark T001, T002, T003, T005 as completed.
- Attempted to update docs/WRITER-HANDOFF.md with ground rules link; tool failed to patch due to fenced block; proposed addendum instead.

## Outcome

- ✅ Impact: Repository now has robots.txt and explicit sitemap config; research includes seed registry; tasks reflect progress.
- 🧪 Tests: None run yet; validation to follow with JSON-LD and feed tasks.
- 📁 Files: robots.txt created; docusaurus.config.ts updated; tasks.md updated; research-addendum.md created.
- 🔁 Next prompts: Implement JSON-LD injection (T006/T011), scaffold retrieval feed (T009/T010), run lint/validator (T012/T015).
- 🧠 Reflection: Keep edits minimal and evidence-first; revisit WRITER-HANDOFF.md patch with a safer approach or separate addendum doc.

## Evaluation notes (flywheel)

- Failure modes observed: Patch tool failed editing large fenced Markdown files.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Create a WRITER-HANDOFF addendum file and link it from CoWriter Guide if needed.
