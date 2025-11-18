# Quickstart — SEO/AEO/GEO Optimization

This quickstart outlines the minimal steps to validate and iterate.

1. Validate answerability signals (no content edits)

- Confirm Article JSON‑LD is injected server-side on 10 target pages (no edits to MDX).
- Use the JSON‑LD presence check script to verify exactly one jsonld-article script in page HTML.
- Record results in tests/unit/answerability.lint.md (report-only; do not edit content).

2. Expose retrieval feed

- Implement read-only paged JSON feed for KnowledgeUnits (see contracts/retrieval-feed.openapi.yaml).
- Generate stable IDs and timestamps.

3. Robots and Sitemaps

- Add/verify static robots.txt under site static assets; allow crawling of public docs and the feed.
- Ensure sitemaps are generated and include canonical URLs; submit to major engines.

4. Channel registry and evaluator

- Seed query sets for ChatGPT, Perplexity, Google AIO; run monthly evaluator; capture citations and referrals.

5. Analytics

Appendix — Structured‑data validator steps (US1)

- Choose 10 target pages (e.g., Preface, Part intros, core method pages).
- For each URL:
  - Check presence of one Article JSON‑LD script in HTML source (no duplicates).
  - Capture any validator errors/warnings and suggested optional frontmatter fields (canonical_url, license, last_updated).
- Record per-page results in tests/unit/answerability.lint.md.
- Only after a green report, consider optional edits via PR with evidence attached.

- Log assistant citations/referrals and feed usage; review monthly against targets.
