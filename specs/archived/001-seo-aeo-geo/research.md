# Research — SEO/AEO/GEO Opportunities

Date: 2025-11-04  
Branch: 001-seo-aeo-geo  
Spec: specs/001-seo-aeo-geo/spec.md

## Decisions and Rationale

### D1 — Terminology: GRO → GEO

- Decision: Treat GRO as GEO (Generative Engine Optimization); use SEO/AEO/GEO consistently.
- Rationale: Avoid term drift; GEO is recognized for generative/assistant ranking.
- Alternatives: Keep GRO distinct (rejected — increases ambiguity without added scope).

### D2 — Provenance policy (initial)

- Decision: Start with lightweight signed metadata; evaluate C2PA for public assets as a later milestone.
- Rationale: Faster to implement; preserves provenance intent; defer heavier ops until needed.
- Alternatives: Immediate C2PA (higher ops cost now), no provenance (reduces citation trust).

### D3 — KPI targets (first 60 days)

- Decision: ≥50 assistant citations/week; ≥5% assistant-attributed referrals among organic sessions; revisit monthly.
- Rationale: Ambitious but achievable; provides clear targets to steer iterations.
- Alternatives: Tighter/looser targets (adjust after first evaluator run).

### D4 — Channel registry and eval sets

- Decision: Maintain a registry for ChatGPT, Perplexity, Google AIO, etc., and seed query sets per channel.
- Rationale: Enables mapping/monitoring of rankings and consistency across channels.
- Alternatives: Ad hoc/manual tracking (incomplete, inconsistent).

### D5 — Robots and Sitemaps strategy

- Decision: Provide robots directives via a static robots.txt under site static assets; ensure sitemap generation is enabled and includes canonical URLs; submit sitemaps.
- Rationale: Standard SEO controls; aligns with static site patterns.
- Alternatives: Framework-specific programmatic robots (unnecessary for static site).

## Open Questions (to monitor, not blocking)

- QO1: Multilingual/hreflang support — not in scope now; plan for future.
- QO2: Assistant-specific submission mechanisms — monitor vendor policies; begin with public sitemaps/feed.

## References (general, not vendor-linked)

- schema.org Article, FAQPage, HowTo
- JSON-LD for structured data
- robots.txt directives (Allow/Disallow, Sitemap)
- Sitemaps protocol (index + per-section)
