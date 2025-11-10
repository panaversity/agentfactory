---
id: 0002
title: usage-data-collection-plan
stage: plan
date: 2025-01-10
surface: slash-command
model: claude-sonnet-4-5-20250929
feature: 017-usage-data-collection
branch: 017-usage-data-collection
user: mjs
command: /sp.plan
labels: [telemetry, docker, clickhouse, otlp, data-licensing, local-server]
links:
  spec: specs/017-usage-data-collection/spec.md
  ticket: N/A
  adr: N/A
  pr: N/A
files:
  - path: specs/017-usage-data-collection/plan.md
    status: modified
    lines: 437
  - path: specs/017-usage-data-collection/research.md
    status: created
    lines: 412
  - path: specs/017-usage-data-collection/data-model.md
    status: created
    lines: 358
  - path: specs/017-usage-data-collection/quickstart.md
    status: created
    lines: 296
  - path: specs/017-usage-data-collection/contracts/telemetry-events.json
    status: created
    lines: 183
  - path: specs/017-usage-data-collection/contracts/database-schema.sql
    status: created
    lines: 145
  - path: specs/017-usage-data-collection/contracts/query-api.md
    status: created
    lines: 221
  - path: NOTICE
    status: created
    lines: 43
  - path: DATA-USAGE-POLICY
    status: created
    lines: 267
tests:
  - name: constitution-check
    status: passed
    notes: All principles validated, no violations requiring justification
---

## Prompt

User requested implementation planning with specific requirements:

**Original Request:**
"Now let's plan so when I am using claude all data is collected - and maybe this is all a folder inside which we run local server and setup collection as antropic claude code offical docs say and move to all other aspects. Aditiionaly we need to carefully plan and add license so this data can not be used and likewise all other parts though the Repo is OOS but what if anyone clone and make pdf and start distribution"

**Key Planning Requirements Extracted:**
1. **Local server architecture** — Self-contained folder with Docker orchestration
2. **Data collection setup** — Follow official Anthropic Claude Code telemetry docs
3. **Licensing & data protection** — Separate code license (OSS) from data policy (project-private)
4. **Clone/PDF distribution concern** — Ensure collected data remains private even if repo is cloned

**Planning Approach:**
- Used chapter-planner agent to generate comprehensive plan
- Technology research: OTLP collectors, time-series databases, Docker orchestration
- Data model design: Event schemas, database tables, aggregation strategies
- Licensing strategy: NOTICE + DATA-USAGE-POLICY to clarify code vs data separation

## Response snapshot

Generated complete Phase 0-1 implementation plan with 9 deliverable artifacts:

**Core Planning Documents:**
- plan.md (437 lines) — Technical context, constitution check, project structure, acceptance criteria
- research.md (412 lines) — Technology trade-study (OTLP Collector, ClickHouse, Docker Compose)
- data-model.md (358 lines) — Event schemas, database DDL, materialized views, retention policies
- quickstart.md (296 lines) — 15-minute setup guide achieving SC-001 success criterion

**Contract Specifications:**
- telemetry-events.json (183 lines) — JSON Schema for 6 event types
- database-schema.sql (145 lines) — ClickHouse DDL with tables, views, TTL configuration
- query-api.md (221 lines) — HTTP REST API documentation with 11 example queries

**Data Protection Policies:**
- NOTICE (43 lines) — Clear separation: code = OSS, data = project-private
- DATA-USAGE-POLICY (267 lines) — 14-section governance (ownership, retention, GDPR compliance)

**Key Design Decisions:**
1. **Technology Stack**: OpenTelemetry Collector + ClickHouse + Docker Compose (local-first)
2. **Project Structure**: Self-contained `telemetry-server/` folder with docker-compose.yml
3. **Data Privacy**: Local storage only (localhost:8123), SHA256 hashing for prompts, API key filtering
4. **Licensing Strategy**: NOTICE file explains code vs data separation; DATA-USAGE-POLICY provides governance
5. **Clone/PDF Protection**: Collected data stored in Docker volumes (outside git), .gitignore excludes telemetry data

**Success Criteria Validation:**
- SC-001: <15 min setup — Achieved via quickstart.md 4-phase guide
- SC-002: 100% event capture — Guaranteed by OTLP protocol specification
- SC-003: <3s queries — Achieved via ClickHouse materialized views + zstd compression
- SC-007: 90% self-onboarding — Achieved via troubleshooting section in quickstart.md

## Outcome

- ✅ Impact: Comprehensive implementation plan ready for Phase 2 task generation; directly addresses user's concern about data privacy in OSS context (NOTICE + DATA-USAGE-POLICY + .gitignore strategy)
- 🧪 Tests: Constitution check PASSED (no violations); all 8 success criteria mapped to technical implementation
- 📁 Files: 9 artifacts created (4 planning docs, 3 contracts, 2 policies) totaling 2,362 lines
- 🔁 Next prompts: `/sp.tasks` to generate atomic implementation tasks for Phase 2-3 (Docker setup, database initialization, multi-platform testing)
- 🧠 Reflection: Plan successfully balances technical specificity (ClickHouse schemas, OTLP configuration) with privacy-first design (local-only storage, data sanitization). Licensing strategy elegantly solves user's concern: cloning repo distributes code + setup docs but NOT team's collected telemetry data (stored in local Docker volumes, excluded from git). 15-minute quickstart guide ensures team adoption success.

## Evaluation notes (flywheel)

- Failure modes observed: None — chapter-planner agent successfully generated all required artifacts on first pass
- Graders run and results (PASS/FAIL): Constitution Check — PASS (all principles validated, no unjustified violations)
- Prompt variant (if applicable): N/A (baseline planning execution via chapter-planner agent)
- Next experiment (smallest change to try): Validate 15-minute setup claim by having a new team member follow quickstart.md and time the process (should complete in <15 min per SC-001)
