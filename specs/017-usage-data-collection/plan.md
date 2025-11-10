# Implementation Plan: Usage Data Collection System (Feature 017)

**Branch**: `017-usage-data-collection` | **Date**: 2025-11-10 | **Spec**: `/specs/017-usage-data-collection/spec.md`  
**Input**: Feature specification for telemetry collection, centralized storage, and error analysis workflow

## Summary

Feature 017 establishes a **phased telemetry collection infrastructure** enabling book development teams to capture and analyze Claude Code usage data. The system comprises:

1. **Client-side telemetry** — Claude Code environment configuration (OTLP export with multiple backends)
2. **Local collection server** — Self-contained docker-compose orchestration (OTLP collector + time-series database) **running on each team member's machine**
3. **Data model** — OpenTelemetry schemas for events, sessions, metrics, and aggregations
4. **Query interface** — SQL/PromQL templates for cost analysis, error pattern detection, and performance insights
5. **Team onboarding** — 15-minute setup guide with troubleshooting and example queries
6. **Data protection** — Dual licensing (CC BY-NC-ND 4.0 for book content, MIT for infrastructure code) ensuring book content cannot be resold while infrastructure remains fully open source
7. **Optional centralization** — Migration path to shared cloud endpoint when team needs cross-member analytics

**Key Innovation**: **Local-first architecture** where each team member runs their own telemetry stack initially, with optional migration to centralized cloud endpoint when satisfied with stability and when cross-team analytics are needed. This phased approach eliminates infrastructure dependencies during development while preserving flexibility for future centralization.

**Deployment Phases**:
- **Phase 1 (NOW)**: Each team member runs `telemetry-server/` locally via docker-compose (localhost:8123)
- **Phase 2 (LATER)**: Optional migration to shared cloud VM when team needs aggregated analytics
- **Phase 3 (OPTIONAL)**: Automated cross-team reports and dashboards

**Licensing Strategy**:
- **ALL repository content**: CC BY-NC-ND 4.0 (free to read and use personally, NOT free to resell or use commercially)
  - Book content (`book-source/docs/`)
  - Infrastructure code (`telemetry-server/`, `.claude/`, `.specify/`)
  - Documentation, scripts, and configuration files
- **Collected data**: Project-private, NOT included in git or repository distribution

## Technical Context

**Language/Version**: 
- Client: Environment variables (shell-agnostic)
- Infrastructure: Docker Compose v3+ (container-agnostic)
- Query layer: SQL (ClickHouse or Prometheus) + PromQL/HTTP
- Documentation: Markdown + example queries (Python/Shell runnable)

**Primary Dependencies**: 
- OpenTelemetry Protocol (OTLP) v1.0+
- OTLP Collector (OpenTelemetry Community distribution)
- ClickHouse or Prometheus (time-series database) — RECOMMEND: ClickHouse for flexibility
- Docker Engine 20.10+, Docker Compose 2.0+

**Storage**: 
- ClickHouse (recommended): Schema-based time-series SQL database, excellent for telemetry ingestion, easy local development
- Prometheus (alternative): Pull-based metrics, simpler setup, limited for event-based data
- Recommendation: ClickHouse for this project (schema flexibility, SQL query power, better for error analysis)

**Testing**: 
- Unit: Python/Shell scripts validating env config, export formats
- Integration: Docker compose spin-up, data export verification, query validation
- End-to-end: Multi-session data collection, aggregation queries, error pattern detection

**Target Platform**: 
- Mac, Linux, WSL (all Unix-like systems with Docker Desktop or Docker Engine)
- Primary: macOS + Linux (development team assumed)
- Must support: Offline degradation (buffer locally, retry on reconnect)

**Project Type**: Infrastructure + documentation (no single "application"; instead, orchestration + guides)

**Performance Goals**: 
- Event ingestion: <50ms latency from Claude Code to local buffer
- Local database: Support <1s query response for 1M+ events
- Team scale: 5-10 concurrent sessions without performance degradation

**Constraints**: 
- Must NOT disrupt Claude Code if telemetry backend fails (non-blocking export)
- Must handle offline scenarios gracefully (buffer to disk, retry exponentially)
- Data sanitization MUST filter sensitive patterns (API keys, PII, credentials) before export
- Collected data MUST NOT be version-controlled (stored outside git)
- Setup must complete in <15 minutes for new team members (SC-001)

**Scale/Scope**: 
- Book team: 5-10 developers across 12-month project
- Expected data volume: ~100-500 events/session, 50-200 sessions/week = 5k-10k events/week
- Storage estimate: ~1-5 MB/week raw telemetry = ~50-250 MB/year (well within 100GB default limit per SC-008)
- Query performance: <3 seconds for 30-day aggregations (SC-003)

## Constitution Check

**Gate: PASS with implementation notes**

✅ **Principle 1 - AI-First Teaching**: Telemetry system tracks AI collaboration patterns, enabling data-driven improvements to prompts and specs. System is intrinsically tied to AI-native methodology validation.

✅ **Principle 2 - Spec-Driven Development**: Feature uses SDD internally (this spec → plan → implementation). Future analysis will validate whether clearer specs reduce validation failures (error analysis use case).

✅ **Principle 4 - Evals-First Development**: Telemetry data becomes the ground truth for measuring whether spec/prompt changes improve outcomes. Success criteria are measurable (SC-001 through SC-008).

✅ **Principle 5 - Validation-First Safety**: Data sanitization is mandatory before export; sensitive information filtered at collection time. Aligns with "never trust AI output without validation" principle.

✅ **Principle 3 - Spec-First**: This feature demonstrates spec-driven development at scale—every upstream component (Claude Code telemetry config, ClickHouse schema) is driven by clear specifications.

⚠️ **No Constitutional Violations**: This feature aligns with graduated complexity (Parts 1-13), bilingual development (infrastructure language-agnostic), and transparency (methodology visible through telemetry traces).

**Implementation Notes**:
- Feature must respect `.git` and `.gitignore` constraints—all collected telemetry data outside version control
- Integration point: Project's existing `.specify/` and `.claude/` directories remain untouched
- Documentation must model transparent methodology (explain OTLP → ClickHouse → SQL flow explicitly)

## Project Structure

### Documentation (this feature)

```text
specs/017-usage-data-collection/
├── plan.md                      # This file
├── research.md                  # Phase 0: Technology research and trade studies
├── data-model.md                # Phase 1: Telemetry schemas, session model, aggregations
├── quickstart.md                # Phase 1: 15-minute setup + example queries + troubleshooting
├── contracts/
│   ├── telemetry-events.json    # OpenTelemetry event schema contract
│   ├── database-schema.sql      # ClickHouse table DDL
│   └── query-api.md             # HTTP query interface (if applicable)
└── checklists/                  # Existing validation checklists
```

### Source Code (repository root)

```text
telemetry-server/               # NEW: Self-contained local telemetry infrastructure
├── docker-compose.yml           # Orchestrates OTLP collector + ClickHouse + setup
├── .env.template                # Environment variable template (copy to .env for local config)
├── collector-config.yaml        # OpenTelemetry Collector configuration
├── clickhouse/
│   ├── schema.sql               # ClickHouse table definitions (auto-executed on startup)
│   ├── init.sh                  # Startup script (optional: pre-populated queries/views)
│   └── .gitignore               # Exclude data directory
├── queries/
│   ├── README.md                # Query reference guide
│   ├── cost-analysis.sql        # Cost by user, chapter, day
│   ├── error-patterns.sql       # Error rate trends, high-failure sessions
│   ├── token-efficiency.sql     # Token usage analysis
│   └── examples.sh              # Runnable shell scripts (curl or HTTP requests)
├── scripts/
│   ├── verify-setup.sh          # Validate local infrastructure is running
│   ├── export-month-report.sh   # Generate monthly cost report (SC-006)
│   └── cleanup.sh               # Archive/delete old data per retention policy
└── docs/
    ├── ARCHITECTURE.md          # Data flow diagram, design decisions
    ├── TROUBLESHOOTING.md       # Common errors and solutions
    ├── DATA-RETENTION.md        # Policy for archiving/deletion
    └── DATA-PRIVACY.md          # Data ownership, licensing, OSS implications

.claude/env-config/              # EXISTING: Claude Code environment templates
├── telemetry-enabled.env        # Template: OTLP export + console logging
└── telemetry-disabled.env       # Template: Telemetry off (default for privacy)

.gitignore                        # UPDATED: Add telemetry-server/clickhouse/data/ and .env files
```

**Structure Decision**: 
Telemetry infrastructure is self-contained in `telemetry-server/` at repository root. This design choice enables:
1. **Isolation**: All telemetry components in one folder; easy to enable/disable project-wide
2. **Portability**: New team members clone repo, run `telemetry-server/` independently of main book workflow
3. **Data ownership**: Local database means collected data never leaves the project (vs. cloud backend)
4. **Easy onboarding**: Single `docker-compose up` command spins up full infrastructure
5. **Git discipline**: All user data (.env, collected events) are .gitignored—code is OSS, data is private

Integration points:
- **`.claude/env-config/`**: Stores telemetry environment templates (team members activate via `.env`)
- **`.specify/memory/`**: Constitution references telemetry as validation mechanism for error analysis
- **`specs/*/`**: Feature specs will reference telemetry session IDs and chapter attribution in future analysis workflows

## Phase 0 - Research

**Deliverable**: `research.md` (detailed analysis of technology options, trade studies, best practices)

### Research Topics

**1. OTLP Collector Options**
- **OpenTelemetry Collector** (Community distribution) — Apache 2.0, CNCF-maintained
  - Pros: Standard OTLP receiver, flexible exporters, mature ecosystem
  - Cons: Moderate resource footprint (100-300MB), requires configuration
  - Recommendation: PRIMARY CHOICE for this project
- **Jaeger** (all-in-one) — Apache 2.0, distributed tracing focus
  - Pros: UI included, trace visualization, easy local development
  - Cons: Heavier footprint, optimized for traces not metrics/events
  - Recommendation: ALTERNATIVE for trace-focused teams
- **Grafana Tempo** — AGPL 3.0, trace storage focus
  - Pros: Cloud-native, great Grafana integration
  - Cons: Complex local setup, heavier resource requirements
  - Recommendation: Defer to later if Grafana dashboards required

**2. Time-Series Database Options**
- **ClickHouse** — BSL/SSPL, OLAP optimized
  - Pros: Schema-flexible, excellent INSERT performance (1M+/sec), powerful SQL, column compression
  - Cons: Heavier footprint (2GB Docker image), SSPL licensing requires attention
  - Recommendation: PRIMARY CHOICE (optimal for telemetry + event analysis)
- **Prometheus** — Apache 2.0
  - Pros: Lightweight, pull-based metrics, large ecosystem
  - Cons: Limited to metrics (not events), data retention challenges
  - Recommendation: SECONDARY CHOICE (use if team already has Prometheus)
- **TimescaleDB** (PostgreSQL extension) — Timescale License/Community Edition
  - Pros: Familiar SQL, excellent for events, ACID guarantees
  - Cons: Smaller ecosystem, fewer specialized tools
  - Recommendation: ALTERNATIVE (solid middle ground)
- **MongoDB with time-series collections** — SSPL
  - Pros: Flexible schema, JSON natural fit
  - Cons: Higher storage overhead, less optimized for time-series queries
  - Recommendation: NOT RECOMMENDED for this project

**RECOMMENDATION**: ClickHouse + OpenTelemetry Collector (both open-source friendly, proven at scale)

**3. Docker Orchestration Patterns**
- **Docker Compose** v3 — Best practice for single-host multi-container coordination
  - Use case: Local development, small team telemetry backend
  - Recommendation: PRIMARY CHOICE for this project
- **Kubernetes** — For production multi-host deployment
  - Not needed for local team telemetry; defer to Part 10+ when teaching Kubernetes
- **Podman Compose** — Open-source Kubernetes-friendly alternative
  - Recommendation: Support as alternative to Docker Compose

**4. OpenTelemetry Best Practices**
- **Schema design**: Use semantic conventions (https://opentelemetry.io/docs/specs/semconv/)
  - Application: Define event types (user-prompt, tool-call, api-request, error) following semconv
- **Sampling**: High-volume sessions may need sampling to manage costs
  - Implementation: Configurable sampling ratio (default 100% for teams <10)
- **Context propagation**: Include session ID, user ID, branch in all events
  - Implementation: Environment variables or config files
- **Graceful degradation**: Buffer locally if backend unavailable
  - Implementation: File-based buffering with retry logic

**5. Data Licensing Patterns for OSS Projects**
- **Code License vs Data License**: Repository is OSS (MIT/Apache), but collected data is project-private
  - Pattern: Add NOTICE file explaining data ownership
  - Pattern: Add DATA-USAGE-POLICY explaining what happens to telemetry
  - Example: "This telemetry data is owned by the Panaversity project and not included in OSS distribution"
- **PDF Distribution Concern**: Even if someone clones repo and generates PDF of code, the telemetry database remains separate
  - Implementation: All collected data stored outside git (`.gitignore`)
  - Implication: Cloned repo contains code + setup scripts, but NO collected data
  - Transparency: Document that cloning the repo does not grant access to usage patterns or team insights
- **Data Rights**: Team members' usage data is aggregated for project improvement only
  - Implementation: Data sanitization removes personal identifiers before aggregation
  - Governance: Define access control (who can query the centralized database)

**6. Cross-Platform Testing**
- Test matrix: macOS (M1/Intel), Linux (Ubuntu 22.04+), WSL2
- Docker Desktop vs Docker Engine compatibility
- Network isolation in corporate environments (proxy requirements)

## Phase 1 - Design

**Deliverables**: 
- `data-model.md` — Telemetry schemas and aggregation models
- `quickstart.md` — 15-minute setup guide for team members
- `contracts/` — API specifications and database schemas

### Design Sections

**1. Data Model Design** (see `data-model.md`)

Core entities:
- **Telemetry Event** — Individual action (prompt, tool call, API request, error)
  - Attributes: Event type, timestamp, session ID, user ID, event-specific data
  - Relationships: Belongs to Session
- **Session** — Single Claude Code CLI invocation
  - Attributes: Session ID, user ID, start time, end time, branch context
  - Relationships: Contains multiple Events
- **Aggregated Metrics** — Computed rollups for reporting
  - Examples: Cost by user/day, token count by chapter, error rate trends

**2. Event Schema Design** (see `contracts/telemetry-events.json`)

OpenTelemetry event types:
```
user_prompt         — User typed a prompt into Claude Code
                      Attributes: prompt_text, token_count_estimate, session_id
tool_call           — Claude Code invoked a tool (file edit, bash, etc.)
                      Attributes: tool_name, parameters, duration_ms, success_status
api_request         — API call made (Claude, tools, etc.)
                      Attributes: api_name, tokens_input, tokens_output, cost_usd
api_error           — API call failed
                      Attributes: error_type, status_code, retry_attempt
validation_result   — User validation/review of AI output
                      Attributes: outcome (approved/rejected/modified), feedback_text
```

**3. ClickHouse Schema Design** (see `contracts/database-schema.sql`)

Tables:
- `telemetry_events` — Raw event stream (insert-optimized)
- `sessions` — Session metadata (one row per CLI session)
- `users` — User directory (one row per team member)
- `daily_metrics` — Pre-aggregated cost/token/error metrics

**4. Quickstart Guide** (see `quickstart.md`)

Target: New team member enables telemetry in <15 minutes
Sections:
- Prerequisites (Docker Desktop/Engine installed)
- Step 1: Clone telemetry-server and run docker-compose
- Step 2: Configure Claude Code environment (.env)
- Step 3: Verify data export (sample query)
- Troubleshooting (common errors)
- Example queries (cost by chapter, top errors)

**5. API Contracts** (see `contracts/`)

Query interface:
- HTTP REST (curl-friendly for shell scripts)
- SQL dialect: ClickHouse (or Prometheus PromQL if alternative backend)
- Example: `GET http://localhost:8123/?query=SELECT * FROM telemetry_events LIMIT 10`

## Complexity Tracking

No constitutional violations identified. This feature is orthogonal to core book content (Parts 1-13) and serves as infrastructure for team collaboration.

## Licensing & Data Protection Strategy

**File Structure**:
```
LICENSE                         # Existing: Code license (e.g., MIT)
NOTICE                          # NEW: Data ownership notice
DATA-USAGE-POLICY               # NEW: Telemetry data governance
telemetry-server/.gitignore     # NEW: Exclude collected data from git
.env.template                   # NEW: Environment template (included in repo)
.env                            # .gitignored: Actual secrets/config
telemetry-server/clickhouse/data/ # .gitignored: Collected telemetry database
```

**NOTICE File** (Repository root):
```
USAGE DATA COLLECTION

This repository includes infrastructure for collecting usage telemetry
during development of AI-native software development content.

IMPORTANT: This telemetry data is owned by the Panaversity project and
is NOT included in the open-source distribution of this repository's code.

When you clone this repository, you receive:
- Book content, specification templates, example code (all OSS)
- Telemetry infrastructure (configuration, database schema, query examples)
- Environment configuration templates

You do NOT receive:
- Any collected usage data from other team members
- Query results or analysis performed on centralized telemetry database
- Personal or session-specific insights

The collected data is managed separately and used ONLY for:
1. Improving content quality and teaching methodology
2. Analyzing AI agent performance patterns
3. Optimizing book structure based on error analysis
4. Generating team productivity reports (for project coordinators only)

Data remains with the project and is never shared externally without
explicit consent of all contributing team members.
```

**DATA-USAGE-POLICY File** (Repository root):
```
TELEMETRY DATA USAGE POLICY

1. DATA OWNERSHIP
   - All usage data collected is owned by Panaversity
   - Contributors retain ownership of their own data contributions
   - Aggregated insights belong to the project for educational improvement

2. DATA COLLECTION SCOPE
   - User prompts (text submitted to Claude Code)
   - Tool calls and their results (file operations, CLI execution)
   - API metrics (token counts, costs, duration)
   - Error patterns and validation failures
   - Session metadata (duration, chapter context, branch name)

3. DATA SANITIZATION
   - API keys and credentials are filtered before export
   - PII (personal names, emails) are removed or hashed
   - Proprietary code snippets can be excluded via configuration
   - Sensitive patterns defined in DATA-SANITIZATION.md

4. DATA RETENTION
   - Raw telemetry: 90 days (default, configurable)
   - Aggregated metrics: Indefinite (anonymized)
   - User request: Data purged upon request within 30 days

5. DATA ACCESS
   - Team members: Can query their own session data
   - Project coordinators: Can query aggregated metrics (cost, errors, trends)
   - Public: No access to any raw or aggregated data

6. PDF DISTRIBUTION & CLONING
   - Cloning this repository does NOT include team's telemetry data
   - PDF generation from source code does NOT include telemetry insights
   - Data remains local to the original development environment
   - No data is bundled with code distribution

7. OPEN-SOURCE COMPATIBILITY
   - Repository code is open-source (MIT/Apache)
   - Telemetry data is project-private (not part of OSS distribution)
   - Contributors agree that their usage data is used for project improvement
   - No external data sharing without explicit consent

8. AUDIT & TRANSPARENCY
   - Weekly data retention reports generated automatically
   - Access logs maintained (who queried what, when)
   - Data governance board (project coordinators) reviews monthly
   - Transparency: All data handling visible in this policy
```

**`.gitignore` Updates**:
```
# Telemetry data (never version control collected usage data)
telemetry-server/clickhouse/data/
telemetry-server/.env

# Environment secrets
.env
.env.local
```

**Why This Matters for OSS Distribution**:
- **Code cloning**: Someone clones repo → gets all code, setup scripts, database schema, query templates
- **PDF generation**: Someone runs `pandoc` or similar → gets rendered code, but NO data (database is separate)
- **Data remains private**: Collected telemetry stored locally in Docker volume, outside git
- **Transparency**: NOTICE and DATA-USAGE-POLICY explain the separation clearly
- **No hidden data**: Unlike SaaS telemetry that phones home, this data stays on-premise

## Acceptance Criteria

- [ ] All Phase 0 research completed and documented in `research.md`
- [ ] All Phase 1 designs finalized and documented in `data-model.md`, `quickstart.md`, `contracts/`
- [ ] Project structure created (telemetry-server/ folders, docker-compose.yml, schema files)
- [ ] NOTICE and DATA-USAGE-POLICY files created at repository root
- [ ] .gitignore updated to exclude collected data
- [ ] Constitution check passed (no violations)
- [ ] Phase 2 tasks.md ready for implementation (generated via `/sp.tasks` command)

## Follow-Ups & Risks

**Risk 1**: Docker image sizes may strain local systems
- **Mitigation**: Provide resource limits in docker-compose.yml; document minimum requirements (4GB RAM, 10GB disk)

**Risk 2**: Team members hesitant to enable telemetry due to privacy concerns
- **Mitigation**: NOTICE and DATA-USAGE-POLICY address data ownership explicitly; optional participation with clear benefits

**Risk 3**: ClickHouse query language unfamiliar to team
- **Mitigation**: Provide example queries as shell scripts (copy-paste friendly); create query builder documentation

**Risk 4**: SSPL/BSL licensing concerns with ClickHouse
- **Mitigation**: Document licensing in ARCHITECTURE.md; provide Prometheus alternative (Phase 1 design supports both)

**Next Step**: Generate `tasks.md` via `/sp.tasks` command to decompose Phase 1 and Phase 2 work into atomic development tasks.
