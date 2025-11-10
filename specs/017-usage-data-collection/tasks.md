# Tasks: Usage Data Collection System (Feature 017)

**Input**: Design documents from `/specs/017-usage-data-collection/`  
**Prerequisites**: plan.md ✅, spec.md ✅, research.md ✅, data-model.md ✅, contracts/ ✅

**Core Focus**: Local-first telemetry data collection infrastructure with phased deployment strategy

**User Context**: "Now let's break down the tasks the core is data collection right now so carefully plan the implementation"

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4)
- File paths use repository root as base

## User Stories Summary

| Priority | Story | Goal | Independent Test |
|----------|-------|------|------------------|
| **P1** 🎯 | **US1**: Enable Usage Data Collection | Local telemetry collection per team member | Run Claude Code with telemetry → verify events in ClickHouse |
| **P2** | **US2**: Centralized Data Storage | Aggregate team data | Multiple members export to shared endpoint → query aggregates |
| **P3** | **US3**: Error Analysis Workflow | Identify failure patterns | Query for high-retry sessions → review traces |
| **P4** | **US4**: Team Documentation & Onboarding | <15min setup | New member follows docs → telemetry working |

**MVP Scope**: User Story 1 only (local collection infrastructure)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and directory structure

**Duration**: ~30 minutes

- [ ] T001 Create `telemetry-server/` directory in repository root
- [ ] T002 Create `.gitignore` entries for telemetry data (`telemetry-server/clickhouse_data/`, `telemetry-server/otel_logs/`, `*.db`, `*.log`)
- [ ] T003 [P] Create `telemetry-server/docs/` directory for documentation
- [ ] T004 [P] Create `telemetry-server/queries/` directory for example SQL queries
- [ ] T005 [P] Create `telemetry-server/scripts/` directory for utility scripts
- [ ] T006 Create `telemetry-server/.env.template` with placeholder environment variables

**Checkpoint**: Directory structure ready for implementation

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

**Duration**: ~2-3 hours

### Docker Infrastructure

- [ ] T007 Create `telemetry-server/docker-compose.yml` with services: clickhouse, otel-collector
- [ ] T008 Create `telemetry-server/clickhouse/Dockerfile` (if custom image needed, otherwise use official)
- [ ] T009 Create `telemetry-server/otel-collector/config.yaml` with OTLP receivers and ClickHouse exporter configuration
- [ ] T010 Configure Docker volumes for persistent data: `clickhouse_data` and `otel_logs`
- [ ] T011 Configure Docker networks for service communication
- [ ] T012 Create `telemetry-server/docker-compose.override.yml.example` for local customization

### ClickHouse Schema (from contracts/database-schema.sql)

- [ ] T013 Create `telemetry-server/clickhouse/schema.sql` with complete database schema from `/specs/017-usage-data-collection/contracts/database-schema.sql`
- [ ] T014 Create `telemetry-server/clickhouse/init-db.sh` script to initialize database on first run
- [ ] T015 Test ClickHouse schema creation: `docker-compose up clickhouse` → verify schema loads without errors

### OTLP Collector Configuration

- [ ] T016 Configure OTLP receiver in `otel-collector/config.yaml` (port 4317 for gRPC, 4318 for HTTP)
- [ ] T017 Configure ClickHouse exporter in `otel-collector/config.yaml` with connection details
- [ ] T018 Configure console exporter in `otel-collector/config.yaml` for debugging (optional, stderr output)
- [ ] T019 Configure telemetry pipelines: logs → ClickHouse, metrics → ClickHouse
- [ ] T020 Test OTLP Collector: `docker-compose up otel-collector` → verify starts without errors and connects to ClickHouse

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Enable Usage Data Collection (Priority: P1) 🎯 MVP

**Goal**: Local telemetry collection on each team member's machine

**Independent Test**: Start Claude Code with `CLAUDE_CODE_ENABLE_TELEMETRY=1` and `OTEL_EXPORTER_OTLP_ENDPOINT=http://localhost:4317`, execute prompts and tool calls, query ClickHouse to verify events are captured

**Why P1**: Foundational capability - without local collection, no other stories possible

**Duration**: ~4-6 hours

### Environment Configuration (Client-Side)

- [ ] T021 [US1] Create `.claude/env-config/telemetry-enabled.env` template with `CLAUDE_CODE_ENABLE_TELEMETRY=1` and `OTEL_EXPORTER_OTLP_ENDPOINT=http://localhost:4317`
- [ ] T022 [US1] Create `.claude/env-config/telemetry-disabled.env` template with `CLAUDE_CODE_ENABLE_TELEMETRY=0` (default)
- [ ] T023 [US1] Document in `telemetry-server/README.md` how to enable telemetry (export env vars in shell profile)

### Docker Compose Orchestration

- [ ] T024 [US1] Add healthchecks to `docker-compose.yml` for clickhouse (port 8123) and otel-collector (port 4317)
- [ ] T025 [US1] Configure restart policies for services: `unless-stopped` or `always`
- [ ] T026 [US1] Add resource limits to docker-compose.yml (memory: 1GB for ClickHouse, 256MB for OTLP Collector)

### Data Collection Verification

- [ ] T027 [US1] Create `telemetry-server/scripts/verify-collection.sh` to check if events are flowing (query ClickHouse for recent events)
- [ ] T028 [US1] Create `telemetry-server/scripts/start-telemetry.sh` wrapper for `docker-compose up -d` with pre-flight checks
- [ ] T029 [US1] Create `telemetry-server/scripts/stop-telemetry.sh` wrapper for `docker-compose down`

### Event Schema Validation (Client Contracts)

- [ ] T030 [P] [US1] Document expected event schema in `telemetry-server/docs/EVENT-SCHEMA.md` (copy from `/specs/017-usage-data-collection/data-model.md`)
- [ ] T031 [P] [US1] Create example events JSON in `telemetry-server/docs/example-events.json` with sample user_prompt, tool_call, api_request events

### Data Sanitization Configuration

- [ ] T032 [US1] Create `telemetry-server/otel-collector/filters.yaml` with regex patterns for API keys, credentials, PII
- [ ] T033 [US1] Configure OTLP Collector processors to apply filters before export to ClickHouse
- [ ] T034 [US1] Test data sanitization: Send event with fake API key → verify it's hashed/filtered in ClickHouse

### Integration Testing

- [ ] T035 [US1] Test full local collection workflow: Start services → Enable telemetry → Run Claude Code session → Query events in ClickHouse
- [ ] T036 [US1] Test offline graceful degradation: Stop Docker services → Run Claude Code → Verify no crashes (events should buffer or be skipped)
- [ ] T037 [US1] Test high-volume session: Generate 1000+ events → Verify no data loss or performance degradation

**US1 Checkpoint**: Local telemetry collection working end-to-end on single machine

---

## Phase 4: User Story 2 - Centralized Data Storage (Priority: P2)

**Goal**: Aggregate multiple team members' data into shared database

**Independent Test**: Configure 2+ Claude Code instances to export to same endpoint → Query aggregated metrics across users

**Dependencies**: **MUST complete US1 first** (local collection validates before centralization)

**Duration**: ~3-4 hours

### Cloud/Shared Deployment Configuration

- [ ] T038 [US2] Create `telemetry-server/deployment/cloud-docker-compose.yml` for shared server deployment (expose ports 4317, 8123)
- [ ] T039 [US2] Create `telemetry-server/deployment/README-CLOUD.md` with instructions for deploying to cloud VM (AWS, DigitalOcean, Azure)
- [ ] T040 [US2] Document firewall configuration in cloud README (allow ports 4317/TCP for OTLP, 8123/TCP for HTTP queries)

### Multi-User Configuration

- [ ] T041 [US2] Update `.env.template` with `OTEL_EXPORTER_OTLP_ENDPOINT` placeholder for cloud endpoint (e.g., `https://telemetry.yourproject.com:4317`)
- [ ] T042 [US2] Document in `telemetry-server/docs/MIGRATION.md` how to transition from local → cloud (export local data, reconfigure env vars)
- [ ] T043 [US2] Create `telemetry-server/scripts/export-local-data.sh` to dump local ClickHouse data to CSV

### Aggregation Queries (from contracts/query-api.md)

- [ ] T044 [P] [US2] Create `telemetry-server/queries/cost-by-user.sql` for total cost per team member
- [ ] T045 [P] [US2] Create `telemetry-server/queries/cost-by-chapter.sql` for cost grouped by feature/chapter context
- [ ] T046 [P] [US2] Create `telemetry-server/queries/token-usage-by-user.sql` for token consumption per user
- [ ] T047 [P] [US2] Create `telemetry-server/queries/token-usage-trends.sql` for daily/weekly token usage trends
- [ ] T048 [P] [US2] Create `telemetry-server/queries/sessions-summary.sql` for session count, duration, events per user

### Role-Based Access Control (Optional, for shared deployment)

- [ ] T049 [US2] Document in `MIGRATION.md` how to configure ClickHouse user accounts for role-based access (contributors see own data, coordinators see all)
- [ ] T050 [US2] Create example ClickHouse user configuration in `telemetry-server/clickhouse/users.xml` (if using XML config)

### Integration Testing

- [ ] T051 [US2] Test centralized collection: Start cloud deployment → Configure 2 local Claude Code instances with cloud endpoint → Verify both users' data appears in shared database
- [ ] T052 [US2] Test aggregation queries: Run cost-by-user.sql with multi-user data → Verify correct grouping and totals

**US2 Checkpoint**: Multi-user centralized data storage operational

---

## Phase 5: User Story 3 - Error Analysis Workflow (Priority: P3)

**Goal**: Query workflow traces to identify failure patterns

**Independent Test**: Insert mock failure events → Query for high-retry sessions → Verify trace retrieval

**Dependencies**: **MUST complete US2 first** (needs centralized data with multiple sessions)

**Duration**: ~2-3 hours

### Error Pattern Queries

- [ ] T053 [P] [US3] Create `telemetry-server/queries/high-retry-sessions.sql` to find sessions with >3 API retries
- [ ] T054 [P] [US3] Create `telemetry-server/queries/validation-failures.sql` to find sessions with validation errors
- [ ] T055 [P] [US3] Create `telemetry-server/queries/common-errors.sql` to group errors by type and count occurrences
- [ ] T056 [P] [US3] Create `telemetry-server/queries/workflow-trace.sql` to retrieve complete trace for a given session_id

### Analysis Documentation

- [ ] T057 [US3] Create `telemetry-server/docs/ERROR-ANALYSIS.md` documenting Andrew Ng's error analysis methodology applied to this data
- [ ] T058 [US3] Document in ERROR-ANALYSIS.md how to: 1) identify failure patterns, 2) extract traces, 3) document improvements, 4) measure impact
- [ ] T059 [US3] Create example error analysis workflow in docs with sample queries and interpretation

### Trace Visualization (Optional, Simple Text Output)

- [ ] T060 [US3] Create `telemetry-server/scripts/print-trace.sh` to format workflow trace as readable timeline (session_id as input, prints events chronologically)

### Integration Testing

- [ ] T061 [US3] Test error analysis queries: Insert mock events with failures → Run high-retry-sessions.sql → Verify correct sessions returned
- [ ] T062 [US3] Test workflow trace retrieval: Use real session_id → Run workflow-trace.sql → Verify complete event sequence

**US3 Checkpoint**: Error analysis queries operational and documented

---

## Phase 6: User Story 4 - Team Documentation & Onboarding (Priority: P4)

**Goal**: <15 minute setup for new team members

**Independent Test**: Give new person quickstart guide → Time setup → Target <15 min

**Dependencies**: **US1 complete** (local setup is what's documented)

**Duration**: ~2-3 hours (mostly writing, minimal code)

### Quickstart Documentation

- [ ] T063 [US4] Create `telemetry-server/QUICKSTART.md` from `/specs/017-usage-data-collection/quickstart.md` template
- [ ] T064 [US4] Document 4-phase setup in QUICKSTART: 1) Prerequisites, 2) Docker startup, 3) Claude Code config, 4) Verification
- [ ] T065 [US4] Include verification checklist in QUICKSTART (6 items: Docker running, ClickHouse accessible, OTLP listening, events flowing, queries working, no errors)

### Troubleshooting Guide

- [ ] T066 [P] [US4] Create `telemetry-server/docs/TROUBLESHOOTING.md` with 8+ common issues and solutions
- [ ] T067 [P] [US4] Document in TROUBLESHOOTING: Port conflicts (4317, 8123), Docker not running, ClickHouse permission errors, OTLP connection refused, No events appearing, Query syntax errors, Disk space issues, Environment variables not set

### Example Queries Documentation

- [ ] T068 [P] [US4] Create `telemetry-server/docs/QUERY-EXAMPLES.md` with 11 copy-paste ready queries from `/specs/017-usage-data-collection/contracts/query-api.md`
- [ ] T069 [P] [US4] Document query categories in QUERY-EXAMPLES: Cost analysis (3 queries), Token usage (2 queries), Error patterns (3 queries), Performance (2 queries), Session analysis (1 query)
- [ ] T070 [P] [US4] Add usage instructions for each query (how to run via curl or clickhouse-client)

### Architecture Documentation

- [ ] T071 [US4] Create `telemetry-server/docs/ARCHITECTURE.md` explaining data flow: Claude Code → OTLP Collector → ClickHouse → Queries
- [ ] T072 [US4] Document component responsibilities in ARCHITECTURE: OTLP Collector (receive, filter, export), ClickHouse (store, query, aggregate), Docker Compose (orchestrate)
- [ ] T073 [US4] Include architecture diagram in ARCHITECTURE.md (ASCII art or link to external diagram)

### FAQ Documentation

- [ ] T074 [P] [US4] Create `telemetry-server/docs/FAQ.md` with 6+ common questions and answers
- [ ] T075 [P] [US4] Document in FAQ: How to enable/disable telemetry, How to check if it's working, How to query my own data, How to delete my data, How to troubleshoot, Where to get support

### Integration Testing

- [ ] T076 [US4] Test quickstart guide: Follow QUICKSTART.md on clean system → Verify <15 minute setup (SC-001 success criterion)
- [ ] T077 [US4] Test troubleshooting guide: Simulate common errors → Verify documented solutions work

**US4 Checkpoint**: Documentation complete and validated

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements, licensing, and repository integration

**Duration**: ~2-3 hours

### Licensing and Data Protection

- [ ] T078 Update repository root `.gitignore` to exclude all telemetry data directories
- [ ] T079 [P] Verify `LICENSE` file includes AI training restrictions and CC BY-NC-ND 4.0 terms (already done, just verify)
- [ ] T080 [P] Verify `NOTICE` file explains telemetry data is NOT included in repository distribution (already done, just verify)
- [ ] T081 [P] Verify `DATA-USAGE-POLICY` governance file exists and references telemetry data (already done, just verify)

### Repository README Integration

- [ ] T082 Update repository root `README.md` to mention telemetry-server/ infrastructure (optional, link to telemetry-server/README.md)

### Data Retention Automation

- [ ] T083 Create `telemetry-server/scripts/cleanup-old-data.sh` to delete events older than 90 days (per retention policy in data-model.md)
- [ ] T084 Document in `telemetry-server/docs/MAINTENANCE.md` how to schedule cleanup script (cron, systemd timer, or manual)

### Performance Optimization

- [ ] T085 [P] Configure ClickHouse materialized views in `schema.sql` for common aggregation queries (cost by day, tokens by user)
- [ ] T086 [P] Configure ClickHouse table compression settings (zstd codec) to reduce storage (10-20x compression expected)

### Multi-Platform Testing

- [ ] T087 Test telemetry infrastructure on macOS (Docker Desktop)
- [ ] T088 Test telemetry infrastructure on Linux (Docker Engine)
- [ ] T089 Test telemetry infrastructure on WSL (Windows Subsystem for Linux with Docker Desktop)

### Final Validation

- [ ] T090 Run complete end-to-end test: Fresh clone → Follow QUICKSTART.md → Collect data → Run queries → Verify all 8 success criteria (SC-001 through SC-008)
- [ ] T091 Generate final report: Task completion summary, success criteria validation, known issues, next steps

**Polish Checkpoint**: Feature 017 complete and ready for team use

---

## Dependencies and Execution Strategy

### Story Dependencies (Completion Order)

```
Phase 1: Setup (prerequisite for all)
  ↓
Phase 2: Foundational (prerequisite for all user stories)
  ↓
Phase 3: US1 - Enable Usage Data Collection (MVP, can ship here) 🎯
  ↓
Phase 4: US2 - Centralized Data Storage (depends on US1)
  ↓
Phase 5: US3 - Error Analysis Workflow (depends on US2)
  ↓
Phase 6: US4 - Team Documentation (depends on US1, can be parallel with US2/US3)
  ↓
Phase 7: Polish (can be parallel with US4)
```

### Parallel Execution Opportunities

**Within Phase 2 (Foundational)**:
- T007-T012 (Docker infra) can run parallel with T013-T015 (ClickHouse schema)
- T016-T020 (OTLP config) can run parallel with schema work

**Within Phase 3 (US1)**:
- T030-T031 (Documentation) can run parallel with T021-T023 (Env config)
- T024-T026 (Docker enhancements) can run parallel with T027-T029 (Scripts)
- T032-T034 (Data sanitization) can run parallel with T030-T031 (Docs)

**Within Phase 4 (US2)**:
- T044-T048 (Query files) are all parallelizable (different files)
- T038-T040 (Cloud config) can run parallel with query creation

**Within Phase 5 (US3)**:
- T053-T056 (Error queries) are all parallelizable (different files)
- T057-T059 (Documentation) can run parallel with query creation

**Within Phase 6 (US4)**:
- T066-T067 (Troubleshooting), T068-T070 (Query examples), T074-T075 (FAQ) are all parallelizable

**Within Phase 7 (Polish)**:
- T079-T081 (Verification tasks) are all parallelizable
- T085-T086 (Performance) can run parallel with T087-T089 (Testing)

### MVP Scope (Minimum Viable Product)

**Recommended MVP**: Complete through **Phase 3 (US1)** only

**MVP Delivers**:
- ✅ Local telemetry collection infrastructure
- ✅ Docker Compose orchestration
- ✅ ClickHouse database with schema
- ✅ OTLP Collector configuration
- ✅ Data sanitization
- ✅ Verification scripts
- ✅ Basic documentation

**MVP Validation**: Single team member can collect and query their own telemetry data locally

**Post-MVP**: Phases 4-7 add centralization, error analysis, comprehensive docs, and polish

---

## Task Summary

**Total Tasks**: 91

**By Phase**:
- Phase 1 (Setup): 6 tasks (~30 min)
- Phase 2 (Foundational): 14 tasks (~2-3 hours)
- Phase 3 (US1 - MVP): 17 tasks (~4-6 hours) 🎯
- Phase 4 (US2): 15 tasks (~3-4 hours)
- Phase 5 (US3): 10 tasks (~2-3 hours)
- Phase 6 (US4): 15 tasks (~2-3 hours)
- Phase 7 (Polish): 14 tasks (~2-3 hours)

**Estimated Total Effort**: 20-26 hours (assuming single developer, sequential execution)

**With Parallelization**: 15-20 hours (optimistic, 3-4 developers working concurrently)

**Critical Path**: Phase 1 → Phase 2 → US1 (MVP) = ~7-10 hours minimum

---

## Success Criteria Mapping

| Success Criterion | Validated By Tasks | Status |
|-------------------|-------------------|--------|
| SC-001: <15 min setup | T063-T077 (US4 documentation + QUICKSTART guide) | Tasks defined ✅ |
| SC-002: 100% event capture | T035 (US1 integration testing verifies no data loss) | Tasks defined ✅ |
| SC-003: <3s queries for 30-day data | T085-T086 (materialized views + compression) | Tasks defined ✅ |
| SC-004: Error analysis → improvements | T053-T062 (US3 error queries + workflow) | Tasks defined ✅ |
| SC-005: High-volume (1000+ events) | T037 (US1 high-volume session test) | Tasks defined ✅ |
| SC-006: Monthly reports in <5 min | T044-T048 (US2 aggregation queries) | Tasks defined ✅ |
| SC-007: 90% self-onboarding | T076 (US4 quickstart validation) | Tasks defined ✅ |
| SC-008: Stay within 100GB limit | T083-T084 (data retention automation + cleanup) | Tasks defined ✅ |

---

## Implementation Notes

### Key Decisions from Research

- **Database Choice**: ClickHouse (over Prometheus) for schema flexibility and SQL query power
- **Orchestration**: Docker Compose (over Kubernetes) for simplicity in local development
- **Data Flow**: Claude Code → OTLP Collector → ClickHouse (standard OpenTelemetry pattern)
- **Deployment**: Local-first (Phase 1), optional cloud migration (Phase 2)
- **Retention**: 90 days raw events, 1 year sessions, indefinite aggregates

### Critical Path Items

1. **Phase 2 completion is blocker** for all user stories (foundational infra must work)
2. **US1 completion is blocker** for US2 (validate local before centralization)
3. **US2 completion is blocker** for US3 (need centralized data for error analysis)
4. **US4 can start after US1** (documents local setup, independent of US2/US3)

### Risk Mitigation

- **ClickHouse complexity**: Start with simple schema, add materialized views in Polish phase
- **OTLP configuration**: Use official OpenTelemetry examples as templates
- **Docker networking**: Test connectivity between services early (T020)
- **Data sanitization**: Implement filters conservatively (hash more, risk less exposure)
- **Multi-platform**: Test on all 3 platforms (Mac/Linux/WSL) before declaring done

---

**Ready for implementation!** Start with Phase 1 (Setup) and proceed sequentially through foundational work, then implement US1 (MVP) for immediate value.
