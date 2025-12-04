# Tasks: PanaversityFS Production Hardening

**Input**: Design documents from `/specs/039-panaversity-fs-hardening/`
**Prerequisites**: plan.md (complete), spec.md (complete)
**Generated**: 2025-12-04

**Organization**: Tasks grouped by user story to enable independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1-US6)
- All file paths relative to `panaversity-fs/` project root

---

## Phase 1: Setup (Project Initialization)

**Purpose**: Add dependencies and create project scaffolding

- [ ] T001 Use `uv add sqlalchemy asyncpg aiosqlite alembic prometheus-client hypothesis` to add all dependencies. Verify with `uv pip list | grep -E "sqlalchemy|asyncpg|aiosqlite|alembic|prometheus|hypothesis"`.
- [ ] T002 [P] Use `alembic init src/panaversity_fs/database/migrations` to scaffold Alembic migrations directory. **Doc**: Fetch Alembic docs via Context7 for async migration configuration. Configure `alembic.ini` at project root to point to migrations directory.
- [ ] T003 [P] Create `src/panaversity_fs/database/__init__.py` module init with exports for models and session factory.
- [ ] T004 Update `src/panaversity_fs/config.py` to add `database_url` field with DATABASE_URL env var support. **Doc**: Fetch Pydantic docs via Context7 for settings patterns with env vars.

**Checkpoint**: Dependencies installed, project structure ready for database layer

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**CRITICAL**: No user story work can begin until this phase is complete

### 2.1 Database Layer

- [ ] T005 Create `src/panaversity_fs/database/models.py` with FileJournal and AuditLog SQLAlchemy models per spec entities. **Doc**: Fetch SQLAlchemy docs via Context7 for `DeclarativeBase`, `Mapped[]`, and async model patterns.
- [ ] T006 Create `src/panaversity_fs/database/connection.py` with async engine factory and `get_session()` sessionmaker. **Doc**: Fetch SQLAlchemy docs via Context7 for `create_async_engine` and `async_sessionmaker` patterns.
- [ ] T007 Configure `src/panaversity_fs/database/migrations/env.py` for async SQLAlchemy (file created by T002's `alembic init`). **Doc**: Fetch Alembic docs via Context7 for async `run_migrations_online()` configuration.
- [ ] T008 Use `alembic revision --autogenerate -m "initial FileJournal and AuditLog schema"` to generate migration. Review generated file and manually add CHECK constraints (`agent_id != 'system'`, `agent_id != ''`) if autogenerate misses them. **Doc**: Fetch Alembic docs via Context7 for CHECK constraint syntax.
- [ ] T009 [P] Create `tests/unit/test_journal.py` with FileJournal CRUD tests. **Doc**: Fetch pytest-asyncio docs via Context7 for async test fixtures.

### 2.2 Path Validation

- [ ] T010 [P] Create `src/panaversity_fs/path_utils.py` with CONTENT_PATH_PATTERN, ASSET_PATH_PATTERN regex constants per spec FR-007, FR-008.
- [ ] T011 [P] Add `validate_content_path()`, `validate_asset_path()`, `validate_overlay_path()` functions to `src/panaversity_fs/path_utils.py` per spec interface.
- [ ] T012 [P] Add path conversion helpers `convert_base_to_overlay()`, `convert_overlay_to_base()`, `extract_user_id_from_overlay()` to `src/panaversity_fs/path_utils.py`.
- [ ] T013 [P] Update `src/panaversity_fs/errors.py` to add SchemaViolationError, HashRequiredError, ConflictError exception classes.
- [ ] T014 [P] Create `tests/unit/test_path_utils.py` with regex validation edge cases including path traversal attacks (`..`, null bytes).

### 2.3 Instrumentation

- [ ] T015 [P] Create `src/panaversity_fs/metrics.py` with Prometheus registry, counters (write_total, archive_total), histograms (archive_duration_seconds, write_duration_seconds), gauges (archive_memory_bytes, journal_entries_total). **Doc**: Fetch prometheus-client docs via Context7 for Counter, Histogram, Gauge patterns.
- [ ] T016 [P] Add `@instrument_write` and `@instrument_archive` decorators to `src/panaversity_fs/metrics.py`. **Doc**: Fetch prometheus-client docs via Context7 for decorator instrumentation patterns.
- [ ] T017 [P] Create `tests/unit/test_metrics.py` with decorator behavior tests.

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Docusaurus Build Fetches Complete Book (Priority: P1) MVP

**Goal**: Reliable book archive downloads within 60s/<64MB memory for CI/CD pipeline

**Independent Test**: Trigger archive of 50MB book (300 lessons), verify completion <60s with memory monitoring

### Implementation for User Story 1

- [ ] T018 [US1] Refactor `src/panaversity_fs/tools/bulk.py` `get_book_archive()` to use chunked streaming with io.BytesIO buffer capped at 64MB (SC-001/R4). **Doc**: Fetch Python zipfile docs for streaming write patterns.
- [ ] T019 [US1] Add timeout detection (60s) to `get_book_archive()` returning partial result with error manifest per FR-014 (SC-001/R4).
- [ ] T020 [US1] Add `archive_memory` gauge tracking throughout archive generation in `src/panaversity_fs/tools/bulk.py` (SC-001/R4).
- [ ] T021 [US1] Apply `@instrument_archive` decorator to `get_book_archive()` in `src/panaversity_fs/tools/bulk.py` (SC-001/R4).
- [ ] T022 [P] [US1] Create `tests/integration/test_streaming_archive.py` with real ZIP generation tests (SC-001/R4).
- [ ] T023 [P] [US1] Create `tests/performance/test_archive_throughput.py` with SC-001/R4 validation (500 files/200MB <60s <64MB).

**Checkpoint**: User Story 1 complete - archive downloads reliable for CI/CD

---

## Phase 4: User Story 2 - Agent Updates Lesson with Conflict Detection (Priority: P1)

**Goal**: Hash-based conflict detection for write operations with clear create/update distinction

**Independent Test**: Call `write_content` with/without `expected_hash`, verify journal entries and conflict responses

### Implementation for User Story 2

- [ ] T024 [US2] Update `src/panaversity_fs/models.py` to add `expected_hash: str | None` field to WriteContentInput. **Doc**: Fetch Pydantic docs via Context7 for Optional field patterns.
- [ ] T025 [US2] Refactor `src/panaversity_fs/tools/content.py` `write_content()` to query FileJournal before write per FR-002.
- [ ] T026 [US2] Add conflict detection logic to `write_content()`: reject if expected_hash mismatches with ConflictError containing current hash per FR-003.
- [ ] T027 [US2] Add hash-required enforcement to `write_content()`: reject existing file updates without expected_hash with HashRequiredError per FR-004.
- [ ] T028 [US2] Add atomic journal+storage transaction wrapper to `write_content()` per FR-002 (rollback both on failure). **Doc**: Fetch SQLAlchemy docs via Context7 for async transaction commit/rollback patterns.
- [ ] T029 [US2] Update `write_content()` response to include `mode: "created"|"updated"` per FR-005.
- [ ] T030 [US2] Apply `@instrument_write` decorator to `write_content()` in `src/panaversity_fs/tools/content.py`.
- [ ] T031 [P] [US2] Create `tests/integration/test_journal_storage_atomic.py` with transaction rollback and fault injection tests for SC-002.
- [ ] T032 [P] [US2] Create `tests/integration/test_conflict_detection.py` with concurrent write conflict tests for SC-003.
- [ ] T033 [P] [US2] Create `tests/property/test_invariant_r2_journal.py` with hypothesis journal-storage consistency tests. **Doc**: Fetch hypothesis docs via Context7 for async test strategies.

**Checkpoint**: User Story 2 complete - agents have reliable conflict detection

---

## Phase 5: User Story 3 - System Administrator Queries Audit Trail (Priority: P2)

**Goal**: Append-only audit with hash chain integrity and real agent IDs

**Independent Test**: Perform operations, query audit log, verify hash chain and agent_id fields

### Implementation for User Story 3

- [ ] T034 [US3] Refactor `src/panaversity_fs/audit.py` `log_operation()` to use append-only INSERT (no read-modify-write) per FR-023.
- [ ] T035 [US3] Add hash chain logic to `log_operation()`: query prev_hash from previous entry on same (book_id, path, user_id) per FR-022.
- [ ] T036 [US3] Create `extract_agent_id_from_context()` function in `src/panaversity_fs/audit.py` to get agent_id from MCP context per FR-021. **Doc**: Fetch FastMCP docs via Context7 for request context access patterns.
- [ ] T037 [US3] Update all tool files to pass agent_id to `log_operation()` calls (content.py, bulk.py, assets.py).
- [ ] T038 [US3] Add `query_audit_log()` function to `src/panaversity_fs/audit.py` with filters: agent_id, date_range, path, operation per FR-024. **Doc**: Fetch SQLAlchemy docs via Context7 for dynamic filter building.
- [ ] T039 [P] [US3] Create `tests/unit/test_audit_chain.py` with hash chain integrity tests.
- [ ] T040 [P] [US3] Create `tests/property/test_invariant_r6_audit.py` with hypothesis chain validation (3-5 operations). **Doc**: Fetch hypothesis docs via Context7 for stateful testing.
- [ ] T041 [P] [US3] Create `tests/property/test_invariant_r7_agent.py` with hypothesis agent provenance tests (no 'system' or empty).

**Checkpoint**: User Story 3 complete - audit trail provides complete provenance

---

## Phase 6: User Story 4 - Personalized Book for Individual User (Priority: P2)

**Goal**: User-specific overlays with fallback to base content

**Independent Test**: Create base lesson, create overlay for user1, verify reads resolve correctly per user

### Implementation for User Story 4

- [ ] T042 [US4] Update `src/panaversity_fs/models.py` to add `user_id: str | None` field to ReadContentInput, WriteContentInput, DeleteContentInput.
- [ ] T043 [US4] Refactor `src/panaversity_fs/tools/content.py` `write_content()` to write to overlay namespace when user_id provided per FR-017.
- [ ] T044 [US4] Refactor `src/panaversity_fs/tools/content.py` `read_content()` to check overlay first, fall back to base per FR-016.
- [ ] T045 [US4] Refactor `src/panaversity_fs/tools/content.py` `delete_content()` to delete overlay only, never affect base per FR-018.
- [ ] T046 [US4] Add overlay path validation using `validate_overlay_path()` from path_utils per FR-019.
- [ ] T047 [P] [US4] Create `tests/integration/test_overlay_isolation.py` with overlay CRUD and isolation tests.
- [ ] T048 [P] [US4] Create `tests/property/test_invariant_r5_overlay.py` with hypothesis overlay exclusivity tests (2 users, 2 lessons). **Doc**: Fetch hypothesis docs via Context7 for composite strategies.
- [ ] T049 [P] [US4] Create `tests/performance/test_overlay_latency.py` with SC-006 validation (overlay read <10ms vs base).

**Checkpoint**: User Story 4 complete - personalization layer working

---

## Phase 7: User Story 5 - Book Schema Validation on Write (Priority: P2)

**Goal**: Reject invalid paths with clear error messages

**Independent Test**: Attempt writes with valid/invalid paths, verify rejections

### Implementation for User Story 5

- [ ] T050 [US5] Integrate `validate_content_path()` call into `write_content()` in `src/panaversity_fs/tools/content.py` per FR-007.
- [ ] T051 [US5] Integrate `validate_asset_path()` call into asset operations in `src/panaversity_fs/tools/assets.py` per FR-008.
- [ ] T052 [US5] Add path traversal rejection (contains `..`, leading `/`, null bytes) to path validation per FR-009.
- [ ] T053 [US5] Create `validate_book_structure` MCP tool in `src/panaversity_fs/tools/content.py` that scans book and reports violations per FR-010. **Doc**: Fetch FastMCP docs via Context7 for tool registration patterns.
- [ ] T054 [P] [US5] Create `tests/property/test_invariant_r1_schema.py` with hypothesis schema enforcement tests. **Doc**: Fetch hypothesis docs via Context7 for from_regex strategy.
- [ ] T055 [P] [US5] Update `tests/unit/test_path_utils.py` with path traversal attack test cases.

**Checkpoint**: User Story 5 complete - schema enforced on all writes

---

## Phase 8: User Story 6 - Delta Build Detection (Priority: P3)

**Goal**: Incremental builds via manifest hash comparison

**Independent Test**: Call `plan_build` with known manifest hash, verify only changed files returned

### Implementation for User Story 6

- [ ] T056 [US6] Add `PlanBuildInput` model to `src/panaversity_fs/models.py` with book_id, target_manifest_hash fields.
- [ ] T057 [US6] Create `compute_manifest_hash()` function in `src/panaversity_fs/tools/bulk.py` per spec algorithm (filter base, sort, concat, SHA256).
- [ ] T058 [US6] Create `plan_build` MCP tool in `src/panaversity_fs/tools/bulk.py` returning status/files/manifest_hash per FR-025, FR-026. **Doc**: Fetch FastMCP docs via Context7 for tool registration patterns.
- [ ] T059 [P] [US6] Create `tests/unit/test_manifest_hash.py` with deterministic computation tests.
- [ ] T060 [P] [US6] Create `tests/integration/test_delta_build.py` with changed file detection tests.

**Checkpoint**: User Story 6 complete - incremental builds enabled

---

## Phase 9: Idempotent Delete (Cross-Cutting)

**Purpose**: Ensure delete operations are idempotent per spec assumption

- [ ] T061 Refactor `src/panaversity_fs/tools/content.py` `delete_content()` to remove journal entry and return success even if file doesn't exist.
- [ ] T062 [P] Create `tests/property/test_invariant_r3_delete.py` with hypothesis idempotent delete tests (double delete succeeds). **Doc**: Fetch hypothesis docs via Context7 for stateful testing patterns.

---

## Phase 10: Polish & Cross-Cutting Concerns

**Purpose**: Final validation of tests created in earlier phases. **No new test code here**—this phase runs the existing test suites to verify full-system integration.

- [ ] T063 Run all integration tests (created in T022, T031-T032, T039, T047, T060) on both PostgreSQL and SQLite backends: `pytest tests/integration/ --db-url=<postgres>` and `pytest tests/integration/` (SQLite default).
- [ ] T064 Run all property-based tests R1-R3, R5-R7 (created in T033, T040-T041, T048, T054, T062) with `pytest tests/property/ -v`.
- [ ] T065 Run performance benchmarks SC-001/R4, SC-006 (created in T023, T049) with `pytest tests/performance/ -v --benchmark` and log results.
- [ ] T066 [P] Update `README.md` with new features (expected_hash, user_id, plan_build).
- [ ] T067 [P] **CI sanity check only** (not for production): Verify migration cycle with `alembic downgrade base && alembic upgrade head`. Production uses fresh-start deployment per plan—never run downgrade in prod.
- [ ] T068 Final acceptance test: Run all 6 user story acceptance scenarios from spec.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phases 3-8)**: All depend on Foundational phase completion
  - US1 and US2 are P1 priority - complete first
  - US3, US4, US5 are P2 priority - can proceed after P1 stories
  - US6 is P3 priority - complete last
- **Polish (Phase 10)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Phase 2 - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Phase 2 - No dependencies on other stories
- **User Story 3 (P2)**: Depends on US2 (uses journal for prev_hash lookup)
- **User Story 4 (P2)**: Can start after Phase 2 - Independent
- **User Story 5 (P2)**: Can start after Phase 2 - Uses path_utils from Phase 2
- **User Story 6 (P3)**: Depends on US2 (uses journal for manifest computation)

### Parallel Opportunities

Within Phase 2 (Foundational):
- T011-T015 (path validation) can run in parallel with T005-T010 (database)
- T016-T018 (metrics) can run in parallel with both

Within each User Story:
- All test tasks marked [P] can run in parallel after implementation tasks

---

## Parallel Example: Phase 2 Foundations

```bash
# Launch in parallel - different files, no dependencies:
Task T005: Create database/models.py
Task T011: Create path_utils.py
Task T016: Create metrics.py

# Then launch tests in parallel:
Task T010: Create tests/unit/test_journal.py
Task T015: Create tests/unit/test_path_utils.py
Task T018: Create tests/unit/test_metrics.py
```

---

## Implementation Strategy

### MVP First (User Stories 1 + 2 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Archive downloads)
4. Complete Phase 4: User Story 2 (Conflict detection)
5. **STOP and VALIDATE**: Test US1 + US2 independently
6. Deploy/demo - CI/CD pipeline and agent writes now reliable

### Incremental Delivery

1. Setup + Foundational → Foundation ready
2. Add US1 + US2 → Test → Deploy (MVP!)
3. Add US3 (Audit) → Test → Deploy
4. Add US4 (Overlays) → Test → Deploy
5. Add US5 (Schema) → Test → Deploy
6. Add US6 (Delta) → Test → Deploy
7. Each story adds value without breaking previous stories

### Suggested MVP Scope

**MVP = Phase 1 + Phase 2 + Phase 3 (US1) + Phase 4 (US2)**

This delivers:
- Reliable archive downloads (fixes 502 timeouts)
- Conflict detection (fixes uncertain updates)
- Foundation for all future stories

Estimated: ~10 tasks for MVP core functionality

---

## Summary

| Metric | Count |
|--------|-------|
| **Total Tasks** | 68 |
| **Setup Tasks** | 4 |
| **Foundational Tasks** | 13 |
| **US1 Tasks** | 6 |
| **US2 Tasks** | 10 |
| **US3 Tasks** | 8 |
| **US4 Tasks** | 8 |
| **US5 Tasks** | 6 |
| **US6 Tasks** | 5 |
| **Cross-Cutting Tasks** | 2 |
| **Polish Tasks** | 6 |
| **Parallelizable [P] Tasks** | 27 |

**Changes from original**:
- Removed manual `script.py.mako` creation (auto-generated by `alembic init`)
- Combined Alembic setup into T002 using `alembic init` CLI
- Changed manual migration file creation to `alembic revision --autogenerate`
- Added **Doc** references for Context7 lookups on SQLAlchemy, Alembic, prometheus-client, hypothesis, FastMCP, Pydantic
- All CLI commands now explicit with verification steps

---

## Notes

- [P] tasks = different files, no dependencies on incomplete tasks
- [Story] label maps task to specific user story for traceability
- Each user story is independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- All file paths relative to `panaversity-fs/` project root
