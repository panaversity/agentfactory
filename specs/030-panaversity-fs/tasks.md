# Tasks: PanaversityFS

**Input**: Design documents from `/specs/030-panaversity-fs/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: NOT REQUESTED in specification - No test tasks included per template guidance

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

**⚠️ IMPLEMENTATION NOTE**: Using **Python MCP** implementation with live documentation:
- Python 3.11+ with uv package manager
- Python MCP SDK (`mcp` package) - **docs via Context7 MCP server**
- **MCP Transport**: Stateless Streamable HTTP protocol (`STATELESS_HTTP=true`)
- OpenDAL Python bindings - **docs via Context7 MCP server**
- Pydantic for validation
- **mcp-builder skill** for architecture guidance
- Development approach: consult live docs (Context7) + architectural patterns (mcp-builder) instead of pre-research
- Deployment via Docker/systemd (HTTP endpoint)

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

**UPDATED**: Using Python MCP implementation (not Node.js/Cloudflare Workers):
- Source: `panaversity-fs/src/`
- Tests: `panaversity-fs/tests/`
- CLI: `panaversity-fs/cli/`
- Examples: `panaversity-fs/examples/`

---

## Phase 0: Learn MCP Architecture with Live Docs (PREREQUISITE)

**Purpose**: Understand Python MCP SDK + OpenDAL using Context7 MCP server and mcp-builder skill

**⚠️ CRITICAL**: Must understand MCP concepts before writing any code

**Tools Available**:
- `mcp__context7__resolve-library-id` - Find correct library IDs for mcp, opendal
- `mcp__context7__get-library-docs` - Get live API docs with code examples
- Skill: `mcp-builder` - MCP server architecture guidance

- [ ] T001 Use Context7 to get Python MCP SDK docs (resolve "mcp" library ID, fetch server/tool/resource concepts with mode='code', specifically Stateless Streamable HTTP transport)
- [ ] T002 Use mcp-builder skill to understand MCP server patterns (invoke skill, ask: "How do I structure a Python MCP server with 15 tools organized by category using Stateless Streamable HTTP transport?")
- [ ] T003 Use Context7 to get OpenDAL Python docs (resolve "opendal" library ID, fetch Operator API: read/write/list/stat/presign with mode='code')
- [ ] T004 Test integration pattern (create minimal MCP server with STATELESS_HTTP=true, 1 tool using OpenDAL Operator to read a file, verify with MCP inspector)
- [ ] T005 Consult mcp-builder for async patterns (ask: "Should I use @mcp.tool() decorator or class-based tools? How to handle async storage operations in Stateless Streamable HTTP MCP server?")

**Checkpoint**: Understand MCP + OpenDAL patterns through live docs + skill guidance before building PanaversityFS

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization using uv

- [ ] T006 Initialize Python project with `uv init panaversity-fs` at repository root
- [ ] T007 Install core dependencies: `uv add opendal mcp pydantic pyyaml python-frontmatter python-magic`
- [ ] T008 Install dev dependencies: `uv add --dev pytest pytest-asyncio ruff mypy`
- [ ] T009 Create basic project structure (src/panaversity_fs/, tests/, cli/, examples/)
- [ ] T010 Create README.md with project overview and development setup instructions

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core domain models and MCP server setup - let OpenDAL handle storage abstraction

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

**Domain Models** (what we store):
- [ ] T011 Create Pydantic models in src/panaversity_fs/models.py (Registry, Book, Lesson, Asset, Summary, AuditEntry per data-model.md)

**Storage Layer** (delegate to OpenDAL):
- [ ] T012 Create storage config in src/panaversity_fs/storage.py (initialize OpenDAL Operator based on STORAGE_BACKEND env var: fs, s3, or supabase)
- [ ] T013 Create storage helper utilities in src/panaversity_fs/storage_utils.py (file hash computation, path validation, error handling wrappers around OpenDAL operations)

**Audit Layer**:
- [ ] T014 Implement JSONL audit logger in src/panaversity_fs/audit.py (async append using OpenDAL, daily rotation, query with filters)

**MCP Server**:
- [ ] T015 Create MCP server entry point in src/panaversity_fs/server.py (initialize MCP Server from Python SDK with STATELESS_HTTP=true for Stateless Streamable HTTP transport, load storage config)
- [ ] T016 Configure environment handler in src/panaversity_fs/config.py (load from .env using pydantic-settings: STATELESS_HTTP=true, STORAGE_BACKEND, R2_BUCKET, OAUTH_TOKEN, etc.)

**Cross-Cutting**:
- [ ] T017 [P] Implement error handling in src/panaversity_fs/errors.py (custom exceptions, MCP error response formatting)
- [ ] T018 [P] Implement logging setup in src/panaversity_fs/logging.py (structured logs, tool invocation tracking)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - AI Agent Updates Lesson Content (Priority: P1) 🎯 MVP

**Goal**: Enable AI agents to perform CRUD operations on lesson markdown files with audit trail tracking

**Independent Test**: Invoke `update_content` MCP tool with lesson path and new content, verify file write and audit log entry

### Implementation for User Story 1

- [ ] T019 [P] [US1] Implement `read_content` MCP tool in src/panaversity_fs/tools/content.py (@mcp.tool decorator, readOnlyHint=true, use OpenDAL operator.read(), return markdown + metadata: file_size, last_modified, file_hash per FR-009)
- [ ] T020 [P] [US1] Implement `write_content` MCP tool in src/panaversity_fs/tools/content.py (@mcp.tool decorator, idempotentHint=true, upsert operation: if file_hash provided detect conflicts per FR-008 else create/overwrite, use OpenDAL operator.write(), log audit entry per FR-007)
- [ ] T021 [P] [US1] Implement `delete_content` MCP tool in src/panaversity_fs/tools/content.py (@mcp.tool decorator, destructiveHint=true, use OpenDAL operator.delete(), log audit entry)
- [ ] T022 [US1] Add frontmatter parsing using python-frontmatter in content tools (validate lesson_id matches filename per data-model.md)
- [ ] T023 [US1] Register content tools with MCP server (3 tools auto-discovered via @mcp.tool decorator in server.py)

**Checkpoint**: At this point, User Story 1 should be fully functional - agents can CRUD lessons with audit trail

---

## Phase 4: User Story 2 - Upload and Manage Book Assets (Priority: P1) 🎯 MVP

**Goal**: Enable AI agents to upload binary assets (images, slides, videos, audio) and retrieve CDN URLs

**Independent Test**: Upload PDF slide deck via `add_asset`, verify storage backend write, retrieve public CDN URL via `get_asset`

### Implementation for User Story 2

- [ ] T024 [P] [US2] Implement `upload_asset` MCP tool in src/panaversity_fs/tools/assets.py (@mcp.tool decorator, destructiveHint=true, hybrid pattern: if binary_data provided upload directly <10MB via OpenDAL operator.write(), if file_size≥10MB return presigned URL via OpenDAL operator.presign() per research.md Decision 3, return cdn_url + method per FR-012)
- [ ] T025 [P] [US2] Implement `get_asset` MCP tool in src/panaversity_fs/tools/assets.py (@mcp.tool decorator, readOnlyHint=true, use OpenDAL operator.stat(), return single asset with cdn_url + metadata per FR-012)
- [ ] T026 [P] [US2] Implement `list_assets` MCP tool in src/panaversity_fs/tools/assets.py (@mcp.tool decorator, readOnlyHint=true, use OpenDAL operator.list(), filter by asset_type optional, return array of assets per FR-011)
- [ ] T027 [US2] Add MIME type validation using python-magic and size checks in upload_asset (reject >100MB per FR-010)
- [ ] T028 [US2] Register asset tools with MCP server (3 tools auto-discovered via @mcp.tool decorator)

**Checkpoint**: At this point, User Story 2 should be fully functional - agents can upload/retrieve assets with CDN URLs

---

## Phase 5: User Story 6 - Docusaurus Build Integration (Priority: P1) 🎯 MVP

**Goal**: Enable Docusaurus static site generator to fetch complete book content during build via bulk download

**Independent Test**: Run `docusaurus build` with hydration script calling `get_book_archive`, verify all lessons/assets download to `book-source/`, build succeeds

### Implementation for User Story 6

- [ ] T029 [US6] Implement `get_book_archive` MCP tool in src/panaversity_fs/tools/bulk.py (@mcp.tool decorator, readOnlyHint=true, use OpenDAL operator.list() recursively, create ZIP using zipfile with ZIP64 support per research.md Decision 4)
- [ ] T030 [US6] Implement streaming ZIP creation (write files to temp ZIP, upload to `.temp/{book_id}-{timestamp}.zip` via OpenDAL, generate presigned URL with 1-hour expiry per FR-029)
- [ ] T031 [US6] Add archive timeout handling using asyncio.timeout (60-second limit per FR-030, return error with fallback per FR-031)
- [ ] T032 [US6] Register bulk tool with MCP server (auto-discovered via @mcp.tool decorator)
- [ ] T033 [US6] Create Docusaurus hydration script in examples/docusaurus-hydration.py (call MCP get_book_archive, download ZIP via presigned URL, extract to book-source/ per FR-037)
- [ ] T034 [US6] Add archive cleanup task (scheduled via APScheduler, delete `.temp/*.zip` older than 2 hours)

**Checkpoint**: At this point, Docusaurus integration should work - builds can hydrate content in <60 seconds via bulk download

---

## Phase 6: User Story 4 - Multi-Book Registry Management (Priority: P2)

**Goal**: Enable system administrators to add new books to platform registry

**Independent Test**: Call `list_books`, verify registry.yaml entries returned

### Implementation for User Story 4

- [ ] T036 [US4] Implement `list_books` MCP tool in src/panaversity_fs/tools/registry.py (read registry.yaml via OpenDAL operator.read(), parse YAML, return books array)
- [ ] T037 [US4] Implement add_book_to_registry helper in src/panaversity_fs/registry.py (atomic registry.yaml update, create book directory structure via OpenDAL)
- [ ] T038 [US4] Register registry tool with MCP server
- [ ] T039 [US4] Add registry-storage consistency check on server startup (detect orphans, auto-repair)

---

## Phase 7: User Story 5 - Search Book Content (Priority: P2)

**Goal**: Enable AI agents to find lessons by keywords or file patterns

**Independent Test**: Call `grep_search` with pattern, verify matches include file paths

### Implementation for User Story 5

- [ ] T040 [P] [US5] Implement `glob_search` MCP tool in src/panaversity_fs/tools/search.py (use OpenDAL operator.list() with pattern matching)
- [ ] T041 [P] [US5] Implement `grep_search` MCP tool in src/panaversity_fs/tools/search.py (read files via OpenDAL, regex search, return matches with line numbers)
- [ ] T042 [US5] Register search tools with MCP server

---

## Phase 8: User Story 3 - Query Audit Trail (Priority: P2)

**Goal**: Enable developers to inspect operations for debugging

**Independent Test**: Perform operations, call `get_audit_log`, verify entries match

### Implementation for User Story 3

- [ ] T043 [US3] Implement `get_audit_log` MCP tool in src/panaversity_fs/tools/audit.py (read .audit/*.jsonl via OpenDAL, filter, paginate per FR-020)
- [ ] T044 [US3] Register audit tool with MCP server

---

## Phase 9: Summary Management (Storage Only)

**Purpose**: Enable external agents to store pre-generated summaries

- [ ] T045 [P] Implement 4 summary MCP tools in src/panaversity_fs/tools/summaries.py (add/update/get/list using OpenDAL operator read/write/list)
- [ ] T046 Register summary tools with MCP server

---

## Phase 10: Migration Tooling (CLI)

**Purpose**: Seed initial content from Git repository

- [ ] T047 Create CLI entry point in cli/migrate.py (argparse for --from ./repo --to r2 --book-id)
- [ ] T048 Implement Git → PanaversityFS migration (scan repo, upload via OpenDAL, create registry entry)
- [ ] T049 Add validation and dry-run mode

---

## Phase 11: Documentation & Deployment

**Purpose**: Production readiness

- [ ] T050 [P] Update quickstart.md (Python/uv setup, MCP client examples)
- [ ] T051 [P] Create Dockerfile for production deployment
- [ ] T052 [P] Setup monitoring (Sentry, Prometheus metrics)
- [ ] T053 Production deployment test (invoke all 15 tools, verify audit logs)

---
## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-9)**: All depend on Foundational phase completion
  - **Phase 3 (US1 - Lesson CRUD)**: Core content operations - MVP foundation
  - **Phase 4 (US2 - Asset Management)**: Can start after Foundational, parallel to Phase 3
  - **Phase 5 (US6 - Docusaurus Integration)**: Can start after Foundational, parallel to Phases 3-4
  - **Phase 6 (US4 - Registry)**: Can start after Foundational, parallel to other phases
  - **Phase 7 (US5 - Search)**: Can start after Foundational, parallel to other phases
  - **Phase 8 (US3 - Audit Query)**: Can start after Foundational (audit logger already in Foundational), parallel to other phases
  - **Phase 9 (Summary Management)**: Can start after Foundational, parallel to other phases
- **Cross-Cutting (Phases 10-13)**: Depend on all user story phases being complete

### User Story Dependencies

- **User Story 1 (P1) - Lesson CRUD**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1) - Asset Management**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 6 (P1) - Docusaurus Integration**: Can start after Foundational (Phase 2) - No dependencies on other stories (bulk download is independent)
- **User Story 4 (P2) - Registry**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 5 (P2) - Search**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 3 (P2) - Audit Query**: Can start after Foundational (Phase 2) - Audit logger already exists in Foundational

### Within Each User Story

- Models/schemas before tools (already in Foundational)
- Storage adapter before tools (already in Foundational)
- Tools can be implemented in parallel (marked with [P])
- Registration in server.ts after tools complete
- Error handling and validation after core implementation

### Parallel Opportunities

- **Phase 1 (Setup)**: T003, T004, T005, T006 (all marked [P]) can run in parallel
- **Phase 2 (Foundational)**: T010-T015 (schemas), T017-T020 (backends), T021-T022 (utils), T024, T026 can all run in parallel (marked [P])
- **Phase 3 (US1)**: T031-T034 (all content tools marked [P]) can run in parallel
- **Phase 4 (US2)**: T038-T040 (all asset tools marked [P]) can run in parallel
- **Phase 7 (US5)**: T065-T066 (both search tools marked [P]) can run in parallel
- **Phase 9 (Summary)**: T077-T080 (all summary tools marked [P]) can run in parallel
- **Phase 10 (Observability)**: T084-T085 (Analytics + Sentry marked [P]) can run in parallel
- **Phase 11 (Migration)**: T090-T091 (reader + registry marked [P]) can run in parallel
- **Phase 12 (Documentation)**: T099-T102 (all docs marked [P]) can run in parallel
- **Once Foundational completes**: All user story phases (3-9) can start in parallel if team capacity allows

---

## Parallel Example: User Story 1 (Lesson CRUD)

```bash
# Launch all content tools together (different files, no dependencies):
Task T031: "Implement add_content MCP tool in src/tools/content/add_content.ts"
Task T032: "Implement update_content MCP tool in src/tools/content/update_content.ts"
Task T033: "Implement read_content MCP tool in src/tools/content/read_content.ts"
Task T034: "Implement delete_content MCP tool in src/tools/content/delete_content.ts"

# After tools complete, register in server (sequential):
Task T035: "Register all content tools in src/server.ts MCP server"
```

---

## Parallel Example: Foundational Phase (Storage Backends)

```bash
# Launch all storage backends together (different files, no dependencies):
Task T017: "Implement Local filesystem backend in src/storage/backends/local.ts"
Task T018: "Implement Cloudflare R2 backend in src/storage/backends/r2.ts"
Task T019: "Implement AWS S3 backend in src/storage/backends/s3.ts"
Task T020: "Implement Supabase Storage backend in src/storage/backends/supabase.ts"
```

---

## Implementation Strategy

### MVP First (Phases 1-5: Setup + Foundational + US1 + US2 + US6)

1. Complete Phase 1: Setup (project initialization)
2. Complete Phase 2: Foundational (storage, auth, audit, MCP server - CRITICAL, blocks all stories)
3. Complete Phase 3: User Story 1 (lesson CRUD operations)
4. Complete Phase 4: User Story 2 (asset management)
5. Complete Phase 5: User Story 6 (Docusaurus integration)
6. **STOP and VALIDATE**: Test all 3 P1 user stories independently (content CRUD, assets, hydration)
7. Deploy to production (Wrangler deploy)
8. Run end-to-end test: Create lesson → Upload asset → Download book archive → Hydrate Docusaurus → Build succeeds

**MVP Deliverables**: Agents can manage content + assets, Docusaurus can build from PanaversityFS storage

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready (storage abstraction works)
2. Add User Story 1 → Test independently → Content CRUD works
3. Add User Story 2 → Test independently → Asset management works
4. Add User Story 6 → Test independently → Docusaurus integration works (**MVP COMPLETE**)
5. Add User Story 4 → Test independently → Multi-book registry works
6. Add User Story 5 → Test independently → Search functionality works
7. Add User Story 3 → Test independently → Audit query works
8. Add Summary Management → Test independently → Summary storage works
9. Add Observability → Production monitoring enabled
10. Add Migration Tooling → Git content can be seeded
11. Add Documentation → Developer onboarding complete
12. Production Deployment → Live on Cloudflare Workers

Each phase adds value without breaking previous phases.

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together (must be sequential - blocking prerequisite)
2. Once Foundational is done:
   - **Developer A**: User Story 1 (Lesson CRUD) - Phase 3
   - **Developer B**: User Story 2 (Asset Management) - Phase 4
   - **Developer C**: User Story 6 (Docusaurus Integration) - Phase 5
   - **Developer D**: User Story 4 (Registry) - Phase 6
3. Stories complete and integrate independently
4. After MVP (Phases 3-5):
   - **Developer A**: User Story 5 (Search) - Phase 7
   - **Developer B**: User Story 3 (Audit Query) - Phase 8
   - **Developer C**: Summary Management - Phase 9
5. Final phases (Observability, Migration, Docs, Deployment) can be parallelized by different team members

---

## Task Count Summary

- **Total Tasks**: 114
- **Phase 1 (Setup)**: 8 tasks (6 parallelizable)
- **Phase 2 (Foundational)**: 22 tasks (15 parallelizable) - CRITICAL BLOCKER
- **Phase 3 (US1 - Lesson CRUD)**: 7 tasks (4 parallelizable) - MVP
- **Phase 4 (US2 - Asset Management)**: 8 tasks (3 parallelizable) - MVP
- **Phase 5 (US6 - Docusaurus Integration)**: 11 tasks - MVP
- **Phase 6 (US4 - Registry)**: 8 tasks
- **Phase 7 (US5 - Search)**: 6 tasks (2 parallelizable)
- **Phase 8 (US3 - Audit Query)**: 6 tasks
- **Phase 9 (Summary Management)**: 7 tasks (4 parallelizable)
- **Phase 10 (Observability)**: 5 tasks (2 parallelizable)
- **Phase 11 (Migration)**: 10 tasks (2 parallelizable)
- **Phase 12 (Documentation)**: 6 tasks (4 parallelizable)
- **Phase 13 (Deployment)**: 10 tasks

**Parallel Opportunities**: 42 tasks marked [P] can run in parallel within their phases
**MVP Scope**: Phases 1-5 (56 tasks) deliver complete content management + Docusaurus integration

---

## Task Count Summary (Revised)

**Total: 53 tasks** (simplified from original 114 by leveraging OpenDAL/MCP built-ins)

**Key Simplifications**:
1. **No custom storage backends** - OpenDAL Operator handles fs/s3/supabase abstraction
2. **No manual OAuth middleware** - MCP SDK provides built-in auth patterns
3. **Consolidated tool files** - Related tools grouped (content.py has 4 tools, not 4 files)
4. **Removed redundant utils** - SHA256, presigned URLs provided by OpenDAL
5. **Learning phase added** - Phase 0 ensures understanding before building

**MVP Scope**: Phases 0-5 = 35 tasks (Learn MCP + Setup + Foundation + 3 P1 user stories)

---

## Notes

- **Phase 0 is CRITICAL** - must understand MCP + OpenDAL before coding
- [P] tasks = parallelizable (different files, no dependencies)
- Use `uv init` not manual pyproject.toml creation
- OpenDAL Operator methods: read(), write(), delete(), list(), stat(), presign()
- MCP SDK provides @mcp.tool() decorator for tool registration
- Tests NOT included per spec (no TDD requirement)
- Storage backend choice per book via STORAGE_BACKEND env var
- Audit logs use direct JSONL writes (eventual consistency per research.md Decision 2)
- Asset uploads use hybrid pattern (direct <10MB, presigned ≥10MB per research.md Decision 3)
- Archive generation uses streaming ZIP (memory-efficient per research.md Decision 4)
