---
id: 0002
title: PanaversityFS Task Generation
stage: tasks
date: 2025-11-24
surface: claude-code
model: claude-sonnet-4-5
feature: 030-panaversity-fs
branch: 030-panaversity-fs
user: mjs
command: /sp.tasks panaversity-fs
labels: [infrastructure, mcp-server, task-generation, implementation-planning]
links:
  spec: specs/030-panaversity-fs/spec.md
  ticket: N/A
  adr: N/A
  pr: N/A
files:
  - path: specs/030-panaversity-fs/tasks.md
    action: created
    lines: 578
tests:
  - none (task generation phase, no test requirements in spec)
---

## Prompt

User requested execution of `/sp.tasks panaversity-fs` command to generate actionable, dependency-ordered implementation tasks from completed specification and planning artifacts.

**Context**: Planning phase completed with all design documents:
- spec.md (448 lines): 6 user stories (3 P1, 3 P2), 38 functional requirements, 18 success criteria
- plan.md (223 lines): Technical context, project structure, performance goals
- research.md (382 lines): 4 technical decisions (Node.js + OpenDAL, direct JSONL audit, hybrid asset upload, streaming ZIP archives)
- data-model.md (689 lines): 6 entities (Registry, Book, Lesson, Asset, Summary, AuditEntry)
- contracts/ (16 JSON files): Complete schemas for all 15 MCP tools

**Command workflow**:
1. Read all design documents from feature directory
2. Extract user stories with priorities from spec.md
3. Map entities and contracts to user stories
4. Generate tasks organized by user story (enable independent implementation/testing)
5. Create dependency graph showing completion order
6. Identify parallel execution opportunities
7. Define MVP scope (typically User Story 1)
8. Output tasks.md with strict checklist format

## Response snapshot

**Task generation completed successfully.**

### Task Organization

**Total**: 114 tasks across 13 phases
**Format**: Strict checklist format - `- [ ] [ID] [P?] [Story] Description with file path`

### Phase Breakdown

1. **Phase 1 - Setup** (8 tasks): Project initialization, dependencies, configuration
2. **Phase 2 - Foundational** (22 tasks): CRITICAL BLOCKER - Storage abstraction, MCP server, auth, audit, schemas
3. **Phase 3 - US1 Lesson CRUD** (7 tasks, P1 MVP): Content operations (add/update/read/delete)
4. **Phase 4 - US2 Asset Management** (8 tasks, P1 MVP): Binary uploads, CDN URLs, presigned URLs
5. **Phase 5 - US6 Docusaurus Integration** (11 tasks, P1 MVP): Bulk download, streaming ZIP, hydration script
6. **Phase 6 - US4 Registry** (8 tasks, P2): Multi-book management, registry.yaml operations
7. **Phase 7 - US5 Search** (6 tasks, P2): Glob and grep search across content
8. **Phase 8 - US3 Audit Query** (6 tasks, P2): Filter and paginate audit logs
9. **Phase 9 - Summary Management** (7 tasks): Storage-only operations (no intelligence)
10. **Phase 10 - Observability** (5 tasks): Analytics Engine, Sentry, health checks
11. **Phase 11 - Migration Tooling** (10 tasks): CLI for Git → PanaversityFS seeding
12. **Phase 12 - Documentation** (6 tasks): Quickstart verification, examples, guides
13. **Phase 13 - Deployment** (10 tasks): Cloudflare Workers production deployment

### Key Mappings

**User Stories → Phases**:
- US1 (Lesson CRUD, P1) → Phase 3 (7 tasks)
- US2 (Asset Management, P1) → Phase 4 (8 tasks)
- US6 (Docusaurus Integration, P1) → Phase 5 (11 tasks)
- US4 (Registry, P2) → Phase 6 (8 tasks)
- US5 (Search, P2) → Phase 7 (6 tasks)
- US3 (Audit Query, P2) → Phase 8 (6 tasks)

**Entities → Tasks**:
- Registry (data-model.md) → T057-T064 (list_books, add_book_to_registry, validation)
- Lesson (data-model.md) → T031-T037 (4 MCP tools: add/update/read/delete_content)
- Asset (data-model.md) → T038-T045 (3 MCP tools: add/get/list_assets, hybrid upload pattern)
- Summary (data-model.md) → T077-T083 (4 MCP tools: add/update/get/list_summaries)
- AuditEntry (data-model.md) → T023-T024, T071-T076 (JSONL logger, query tool)

**Contracts → Tools**:
- 4 content tools (contracts/add_content.json, update_content.json, read_content.json, delete_content.json) → T031-T034
- 3 asset tools (contracts/add_asset.json, get_asset.json, list_assets.json) → T038-T040
- 4 summary tools (contracts/add_summary.json, update_summary.json, get_summary.json, list_summaries.json) → T077-T080
- 1 bulk tool (contracts/get_book_archive.json) → T046-T051
- 1 registry tool (contracts/list_books.json) → T057
- 2 search tools (contracts/glob_search.json, grep_search.json) → T065-T066
- 1 audit tool (contracts/get_audit_log.json) → T071

### Parallel Opportunities

**42 tasks marked [P]** can run in parallel within their phases:
- Phase 1: 6 parallelizable (dependencies, config files)
- Phase 2: 15 parallelizable (schemas, storage backends, utils)
- Phase 3: 4 parallelizable (all content MCP tools)
- Phase 4: 3 parallelizable (all asset MCP tools)
- Phase 7: 2 parallelizable (glob + grep search tools)
- Phase 9: 4 parallelizable (all summary MCP tools)
- Phase 10: 2 parallelizable (Analytics Engine + Sentry)
- Phase 11: 2 parallelizable (Git reader + registry init)
- Phase 12: 4 parallelizable (all documentation tasks)

**User story independence**: After Foundational phase (Phase 2) completes, all user story phases (3-9) can proceed in parallel if team capacity allows.

### Dependencies

**Critical Path**:
1. Phase 1 (Setup) → MUST complete first
2. Phase 2 (Foundational) → BLOCKS all user stories (storage adapter, MCP server, auth, audit)
3. Phases 3-9 (User Stories) → All depend on Phase 2, independent of each other
4. Phases 10-13 (Cross-Cutting) → Depend on relevant user stories

**MVP Scope**: Phases 1-5 (56 tasks) = Setup + Foundational + US1 (Lesson CRUD) + US2 (Assets) + US6 (Docusaurus)

### Implementation Strategies Documented

1. **MVP First**: Phases 1-5 → Test → Deploy (complete content management + Docusaurus integration)
2. **Incremental Delivery**: Complete each user story → Test independently → Deploy (8 incremental releases)
3. **Parallel Team**: Foundational together, then split user stories across developers

## Outcome

- ✅ Impact: **Complete implementation roadmap generated (114 tasks, 578 lines).** All 6 user stories mapped to executable tasks with clear dependencies, parallel opportunities, and MVP scope defined (56 tasks). **UPDATED to Python MCP** per user direction (originally Node.js/TypeScript). Ready for implementation via `/sp.implement` command.
- 🧪 Tests: No test tasks included per spec (no TDD requirement mentioned in specification). Contract/integration/e2e test strategy defined in plan.md but tests not requested for implementation.
- 📁 Files: **1 file created + updated**: tasks.md (578 lines) with 13 phases, 114 tasks, dependency graph, 3 execution strategies. **Updated all file extensions (.ts → .py), dependencies (Zod → Pydantic, @modelcontextprotocol/sdk → mcp), deployment (Cloudflare Workers → Docker/systemd).**
- 🔁 Next prompts: Execute `/sp.implement panaversity-fs` to begin implementation starting with Phase 1 (Setup), or manually execute tasks in order (Setup → Foundational → MVP user stories)
- 🧠 Reflection: Task generation successfully organized by user story for independent implementation/testing. All 15 MCP tools mapped from contracts to tasks with exact file paths. Research decisions (JSONL audit, hybrid assets, streaming ZIP) propagated into implementation tasks. **Runtime decision changed from Node.js to Python MCP** per user guidance ("use python mcp unless theres a spreclisl readsob"). Python provides: native OpenDAL bindings, mature async/await with asyncio, Pydantic validation, simpler deployment (Docker/systemd vs Cloudflare Workers constraints). MVP scope clearly defined (3 P1 user stories = 26 tasks after Foundational). Parallel opportunities maximized (42 [P] tasks across phases).

## Evaluation notes (flywheel)

- Failure modes observed: None. All design documents loaded successfully, all user stories mapped to tasks, all entities and contracts covered.
- Graders run and results (PASS/FAIL): No formal graders (infrastructure task generation, not educational content). Manual validation: All 6 user stories have complete task phases, all 15 MCP tools have implementation tasks, all 6 entities have CRUD tasks, Foundational phase correctly identified as blocking prerequisite, 42 parallel tasks correctly marked.
- Prompt variant (if applicable): Default `/sp.tasks` workflow (load design docs → extract user stories → map entities/contracts → generate phases → document dependencies)
- Next experiment (smallest change to try): N/A - workflow succeeded. For future infrastructure features, consider adding "performance benchmarking tasks" as final phase (measure p95 latency against success criteria before deployment).
