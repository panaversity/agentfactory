# Requirements Checklist: PanaversityFS

**Feature**: Agent-Native Multi-Book Storage System
**Spec Version**: Draft (2025-11-24)
**Architecture**: OpenDAL + AgentFS + Cloudflare MCP

---

## Storage Abstraction (OpenDAL)

- [ ] **FR-001**: System provides unified storage API supporting Local filesystem, Cloudflare R2, Supabase Storage, and AWS S3 backends via OpenDAL Python library
- [ ] **FR-002**: Storage backend is configurable per-book via `book.yaml` `storage_backend` field with values: [local, r2, supabase, s3]
- [ ] **FR-003**: System handles storage backend failures with 3 retries using exponential backoff (100ms, 200ms, 400ms) before returning error to MCP client

---

## MCP Server Architecture

- [ ] **FR-004**: System implements MCP server using Cloudflare MCP architecture patterns (OAuth authentication, metrics tracking via Analytics Engine, error handling with Sentry)
- [ ] **FR-005**: MCP server exposes 13 tools: `add_content`, `update_content`, `read_content`, `delete_content`, `add_asset`, `get_asset`, `list_assets`, `add_summary`, `get_summary`, `update_summary`, `list_books`, `glob_search`, `grep_search`, `get_audit_log`
- [ ] **FR-006**: All MCP tool calls are authenticated via OAuth 2.0 with token validation against Cloudflare Workers OAuth Provider

---

## Content Operations

- [ ] **FR-007**: `add_content` tool creates new lesson markdown file at path `books/[book-id]/lessons/[part]/[chapter]/[lesson].md`, writes content, logs operation to audit trail, and triggers summary regeneration for parent chapter
- [ ] **FR-008**: `update_content` tool detects concurrent modifications by comparing file hash before write, returns conflict error if hash changed, and suggests merge strategy
- [ ] **FR-009**: `read_content` tool returns lesson markdown content plus metadata: {file_size, last_modified, storage_backend, file_hash_sha256}

---

## Asset Management

- [ ] **FR-010**: `add_asset` tool accepts binary file uploads up to 100MB, stores in path `books/[book-id]/assets/[type]/[filename]`, returns public CDN URL, and logs operation
- [ ] **FR-011**: Asset types are categorized as: slides (PDF), images (PNG/JPG/SVG), videos (MP4), audio (MP3/WAV) with separate storage subdirectories
- [ ] **FR-012**: `get_asset` tool returns asset metadata including: {cdn_url, file_size, mime_type, upload_timestamp, uploaded_by_agent_id}
- [ ] **FR-013**: CDN URLs use format: `https://cdn.panaversity.com/books/[book-id]/assets/[type]/[filename]` for Cloudflare R2 backend with public bucket access

---

## Summary Generation (Vertical Intelligence)

- [ ] **FR-014**: System auto-generates chapter summaries by concatenating all lesson content within chapter and invoking LLM with prompt: "Summarize this chapter in 200 words focusing on key learning outcomes"
- [ ] **FR-015**: Summary regeneration triggers automatically when any lesson within chapter is added/updated/deleted
- [ ] **FR-016**: `get_summary` tool returns summary markdown plus metadata: {generated_at, lessons_included_count, summary_status: [current, stale, generating]}
- [ ] **FR-017**: If summary generation fails due to LLM API unavailability, system marks summary as "stale", continues content operation successfully, and queues regeneration for retry within 5 minutes

---

## Audit Trail (AgentFS Pattern)

- [ ] **FR-018**: All content/asset operations append entry to daily JSONL audit log at path `.audit/YYYY-MM-DD.jsonl` with fields: {timestamp, agent_id, operation, path, status, error_message?}
- [ ] **FR-019**: Audit log rotation occurs at 00:01 UTC daily, archiving previous day's log and creating new file atomically
- [ ] **FR-020**: `get_audit_log` tool supports filters: date_range, agent_id, operation_type, status, and returns paginated results (max 100 entries per response with continuation token)
- [ ] **FR-021**: Audit logs are immutable append-only files stored in same storage backend as content

---

## Multi-Book Registry

- [ ] **FR-022**: System maintains central `registry.yaml` listing all books with schema: `books: [{book_id, title, storage_backend, created_at, status}]`
- [ ] **FR-023**: Each book has isolated `book.yaml` at `books/[book-id]/book.yaml` defining: {title, author, version, storage_backend, content_structure: {parts: [{part_id, chapters: [chapter_id]}]}}
- [ ] **FR-024**: `list_books` tool returns array of all registered books with metadata from `registry.yaml` including status: [active, archived, migrating]
- [ ] **FR-025**: Adding new book via `add_book_to_registry` tool creates directory structure: `books/[book-id]/{book.yaml, parts/, chapters/, lessons/, assets/}` and updates registry atomically

---

## Search Capabilities

- [ ] **FR-026**: `glob_search` tool supports glob patterns (e.g., `**/*.md`, `assets/images/**/*.png`) and returns matching file paths within specified book scope
- [ ] **FR-027**: `grep_search` tool supports regex patterns, searches across all lesson markdown files, and returns matches as: `[{file_path, line_number, matched_line_content}]`
- [ ] **FR-028**: Search operations are scoped to single book by default, with optional `all_books: true` parameter for cross-book search

---

## Observability (Cloudflare MCP Pattern)

- [ ] **FR-029**: MCP server logs all tool invocations to Cloudflare Analytics Engine with fields: {tool_name, user_id, execution_time_ms, error_code?, timestamp}
- [ ] **FR-030**: Errors are reported to Sentry with context: {tool_name, agent_id, operation, error_stack_trace, storage_backend}
- [ ] **FR-031**: System exposes health check endpoint `/health` returning: {status: "healthy", storage_backends: {local: "up", r2: "up"}, last_operation_timestamp}

---

## Docusaurus Integration

- [ ] **FR-032**: Hydration script (`docusaurus-hydration.ts`) runs during `docusaurus build` phase, authenticates with PanaversityFS MCP server using OAuth token from `PANAVERSITY_OAUTH_TOKEN` environment variable
- [ ] **FR-033**: Hydration script downloads all lessons to `apps/learn-app/docs/[part]/[chapter]/[lesson].md` and all assets to `book-source/static/assets/`, preserving directory structure
- [ ] **FR-034**: If hydration fails (storage unavailable, auth failure), Docusaurus build fails with clear error message indicating which lesson/asset failed to fetch

---

## Success Criteria

### Performance

- [ ] **SC-001**: AI agents can create/read/update/delete lesson content in under 500ms (95th percentile) for files up to 50KB
- [ ] **SC-002**: Asset uploads complete in under 3 seconds for files up to 10MB over typical network connections (5 Mbps upload)
- [ ] **SC-009**: Search operations (`glob_search`, `grep_search`) return results within 2 seconds for books containing up to 500 lessons
- [ ] **SC-010**: Docusaurus hydration script downloads complete book (50 lessons, 100 assets) in under 5 minutes during CI/CD build

### Reliability

- [ ] **SC-003**: Storage backend failures trigger automatic retry with exponential backoff, succeeding on retry 95% of the time for transient errors
- [ ] **SC-005**: Concurrent modifications detected with 100% accuracy via file hash comparison, returning conflict error before data corruption occurs
- [ ] **SC-007**: Stale summaries (LLM API unavailable) retry generation successfully within 5 minutes of API restoration
- [ ] **SC-012**: OAuth token refresh succeeds automatically on expiration with <1 second delay before operation retry
- [ ] **SC-014**: Storage backend migration completes for single book (100 lessons, 200 assets) within 15 minutes with zero data loss
- [ ] **SC-015**: System handles 100 concurrent agent operations (different lessons) without conflicts or performance degradation

### Observability

- [ ] **SC-004**: All operations append audit log entry within 50ms of completion with complete metadata (timestamp, agent_id, operation, status)
- [ ] **SC-013**: Audit log queries with date range filters return paginated results (100 entries) in under 500ms
- [ ] **SC-016**: Error reporting to Sentry includes full context (tool name, agent ID, storage backend, stack trace) for 100% of errors
- [ ] **SC-017**: Analytics Engine logs capture all tool invocations with execution time, enabling 95th percentile latency monitoring

### Intelligence Layer

- [ ] **SC-006**: Chapter summary regeneration completes within 10 seconds of lesson update using LLM API (Claude Haiku)

### Data Integrity

- [ ] **SC-008**: Multi-book registry supports at least 50 books without performance degradation on `list_books` operation (response time <200ms)
- [ ] **SC-011**: Hydration script failures provide actionable error messages identifying exact lesson/asset that failed with retry instructions
- [ ] **SC-018**: Registry-storage consistency check on startup detects and auto-repairs desync conditions (orphan registry entries or missing directories) within 10 seconds

---

## User Stories Validation

### Priority P1 (Critical Path)

- [ ] **US-1**: AI Agent Updates Lesson Content — Agent can modify lesson content with audit tracking and summary regeneration
- [ ] **US-2**: Upload and Manage Book Assets — Agent can upload images/slides/videos and retrieve CDN URLs
- [ ] **US-6**: Docusaurus Build Integration — Hydration script fetches complete book content during build

### Priority P2 (Enables Advanced Workflows)

- [ ] **US-3**: Query Audit Trail for Operations — Developer can inspect recent operations for debugging
- [ ] **US-4**: Multi-Book Registry Management — Admin can add new books and configure storage backends
- [ ] **US-5**: Search Book Content — Agent can find lessons via keyword/pattern search

---

## Edge Cases Coverage

- [ ] **Storage backend failure**: R2 503 error triggers 3 retries with exponential backoff, fails gracefully with audit log entry
- [ ] **Concurrent modifications**: Two agents updating same lesson within 100ms triggers conflict detection via file hash comparison
- [ ] **Large asset uploads**: 500MB video file rejected with error and chunked upload instructions
- [ ] **Missing summary generation**: LLM API unavailable marks summary as "stale", queues retry, continues content operation
- [ ] **Storage backend migration**: Operations queue in-memory during migration, replay against new backend after cutover
- [ ] **Audit log rotation**: Midnight boundary uses atomic file append, rotation at 00:01, no operations lost
- [ ] **Registry desync**: Startup detects orphan registry entries or missing directories, auto-repairs via audit log history
- [ ] **OAuth token expiration**: 401 response triggers token refresh using refresh token, retries operation once
- [ ] **Docusaurus build timeout**: Hydration script supports configurable timeout (default 15 min), logs progress every 60s

---

## Dependencies Verification

### External Libraries

- [ ] **OpenDAL** (v0.45.0+): Python library for unified storage API (Local, R2, S3, Supabase)
- [ ] **@modelcontextprotocol/sdk** (v1.20.2+): TypeScript SDK for MCP server implementation
- [ ] **@cloudflare/workers-oauth-provider** (v0.0.13+): OAuth 2.0 authentication library
- [ ] **zod** (v3.24.2+): Schema validation for MCP tool parameters
- [ ] **Anthropic SDK** (v0.40.0+): Python client for Claude API (summary generation)

### Infrastructure Services

- [ ] **Cloudflare Workers**: MCP server runtime environment (OAuth, Analytics Engine, R2)
- [ ] **Cloudflare R2**: Primary storage backend (production)
- [ ] **Cloudflare Analytics Engine**: Tool invocation metrics
- [ ] **Sentry**: Error tracking and alerting
- [ ] **Supabase Storage**: Alternative storage backend (optional)
- [ ] **AWS S3**: Backup storage backend (optional)

### Development Tools

- [ ] **TypeScript** (v5.5.4+): MCP server and hydration script implementation
- [ ] **Python** (v3.11+): OpenDAL integration, summary generation scripts
- [ ] **Wrangler** (v4.10.0+): Cloudflare Workers deployment CLI
- [ ] **Vitest** (v3.0.9+): Testing framework for MCP server evaluation

---

## Non-Goals Confirmation

- [ ] **Vector search** (LanceDB) — Explicitly deferred to later phase
- [ ] **Version control integration** (Git history) — Out of scope, use external Git workflows
- [ ] **Real-time collaboration** (WebSockets) — Asynchronous agent workflows only
- [ ] **Content validation** (markdown schema) — Agent responsibility, not storage layer
- [ ] **Multi-tenancy** (user isolation) — Single-tenant architecture
- [ ] **Built-in CDN** (cache invalidation) — Assumes external CDN setup
- [ ] **Content transformation** (markdown-to-PDF) — Docusaurus handles rendering
- [ ] **Analytics dashboard** (built-in UI) — Use external tools (Grafana, Sentry UI)

---

## Open Questions (Requires Decision Before Phase 2)

- [ ] **Q1**: LLM model selection — Claude Haiku (faster, cheaper) vs Sonnet (higher quality) for summaries?
- [ ] **Q2**: OAuth scope granularity — Per-book scopes vs full registry access?
- [ ] **Q3**: Audit log retention — Indefinite storage vs auto-archive after 90 days?
- [ ] **Q4**: Asset versioning — Immutable (delete + re-upload) vs in-place updates with version history?
- [ ] **Q5**: Content validation hooks — Webhook for external validation vs post-processing only?
- [ ] **Q6**: Multi-region replication — Defer until scale requirements or implement now?
- [ ] **Q7**: Migration tooling — CLI tool in Phase 1 vs manual process documented?
- [ ] **Q8**: Docusaurus cache invalidation — Checksum comparison to skip unchanged files vs always fetch full tree?

---

**Checklist Status**: 34 Functional Requirements | 18 Success Criteria | 6 User Stories | 9 Edge Cases | 8 Open Questions
**Last Updated**: 2025-11-24
**Next Review**: After Phase 1 implementation planning
