# Feature Specification: PanaversityFS - Agent-Native Multi-Book Storage System

**Feature Branch**: `030-panaversity-fs`
**Created**: 2025-11-24
**Status**: Implementation
**Architecture**: Python MCP + OpenDAL (Storage) + JSONL Audit + Stateless HTTP Transport

**ARCHITECTURE NOTE**: Originally designed for Node.js/Cloudflare Workers. Revised to Python MCP per user direction (2025-11-24). Deployment: Docker/systemd HTTP service, not edge runtime.

## User Scenarios & Testing _(mandatory)_

### User Story 1 - AI Agent Updates Lesson Content (Priority: P1)

An AI agent needs to modify lesson content within a book chapter, ensuring changes are tracked and the system remains synchronized with Docusaurus builds.

**Why this priority**: Core CRUD operations are the foundation of all agent interactions with educational content. Without content modification capabilities, no other features have value.

**Independent Test**: Can be fully tested by invoking `update_content` MCP tool with lesson path and new content, verifying file write and audit log entry. Delivers immediate value for content management workflows.

**Acceptance Scenarios**:

1. **Given** a lesson file exists at `books/ai-native-software-development/lessons/part-1/chapter-01/lesson-01.md`, **When** agent calls `write_content` with updated markdown and file_hash, **Then** file is written and audit log records operation with timestamp/agent_id
2. **Given** an agent attempts to write non-existent lesson, **When** `write_content` is called without file_hash, **Then** system creates new file and returns success
3. **Given** concurrent agents update same lesson, **When** both call `write_content` within 100ms, **Then** second operation detects conflict via file_hash mismatch and returns retry-with-merge suggestion

---

### User Story 2 - Upload and Manage Book Assets (Priority: P1)

An AI agent needs to upload images, slides, or videos to accompany lesson content and retrieve CDN URLs for embedding in markdown.

**Why this priority**: Educational content requires rich media assets. Asset management is essential for production-quality books alongside text content.

**Independent Test**: Can be tested by uploading a PDF slide deck via `add_asset`, verifying storage backend write, and retrieving public CDN URL via `get_asset`. Delivers value for multimedia educational content.

**Acceptance Scenarios**:

1. **Given** a PDF file "chapter-01-slides.pdf", **When** agent calls `add_asset` with book/chapter context and binary data, **Then** file uploads to storage backend, CDN URL returns (e.g., `https://cdn.panaversity.com/books/ai-native-software-development/assets/slides/chapter-01-slides.pdf`), and audit log records upload
2. **Given** an asset exceeds 100MB size limit, **When** `add_asset` is called, **Then** system returns error "Asset too large" with chunked upload instructions
3. **Given** agent requests non-existent asset, **When** `get_asset` is called, **Then** system returns 404 with list of available assets in that chapter

---

### User Story 3 - Query Audit Trail for Operations (Priority: P2)

A developer or monitoring agent needs to inspect recent operations performed on book content to debug issues or analyze agent behavior.

**Why this priority**: Observability is critical for production systems but not required for basic content operations. Enables debugging after core features work.

**Independent Test**: Can be tested by performing several content operations, then calling `get_audit_log` with filters (date range, agent_id, operation type), verifying returned JSONL entries match expected operations.

**Acceptance Scenarios**:

1. **Given** 50 operations occurred in last 24 hours, **When** agent calls `get_audit_log` with filter `date_range: "2025-11-24"`, **Then** system returns JSONL array of all operations with metadata (timestamp, agent_id, operation, path, status)
2. **Given** audit log spans multiple daily files, **When** query crosses midnight boundary, **Then** system aggregates entries from both files transparently
3. **Given** audit log exceeds 10,000 entries for date range, **When** `get_audit_log` is called, **Then** system returns paginated response with continuation token

---

### User Story 4 - Multi-Book Registry Management (Priority: P2)

A system administrator needs to add new books to the platform registry and configure storage backends per book.

**Why this priority**: Multi-book capability is essential for platform scalability but not needed for single-book MVP. Enables future growth without architectural rework.

**Independent Test**: Can be tested by calling `add_book_to_registry` with book metadata, verifying `registry.yaml` update, book directory creation, and `book.yaml` initialization.

**Acceptance Scenarios**:

1. **Given** registry contains 3 existing books, **When** admin calls MCP tool `add_book_to_registry` with `book_id: "generative-ai-fundamentals"`, `title: "Generative AI Fundamentals"`, `storage_backend: "r2"`, **Then** `registry.yaml` adds new entry, `books/generative-ai-fundamentals/` directory initializes with `book.yaml`, and audit log records registry update
2. **Given** book_id conflicts with existing book, **When** `add_book_to_registry` is called, **Then** system returns error "Book ID already exists" with suggested alternatives
3. **Given** invalid storage backend specified, **When** `add_book_to_registry` is called, **Then** system returns error listing valid backends: [local, r2, supabase, s3]

---

### User Story 5 - Search Book Content (Priority: P2)

An AI agent needs to find lessons containing specific keywords or matching file patterns to analyze content structure or locate references.

**Why this priority**: Search capabilities enhance agent intelligence but core content operations work without it. Enables advanced agent workflows like cross-referencing and content analysis.

**Independent Test**: Can be tested by calling `grep_search` with pattern "OpenDAL" and verifying returned matches include file paths and line numbers from all lessons mentioning the term.

**Acceptance Scenarios**:

1. **Given** 20 lessons across 5 chapters, **When** agent calls `grep_search` with pattern "OpenDAL", **Then** system returns array of matches: `[{file: "lessons/part-4/chapter-12/lesson-03.md", line: 42, content: "...OpenDAL provides..."}]`
2. **Given** agent searches for TypeScript files, **When** `glob_search` called with pattern `**/*.ts`, **Then** system returns all TypeScript file paths in book assets directory
3. **Given** search pattern matches 1000+ files, **When** `grep_search` is called, **Then** system returns first 100 results with warning "Results truncated, refine pattern"

---

### User Story 6 - Docusaurus Build Integration (Priority: P1)

The Docusaurus static site generator needs to fetch complete book content from PanaversityFS storage during build time to generate the published website.

**Why this priority**: Publishing pipeline integration is essential for the complete workflow. Without it, content management has no public output, breaking the production chain.

**Independent Test**: Can be tested by running `docusaurus build` with hydration script that calls PanaversityFS MCP tools, verifying all lessons/assets download to `apps/learn-app/docs/`, and build completes successfully.

**Acceptance Scenarios**:

1. **Given** PanaversityFS contains 50 lessons across 10 chapters, **When** `hydration-script.ts` runs during `docusaurus build`, **Then** all lessons download to `apps/learn-app/docs/[part]/[chapter]/[lesson].md`, all assets download to `book-source/static/assets/`, and Docusaurus build succeeds
2. **Given** a lesson file is corrupted in storage, **When** hydration script attempts download, **Then** build fails with clear error: "Failed to fetch lesson: part-1/chapter-01/lesson-01.md - storage returned 500"
3. **Given** build runs in CI/CD environment, **When** hydration script authenticates, **Then** OAuth token retrieves from environment variable `PANAVERSITY_OAUTH_TOKEN` and MCP connection succeeds

---

### Edge Cases

- **Storage backend failure**: What happens when Cloudflare R2 returns 503 during content write? System must retry 3 times with exponential backoff, then fail gracefully with audit log entry marking operation as "failed_storage_error"
- **Concurrent modifications**: How does system handle two agents updating same lesson within 100ms? Second operation detects conflict via file hash comparison and returns error with merge suggestion
- **Large asset uploads**: What happens when agent uploads 500MB video file? System rejects with error if >100MB, provides chunked upload instructions referencing multipart upload API
- **Missing summary files**: How does system handle requests for summaries that don't exist? Returns 404 error with message "Summary not found at [path]. Use add_summary tool to create it."
- **Storage backend migration**: What happens when moving book from Local to R2 mid-operation? Operations queue in-memory, migration script runs, queued operations replay against new backend after cutover
- **Audit log rotation**: How does system handle midnight boundary during audit log write? Uses atomic file append to current day's JSONL file, log rotation script runs at 00:01 to archive previous day, no operations lost
- **Registry desync**: What happens if `registry.yaml` lists book but `books/[book-id]/` directory missing? System detects on startup, logs warning "Registry-storage desync detected", auto-repairs by creating missing directory structure or removing orphan registry entry based on audit log history
- **OAuth token expiration**: What happens when MCP client's token expires mid-operation? System detects 401 response, attempts token refresh using refresh token, retries operation once, fails with "authentication_expired" if refresh fails
- **Docusaurus build timeout**: What happens if hydration script takes >10 minutes to download all content? Build system must support configurable timeout via environment variable, default 15 minutes, logs progress every 60 seconds to prevent CI timeout perception

## Requirements _(mandatory)_

### Functional Requirements

**Storage Abstraction (OpenDAL)**

- **FR-001**: System MUST provide unified storage API supporting Local filesystem, Cloudflare R2, Supabase Storage, and AWS S3 backends using storage abstraction layer compatible with Cloudflare Workers runtime (Rust/WASM or Node.js with OpenDAL bindings)
- **FR-002**: Storage backend MUST be configurable per-book via `book.yaml` `storage_backend` field with values: [local, r2, supabase, s3]
- **FR-003**: System MUST handle storage backend failures with 3 retries using exponential backoff (100ms, 200ms, 400ms) before returning error to MCP client

**MCP Server Architecture**

- **FR-004**: System MUST implement MCP server using Python MCP SDK (FastMCP) with Stateless Streamable HTTP transport. Authentication via API key for MVP (OAuth deferred to future enhancement). Error handling with structured logging and Sentry integration.
- **FR-005**: MCP server MUST expose 9 tools: `read_content`, `write_content` (upsert), `delete_content`, `upload_asset`, `get_asset`, `list_assets`, `get_book_archive`, `list_books`, `glob_search`, `grep_search`. Note: Summary operations use content tools with `.summary.md` naming convention (see ADR-0018).
- **FR-006**: (DEFERRED) Authentication via API key in `Authorization` header for MVP. OAuth 2.0 implementation deferred to post-MVP. API key validated against `PANAVERSITY_API_KEY` environment variable.

**Content Operations**

- **FR-007**: `write_content` tool MUST support upsert semantics: (a) If `file_hash` parameter provided, verify hash matches current file before update (conflict detection for updates), (b) If `file_hash` omitted, create new file or overwrite existing (idempotent create). Tool writes to path `books/[book-id]/content/[part]/[chapter]/[lesson].md` (Docusaurus-aligned structure per ADR-0018) and logs operation to audit trail. Summaries use same tool with `.summary.md` suffix.
- **FR-008**: `write_content` MUST detect concurrent modifications by comparing provided `file_hash` against current file SHA256 hash before write. If mismatch, return conflict error with current hash and suggest merge strategy.
- **FR-009**: `read_content` tool MUST return lesson markdown content plus metadata: {file_size, last_modified, storage_backend, file_hash_sha256, content}

**Asset Management**

- **FR-010**: `upload_asset` tool MUST support binary file uploads up to 100MB via hybrid pattern: (a) Direct upload through MCP tool for assets <10MB (base64 encoded in `binary_data` parameter), (b) Presigned URL pattern for assets ≥10MB where tool returns temporary upload URL and client uploads directly to storage backend (if OpenDAL supports presign_write, else use boto3 for S3/R2).
- **FR-011**: Asset types MUST be categorized as: slides (PDF), images (PNG/JPG/SVG), videos (MP4), audio (MP3/WAV) with separate storage subdirectories
- **FR-012**: `get_asset` tool MUST return asset metadata including: {cdn_url, file_size, mime_type, upload_timestamp, uploaded_by_agent_id}
- **FR-013**: CDN URLs MUST use format: `https://cdn.panaversity.com/books/[book-id]/assets/[type]/[filename]` for Cloudflare R2 backend with public bucket access

**Summary Management (via Content Tools - ADR-0018)**

- **FR-014**: (REMOVED) Summary operations use `write_content`, `read_content`, `delete_content` with `.summary.md` naming convention. Example: `content/01-Part/01-Chapter/01-lesson.md` has summary at `content/01-Part/01-Chapter/01-lesson.summary.md`
- **FR-015**: (REMOVED) Summaries are regular markdown files, no special metadata beyond content tool response
- **FR-016**: (REMOVED) Use `delete_content` for summary deletion

**Audit Trail (AgentFS Pattern)**

- **FR-018**: All content/asset operations MUST write audit entries with fields: {timestamp, agent_id, operation, path, status, error_message?}. Implementation MUST use either (a) Cloudflare D1 database for hot storage with daily export to JSONL files in `.audit/YYYY-MM-DD.jsonl`, OR (b) direct JSONL writes with eventual consistency guarantees (race conditions acceptable for audit trail)
- **FR-019**: Audit log rotation MUST occur at 00:01 UTC daily, creating new daily partition (D1 table or JSONL file)
- **FR-020**: `get_audit_log` tool MUST support filters: date_range, agent_id, operation_type, status, and return paginated results (max 100 entries per response with continuation token)
- **FR-021**: Audit logs MUST be immutable after daily rotation completes (D1 records or JSONL files become read-only)

**Multi-Book Registry**

- **FR-022**: System MUST maintain central `registry.yaml` listing all books with schema: `books: [{book_id, title, storage_backend, created_at, status}]`
- **FR-023**: Each book MUST have isolated `book.yaml` at `books/[book-id]/book.yaml` defining: {title, author, version, storage_backend, content_structure: {parts: [{part_id, chapters: [chapter_id]}]}}
- **FR-024**: `list_books` tool MUST return array of all registered books with metadata from `registry.yaml` including status: [active, archived, migrating]
- **FR-025**: Adding new book via `add_book_to_registry` tool MUST create directory structure: `books/[book-id]/{book.yaml, parts/, chapters/, lessons/, assets/}` and update registry atomically

**Search Capabilities**

- **FR-026**: `glob_search` tool MUST support glob patterns (e.g., `**/*.md`, `assets/images/**/*.png`) and return matching file paths within specified book scope
- **FR-027**: `grep_search` tool MUST support regex patterns, search across all lesson markdown files, and return matches as: `[{file_path, line_number, matched_line_content}]`
- **FR-028**: Search operations MUST be scoped to single book by default, with optional `all_books: true` parameter for cross-book search

**Bulk Operations**

- **FR-029**: `get_book_archive` tool MUST generate and return presigned URL to download entire book directory as compressed archive (ZIP or TAR format). Archive MUST include all lessons, assets, summaries, and metadata files for specified book. URL MUST be valid for 1 hour and expire after single download or timeout
- **FR-030**: Archive generation MUST complete within 60 seconds for books containing up to 500 files totaling 200MB uncompressed
- **FR-031**: If archive generation fails (timeout, storage error), system MUST return error with fallback suggestion to use individual file download tools

**Observability (Python Logging + Sentry)**

- **FR-032**: (REVISED) MCP server MUST log all tool invocations to structured JSON logs (stdout) with fields: {tool_name, agent_id, execution_time_ms, error_code?, timestamp, storage_backend}. Cloudflare Analytics Engine replaced with standard Python logging. Consider MCP-specific observability tools if available.
- **FR-033**: Errors MUST be reported to Sentry with context: {tool_name, agent_id, operation, error_stack_trace, storage_backend}
- **FR-034**: System MUST expose health check endpoint `/health` returning: {status: "healthy", storage_backends: {fs: "up", s3: "up"}, last_operation_timestamp}

**Docusaurus Integration**

- **FR-035**: Hydration script (`docusaurus-hydration.py`) MUST run during `docusaurus build` phase, authenticate with PanaversityFS MCP server using API key from `PANAVERSITY_API_KEY` environment variable (OAuth deferred)
- **FR-036**: Hydration script SHOULD use `get_book_archive` tool to download entire book in single request for performance. Script MAY fall back to individual file downloads if archive generation fails
- **FR-037**: Hydration script MUST extract downloaded content to `apps/learn-app/docs/[part]/[chapter]/[lesson].md` and assets to `book-source/static/assets/`, preserving directory structure
- **FR-038**: If hydration fails (storage unavailable, auth failure), Docusaurus build MUST fail with clear error message indicating root cause (archive download failure, extraction error, or individual file fetch failure)

### Key Entities

- **Book**: Represents a complete educational book with isolated storage, defined by `book.yaml` containing metadata (title, author, version, storage backend, content structure)
- **Part**: Top-level division within a book (e.g., "Part 1: Foundations"), groups related chapters, defined in `book.yaml` content_structure
- **Chapter**: Collection of lessons teaching a coherent topic, may have associated summary file, maps to directory `books/[book-id]/chapters/[chapter-id]/`
- **Lesson**: Single markdown file teaching specific concept, stored at `books/[book-id]/lessons/[part]/[chapter]/[lesson].md`, has metadata (file_size, last_modified, hash)
- **Asset**: Binary file (slides, images, videos, audio) associated with lessons, stored in `books/[book-id]/assets/[type]/[filename]`, has public CDN URL
- **Summary**: Pre-generated chapter overview stored as markdown file (`.summary.md`), created and updated by external agents, has metadata (file_size, last_modified, hash)
- **Audit Entry**: Immutable JSONL record of operation, stored in `.audit/YYYY-MM-DD.jsonl`, contains timestamp, agent_id, operation, path, status, error details
- **Registry**: Central `registry.yaml` listing all books with metadata (book_id, title, storage_backend, status), single source of truth for book catalog
- **Storage Backend**: One of [Local filesystem, Cloudflare R2, Supabase Storage, AWS S3], configured per-book, abstracted via OpenDAL

## Success Criteria _(mandatory)_

### Measurable Outcomes

- **SC-001**: AI agents can create/read/update/delete lesson content in under 500ms (95th percentile) for files up to 50KB
- **SC-002**: Asset uploads complete in under 3 seconds for files up to 10MB over typical network connections (5 Mbps upload)
- **SC-003**: Storage backend failures trigger automatic retry with exponential backoff, succeeding on retry 95% of the time for transient errors
- **SC-004**: All operations append audit log entry within 50ms of completion with complete metadata (timestamp, agent_id, operation, status)
- **SC-005**: Concurrent modifications detected with 100% accuracy via file hash comparison, returning conflict error before data corruption occurs
- **SC-006**: (UPDATED per ADR-0018) Summary files (`.summary.md`) use content tools; same 500ms performance target as SC-001
- **SC-007**: (UPDATED per ADR-0018) Summary files return same metadata as content files via `read_content`
- **SC-008**: Multi-book registry supports at least 50 books without performance degradation on `list_books` operation (response time <200ms)
- **SC-009**: Search operations (`glob_search`, `grep_search`) return results within 2 seconds for books containing up to 500 lessons
- **SC-010**: Docusaurus hydration script using `get_book_archive` downloads complete book (50 lessons, 100 assets, ~200MB) in under 60 seconds during CI/CD build, compared to 5+ minutes with individual file downloads
- **SC-011**: Hydration script failures provide actionable error messages identifying exact lesson/asset that failed with retry instructions
- **SC-012**: OAuth token refresh succeeds automatically on expiration with <1 second delay before operation retry
- **SC-013**: Audit log queries with date range filters return paginated results (100 entries) in under 500ms
- **SC-014**: Storage backend migration completes for single book (100 lessons, 200 assets) within 15 minutes with zero data loss
- **SC-015**: System handles 100 concurrent agent operations (different lessons) without conflicts or performance degradation
- **SC-016**: Error reporting to Sentry includes full context (tool name, agent ID, storage backend, stack trace) for 100% of errors
- **SC-017**: Analytics Engine logs capture all tool invocations with execution time, enabling 95th percentile latency monitoring
- **SC-018**: Registry-storage consistency check on startup detects and auto-repairs desync conditions (orphan registry entries or missing directories) within 10 seconds

## Non-Goals _(explicit scope boundaries)_

- **AI-powered content generation**: PanaversityFS is a storage system, NOT a content intelligence system. Summary generation, content validation, and other AI operations are the responsibility of external agents (Claude Code, custom scripts, CI/CD pipelines). System only stores pre-generated summaries; it does not invoke LLM APIs or trigger automatic content generation.
- **Vector search**: LanceDB integration for semantic search is explicitly excluded from initial scope. Current grep/glob search provides sufficient keyword-based discovery. Vector embeddings can be added later as Layer 3 intelligence without architectural changes.
- **Version control integration**: Git-based history tracking is not implemented. Audit trail provides operation history, but content versioning (rollback, diffs, branching) is out of scope. Users needing version control should use external Git workflows.
- **Real-time collaboration**: No WebSocket-based live editing or presence indicators. System supports concurrent operations via conflict detection but assumes asynchronous agent workflows, not human real-time collaboration.
- **Content validation**: No schema validation for lesson markdown structure or pedagogical quality checks. System stores content as-is; validation is responsibility of content-generating agents or external pipelines.
- **Multi-tenancy**: Single-tenant architecture. All books belong to one organization (Panaversity). User-level isolation, permissions, or organization-scoped registries are not implemented.
- **Built-in CDN**: System generates CDN URLs assuming external CDN configuration (Cloudflare R2 public bucket). No integrated CDN provisioning, cache invalidation, or origin server included.
- **Content transformation**: No built-in markdown-to-HTML rendering, PDF generation, or format conversion. Docusaurus handles static site generation; other formats are out of scope.
- **Analytics dashboard**: Observability data sent to Cloudflare Analytics Engine and Sentry, but no built-in dashboard UI. Users must use external tools (Grafana, Sentry UI) for visualization.

## Assumptions _(environmental and contextual)_

- **OpenDAL availability**: Assumes OpenDAL Rust core (v0.45.0+) can be compiled to WASM for Cloudflare Workers OR Node.js bindings are available, supporting R2, S3, and Supabase Storage backends with stable APIs
- **MCP SDK maturity**: Assumes @modelcontextprotocol/sdk v1.20.2+ provides stable server implementation with OAuth support
- **Cloudflare Workers environment**: Production deployment assumes Cloudflare Workers environment for OAuth Provider, Analytics Engine, and R2 storage integration
- **OAuth token management**: Assumes clients (Docusaurus hydration script, AI agents) handle token storage and refresh logic correctly using refresh tokens
- **Storage backend configuration**: Assumes storage credentials (API keys, bucket names, endpoints) provided via environment variables or configuration file, not hard-coded
- **Network reliability**: Assumes reasonably stable network for storage operations (transient failures handled via retry, but persistent outages will fail operations)
- **Docusaurus version compatibility**: Assumes Docusaurus v3.9.2+ with TypeScript plugin support for build-time hydration script execution
- **Audit log retention**: Assumes storage backend has sufficient space for daily JSONL logs; no automatic archival/deletion policy implemented (manual cleanup expected)
- **Single-region deployment**: Initial architecture assumes single-region storage (Cloudflare R2 auto-replicates, but no multi-region active-active setup)
- **UTF-8 content encoding**: Assumes all lesson markdown files use UTF-8 encoding; other encodings may cause parsing errors
- **Asset immutability**: Assumes assets are write-once (no in-place updates); updating asset requires deleting old and uploading new with different filename

## Risks & Mitigations

**Risk 1: Storage backend vendor lock-in**

- **Likelihood**: Medium (OpenDAL abstracts storage, but R2-specific features may creep in)
- **Impact**: High (difficult to migrate books if R2 becomes cost-prohibitive or unavailable)
- **Mitigation**: Strictly use OpenDAL's lowest-common-denominator API. Avoid R2-specific features (Workers integration, Cache Reserve). Test with Local and S3 backends regularly to ensure portability. Document storage backend requirements in `book.yaml` schema.

**Risk 2: Summary file desynchronization**

- **Likelihood**: Medium (external agents may fail to update summaries when lessons change)
- **Impact**: Low (summaries become outdated, but doesn't affect storage operations)
- **Mitigation**: Document clear conventions for summary file naming (`.summary.md` suffix). Provide audit log queries to detect lessons modified without corresponding summary updates. External agents responsible for maintaining summary consistency.

**Risk 3: Audit log size growth**

- **Likelihood**: High (daily JSONL logs accumulate indefinitely without cleanup)
- **Impact**: Low (storage cost increases, but functionality not affected)
- **Mitigation**: Document manual archival process in operations guide. Consider future enhancement to compress logs older than 30 days. Monitor storage usage via health check endpoint.

**Risk 4: OAuth token leakage in CI/CD logs**

- **Likelihood**: Low (if developers follow security best practices)
- **Impact**: High (unauthorized access to PanaversityFS, data corruption risk)
- **Mitigation**: Document token handling in security guide: tokens must be stored in CI/CD secrets (GitHub Actions secrets, not environment variable logs). Implement token rotation policy (90-day expiry). Add Sentry alert for unusual API usage patterns.

**Risk 5: Concurrent modification conflicts**

- **Likelihood**: Low (agents typically work on different lessons)
- **Impact**: Medium (operations fail, require retry with merge)
- **Mitigation**: File hash-based conflict detection prevents data corruption. Provide clear error messages with merge suggestions. Consider future enhancement for optimistic locking via ETag headers if conflicts become frequent.

**Risk 6: Docusaurus build timeout on large books**

- **Likelihood**: Medium (hydration script may exceed 10-minute CI timeout for 1000+ lessons)
- **Impact**: High (builds fail, content not published)
- **Mitigation**: Make timeout configurable via `HYDRATION_TIMEOUT_MS` environment variable (default 15 minutes). Implement progress logging every 60 seconds. Consider future parallel download enhancement using Worker threads for books exceeding 500 lessons.

**Risk 7: Storage backend migration data loss**

- **Likelihood**: Low (if migration script tested thoroughly)
- **Impact**: Critical (losing educational content is unacceptable)
- **Mitigation**: Require migration dry-run mode showing file counts before actual transfer. Implement checksum verification after migration. Keep old backend read-only for 7 days post-migration as backup. Document rollback procedure.

## Dependencies

**External Libraries**

- **OpenDAL** (v0.45.0+): Storage abstraction library with Rust core (compiled to WASM for Cloudflare Workers) OR Node.js bindings, providing unified API for Local, R2, S3, Supabase backends
- **@modelcontextprotocol/sdk** (v1.20.2+): TypeScript SDK for MCP server implementation
- **@cloudflare/workers-oauth-provider** (v0.0.13+): OAuth 2.0 authentication library for Cloudflare Workers
- **zod** (v3.24.2+): Schema validation for MCP tool parameters

**Infrastructure Services**

- **Cloudflare Workers**: Runtime environment for MCP server (OAuth, Analytics Engine, R2 integration)
- **Cloudflare R2**: Primary storage backend (production)
- **Cloudflare D1**: Optional SQLite database for hot audit log storage (alternative to direct JSONL writes)
- **Cloudflare Analytics Engine**: Observability metrics for tool invocations
- **Sentry**: Error tracking and alerting
- **Supabase Storage**: Alternative storage backend (optional)
- **AWS S3**: Backup storage backend (optional)

**Panaversity Internal Systems**

- **Docusaurus build pipeline**: Depends on PanaversityFS MCP server for content hydration during static site generation
- **AI agent infrastructure**: Agents (Claude Code, custom subagents) depend on MCP tools for content operations
- **Book authoring workflows**: Current manual Git-based workflows must migrate to MCP tool-based operations

**Development Tools**

- **TypeScript** (v5.5.4+): MCP server and hydration script implementation
- **Python** (v3.11+): OpenDAL integration, summary generation scripts
- **Wrangler** (v4.10.0+): Cloudflare Workers deployment CLI
- **Vitest** (v3.0.9+): Testing framework for MCP server evaluation

## Technical Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                           CONSUMERS                              │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │
│  │  AI Agents   │  │  Docusaurus  │  │  Admin CLI   │          │
│  │  (Claude)    │  │  Hydration   │  │   Tools      │          │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘          │
│         │                 │                  │                   │
└─────────┼─────────────────┼──────────────────┼───────────────────┘
          │                 │                  │
          └─────────────────┴──────────────────┘
                            │
                ┌───────────▼───────────┐
                │   MCP SERVER          │
                │  (Cloudflare Workers) │
                │                       │
                │  • OAuth Provider     │
                │  • 13 MCP Tools       │
                │  • Analytics Engine   │
                │  • Sentry Errors      │
                └───────────┬───────────┘
                            │
          ┌─────────────────┴─────────────────┐
          │                                   │
┌─────────▼──────────┐            ┌──────────▼─────────┐
│  PANAVERSITY FS    │            │  INTELLIGENCE      │
│       CORE         │            │      LAYER         │
│                    │            │                    │
│  • Content Ops     │            │  • Summary Gen     │
│  • Asset Mgmt      │            │    (LLM API)       │
│  • Audit Trail     │            │  • Vector Search   │
│  • Multi-Book      │            │    (Future)        │
│    Registry        │            │                    │
│  • Search          │            │                    │
└─────────┬──────────┘            └────────────────────┘
          │
          │ OpenDAL Unified API
          │
┌─────────▼──────────────────────────────────────────────┐
│               STORAGE LAYER (OpenDAL)                  │
│  ┌────────┐  ┌────────┐  ┌──────────┐  ┌──────────┐  │
│  │ Local  │  │   R2   │  │ Supabase │  │   S3    │   │
│  │  FS    │  │(Primary)│ │ Storage  │  │(Backup) │   │
│  └────────┘  └────────┘  └──────────┘  └──────────┘  │
└───────────────────────────────────────────────────────┘
```

**Storage Directory Structure** (Docusaurus-aligned per ADR-0018):

```
panaversity-storage/
├── registry.yaml                       # Central book catalog
├── .audit/                             # Audit trail logs
│   ├── 2025-11-24.jsonl               # Daily JSONL log
│   └── 2025-11-23.jsonl               # Previous day (archived)
├── books/
│   └── ai-native-software-development/
│       ├── book.yaml                   # Book metadata
│       │
│       ├── content/                    # Maps to Docusaurus docs/
│       │   ├── 01-Introducing-AI-Driven-Development/     # Part
│       │   │   ├── README.md                             # Part intro
│       │   │   ├── 01-what-is-ai-native/                 # Chapter
│       │   │   │   ├── README.md                         # Chapter intro
│       │   │   │   ├── 01-defining-ai-native.md          # Lesson
│       │   │   │   ├── 01-defining-ai-native.summary.md  # Lesson summary
│       │   │   │   ├── 02-core-principles.md
│       │   │   │   └── 02-core-principles.summary.md
│       │   │   └── 02-paradigm-shift/
│       │   │       └── ...
│       │   └── 02-AI-Tool-Landscape/
│       │       └── ...
│       │
│       └── static/                     # Maps to Docusaurus static/
│           ├── img/                    # Images organized by part
│           │   ├── 01-introducing/
│           │   │   └── architecture-diagram.png
│           │   └── 02-tool-landscape/
│           │       └── screenshot.png
│           ├── slides/                 # PDF slide decks
│           │   └── 01-intro-slides.pdf
│           └── videos/                 # MP4 files
│               └── demo-walkthrough.mp4
```

**MCP Tools Summary** (9 tools per ADR-0018):

| Tool Name          | Purpose                           | Key Parameters                                     |
| ------------------ | --------------------------------- | -------------------------------------------------- |
| `read_content`     | Fetch content (lesson or summary) | `book_id`, `path`                                  |
| `write_content`    | Create/update content (upsert)    | `book_id`, `path`, `content`, `file_hash?`         |
| `delete_content`   | Remove content file               | `book_id`, `path`                                  |
| `upload_asset`     | Upload binary asset               | `book_id`, `asset_type`, `filename`, `binary_data` |
| `get_asset`        | Retrieve asset metadata           | `book_id`, `asset_type`, `filename`                |
| `list_assets`      | List all assets                   | `book_id`, `asset_type?`                           |
| `get_book_archive` | Download entire book as ZIP       | `book_id`                                          |
| `list_books`       | Get all books in registry         | None                                               |
| `glob_search`      | Find files by pattern             | `book_id`, `pattern`, `all_books?`                 |
| `grep_search`      | Search content by regex           | `book_id`, `pattern`, `all_books?`                 |

**Note**: Summary files use content tools with `.summary.md` suffix convention. `get_audit_log` is internal/future.

## Implementation Decisions _(resolved)_

The following architectural decisions have been made to unblock development:

1. **OAuth scope granularity**: **DECISION: Full Registry Access**

   - Rationale: For internal MVP, per-book authorization adds complexity without security benefit. Trust API token, audit agent operations. Fine-grained scopes can be added post-MVP if multi-tenant requirements emerge.

2. **Audit log retention policy**: **DECISION: Indefinite Retention**

   - Rationale: Text logs are cheap on R2 (<$0.01/GB/month). Keep all audit data forever for training future agents and compliance. No auto-deletion policy needed.

3. **Asset versioning**: **DECISION: Immutable (Overwrite Only)**

   - Rationale: Educational assets follow "latest is best" pattern. If version tracking needed, use filename conventions (`slides-v2.pdf`). No complex object versioning required for MVP.

4. **Content validation hooks**: **DECISION: Post-Processing Only**

   - Rationale: Don't block write operations. Let agents save drafts freely. Run separate "Linter Agent" or CI checks to flag issues asynchronously. Pre-commit hooks kill velocity.

5. **Multi-region replication**: **DECISION: Defer (R2 is Global by Default)**

   - Rationale: Cloudflare R2 is automatically globally distributed with no regional constraints. No explicit multi-region setup needed. Evaluate if latency issues arise post-launch.

6. **Migration tooling scope**: **DECISION: CLI Tool Required for MVP**

   - Rationale: Cannot manually migrate 50+ lessons via MCP. Build simple CLI (`panaversity-fs migrate ./repo --to r2`) to seed initial content from Git repository.

7. **Docusaurus cache invalidation**: **DECISION: Skip (Use Bulk Download)**
   - Rationale: With `get_book_archive` optimization, downloading 200MB ZIP is faster than checksumming 700 individual files. Always fetch fresh content for simplicity.
