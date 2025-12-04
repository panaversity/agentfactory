# Implementation Plan: PanaversityFS

**Branch**: `030-panaversity-fs` | **Date**: 2025-11-24 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/030-panaversity-fs/spec.md`

## Summary

PanaversityFS is an agent-native multi-book storage system providing unified CRUD operations for educational content across multiple storage backends (Local, Cloudflare R2, AWS S3, Supabase Storage). Core features include:

- **Storage Abstraction**: OpenDAL Python bindings provide unified API for all backends
- **MCP Server**: 15 tools exposing content operations, asset management, summary storage, and audit trail queries
- **Audit Trail**: Direct JSONL writes with eventual consistency (per FR-018 specification)
- **Asset Management**: Hybrid upload pattern (direct <10MB, presigned URLs ≥10MB)
- **Bulk Operations**: Streaming ZIP archive generation with presigned download URLs
- **Docusaurus Integration**: Hydration script downloads complete book during build

**Technical Approach** (from research.md + user direction):
- **Runtime**: Python 3.11+ with Python MCP SDK (simpler than Node.js/Cloudflare Workers)
- **MCP Transport**: Stateless Streamable HTTP protocol (`STATELESS_HTTP=true`)
- **Audit Strategy**: Direct JSONL writes to storage backend (simpler than D1, meets latency requirements)
- **Asset Pattern**: Hybrid based on size threshold (balances DX with memory safety)
- **Archive Generation**: Streaming ZIP with presigned URLs (memory-efficient, universal compatibility)
- **Documentation**: Context7 MCP server for live docs (opendal, mcp SDK)
- **Development Approach**: mcp-builder skill for MCP server patterns

## Technical Context

**Language/Version**: Python 3.11+ / asyncio
**Primary Dependencies**:
- `opendal` (Storage abstraction via OpenDAL Python bindings)
- `mcp` (Python MCP SDK for server/tool implementation)
- `pydantic` (Schema validation and data models)
- `pyyaml` (YAML parsing for registry/book configs)
- `python-frontmatter` (Lesson markdown frontmatter parsing)
- `python-magic` (MIME type detection for assets)

**Storage**:
- **Primary**: Cloudflare R2 (production, S3-compatible API)
- **Alternatives**: Local filesystem (dev), AWS S3, Supabase Storage
- **Abstraction**: OpenDAL Python bindings provide unified Operator interface
- **Audit Logs**: Direct JSONL writes to storage backend (`.audit/YYYY-MM-DD.jsonl`)

**Testing**:
- **Framework**: pytest with pytest-asyncio for async tests
- **Contract Tests**: Validate MCP tool schemas against Pydantic models
- **Integration Tests**: Test against Local + R2 backends
- **E2E Tests**: Full workflow tests (create book → add lessons → hydrate → build Docusaurus)

**Development Tools**:
- **Context7 MCP**: Live documentation for opendal, mcp SDK (via mcp__context7__get-library-docs)
- **mcp-builder skill**: MCP server architecture guidance and patterns
- **Package Manager**: uv (fast Python package management)
- **Linting**: ruff (linter + formatter)
- **Type Checking**: mypy (strict mode)

**Target Platform**:
- **Runtime**: Python 3.11+ with asyncio (NOT Cloudflare Workers)
- **MCP Protocol**: Stateless Streamable HTTP transport (`STATELESS_HTTP=true` in server config)
- **Deployment**: Docker container OR systemd service (HTTP endpoint exposed)
- **Local Dev**: Direct Python execution with local filesystem backend

**Project Type**: Single backend service (Python MCP server over Stateless Streamable HTTP)

**Performance Goals**:
- **Content CRUD**: <500ms p95 for files up to 50KB (FR-SC-001)
- **Asset Uploads**: <3s for 10MB files over 5 Mbps connection (FR-SC-002)
- **Audit Writes**: <50ms append operations (FR-004, research.md Decision 2)
- **Archive Generation**: <60s for 500 files / 200MB books (FR-030)
- **Registry Operations**: <200ms for list_books on 50 books (FR-SC-008)

**Constraints**:
- **Memory**: Streaming for large archives (200MB books), presigned URLs for assets ≥10MB
- **Concurrency**: Support 100 concurrent agent operations without conflicts (FR-SC-015)
- **Conflict Detection**: File hash-based (SHA256 via hashlib) for update operations (FR-008)
- **Binary Size**: Assets capped at 100MB (FR-010)
- **OAuth**: Required in production (MCP SDK auth), disabled for local dev

**Scale/Scope**:
- **Books**: 50 books in registry (FR-SC-008)
- **Lessons per Book**: 500 lessons (FR-SC-009)
- **MCP Tools**: 15 tools (content ops, assets, summaries, search, audit)
- **Audit Retention**: Indefinite (text logs ~$0.01/GB/month on R2)
- **Concurrent Agents**: 100 simultaneous operations (FR-SC-015)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Note**: This is a production infrastructure feature, NOT educational content. Constitution principles for pedagogical design (4-Layer Teaching Method, Progressive Complexity, Anti-Convergence, etc.) are NOT applicable.

**Relevant Principles**:
- **Factual Accuracy**: All technical claims verified via WebSearch (OpenDAL docs, Cloudflare Workers docs)
- **Minimal Sufficient Content**: Spec defines non-goals explicitly (no vector search, no real-time collaboration, no multi-tenancy in MVP)

**Status**: PASS (infrastructure features exempt from pedagogical constitution gates)

## Project Structure

### Documentation (this feature)

```text
specs/030-panaversity-fs/
├── spec.md              # Feature specification (user scenarios, requirements)
├── plan.md              # This file (technical context, structure, decisions)
├── research.md          # Phase 0: Technical research decisions (runtime, audit, assets, archives)
├── data-model.md        # Phase 1: Entity schemas (Registry, Book, Lesson, Asset, Summary, AuditEntry)
├── quickstart.md        # Phase 1: Developer onboarding guide
├── contracts/           # Phase 1: MCP tool JSON schemas (15 files)
│   ├── add_content.json
│   ├── update_content.json
│   ├── read_content.json
│   ├── delete_content.json
│   ├── add_asset.json
│   ├── get_asset.json
│   ├── list_assets.json
│   ├── add_summary.json
│   ├── update_summary.json
│   ├── get_summary.json
│   ├── list_summaries.json
│   ├── get_book_archive.json
│   ├── list_books.json
│   ├── glob_search.json
│   ├── grep_search.json
│   └── get_audit_log.json
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT YET CREATED)
```

### Source Code (repository root)

**Project Type**: Single backend service (Python MCP server)

```text
panaversity-fs/
├── src/
│   └── panaversity_fs/              # Python package
│       ├── __init__.py
│       ├── server.py                # MCP server entry point
│       ├── models.py                # Pydantic models (Registry, Book, Lesson, Asset, Summary, AuditEntry)
│       ├── config.py                # Environment config (pydantic-settings)
│       ├── storage.py               # OpenDAL Operator initialization
│       ├── storage_utils.py         # Storage helpers (hash, path validation)
│       ├── audit.py                 # JSONL audit logger + query
│       ├── errors.py                # Custom exceptions
│       ├── logging.py               # Structured logging setup
│       └── tools/                   # MCP tool implementations
│           ├── __init__.py
│           ├── content.py           # 4 tools: add/update/read/delete_content
│           ├── assets.py            # 3 tools: add/get/list_assets
│           ├── summaries.py         # 4 tools: add/update/get/list_summaries
│           ├── bulk.py              # 1 tool: get_book_archive
│           ├── registry.py          # 1 tool: list_books
│           ├── search.py            # 2 tools: glob/grep_search
│           └── audit.py             # 1 tool: get_audit_log (query interface)
│
├── tests/
│   ├── __init__.py
│   ├── test_content.py              # Content tools tests
│   ├── test_assets.py               # Asset tools tests
│   ├── test_storage.py              # OpenDAL integration tests
│   ├── test_audit.py                # Audit logger tests
│   └── test_e2e.py                  # Full workflow tests
│
├── cli/
│   ├── __init__.py
│   └── migrate.py                   # Git → PanaversityFS migration CLI
│
├── examples/
│   ├── docusaurus_hydration.py     # Docusaurus integration script
│   └── mcp_client.py                # MCP client example
│
├── pyproject.toml                   # uv/pip dependencies
├── .python-version                  # Python 3.11+
├── ruff.toml                        # Linting config
├── mypy.ini                         # Type checking config
├── pytest.ini                       # Test config
├── Dockerfile                       # Container deployment
└── README.md
```

**Structure Decision**:

This is a **single backend service** (Python MCP server). Structure follows:

1. **Consolidated tool files**: Related tools grouped (content.py has 4 @mcp.tool() decorated functions, not 4 files)
2. **OpenDAL abstraction**: storage.py initializes Operator based on STORAGE_BACKEND env var (fs/s3/supabase)
3. **Pydantic models**: models.py defines all entities with validation (no separate schemas/ directory needed)
4. **Test simplicity**: Test files mirror tool files (test_content.py tests all 4 content tools)
5. **CLI separation**: cli/migrate.py for Git seeding (not part of MCP server)

**Key Design Decisions**:
- No frontend (MCP tools consumed by external clients: Claude Code, Docusaurus hydration script, admin CLI)
- Monorepo NOT required (single service with well-defined MCP API boundary)
- **OpenDAL handles storage abstraction** - no custom backend implementations needed
- **MCP SDK handles server lifecycle** - minimal boilerplate in server.py
- **Context7 + mcp-builder for guidance** - consult live docs during development instead of pre-researching

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**No Violations**: This is infrastructure, not educational content. No pedagogical complexity concerns apply.
