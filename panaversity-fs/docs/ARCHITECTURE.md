# PanaversityFS Architecture

> Agent-Native Multi-Book Storage System

**Specification**: [specs/030-panaversity-fs/spec.md](../../specs/030-panaversity-fs/spec.md)
**ADR-0018**: [Docusaurus-Aligned Storage Structure](../../history/adr/0018-panaversityfs-docusaurus-aligned-structure.md)

## Overview

PanaversityFS is an MCP (Model Context Protocol) server that provides unified storage operations for educational content across multiple books. It enables AI agents to read, write, search, and manage lesson content, summaries, and binary assets.

```
┌─────────────────────────────────────────────────────────────────┐
│                        AI Agents                                │
│  (Claude Code, Docusaurus Plugin, Content Generators)           │
└─────────────────────────┬───────────────────────────────────────┘
                          │ MCP Protocol (stdio/HTTP)
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│                   PanaversityFS MCP Server                      │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │                    FastMCP Framework                       │  │
│  │  - Stateless Streamable HTTP transport                    │  │
│  │  - JSON-RPC 2.0 protocol                                  │  │
│  │  - Pydantic v2 input validation                           │  │
│  └───────────────────────────────────────────────────────────┘  │
│                              │                                  │
│  ┌───────────────────────────▼───────────────────────────────┐  │
│  │                     9 MCP Tools (ADR-0018)                │  │
│  │  Content: read, write, delete (+ summaries via naming)    │  │
│  │  Assets:  upload, get, list                               │  │
│  │  Search:  glob, grep                                      │  │
│  │  Registry: list_books                                     │  │
│  │  Bulk:    get_book_archive                                │  │
│  └───────────────────────────┬───────────────────────────────┘  │
│                              │                                  │
│  ┌───────────────────────────▼───────────────────────────────┐  │
│  │                  OpenDAL Abstraction                      │  │
│  │  - Unified async API for all backends                     │  │
│  │  - read(), write(), delete(), stat(), list()              │  │
│  └───────────────┬───────────────┬───────────────┬──────────┘  │
└──────────────────┼───────────────┼───────────────┼──────────────┘
                   │               │               │
         ┌─────────▼───┐   ┌───────▼─────┐   ┌────▼──────┐
         │ Filesystem  │   │ Cloudflare  │   │ Supabase  │
         │   (local)   │   │     R2      │   │  Storage  │
         └─────────────┘   └─────────────┘   └───────────┘
```

## Design Principles

### 1. Agent-Native Design

Every design decision optimizes for AI agent consumption:

- **Structured JSON responses**: All tools return JSON for easy parsing
- **Semantic error types**: `ContentNotFoundError`, `ConflictError` for agent decision-making
- **Idempotent operations**: Safe to retry without side effects
- **Hash-based conflict detection**: Enable optimistic concurrency

### 2. Storage Abstraction

OpenDAL provides unified interface across storage backends:

```python
# Same code works for fs, S3, Supabase
op = get_operator()
await op.write("books/test/lesson.md", content)
await op.read("books/test/lesson.md")
await op.delete("books/test/lesson.md")
```

### 3. Multi-Book Architecture (ADR-0018)

Each book is isolated with Docusaurus-aligned structure:

```
storage-root/
├── archives/                  # Generated ZIP archives
└── books/                     # Book directories (list_books scans this)
    └── {book-id}/
        ├── content/           # Maps to Docusaurus docs/
        │   └── {NN-Part}/
        │       ├── README.md              # Part intro
        │       └── {NN-Chapter}/
        │           ├── README.md          # Chapter intro
        │           ├── {NN-lesson}.md     # Lesson content
        │           └── {NN-lesson}.summary.md  # Lesson summary (sibling)
        └── static/            # Maps to Docusaurus static/
            ├── images/
            ├── slides/
            ├── videos/
            └── audio/
```

**Summary Convention (ADR-0018)**: Summaries use `.summary.md` suffix as sibling files to lessons (e.g., `01-intro.summary.md` for `01-intro.md`).

### 4. Tool Categories (ADR-0018: 9 Tools)

| Category | Tools | Purpose |
|----------|-------|---------|
| **Content** | `read_content`, `write_content`, `delete_content` | Lesson/summary CRUD with conflict detection |
| **Assets** | `upload_asset`, `get_asset`, `list_assets` | Binary asset management with CDN URLs |
| **Search** | `glob_search`, `grep_search` | File pattern and content search |
| **Registry** | `list_books` | Book discovery from registry.yaml |
| **Bulk** | `get_book_archive` | ZIP archive generation |

**Note**: Summary tools were removed in ADR-0018. Summaries are now managed via content tools using the `.summary.md` naming convention.

## Core Components

### FastMCP Server (`server.py`)

Entry point that initializes the MCP server with Stateless Streamable HTTP transport:

```python
mcp = FastMCP(
    "panaversity_fs",
    stateless_http=True,    # Stateless HTTP transport
    json_response=True       # JSON responses (no SSE)
)
```

### Configuration (`config.py`)

Environment-based configuration with Pydantic validation:

```python
class Config(BaseSettings):
    model_config = SettingsConfigDict(env_prefix="PANAVERSITY_")

    storage_backend: Literal["fs", "s3", "supabase"] = "fs"
    storage_root: str = "/tmp/panaversity-fs-data"

    # S3/R2 configuration
    s3_bucket: str | None = None
    s3_endpoint: str | None = None  # For Cloudflare R2
    s3_access_key_id: str | None = None
    s3_secret_access_key: str | None = None

    # Supabase configuration
    supabase_bucket: str | None = None
    supabase_url: str | None = None
    supabase_service_key: str | None = None
```

### Storage Abstraction (`storage.py`)

Singleton OpenDAL operator configured per backend:

```python
def get_operator() -> opendal.AsyncOperator:
    """Returns configured async operator for current backend."""
    config = get_config()

    if config.storage_backend == "fs":
        return opendal.AsyncOperator("fs", root=config.storage_root)
    elif config.storage_backend == "s3":
        return opendal.AsyncOperator("s3",
            bucket=config.s3_bucket,
            endpoint=config.s3_endpoint,  # For R2
            access_key_id=config.s3_access_key_id,
            secret_access_key=config.s3_secret_access_key
        )
    elif config.storage_backend == "supabase":
        return opendal.AsyncOperator("supabase",
            bucket=config.supabase_bucket,
            endpoint=config.supabase_url,
            key=config.supabase_service_key
        )
```

### Pydantic Models (`models.py`)

Type-safe input/output validation for all tools:

```python
class WriteContentInput(BaseModel):
    model_config = ConfigDict(str_strip_whitespace=True, extra='forbid')

    book_id: str = Field(..., pattern=r'^[a-z0-9-]+$', min_length=3, max_length=50)
    path: str = Field(..., description="Relative path within book")
    content: str = Field(..., min_length=1, max_length=1_000_000)
    file_hash: str | None = Field(None, description="SHA256 for conflict detection")
```

### Audit Logging (`audit.py`)

All operations logged with execution metrics:

```python
await log_operation(
    operation=OperationType.WRITE_CONTENT,
    path=full_path,
    agent_id="system",
    status=OperationStatus.SUCCESS,
    execution_time_ms=execution_time
)
```

## Key Patterns

### 1. Conflict Detection (Optimistic Concurrency)

```python
# Agent reads content with hash
content = await read_content(book_id="book", path="lesson.md")
# Returns: {"content": "...", "file_hash_sha256": "abc123..."}

# Agent writes with hash for conflict detection
result = await write_content(
    book_id="book",
    path="lesson.md",
    content="updated content",
    file_hash="abc123..."  # Must match current hash
)
# Raises ConflictError if hash mismatch
```

### 2. Idempotent Operations

All tools designed for safe retry:

- `write_content`: Upsert semantics (create or update) - works for lessons and summaries
- `delete_content`: No error if file doesn't exist
- `upload_asset`: Overwrites existing asset

### 3. Hybrid Asset Upload

```python
# Small files (<10MB): Direct base64 upload
await upload_asset(
    book_id="book",
    asset_type="images",
    filename="diagram.png",
    binary_data="iVBORw0KGgo..."  # Base64
)

# Large files (≥10MB): Presigned URL (planned)
await upload_asset(
    book_id="book",
    asset_type="videos",
    filename="tutorial.mp4",
    file_size=52428800  # Returns presigned upload URL
)
```

## Technology Stack

| Component | Technology | Purpose |
|-----------|------------|---------|
| **MCP Framework** | FastMCP | MCP server implementation |
| **Storage** | OpenDAL | Unified storage abstraction |
| **Validation** | Pydantic v2 | Input/output validation |
| **Config** | pydantic-settings | Environment configuration |
| **Testing** | pytest-asyncio | Async test support |

## Performance Constraints

From specification (FR-030):

| Metric | Requirement |
|--------|-------------|
| Archive generation | <60s for 500 files / 200MB |
| Direct upload | <10MB per asset |
| Presigned URL | ≥10MB assets |
| Search results | Max 1000 matches |

## Security Model

### Current (MVP)

- Optional API key authentication (`PANAVERSITY_API_KEY`)
- Path traversal prevention in all tools
- Filename sanitization for assets

### Planned

- Per-book access control
- Agent-specific permissions
- Audit trail with agent identification

## Related Documentation

- **[MCP Tools Reference](./MCP-TOOLS.md)**: Complete tool API documentation
- **[Setup Guide](./SETUP.md)**: Backend configuration instructions
- **[Specification](../../specs/030-panaversity-fs/spec.md)**: Full requirements
