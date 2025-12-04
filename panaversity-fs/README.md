# PanaversityFS

Agent-Native Multi-Book Storage System - MCP server for educational content management.

**[Specification](../specs/030-panaversity-fs/spec.md)** | **[Architecture](docs/ARCHITECTURE.md)** | **[MCP Tools](docs/MCP-TOOLS.md)** | **[Setup](docs/SETUP.md)** | **[ADR-0018](../history/adr/0018-panaversityfs-docusaurus-aligned-structure.md)**

## Features

- **9 MCP Tools**: Content, assets, search, bulk operations (ADR-0018)
- **Bulk Content Reads**: `read_content` supports `scope` parameter (file/chapter/part)
- **Binary Asset Download**: `get_asset` with `include_binary=true` for direct data
- **3 Storage Backends**: Local filesystem, Cloudflare R2, Supabase
- **60 Tests**: Unit, integration, e2e, edge cases (100% passing)
- **Docusaurus-Aligned**: Storage structure mirrors Docusaurus docs/ convention

## Quick Start

```bash
# Install
cd panaversity-fs && uv sync

# Configure
export PANAVERSITY_STORAGE_BACKEND=fs
export PANAVERSITY_STORAGE_ROOT=/tmp/panaversity-test

# Test
uv run pytest tests/ -q
# Expected: 60 passed

# Run server
uv run python -m panaversity_fs.server
```

## MCP Tools (9 Total - ADR-0018)

| Category | Tools | Description |
|----------|-------|-------------|
| Content | `read_content`, `write_content`, `delete_content` | Lesson/summary CRUD with conflict detection. Supports `scope` for bulk reads |
| Assets | `upload_asset`, `get_asset`, `list_assets` | Binary assets with CDN URLs. `get_asset` supports `include_binary` |
| Search | `glob_search`, `grep_search` | File pattern and content search |
| Registry | `list_books` | Dynamic book discovery (no registry.yaml required) |
| Bulk | `get_book_archive` | ZIP archive generation |

**Note**: Summary operations use content tools with `.summary.md` naming convention (ADR-0018).

See **[MCP Tools Reference](docs/MCP-TOOLS.md)** for complete API documentation.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      AI Agents                              │
│  (Claude Code, Docusaurus Plugin, Content Generators)       │
└─────────────────────────┬───────────────────────────────────┘
                          │ MCP Protocol
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                 PanaversityFS MCP Server                    │
│  FastMCP + Pydantic v2 + OpenDAL                           │
└─────────────────────────┬───────────────────────────────────┘
                          │
         ┌────────────────┼────────────────┐
         ▼                ▼                ▼
    Filesystem      Cloudflare R2     Supabase
```

See **[Architecture Guide](docs/ARCHITECTURE.md)** for design details.

## Project Structure

```
panaversity-fs/
├── src/panaversity_fs/
│   ├── server.py       # MCP server entry point
│   ├── config.py       # Environment configuration
│   ├── models.py       # Pydantic input/output models
│   ├── storage.py      # OpenDAL storage abstraction
│   ├── audit.py        # Operation logging
│   ├── errors.py       # Custom error types
│   └── tools/          # 9 MCP tool implementations (ADR-0018)
│       ├── content.py  # read/write/delete_content (handles summaries too)
│       ├── assets.py   # upload/get/list_assets
│       ├── search.py   # glob/grep_search
│       ├── registry.py # list_books
│       └── bulk.py     # get_book_archive
├── tests/
│   ├── unit/           # Component tests
│   ├── integration/    # Workflow tests
│   ├── e2e/            # End-to-end tests
│   └── edge_cases/     # Production-like scenario tests
└── docs/
    ├── ARCHITECTURE.md # System design
    ├── MCP-TOOLS.md    # Tool API reference
    └── SETUP.md        # Backend configuration
```

## Storage Backends

### Local Filesystem (Default)
```bash
export PANAVERSITY_STORAGE_BACKEND=fs
export PANAVERSITY_STORAGE_ROOT=/tmp/panaversity-data
```

### Cloudflare R2
```bash
export PANAVERSITY_STORAGE_BACKEND=s3
export PANAVERSITY_S3_BUCKET=your-bucket
export PANAVERSITY_S3_ENDPOINT=https://xxx.r2.cloudflarestorage.com
export PANAVERSITY_S3_ACCESS_KEY_ID=your-key
export PANAVERSITY_S3_SECRET_ACCESS_KEY=your-secret
export PANAVERSITY_S3_REGION=auto
```

### Supabase
```bash
export PANAVERSITY_STORAGE_BACKEND=supabase
export PANAVERSITY_SUPABASE_URL=https://xxx.supabase.co
export PANAVERSITY_SUPABASE_SERVICE_ROLE_KEY=your-service-key
export PANAVERSITY_SUPABASE_BUCKET=panaversity-books
```

See **[Setup Guide](docs/SETUP.md)** for detailed instructions.

## Running Tests

```bash
# All tests
uv run pytest tests/ -v

# By category
uv run pytest tests/unit/ -v
uv run pytest tests/integration/ -v
uv run pytest tests/e2e/ -v
uv run pytest tests/edge_cases/ -v
```

## Technology Stack

| Component | Technology | Purpose |
|-----------|------------|---------|
| MCP Framework | FastMCP | MCP server implementation |
| Storage | OpenDAL | Unified storage abstraction |
| Validation | Pydantic v2 | Input/output validation |
| Config | pydantic-settings | Environment configuration |
| Testing | pytest-asyncio | Async test support |

## Container Deployment

```dockerfile
FROM python:3.13-slim
RUN apt-get update && apt-get install -y libmagic1 && rm -rf /var/lib/apt/lists/*
WORKDIR /app
COPY . .
RUN pip install uv && uv sync --frozen
CMD ["uv", "run", "python", "-m", "panaversity_fs.server"]
```

**System dependency**: `libmagic` required for MIME type detection.

## License

MIT
