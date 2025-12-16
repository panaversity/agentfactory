# PanaversityFS

Agent-Native Multi-Book Storage System - MCP server for educational content management.

**[Developer Guide](docs/guide/)** | **[Tools Reference](docs/guide/03-tools-reference.md)** | **[Architecture](docs/guide/02-architecture.md)** | **[Setup](docs/SETUP.md)**

## Features

- **12 MCP Tools**: Content, assets, search, bulk, validation, delta builds
- **301 Tests**: Unit, integration, property, performance, e2e
- **User Overlays**: Per-user content personalization (FR-016/017/018)
- **Conflict Detection**: Hash-based optimistic concurrency (FR-002/003/004)
- **Audit Trail**: Append-only hash chain with agent provenance
- **3 Storage Backends**: Filesystem, Cloudflare R2, Supabase
- **Schema Validation**: Path enforcement (FR-007/008/009)

## Quick Start

```bash
# Install
cd panaversity-fs && uv sync

# Configure local backend
export PANAVERSITY_STORAGE_BACKEND=fs
export PANAVERSITY_STORAGE_ROOT=/tmp/panaversity-test

# Run tests
uv run pytest tests/ -q
# Expected: 301 passed

# Start server
uv run python -m panaversity_fs.server
# Server at http://localhost:8000/mcp
```

## MCP Tools (12 Total)

| Category | Tools | Description |
|----------|-------|-------------|
| **Content** | `read_content`, `write_content`, `delete_content` | Lesson/summary CRUD with overlay support |
| **Assets** | `upload_asset`, `get_asset`, `list_assets` | Binary assets with CDN URLs |
| **Search** | `glob_search`, `grep_search` | File pattern and content search |
| **Registry** | `list_books` | Dynamic book discovery |
| **Bulk** | `get_book_archive` | ZIP archive generation (<60s) |
| **Validation** | `validate_book`, `delta_build` | Schema check, incremental builds |

See **[Tools Reference](docs/guide/03-tools-reference.md)** for complete API documentation.

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
│  12 Tools + Audit + Metrics + Path Validation               │
└─────────────────────────┬───────────────────────────────────┘
                          │
         ┌────────────────┼────────────────┐
         ▼                ▼                ▼
    Filesystem      Cloudflare R2     Supabase
```

See **[Architecture Guide](docs/guide/02-architecture.md)** for design details.

## Project Structure

```
panaversity-fs/
├── src/panaversity_fs/
│   ├── server.py          # MCP server entry point
│   ├── config.py          # Environment configuration
│   ├── models.py          # Pydantic input/output models
│   ├── storage.py         # OpenDAL storage abstraction
│   ├── path_utils.py      # Path validation (FR-007/008/009)
│   ├── audit.py           # Hash chain audit logging
│   ├── metrics.py         # Prometheus instrumentation
│   ├── database/          # SQLAlchemy + Alembic
│   └── tools/             # 12 MCP tool implementations
├── tests/                 # 301 tests
│   ├── unit/              # Component tests
│   ├── integration/       # Workflow tests
│   ├── property/          # Hypothesis invariant tests
│   ├── performance/       # Latency/throughput tests
│   └── e2e/               # End-to-end tests
└── docs/
    └── guide/             # Developer documentation
```

## Storage Backends

### Local Filesystem
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
```

### Supabase
```bash
export PANAVERSITY_STORAGE_BACKEND=supabase
export PANAVERSITY_SUPABASE_URL=https://xxx.supabase.co
export PANAVERSITY_SUPABASE_SERVICE_ROLE_KEY=your-key
export PANAVERSITY_SUPABASE_BUCKET=panaversity-books
```

See **[Setup Guide](docs/SETUP.md)** for detailed instructions.

## Build Pipeline Scripts

PanaversityFS includes CLI scripts for incremental builds:

### Hydrate (Download from PanaversityFS)

```bash
# Download content for Docusaurus build (incremental)
python scripts/hydrate-book.py --book-id ai-native-python

# Force full download
python scripts/hydrate-book.py --book-id ai-native-python --full-rebuild

# Or use Make
make hydrate BOOK_ID=ai-native-python
```

### Ingest (Upload to PanaversityFS)

```bash
# Upload source content (incremental)
python scripts/ingest-book.py --book-id ai-native-python --source-dir ./book-source/docs

# Preview what would be synced
python scripts/ingest-book.py --book-id ai-native-python --source-dir ./book-source/docs --dry-run

# Or use Make
make ingest BOOK_ID=ai-native-python SOURCE_DIR=./book-source/docs
```

### Full Build Pipeline

```bash
# Ingest + hydrate in one command
make build BOOK_ID=ai-native-python SOURCE_DIR=./book-source/docs
```

## Running Tests

```bash
# All tests
uv run pytest tests/ -v

# By category
uv run pytest tests/unit/ -v           # ~170 component tests
uv run pytest tests/integration/ -v    # 24 workflow tests
uv run pytest tests/property/ -v       # 33 invariant tests
uv run pytest tests/performance/ -v    # 9 benchmark tests
uv run pytest tests/scripts/ -v        # 54 build script tests
```

## Documentation

| Guide | Purpose |
|-------|---------|
| [Quickstart](docs/guide/01-quickstart.md) | Get running in 5 minutes |
| [Architecture](docs/guide/02-architecture.md) | System design & components |
| [Tools Reference](docs/guide/03-tools-reference.md) | All 12 MCP tools |
| [Codebase Map](docs/guide/04-codebase-map.md) | Source code navigation |
| [Testing](docs/guide/05-testing.md) | Test suites & best practices |
| [Extending](docs/guide/06-extending.md) | Adding new features |
| [Operations](docs/guide/07-operations.md) | Deployment & monitoring |

## Technology Stack

| Component | Technology |
|-----------|------------|
| MCP Framework | FastMCP |
| Storage | OpenDAL |
| Database | SQLAlchemy + Alembic |
| Validation | Pydantic v2 |
| Metrics | prometheus-client |
| Testing | pytest + hypothesis |

## License

MIT
