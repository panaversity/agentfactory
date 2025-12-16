# Codebase Map

> Navigate the PanaversityFS source code

**Spec Reference**: [Feature 039: PanaversityFS Production Hardening](../../../../specs/039-panaversity-fs-hardening/spec.md)

This codebase implements the functional requirements (FR-001 to FR-031) and invariants (R1-R7) defined in the authoritative specification.

## Directory Structure

```
panaversity-fs/
├── src/panaversity_fs/           # Main Python package
│   ├── __init__.py               # Package init, version
│   ├── server.py                 # MCP server entry point
│   ├── app.py                    # FastMCP application setup
│   ├── config.py                 # Environment configuration
│   ├── models.py                 # Pydantic input/output models
│   ├── storage.py                # OpenDAL storage abstraction
│   ├── storage_utils.py          # Storage helper functions
│   ├── path_utils.py             # Path validation (FR-007-009)
│   ├── errors.py                 # Custom exception types
│   ├── audit.py                  # Audit logging with hash chain
│   ├── metrics.py                # Prometheus instrumentation
│   ├── auth.py                   # JWT authentication
│   ├── database/                 # Database layer
│   │   ├── __init__.py
│   │   ├── connection.py         # Async session factory
│   │   ├── models.py             # SQLAlchemy ORM models
│   │   └── migrations/           # Alembic migrations
│   │       ├── env.py
│   │       └── versions/
│   └── tools/                    # MCP tool implementations
│       ├── __init__.py           # Tool registration
│       ├── content.py            # read/write/delete_content
│       ├── assets.py             # upload/get/list_assets
│       ├── search.py             # glob/grep_search
│       ├── registry.py           # list_books
│       ├── bulk.py               # get_book_archive
│       ├── validate.py           # validate_book
│       └── delta.py              # delta_build
├── tests/                        # Test suites (301 tests)
│   ├── conftest.py               # Shared fixtures
│   ├── unit/                     # Component tests
│   ├── integration/              # Workflow tests
│   ├── property/                 # Hypothesis property tests
│   ├── performance/              # Latency/throughput tests
│   ├── e2e/                      # End-to-end tests
│   └── edge_cases/               # Production scenarios
├── docs/                         # Documentation
│   ├── guide/                    # Developer guides (you are here)
│   ├── ARCHITECTURE.md           # Legacy architecture doc
│   ├── MCP-TOOLS.md             # Tool API reference
│   ├── SETUP.md                  # Backend setup
│   └── DEVELOPMENT.md            # Development workflow
├── scripts/                      # Utility scripts
├── alembic.ini                   # Alembic configuration
├── pyproject.toml                # Project configuration
└── uv.lock                       # Dependency lock file
```

## Key Files Explained

### Entry Points

| File | Purpose | When to Modify |
|------|---------|----------------|
| `server.py` | MCP server startup | Change server configuration |
| `app.py` | FastMCP app initialization | Add middleware, change transport |
| `config.py` | Environment variables | Add new configuration options |

### Core Modules

| File | Purpose | Key Classes/Functions |
|------|---------|----------------------|
| `models.py` | Pydantic models | `ReadContentInput`, `WriteContentInput`, `DeltaBuildInput` |
| `storage.py` | Storage abstraction | `get_operator()` |
| `path_utils.py` | Path validation | `validate_content_path()`, `validate_asset_path()` |
| `errors.py` | Exception types | `ConflictError`, `HashRequiredError`, `SchemaViolationError` |
| `audit.py` | Audit logging | `log_operation()`, `query_audit_log()` |
| `metrics.py` | Prometheus metrics | `@instrument_write`, `@instrument_archive` |
| `auth.py` | JWT authentication | `verify_token()`, `create_test_token()` |

### Database Layer

| File | Purpose | Key Elements |
|------|---------|--------------|
| `database/connection.py` | Async engine | `get_session()`, `init_db()` |
| `database/models.py` | ORM models | `FileJournal`, `AuditLog` |
| `database/migrations/env.py` | Alembic config | Async migration runner |

### Tools (Mapped to Spec Requirements)

| File | MCP Tools | Key Spec Requirements |
|------|-----------|----------------------|
| `tools/content.py` | `read_content`, `write_content`, `delete_content` | FR-001-006 (journal), FR-015-019 (overlays), R2, R3 |
| `tools/assets.py` | `upload_asset`, `get_asset`, `list_assets` | FR-008 (asset schema) |
| `tools/search.py` | `glob_search`, `grep_search` | - |
| `tools/registry.py` | `list_books` | - |
| `tools/bulk.py` | `get_book_archive` | FR-011-014 (streaming), R4 (throughput) |
| `tools/validate.py` | `validate_book` | FR-007-010 (schema), R1 (paths) |
| `tools/delta.py` | `delta_build` | FR-025-027 (manifest hash) |

## Code Patterns

### Adding a New Tool

1. Create Pydantic models in `models.py`:
```python
class MyToolInput(BaseModel):
    book_id: str = Field(..., pattern=r'^[a-z0-9-]+$')
    # ... other fields
```

2. Implement tool function in `tools/my_tool.py`:
```python
from panaversity_fs.models import MyToolInput
from panaversity_fs.storage import get_operator

async def my_tool(params: MyToolInput) -> str:
    op = get_operator()
    # ... implementation
    return json.dumps({"status": "success"})
```

3. Register in `tools/__init__.py`:
```python
from .my_tool import my_tool
```

4. Add to server in `app.py`:
```python
@mcp.tool(annotations={"readOnlyHint": True})
async def my_tool(params: MyToolInput) -> str:
    """Tool description."""
    return await tools.my_tool(params)
```

### Using the Storage Layer

```python
from panaversity_fs.storage import get_operator

async def example():
    op = get_operator()

    # Write
    await op.write("books/my-book/content/lesson.md", content.encode())

    # Read
    data = await op.read("books/my-book/content/lesson.md")

    # Check existence
    try:
        await op.stat("books/my-book/content/lesson.md")
        exists = True
    except:
        exists = False

    # List files
    async for entry in await op.list("books/my-book/content/"):
        print(entry.path)

    # Delete
    await op.delete("books/my-book/content/lesson.md")
```

### Using the Database

```python
from panaversity_fs.database import get_session, FileJournal

async def example():
    async with get_session() as session:
        # Query
        result = await session.execute(
            select(FileJournal).where(FileJournal.book_id == "my-book")
        )
        entries = result.scalars().all()

        # Insert
        journal = FileJournal(
            book_id="my-book",
            path="content/lesson.md",
            sha256_hash="abc123..."
        )
        session.add(journal)
        await session.commit()
```

### Path Validation

```python
from panaversity_fs.path_utils import (
    validate_content_path,
    validate_asset_path,
    convert_base_to_overlay,
)

# Validate content path
result = validate_content_path("content/01-Part/01-Chapter/01-lesson.md")
if not result.is_valid:
    raise SchemaViolationError(result.errors)

# Convert to overlay path
overlay_path = convert_base_to_overlay(
    "content/01-Part/01-Chapter/01-lesson.md",
    user_id="user123"
)
# Result: "users/user123/content/01-Part/01-Chapter/01-lesson.md"
```

### Error Handling

```python
from panaversity_fs.errors import (
    ConflictError,
    HashRequiredError,
    ContentNotFoundError,
    SchemaViolationError,
)

try:
    await write_content(params)
except ConflictError as e:
    # Hash mismatch - content was modified
    return {"error": "conflict", "current_hash": e.current_hash}
except HashRequiredError as e:
    # Tried to update without expected_hash
    return {"error": "hash_required", "message": str(e)}
except SchemaViolationError as e:
    # Invalid path format
    return {"error": "invalid_path", "violations": e.errors}
```

## Test Organization (Mapped to Spec Invariants)

```
tests/
├── conftest.py                   # Fixtures: setup_fs_backend, etc.
├── unit/                         # ~170 tests
│   ├── test_content_tools.py     # FR-001-006 content operations
│   ├── test_path_utils.py        # FR-007-009 path validation
│   ├── test_journal.py           # FR-001 FileJournal CRUD
│   ├── test_audit_chain.py       # FR-020-024, R6 audit hash chain
│   ├── test_metrics.py           # SC-* instrumentation
│   ├── test_auth.py              # JWT authentication
│   ├── test_overlay_content.py   # FR-015-019 user overlays
│   ├── test_validate_book.py     # FR-010, R1 schema validation
│   └── test_delta_build.py       # FR-025-027 delta detection
├── integration/                  # 24 tests
│   ├── test_conflict_detection.py    # FR-003, SC-003
│   ├── test_journal_storage_atomic.py # R2, SC-002
│   └── test_streaming_archive.py     # FR-011-014
├── property/                     # 33 tests (Alloy invariants via Hypothesis)
│   ├── test_invariant_r1_schema.py    # R1: Path schema enforcement
│   ├── test_invariant_r2_journal.py   # R2: Journal-storage consistency
│   ├── test_invariant_r5_overlay.py   # R5: Overlay isolation
│   ├── test_invariant_r6_audit.py     # R6: Audit hash chain
│   └── test_invariant_r7_agent.py     # R7: Agent provenance
├── performance/                  # 9 tests (Success Criteria)
│   ├── test_archive_throughput.py     # SC-001, R4: Archive <60s
│   └── test_overlay_latency.py        # SC-006: Overlay <10ms
├── e2e/                          # 3 tests
│   └── test_complete_book_workflow.py
└── edge_cases/                   # 13 tests
    └── test_production_structure.py   # User Story edge cases from spec
```

## Configuration

### Environment Variables

```bash
# Storage backend
PANAVERSITY_STORAGE_BACKEND=fs|s3|supabase
PANAVERSITY_STORAGE_ROOT=/path/to/data  # for fs

# S3/R2
PANAVERSITY_S3_BUCKET=bucket-name
PANAVERSITY_S3_ENDPOINT=https://...
PANAVERSITY_S3_ACCESS_KEY_ID=key
PANAVERSITY_S3_SECRET_ACCESS_KEY=secret

# Database (optional, defaults to SQLite)
DATABASE_URL=postgresql+asyncpg://...

# Authentication (optional)
PANAVERSITY_JWT_SECRET=your-secret-key

# Server
PANAVERSITY_SERVER_HOST=0.0.0.0
PANAVERSITY_SERVER_PORT=8000
```

### Database Migrations

```bash
# Generate new migration
uv run alembic revision --autogenerate -m "description"

# Apply migrations
uv run alembic upgrade head

# Rollback one step
uv run alembic downgrade -1

# View current state
uv run alembic current
```
