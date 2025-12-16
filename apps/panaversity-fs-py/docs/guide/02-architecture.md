# Architecture Guide

> How PanaversityFS is designed and why

**Spec Reference**: [Feature 039: PanaversityFS Production Hardening](../../../../specs/039-panaversity-fs-hardening/spec.md)

This architecture implements the requirements and invariants defined in the authoritative specification.

## System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                        AI Agents                                │
│  (Claude Code, Docusaurus Plugin, Content Generators)          │
└─────────────────────────┬───────────────────────────────────────┘
                          │ MCP Protocol (JSON-RPC 2.0)
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│                   PanaversityFS MCP Server                      │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │                 FastMCP Framework                          │  │
│  │  - Stateless HTTP transport                               │  │
│  │  - Pydantic v2 validation                                 │  │
│  └───────────────────────────────────────────────────────────┘  │
│                              │                                  │
│  ┌───────────────────────────▼───────────────────────────────┐  │
│  │                    12 MCP Tools                           │  │
│  │  Content: read, write, delete                             │  │
│  │  Assets:  upload, get, list                               │  │
│  │  Search:  glob, grep                                      │  │
│  │  Registry: list_books                                     │  │
│  │  Bulk:    get_book_archive                                │  │
│  │  Validation: validate_book, delta_build                   │  │
│  └───────────────────────────┬───────────────────────────────┘  │
│                              │                                  │
│  ┌───────────────────────────▼───────────────────────────────┐  │
│  │              Supporting Infrastructure                     │  │
│  │  - Path Validation (FR-007, FR-008, FR-009)              │  │
│  │  - FileJournal (conflict detection)                      │  │
│  │  - AuditLog (hash chain provenance)                      │  │
│  │  - Prometheus Metrics                                     │  │
│  └───────────────────────────┬───────────────────────────────┘  │
│                              │                                  │
│  ┌───────────────────────────▼───────────────────────────────┐  │
│  │                  OpenDAL Abstraction                       │  │
│  └───────────────┬───────────────┬───────────────┬──────────┘  │
└──────────────────┼───────────────┼───────────────┼──────────────┘
                   │               │               │
         ┌─────────▼───┐   ┌───────▼─────┐   ┌────▼──────┐
         │ Filesystem  │   │ Cloudflare  │   │ Supabase  │
         │   (local)   │   │     R2      │   │  Storage  │
         └─────────────┘   └─────────────┘   └───────────┘
```

## Core Design Principles

### 1. Agent-Native Design

Every decision optimizes for AI agent consumption:

| Principle | Implementation |
|-----------|----------------|
| Structured responses | All tools return JSON for easy parsing |
| Semantic errors | `ConflictError`, `HashRequiredError` for agent decision-making |
| Idempotent operations | Safe to retry without side effects |
| Conflict detection | SHA256 hash-based optimistic concurrency |

### 2. Multi-Book Architecture

Each book is isolated with Docusaurus-aligned structure:

```
storage-root/
├── books/
│   └── {book-id}/
│       ├── content/                    # Markdown content
│       │   └── {NN-Part}/
│       │       ├── README.md           # Part intro
│       │       └── {NN-Chapter}/
│       │           ├── README.md       # Chapter intro
│       │           ├── {NN-lesson}.md  # Lesson
│       │           └── {NN-lesson}.summary.md  # Summary
│       ├── static/                     # Binary assets
│       │   ├── img/
│       │   ├── slides/
│       │   ├── videos/
│       │   └── audio/
│       └── users/                      # Overlay personalization
│           └── {user-id}/
│               └── content/            # User's customized content
└── archives/                           # Generated ZIP archives
```

### 3. Overlay Personalization (FR-016, FR-017, FR-018)

Users can have personalized versions of content without modifying the base:

```
Read Priority:
1. Check users/{user_id}/content/... (overlay)
2. Fall back to content/... (base)

Write with user_id:
→ Writes to users/{user_id}/content/... only

Delete with user_id:
→ Deletes overlay only, base is NEVER affected
```

### 4. Conflict Detection (FR-002, FR-003, FR-004)

Hash-based optimistic concurrency control:

```
CREATE (new file):
  expected_hash: omit → succeeds

UPDATE (existing file):
  expected_hash: required
  Matches current → write succeeds
  Mismatches → ConflictError with current hash
  Omitted → HashRequiredError
```

### 5. Audit Trail (FR-021, FR-022, FR-023, FR-024)

Append-only audit log with hash chain integrity:

```
┌─────────────────────────────────────────────────────────────┐
│ AuditLog Entry                                              │
├─────────────────────────────────────────────────────────────┤
│ id: UUID                                                    │
│ book_id: str                                                │
│ path: str                                                   │
│ operation: WRITE | DELETE | READ                            │
│ agent_id: str (extracted from MCP context)                 │
│ user_id: str | null                                         │
│ timestamp: datetime                                         │
│ entry_hash: SHA256(book_id + path + operation + ...)       │
│ prev_hash: hash of previous entry for this (book, path)    │
└─────────────────────────────────────────────────────────────┘
```

## Key Components

### Storage Layer (`storage.py`)

OpenDAL provides unified async API across backends:

```python
from panaversity_fs.storage import get_operator

op = get_operator()
await op.write("books/my-book/content/lesson.md", content)
await op.read("books/my-book/content/lesson.md")
await op.delete("books/my-book/content/lesson.md")
```

### Database Layer (`database/`)

SQLAlchemy async ORM with Alembic migrations:

- **FileJournal**: Tracks file hashes for conflict detection
- **AuditLog**: Immutable audit trail with hash chain

```python
from panaversity_fs.database import get_session, FileJournal

async with get_session() as session:
    journal = FileJournal(
        book_id="my-book",
        path="content/lesson.md",
        sha256_hash="abc123...",
        agent_id="agent-1"
    )
    session.add(journal)
```

### Path Validation (`path_utils.py`)

Enforces content schema (FR-007, FR-008, FR-009):

```python
from panaversity_fs.path_utils import validate_content_path

result = validate_content_path("content/01-Part/01-Chapter/01-lesson.md")
assert result.is_valid

# Rejects path traversal attacks
result = validate_content_path("content/../../../etc/passwd")
assert not result.is_valid
assert "traversal" in result.errors[0].lower()
```

### Metrics (`metrics.py`)

Prometheus instrumentation for observability:

```python
from panaversity_fs.metrics import write_total, archive_duration_seconds

# Counters track operation counts
write_total.labels(book_id="my-book", status="success").inc()

# Histograms track latencies
with archive_duration_seconds.labels(book_id="my-book").time():
    # archive generation
```

## Alloy Invariants (from Spec Section: Formal Verification)

The spec defines formal invariants using Alloy-style notation. These are verified by property tests:

| ID | Invariant | Enforcement | Property Test |
|----|-----------|-------------|---------------|
| R1 | All paths conform to Docusaurus schema | `path_utils.validate_*` | `test_invariant_r1_schema.py` |
| R2 | Journal hash always matches storage content | Atomic transactions | `test_invariant_r2_journal.py` |
| R3 | Delete operations are idempotent | `existed` flag in response | (unit tests) |
| R4 | Archives complete in <60s with <64MB memory | Streaming + timeout | `test_archive_throughput.py` |
| R5 | User overlays are isolated (User A never sees User B's overlay) | Namespace separation | `test_invariant_r5_overlay.py` |
| R6 | Audit log maintains hash chain: `entry[n].new_hash == entry[n+1].prev_hash` | `prev_hash` linking | `test_invariant_r6_audit.py` |
| R7 | No 'system' or empty agent_id in audit entries | DB constraint + validation | `test_invariant_r7_agent.py` |

**Small-Scope Testing**: Each invariant is verified using Hypothesis with 3-5 instance bounds, following the spec's Alloy small-scope methodology.

## Performance Characteristics (Success Criteria from Spec)

| Success Criteria | Requirement | Implementation |
|------------------|-------------|----------------|
| SC-001 | Archive 500 files/200MB in <60s, <64MB memory | Streaming ZIP with chunked buffer |
| SC-002 | Zero orphaned storage writes after failure | Atomic journal+storage transactions |
| SC-003 | 100% conflict detection accuracy | SHA256 hash comparison |
| SC-006 | Overlay reads add <10ms latency | Parallel existence check |
| SC-007 | Delta builds return only changed files | Manifest hash comparison |
| SC-010 | 99.9% R2 operation success rate | Automatic retry on transient failures |

## Security Model

### Path Security (FR-009)
- Path traversal (`..`) rejected
- Null bytes rejected
- Absolute paths rejected
- URL-encoded attacks detected

### Authentication (Optional)
- JWT-based authentication
- Scope-based authorization (read, write, admin)
- Agent ID extraction from JWT `sub` claim

### Data Integrity
- SHA256 hashing for content verification
- Hash chain for audit trail integrity
- Atomic journal+storage transactions
