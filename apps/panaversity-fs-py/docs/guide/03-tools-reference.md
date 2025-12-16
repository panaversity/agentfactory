# MCP Tools Reference

> Complete API documentation for all 12 PanaversityFS tools

**Spec Reference**: [Feature 039: PanaversityFS Production Hardening](../../../../specs/039-panaversity-fs-hardening/spec.md)

## Overview

| Category | Tools | Purpose | Key Requirements |
|----------|-------|---------|------------------|
| Content | `read_content`, `write_content`, `delete_content` | Lesson/summary CRUD | FR-001 to FR-006, FR-015 to FR-019 |
| Assets | `upload_asset`, `get_asset`, `list_assets` | Binary asset management | FR-008 |
| Search | `glob_search`, `grep_search` | File and content search | - |
| Registry | `list_books` | Book discovery | - |
| Bulk | `get_book_archive` | ZIP archive generation | FR-011 to FR-014, R4 |
| Validation | `validate_book`, `delta_build` | Schema validation, incremental builds | FR-010, FR-025 to FR-027 |

---

## Content Tools

### `read_content`

Read markdown content with metadata. Supports bulk reading and user overlays.

**Annotations**: `readOnlyHint=true`, `idempotentHint=true`

| Parameter | Required | Type | Description |
|-----------|----------|------|-------------|
| `book_id` | Yes | string | Book identifier |
| `path` | Yes | string | Content path |
| `scope` | No | string | `file`, `chapter`, `part`, or `book` |
| `user_id` | No | string | User ID for overlay personalization |

**Example - Single File**:
```json
{
  "book_id": "ai-python",
  "path": "content/01-Part/01-Chapter/01-intro.md"
}
```

**Response**:
```json
{
  "content": "# Introduction\n\nWelcome...",
  "file_size": 2345,
  "last_modified": "2025-12-04T10:00:00Z",
  "file_hash_sha256": "abc123...",
  "source": "base"
}
```

**Example - With Overlay**:
```json
{
  "book_id": "ai-python",
  "path": "content/01-Part/01-Chapter/01-intro.md",
  "user_id": "user123"
}
```

Response includes `"source": "overlay"` if user has customized version.

**Example - Bulk Read (Chapter)**:
```json
{
  "book_id": "ai-python",
  "path": "content/01-Part/01-Chapter",
  "scope": "chapter"
}
```

Returns array of all `.md` files in that directory.

---

### `write_content`

Write content with conflict detection and overlay support.

**Implements**: FR-002 (journal before success), FR-003 (hash conflict rejection), FR-004 (HASH_REQUIRED for blind overwrites), FR-005 (create without hash), FR-017 (overlay writes)

**Annotations**: `idempotentHint=true`

| Parameter | Required | Type | Description |
|-----------|----------|------|-------------|
| `book_id` | Yes | string | Book identifier |
| `path` | Yes | string | Content path |
| `content` | Yes | string | Markdown content |
| `expected_hash` | Conditional | string | Required for updates, omit for creates |
| `user_id` | No | string | Write to user overlay |

**Agent Contract (from Spec)**:

The spec defines this mandatory protocol for agents:
```
IF file exists:
    1. Read current file to get file_hash from response
    2. Call write_content WITH expected_hash=<that hash>
    3. Handle CONFLICT → re-read, merge, retry

IF file does NOT exist:
    1. Call write_content WITHOUT expected_hash
    2. System creates file, returns new hash

FORBIDDEN:
    - write_content on existing file WITHOUT expected_hash → HASH_REQUIRED error
    - write_content WITH expected_hash on non-existent file → NOT_FOUND error
```

**Example - Create**:
```json
{
  "book_id": "ai-python",
  "path": "content/01-Part/01-Chapter/02-variables.md",
  "content": "# Variables\n\nLearn about variables..."
}
```

**Example - Update**:
```json
{
  "book_id": "ai-python",
  "path": "content/01-Part/01-Chapter/01-intro.md",
  "content": "# Updated Introduction\n\n...",
  "expected_hash": "abc123..."
}
```

**Example - User Overlay**:
```json
{
  "book_id": "ai-python",
  "path": "content/01-Part/01-Chapter/01-intro.md",
  "content": "# My Notes\n\nPersonalized content...",
  "user_id": "user123"
}
```

**Response**:
```json
{
  "status": "success",
  "mode": "created",
  "file_hash": "def456...",
  "file_size": 345,
  "namespace": "base"
}
```

---

### `delete_content`

Delete content (idempotent - succeeds even if file doesn't exist).

**Annotations**: `destructiveHint=true`, `idempotentHint=true`

| Parameter | Required | Type | Description |
|-----------|----------|------|-------------|
| `book_id` | Yes | string | Book identifier |
| `path` | Yes | string | Content path |
| `user_id` | No | string | Delete overlay only (base never deleted) |

**Example**:
```json
{
  "book_id": "ai-python",
  "path": "content/01-Part/01-Chapter/01-intro.md"
}
```

**Response**:
```json
{
  "status": "success",
  "path": "books/ai-python/content/01-Part/01-Chapter/01-intro.md",
  "existed": true
}
```

---

## Asset Tools

### `upload_asset`

Upload binary asset (images, slides, videos, audio).

**Annotations**: `destructiveHint=true`

| Parameter | Required | Type | Description |
|-----------|----------|------|-------------|
| `book_id` | Yes | string | Book identifier |
| `asset_type` | Yes | string | `img`, `slides`, `videos`, `audio` |
| `filename` | Yes | string | Original filename |
| `binary_data` | Yes | string | Base64-encoded content |

**Example**:
```json
{
  "book_id": "ai-python",
  "asset_type": "img",
  "filename": "diagram.png",
  "binary_data": "iVBORw0KGgo..."
}
```

**Response**:
```json
{
  "status": "success",
  "method": "direct",
  "cdn_url": "https://cdn.example.com/books/ai-python/static/img/diagram.png",
  "file_size": 45231,
  "mime_type": "image/png"
}
```

---

### `get_asset`

Get asset metadata and optionally binary data.

**Annotations**: `readOnlyHint=true`, `idempotentHint=true`

| Parameter | Required | Type | Description |
|-----------|----------|------|-------------|
| `book_id` | Yes | string | Book identifier |
| `asset_type` | Yes | string | `img`, `slides`, `videos`, `audio` |
| `filename` | Yes | string | Asset filename |
| `include_binary` | No | bool | Include base64 data (default: false) |

**Example**:
```json
{
  "book_id": "ai-python",
  "asset_type": "img",
  "filename": "diagram.png",
  "include_binary": true
}
```

**Response**:
```json
{
  "cdn_url": "https://cdn.example.com/...",
  "file_size": 45231,
  "mime_type": "image/png",
  "binary_data": "iVBORw0KGgo..."
}
```

---

### `list_assets`

List all assets in a book.

**Annotations**: `readOnlyHint=true`, `idempotentHint=true`

| Parameter | Required | Type | Description |
|-----------|----------|------|-------------|
| `book_id` | Yes | string | Book identifier |
| `asset_type` | No | string | Filter by type |

**Response**: Array of asset metadata objects.

---

## Search Tools

### `glob_search`

Find files matching glob pattern.

**Annotations**: `readOnlyHint=true`, `idempotentHint=true`

| Parameter | Required | Type | Description |
|-----------|----------|------|-------------|
| `book_id` | Yes | string | Book identifier |
| `pattern` | Yes | string | Glob pattern |
| `all_books` | No | bool | Search all books |

**Example**:
```json
{
  "book_id": "ai-python",
  "pattern": "content/**/*.summary.md"
}
```

**Response**: Array of matching file paths.

---

### `grep_search`

Search content by regex pattern.

**Annotations**: `readOnlyHint=true`, `idempotentHint=true`

| Parameter | Required | Type | Description |
|-----------|----------|------|-------------|
| `book_id` | Yes | string | Book identifier |
| `pattern` | Yes | string | Regex pattern |
| `all_books` | No | bool | Search all books |
| `max_results` | No | int | Limit results (default: 100, max: 1000) |

**Response**:
```json
[
  {
    "file_path": "books/ai-python/content/01-Part/01-Chapter/01-intro.md",
    "line_number": 42,
    "matched_line": "Python is a powerful language..."
  }
]
```

---

## Registry Tools

### `list_books`

Discover available books by scanning `books/` directory.

**Annotations**: `readOnlyHint=true`, `idempotentHint=true`

**Response**:
```json
[
  {
    "book_id": "ai-python",
    "storage_backend": "fs"
  },
  {
    "book_id": "robotics-fundamentals",
    "storage_backend": "fs"
  }
]
```

---

## Bulk Tools

### `get_book_archive`

Generate ZIP archive of entire book using streaming (FR-011).

**Implements**: FR-011 (64MB streaming buffer), FR-012 (<60s for 500 files/200MB), FR-013 (scope parameter), FR-014 (partial result on timeout)

**Invariant R4**: Archive throughput bound - verified by `tests/performance/test_archive_throughput.py`

**Annotations**: `readOnlyHint=true`, `idempotentHint=true`

| Parameter | Required | Type | Description |
|-----------|----------|------|-------------|
| `book_id` | Yes | string | Book identifier |
| `scope` | No | string | `content`, `assets`, or `all` (FR-013) |

**Response**:
```json
{
  "status": "success",
  "archive_url": "https://cdn.example.com/archives/ai-python-2025-12-04.zip",
  "expires_at": "2025-12-04T11:00:00Z",
  "file_count": 487,
  "total_size_bytes": 185432100
}
```

**Performance (SC-001)**: <60s for 500 files, <64MB memory.

---

## Validation Tools

### `validate_book`

Validate book structure against Docusaurus-aligned schema.

**Implements**: FR-010 (`validate_book_structure` tool), FR-007 (content path regex), FR-008 (asset path regex), FR-009 (traversal rejection)

**Invariant R1**: All paths conform to schema - verified by `tests/property/test_invariant_r1_schema.py`

**Annotations**: `readOnlyHint=true`, `idempotentHint=true`

| Parameter | Required | Type | Description |
|-----------|----------|------|-------------|
| `book_id` | Yes | string | Book identifier |
| `strict` | No | bool | Fail on first error |
| `include_warnings` | No | bool | Include warnings (default: true) |

**Response**:
```json
{
  "valid": true,
  "book_id": "ai-python",
  "errors": [],
  "warnings": [
    {
      "path": "content/01-Part/setup.md",
      "issue": "Filename doesn't follow NN-name pattern"
    }
  ],
  "summary": {
    "total_files": 45,
    "content_files": 42,
    "error_count": 0,
    "warning_count": 1
  }
}
```

---

### `delta_build`

Detect files changed since last build for incremental Docusaurus builds.

**Implements**: FR-025 (`plan_build` tool), FR-026 (response format with status, files, manifest_hash), FR-027 (download only changed files)

**Annotations**: `readOnlyHint=true`, `idempotentHint=true`

| Parameter | Required | Type | Description |
|-----------|----------|------|-------------|
| `book_id` | Yes | string | Book identifier |
| `since` | Yes | string | ISO 8601 timestamp |
| `include_content` | No | bool | Include file content |
| `user_id` | No | string | Include user overlay changes |

**Example**:
```json
{
  "book_id": "ai-python",
  "since": "2025-01-01T00:00:00Z"
}
```

**Response**:
```json
{
  "changed_count": 3,
  "since": "2025-01-01T00:00:00Z",
  "book_id": "ai-python",
  "changed_files": [
    {
      "path": "content/01-Part/01-Chapter/01-lesson.md",
      "sha256": "abc123...",
      "last_modified": "2025-01-02T10:30:00Z",
      "namespace": "base"
    }
  ]
}
```

---

## Error Types

| Error | When | Resolution |
|-------|------|------------|
| `ContentNotFoundError` | File doesn't exist | Check path, create content first |
| `ConflictError` | Hash mismatch on update | Re-read to get current hash |
| `HashRequiredError` | Update without `expected_hash` | Provide hash from previous read |
| `SchemaViolationError` | Invalid path format | Check path against FR-007/008 patterns |
| `ValidationError` | Invalid input | Check Pydantic model requirements |

---

## MCP Annotations

| Annotation | Meaning |
|------------|---------|
| `readOnlyHint` | Tool only reads, no modifications |
| `destructiveHint` | Tool may delete or overwrite |
| `idempotentHint` | Safe to retry, same result |
