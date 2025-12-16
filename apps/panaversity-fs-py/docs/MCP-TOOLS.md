# MCP Tools Reference

> Complete API documentation for PanaversityFS MCP tools

**Specification**: [specs/030-panaversity-fs/spec.md](../../specs/030-panaversity-fs/spec.md)
**ADR-0018**: [Docusaurus-Aligned Storage Structure](../../history/adr/0018-panaversityfs-docusaurus-aligned-structure.md)

## Overview

PanaversityFS exposes **12 MCP tools** organized into 6 categories (ADR-0018):

| Category | Tools | Count |
|----------|-------|-------|
| [Content](#content-tools) | `read_content`, `write_content`, `delete_content` | 3 |
| [Assets](#asset-tools) | `upload_asset`, `get_asset`, `list_assets` | 3 |
| [Search](#search-tools) | `glob_search`, `grep_search` | 2 |
| [Registry](#registry-tools) | `list_books` | 1 |
| [Bulk](#bulk-tools) | `get_book_archive` | 1 |
| [Validation & Build](#validation--build-tools) | `validate_book`, `delta_build` | 2 |

**Note**: Summary tools were removed in ADR-0018. Summaries are now managed via content tools using the `.summary.md` naming convention.

---

## Content Tools

Content tools handle both lessons and summaries (ADR-0018).

### `read_content`

Read markdown content with metadata. Works for lessons and summaries. Supports bulk reading of entire chapters, parts, or the entire book. Supports user overlay personalization (FR-016).

**Annotations**: `readOnlyHint=true`, `idempotentHint=true`

| Field | Required | Description |
|-------|----------|-------------|
| `book_id` | Yes | Book identifier |
| `path` | Yes | Content path (file, chapter directory, or part directory) |
| `scope` | No | `file` (default), `chapter`, `part`, or `book` |
| `user_id` | No | User ID for overlay personalization (FR-016) |

**Input (Single File - Default)**:
```json
{
  "book_id": "ai-native-python",
  "path": "content/01-Part/01-Chapter/01-intro.md"
}
```

**Input (Entire Chapter)**:
```json
{
  "book_id": "ai-native-python",
  "path": "content/01-Part/01-Chapter",
  "scope": "chapter"
}
```

**Input (Entire Part)**:
```json
{
  "book_id": "ai-native-python",
  "path": "content/01-Part",
  "scope": "part"
}
```

**Output (scope=file)**:
```json
{
  "content": "---\ntitle: Introduction\n---\n\n# Lesson 1...",
  "file_size": 2345,
  "last_modified": "2025-11-24T12:00:00Z",
  "storage_backend": "fs",
  "file_hash_sha256": "a591a6d40bf420404a011733cfb7b190d62c65bf..."
}
```

**Output (scope=chapter or scope=part)** - Array of files:
```json
[
  {
    "path": "content/01-Part/01-Chapter/README.md",
    "content": "# Chapter Overview...",
    "file_size": 1234,
    "last_modified": "2025-11-24T12:00:00Z",
    "storage_backend": "fs",
    "file_hash_sha256": "..."
  },
  {
    "path": "content/01-Part/01-Chapter/01-intro.md",
    "content": "# Introduction...",
    "file_size": 2345,
    "last_modified": "2025-11-24T12:00:00Z",
    "storage_backend": "fs",
    "file_hash_sha256": "..."
  }
]
```

**Scope Behavior**:
- `file`: Read single file (original behavior)
- `chapter`: Read all `.md` files directly in the chapter directory (not subdirectories)
- `part`: Read all `.md` files recursively in the part directory (includes all chapters)
- `book`: Read all `.md` files in the entire book's content/ directory

**Overlay Personalization (FR-016)**:
When `user_id` is provided, checks overlay first and falls back to base:
```json
{
  "book_id": "ai-native-python",
  "path": "content/01-Part/01-Chapter/01-intro.md",
  "user_id": "user123"
}
```

Response includes `source` field indicating where content was read from:
```json
{
  "content": "...",
  "source": "overlay",  // or "base"
  "file_hash_sha256": "..."
}
```

**Errors**:
- `ContentNotFoundError`: File/directory does not exist

---

### `write_content`

Write content with upsert semantics and conflict detection. Works for lessons and summaries. Supports user overlay personalization (FR-017).

**Annotations**: `idempotentHint=true`

**Conflict Detection Protocol**:
- **FR-003**: If `expected_hash` provided, verify it matches current hash before write
- **FR-004**: If `expected_hash` omitted AND file exists, reject with `HashRequiredError`
- **FR-005**: If `expected_hash` omitted AND file doesn't exist, create succeeds

**Input (Lesson)**:
```json
{
  "book_id": "ai-native-python",
  "path": "content/01-Part/01-Chapter/01-intro.md",
  "content": "---\ntitle: Introduction\n---\n\n# Updated content...",
  "file_hash": "a591a6d40bf420404a011733cfb7b190d62c65bf..."
}
```

**Input (Summary - ADR-0018)**:
```json
{
  "book_id": "ai-native-python",
  "path": "content/01-Part/01-Chapter/01-intro.summary.md",
  "content": "# Lesson 1 Summary\n\nKey concepts..."
}
```

| Field | Required | Description |
|-------|----------|-------------|
| `book_id` | Yes | Book identifier (lowercase alphanumeric + hyphens) |
| `path` | Yes | Relative path within book (use `.summary.md` for summaries) |
| `content` | Yes | Markdown content (max 500KB) |
| `expected_hash` | No | SHA256 hash for conflict detection (REQUIRED for updates) |
| `user_id` | No | User ID for overlay personalization (FR-017) |

**Overlay Personalization (FR-017)**:
When `user_id` is provided, writes to user's overlay namespace:
```json
{
  "book_id": "ai-native-python",
  "path": "content/01-Part/01-Chapter/01-intro.md",
  "content": "# My personalized notes...",
  "user_id": "user123"
}
```

Response includes `namespace` field:
```json
{
  "status": "success",
  "mode": "created",
  "namespace": "overlay",
  "file_hash": "..."
}
```

**Output**:
```json
{
  "status": "success",
  "path": "books/ai-native-python/content/01-Part/01-Chapter/01-intro.md",
  "file_size": 2456,
  "file_hash": "b7a9c3d4e5f6...",
  "mode": "updated"
}
```

**Errors**:
- `ConflictError`: Hash mismatch (another agent modified the file)
- `HashRequiredError`: Update attempted without expected_hash (FR-004)
- `InvalidPathError`: Path contains traversal or invalid characters

---

### `delete_content`

Delete content file (lesson or summary). Supports user overlay personalization (FR-018).

**Annotations**: `destructiveHint=true`, `idempotentHint=true`

**Input (Lesson)**:
```json
{
  "book_id": "ai-native-python",
  "path": "content/01-Part/01-Chapter/01-intro.md"
}
```

**Input (Summary - ADR-0018)**:
```json
{
  "book_id": "ai-native-python",
  "path": "content/01-Part/01-Chapter/01-intro.summary.md"
}
```

| Field | Required | Description |
|-------|----------|-------------|
| `book_id` | Yes | Book identifier |
| `path` | Yes | Content path to delete |
| `user_id` | No | User ID for overlay delete (FR-018) |

**Output**:
```json
{
  "status": "success",
  "path": "books/ai-native-python/content/01-Part/01-Chapter/01-intro.md",
  "existed": true,
  "message": "File deleted"
}
```

**Overlay Personalization (FR-018)**:
When `user_id` is provided, ONLY deletes from user's overlay namespace (base content is NEVER deleted):
```json
{
  "book_id": "ai-native-python",
  "path": "content/01-Part/01-Chapter/01-intro.md",
  "user_id": "user123"
}
```

Response includes `namespace` field:
```json
{
  "status": "success",
  "path": "books/ai-native-python/users/user123/content/01-Part/01-Chapter/01-intro.md",
  "existed": true,
  "namespace": "overlay",
  "message": "File deleted"
}
```

This effectively "resets" the user's personalized content back to the base version.

**Note**: Idempotent - returns success even if file doesn't exist (`existed: false`).

---

## Asset Tools

### `upload_asset`

Upload binary asset with hybrid pattern.

**Annotations**: `destructiveHint=true`

**Direct Upload (<10MB)**:
```json
{
  "book_id": "ai-native-python",
  "asset_type": "images",
  "filename": "diagram.png",
  "binary_data": "iVBORw0KGgoAAAANSUhEUgAA..."
}
```

**Presigned URL (≥10MB)** *(planned)*:
```json
{
  "book_id": "ai-native-python",
  "asset_type": "videos",
  "filename": "tutorial.mp4",
  "file_size": 52428800
}
```

| Field | Required | Description |
|-------|----------|-------------|
| `book_id` | Yes | Book identifier |
| `asset_type` | Yes | One of: `images`, `slides`, `videos`, `audio` |
| `filename` | Yes | Original filename (sanitized on server) |
| `binary_data` | For <10MB | Base64-encoded binary content |
| `file_size` | For ≥10MB | File size in bytes (for presigned URL) |

**Output (Direct)**:
```json
{
  "status": "success",
  "method": "direct",
  "cdn_url": "https://cdn.panaversity.com/books/ai-native-python/static/images/diagram.png",
  "file_size": 45231,
  "mime_type": "image/png",
  "path": "books/ai-native-python/static/images/diagram.png"
}
```

**Storage Path**: `books/{book_id}/static/{asset_type}/{filename}` (ADR-0018: Docusaurus-aligned)

---

### `get_asset`

Get asset metadata including CDN URL. Optionally include base64-encoded binary data.

**Annotations**: `readOnlyHint=true`, `idempotentHint=true`

| Field | Required | Description |
|-------|----------|-------------|
| `book_id` | Yes | Book identifier |
| `asset_type` | Yes | One of: `images`, `slides`, `videos`, `audio` |
| `filename` | Yes | Asset filename |
| `include_binary` | No | Include base64 binary data (default: false) |

**Input (Metadata Only - Default)**:
```json
{
  "book_id": "ai-native-python",
  "asset_type": "images",
  "filename": "diagram.png"
}
```

**Input (With Binary Data)**:
```json
{
  "book_id": "ai-native-python",
  "asset_type": "images",
  "filename": "diagram.png",
  "include_binary": true
}
```

**Output (include_binary=false)**:
```json
{
  "cdn_url": "https://cdn.panaversity.com/books/ai-native-python/static/images/diagram.png",
  "file_size": 45231,
  "mime_type": "image/png",
  "upload_timestamp": "2025-11-24T12:00:00Z",
  "uploaded_by_agent_id": "system",
  "asset_type": "images",
  "filename": "diagram.png",
  "binary_data": null
}
```

**Output (include_binary=true)**:
```json
{
  "cdn_url": "https://cdn.panaversity.com/books/ai-native-python/static/images/diagram.png",
  "file_size": 45231,
  "mime_type": "image/png",
  "upload_timestamp": "2025-11-24T12:00:00Z",
  "uploaded_by_agent_id": "system",
  "asset_type": "images",
  "filename": "diagram.png",
  "binary_data": "iVBORw0KGgoAAAANSUhEUgAA..."
}
```

**Use Cases for `include_binary`**:
- Docusaurus plugin fetching assets for local build
- Direct asset download without CDN
- Asset migration between storage backends

**Errors**:
- `ContentNotFoundError`: Asset does not exist

---

### `list_assets`

List assets for a book with optional type filtering.

**Annotations**: `readOnlyHint=true`, `idempotentHint=true`

**Input**:
```json
{
  "book_id": "ai-native-python",
  "asset_type": "images"
}
```

| Field | Required | Description |
|-------|----------|-------------|
| `book_id` | Yes | Book identifier |
| `asset_type` | No | Filter by type (omit for all types) |

**Output**:
```json
[
  {
    "cdn_url": "https://cdn.panaversity.com/books/ai-native-python/static/images/diagram.png",
    "file_size": 45231,
    "mime_type": "image/png",
    "upload_timestamp": "2025-11-24T12:00:00Z",
    "uploaded_by_agent_id": "system",
    "asset_type": "images",
    "filename": "diagram.png"
  },
  {
    "cdn_url": "https://cdn.panaversity.com/books/ai-native-python/static/images/screenshot.png",
    "file_size": 12345,
    "mime_type": "image/png",
    ...
  }
]
```

---

## Search Tools

### `glob_search`

Search for files matching glob pattern.

**Annotations**: `readOnlyHint=true`, `idempotentHint=true`

**Input**:
```json
{
  "book_id": "ai-native-python",
  "pattern": "**/*.md",
  "all_books": false
}
```

| Field | Required | Description |
|-------|----------|-------------|
| `book_id` | Yes | Book identifier |
| `pattern` | Yes | Glob pattern (supports `**`, `*`, `?`) |
| `all_books` | No | Search all books (default: false) |

**Output**:
```json
[
  "books/ai-native-python/content/01-Part/01-Chapter/01-intro.md",
  "books/ai-native-python/content/01-Part/01-Chapter/01-intro.summary.md",
  "books/ai-native-python/content/01-Part/01-Chapter/02-variables.md"
]
```

**Pattern Examples** (ADR-0018: Docusaurus-aligned):
- `**/*.md` - All markdown files
- `content/**/*.md` - All content files (lessons + summaries)
- `content/**/*.summary.md` - All summaries only
- `static/images/**/*.png` - All PNG images
- `content/01-Part/01-Chapter/*` - All files in a specific chapter

---

### `grep_search`

Search content using regex pattern.

**Annotations**: `readOnlyHint=true`, `idempotentHint=true`

**Input**:
```json
{
  "book_id": "ai-native-python",
  "pattern": "OpenDAL",
  "all_books": false,
  "max_results": 100
}
```

| Field | Required | Description |
|-------|----------|-------------|
| `book_id` | Yes | Book identifier |
| `pattern` | Yes | Regex pattern |
| `all_books` | No | Search all books (default: false) |
| `max_results` | No | Maximum results (default: 100, max: 1000) |

**Output**:
```json
[
  {
    "file_path": "books/ai-native-python/content/04-Part/12-Chapter/03-storage.md",
    "line_number": 42,
    "matched_line": "OpenDAL provides unified storage abstraction..."
  },
  {
    "file_path": "books/ai-native-python/content/04-Part/12-Chapter/04-backends.md",
    "line_number": 15,
    "matched_line": "We use OpenDAL to access S3 and local filesystem..."
  }
]
```

**Note**: Only searches `.md` files.

---

## Registry Tools

### `list_books`

List all books by dynamically scanning the `books/` directory.

**Annotations**: `readOnlyHint=true`, `idempotentHint=true`

**Input**:
```json
{}
```

**Output**:
```json
[
  {
    "book_id": "ai-native-python",
    "storage_backend": "fs"
  },
  {
    "book_id": "generative-ai-fundamentals",
    "storage_backend": "fs"
  }
]
```

**Discovery Method**: Books are automatically discovered by scanning subdirectories in the `books/` directory. Any directory matching the book ID pattern (`^[a-z0-9-]+$`) is returned.

**Note**: No `registry.yaml` file is required. This is a zero-configuration approach.

---

## Bulk Tools

### `get_book_archive`

Generate ZIP archive of entire book.

**Annotations**: `readOnlyHint=true`, `idempotentHint=true`

**Input**:
```json
{
  "book_id": "ai-native-python"
}
```

**Output**:
```json
{
  "status": "success",
  "archive_url": "https://cdn.panaversity.com/archives/ai-native-python-2025-11-24.zip",
  "expires_at": "2025-11-24T13:00:00Z",
  "file_count": 487,
  "total_size_bytes": 185432100,
  "format": "zip",
  "valid_for_seconds": 3600
}
```

**Performance Constraints**:
- Must complete in <60 seconds
- Supports up to 500 files / 200MB uncompressed

**Archive Contents** (ADR-0018: Docusaurus-aligned):
- All content (`content/**/*.md`) - lessons and summaries
- All static assets (`static/**/*`)

---

## Validation & Build Tools

### `validate_book`

Validate book structure against the expected schema (FR-007, FR-008).

**Annotations**: `readOnlyHint=true`, `idempotentHint=true`

**Input**:
```json
{
  "book_id": "ai-native-python",
  "strict": false,
  "include_warnings": true
}
```

| Field | Required | Description |
|-------|----------|-------------|
| `book_id` | Yes | Book identifier |
| `strict` | No | Fail on first error (default: false) |
| `include_warnings` | No | Include non-critical warnings (default: true) |

**Output**:
```json
{
  "valid": true,
  "book_id": "ai-native-python",
  "errors": [],
  "warnings": [
    {
      "path": "content/01-Part/setup.md",
      "issue": "Filename doesn't follow NN-name pattern",
      "suggestion": "Consider renaming to '00-setup.md'"
    }
  ],
  "summary": {
    "total_files": 45,
    "content_files": 42,
    "asset_files": 3,
    "error_count": 0,
    "warning_count": 1
  }
}
```

**Validation Rules**:
- **Content paths (FR-007)**: Must match `content/{NN-Name}/{NN-Name}/{NN-name}(.summary)?.md`
- **Asset paths (FR-008)**: Must be in `static/(img|slides|videos|audio)/{path}`

**Errors**:
- `ContentNotFoundError`: Book does not exist

---

### `delta_build`

Detect files changed since a given timestamp for incremental builds (FR-025).

**Annotations**: `readOnlyHint=true`, `idempotentHint=true`

**Input**:
```json
{
  "book_id": "ai-native-python",
  "since": "2025-01-01T00:00:00Z",
  "include_content": false,
  "user_id": null
}
```

| Field | Required | Description |
|-------|----------|-------------|
| `book_id` | Yes | Book identifier |
| `since` | Yes | ISO 8601 timestamp (returns files modified after this) |
| `include_content` | No | Include file content in response (default: false) |
| `user_id` | No | Include user's overlay changes |

**Output (without user_id)**:
```json
{
  "changed_count": 3,
  "since": "2025-01-01T00:00:00Z",
  "book_id": "ai-native-python",
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

**Output (with user_id)**:
```json
{
  "changed_count": 5,
  "since": "2025-01-01T00:00:00Z",
  "book_id": "ai-native-python",
  "user_id": "user123",
  "base_changes": 3,
  "overlay_changes": 2,
  "changed_files": [
    {"path": "...", "namespace": "base", ...},
    {"path": "...", "namespace": "overlay", "user_id": "user123", ...}
  ]
}
```

**Use Cases**:
- Incremental Docusaurus builds (only rebuild changed pages)
- CI/CD pipelines detecting what to deploy
- User-specific change tracking for personalized builds

---

## Error Types

| Error | HTTP Status | Description |
|-------|-------------|-------------|
| `ContentNotFoundError` | 404 | Requested file/asset/summary not found |
| `ConflictError` | 409 | Hash mismatch during write (concurrent modification) |
| `HashRequiredError` | 400 | Update attempted without expected_hash (FR-004) |
| `InvalidPathError` | 400 | Path contains traversal or invalid characters |
| `ValidationError` | 400 | Input validation failed (Pydantic) |

**Error Response Format**:
```json
{
  "error": "ContentNotFoundError",
  "message": "Content not found: books/ai-native-python/lessons/missing.md",
  "path": "books/ai-native-python/lessons/missing.md"
}
```

---

## MCP Annotations Reference

All tools include standard MCP annotations:

| Annotation | Description |
|------------|-------------|
| `readOnlyHint` | Tool only reads data, no modifications |
| `destructiveHint` | Tool may delete or overwrite data |
| `idempotentHint` | Safe to retry, same result on multiple calls |
| `openWorldHint` | Tool may access external resources |

---

## Related Documentation

- **[Architecture](./ARCHITECTURE.md)**: System design and patterns
- **[Setup Guide](./SETUP.md)**: Backend configuration
- **[Specification](../../specs/030-panaversity-fs/spec.md)**: Full requirements
