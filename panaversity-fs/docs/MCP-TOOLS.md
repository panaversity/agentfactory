# MCP Tools Reference

> Complete API documentation for PanaversityFS MCP tools

**Specification**: [specs/030-panaversity-fs/spec.md](../../specs/030-panaversity-fs/spec.md)
**ADR-0018**: [Docusaurus-Aligned Storage Structure](../../history/adr/0018-panaversityfs-docusaurus-aligned-structure.md)

## Overview

PanaversityFS exposes **9 MCP tools** organized into 5 categories (ADR-0018):

| Category | Tools | Count |
|----------|-------|-------|
| [Content](#content-tools) | `read_content`, `write_content`, `delete_content` | 3 |
| [Assets](#asset-tools) | `upload_asset`, `get_asset`, `list_assets` | 3 |
| [Search](#search-tools) | `glob_search`, `grep_search` | 2 |
| [Registry](#registry-tools) | `list_books` | 1 |
| [Bulk](#bulk-tools) | `get_book_archive` | 1 |

**Note**: Summary tools were removed in ADR-0018. Summaries are now managed via content tools using the `.summary.md` naming convention.

---

## Content Tools

Content tools handle both lessons and summaries (ADR-0018).

### `read_content`

Read markdown content with metadata. Works for lessons and summaries. Supports bulk reading of entire chapters or parts.

**Annotations**: `readOnlyHint=true`, `idempotentHint=true`

| Field | Required | Description |
|-------|----------|-------------|
| `book_id` | Yes | Book identifier |
| `path` | Yes | Content path (file, chapter directory, or part directory) |
| `scope` | No | `file` (default), `chapter`, or `part` |

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

**Errors**:
- `ContentNotFoundError`: File/directory does not exist

---

### `write_content`

Write content with upsert semantics and optional conflict detection. Works for lessons and summaries.

**Annotations**: `idempotentHint=true`

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
| `file_hash` | No | SHA256 hash for conflict detection |

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
- `InvalidPathError`: Path contains traversal or invalid characters

**Conflict Detection Flow**:
```
1. Agent reads content → gets file_hash
2. Agent modifies content locally
3. Agent writes with original file_hash
4. Server verifies hash matches current file
5. If match: write succeeds, return new hash
6. If mismatch: raise ConflictError (agent must re-read and merge)
```

---

### `delete_content`

Delete content file (lesson or summary).

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

**Output**:
```json
{
  "status": "success",
  "path": "books/ai-native-python/content/01-Part/01-Chapter/01-intro.md",
  "existed": true,
  "message": "File deleted"
}
```

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

## Error Types

| Error | HTTP Status | Description |
|-------|-------------|-------------|
| `ContentNotFoundError` | 404 | Requested file/asset/summary not found |
| `ConflictError` | 409 | Hash mismatch during write (concurrent modification) |
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
