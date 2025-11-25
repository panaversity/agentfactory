# MCP Tools Reference

> Complete API documentation for PanaversityFS MCP tools

**Specification**: [specs/030-panaversity-fs/spec.md](../../specs/030-panaversity-fs/spec.md)

## Overview

PanaversityFS exposes **12 MCP tools** organized into 6 categories:

| Category | Tools | Count |
|----------|-------|-------|
| [Content](#content-tools) | `read_content`, `write_content`, `delete_content` | 3 |
| [Summary](#summary-tools) | `read_summary`, `write_summary`, `delete_summary` | 3 |
| [Assets](#asset-tools) | `upload_asset`, `get_asset`, `list_assets` | 3 |
| [Search](#search-tools) | `glob_search`, `grep_search` | 2 |
| [Registry](#registry-tools) | `list_books` | 1 |
| [Bulk](#bulk-tools) | `get_book_archive` | 1 |

---

## Content Tools

### `read_content`

Read lesson markdown content with metadata.

**Annotations**: `readOnlyHint=true`, `idempotentHint=true`

**Input**:
```json
{
  "book_id": "ai-native-python",
  "path": "lessons/part-1/chapter-01/lesson-01.md"
}
```

**Output**:
```json
{
  "content": "---\ntitle: Introduction\n---\n\n# Lesson 1...",
  "file_size": 2345,
  "last_modified": "2025-11-24T12:00:00Z",
  "storage_backend": "fs",
  "file_hash_sha256": "a591a6d40bf420404a011733cfb7b190d62c65bf..."
}
```

**Errors**:
- `ContentNotFoundError`: File does not exist

---

### `write_content`

Write lesson content with upsert semantics and optional conflict detection.

**Annotations**: `idempotentHint=true`

**Input**:
```json
{
  "book_id": "ai-native-python",
  "path": "lessons/part-1/chapter-01/lesson-01.md",
  "content": "---\ntitle: Introduction\n---\n\n# Updated content...",
  "file_hash": "a591a6d40bf420404a011733cfb7b190d62c65bf..."
}
```

| Field | Required | Description |
|-------|----------|-------------|
| `book_id` | Yes | Book identifier (lowercase alphanumeric + hyphens) |
| `path` | Yes | Relative path within book |
| `content` | Yes | Markdown content (max 1MB) |
| `file_hash` | No | SHA256 hash for conflict detection |

**Output**:
```json
{
  "status": "success",
  "path": "books/ai-native-python/lessons/part-1/chapter-01/lesson-01.md",
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

Delete lesson content file.

**Annotations**: `destructiveHint=true`, `idempotentHint=true`

**Input**:
```json
{
  "book_id": "ai-native-python",
  "path": "lessons/part-1/chapter-01/lesson-01.md"
}
```

**Output**:
```json
{
  "status": "success",
  "path": "books/ai-native-python/lessons/part-1/chapter-01/lesson-01.md",
  "existed": true,
  "message": "File deleted"
}
```

**Note**: Idempotent - returns success even if file doesn't exist (`existed: false`).

---

## Summary Tools

### `read_summary`

Read chapter summary with metadata.

**Annotations**: `readOnlyHint=true`, `idempotentHint=true`

**Input**:
```json
{
  "book_id": "ai-native-python",
  "chapter_id": "chapter-01"
}
```

**Output**:
```json
{
  "path": "books/ai-native-python/chapters/chapter-01/.summary.md",
  "content": "# Chapter 1 Summary\n\n## Key Concepts...",
  "file_size": 1234,
  "last_modified": "2025-11-24T12:00:00Z",
  "storage_backend": "fs",
  "sha256": "c8d9e0f1a2b3..."
}
```

**Errors**:
- `ContentNotFoundError`: Summary does not exist

**Storage Path**: `books/{book_id}/chapters/{chapter_id}/.summary.md`

---

### `write_summary`

Create or update chapter summary (idempotent upsert).

**Annotations**: `idempotentHint=true`

**Input**:
```json
{
  "book_id": "ai-native-python",
  "chapter_id": "chapter-01",
  "content": "# Chapter 1 Summary\n\n## Key Concepts\n- Python basics..."
}
```

| Field | Required | Description |
|-------|----------|-------------|
| `book_id` | Yes | Book identifier |
| `chapter_id` | Yes | Chapter identifier (format: `chapter-NN`) |
| `content` | Yes | Summary markdown (max 100KB) |

**Output**:
```json
{
  "status": "success",
  "path": "books/ai-native-python/chapters/chapter-01/.summary.md",
  "file_size": 1234,
  "sha256": "d1e2f3a4b5c6..."
}
```

---

### `delete_summary`

Delete chapter summary.

**Annotations**: `destructiveHint=true`, `idempotentHint=true`

**Input**:
```json
{
  "book_id": "ai-native-python",
  "chapter_id": "chapter-01"
}
```

**Output**:
```json
{
  "status": "success",
  "path": "books/ai-native-python/chapters/chapter-01/.summary.md",
  "message": "Summary deleted"
}
```

**Errors**:
- `ContentNotFoundError`: Summary does not exist (not idempotent for delete)

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
  "cdn_url": "https://cdn.panaversity.com/books/ai-native-python/assets/images/diagram.png",
  "file_size": 45231,
  "mime_type": "image/png",
  "path": "books/ai-native-python/assets/images/diagram.png"
}
```

**Storage Path**: `books/{book_id}/assets/{asset_type}/{filename}`

---

### `get_asset`

Get asset metadata including CDN URL.

**Annotations**: `readOnlyHint=true`, `idempotentHint=true`

**Input**:
```json
{
  "book_id": "ai-native-python",
  "asset_type": "images",
  "filename": "diagram.png"
}
```

**Output**:
```json
{
  "cdn_url": "https://cdn.panaversity.com/books/ai-native-python/assets/images/diagram.png",
  "file_size": 45231,
  "mime_type": "image/png",
  "upload_timestamp": "2025-11-24T12:00:00Z",
  "uploaded_by_agent_id": "system",
  "asset_type": "images",
  "filename": "diagram.png"
}
```

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
    "cdn_url": "https://cdn.panaversity.com/.../diagram.png",
    "file_size": 45231,
    "mime_type": "image/png",
    "upload_timestamp": "2025-11-24T12:00:00Z",
    "uploaded_by_agent_id": "system",
    "asset_type": "images",
    "filename": "diagram.png"
  },
  {
    "cdn_url": "https://cdn.panaversity.com/.../screenshot.png",
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
  "books/ai-native-python/lessons/part-1/chapter-01/lesson-01.md",
  "books/ai-native-python/lessons/part-1/chapter-01/lesson-02.md",
  "books/ai-native-python/chapters/chapter-01/.summary.md"
]
```

**Pattern Examples**:
- `**/*.md` - All markdown files
- `lessons/**/*.md` - All lesson files
- `assets/images/**/*.png` - All PNG images
- `chapters/chapter-01/*` - All files in chapter-01

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
    "file_path": "books/ai-native-python/lessons/part-4/chapter-12/lesson-03.md",
    "line_number": 42,
    "matched_line": "OpenDAL provides unified storage abstraction..."
  },
  {
    "file_path": "books/ai-native-python/lessons/part-4/chapter-12/lesson-04.md",
    "line_number": 15,
    "matched_line": "We use OpenDAL to access S3 and local filesystem..."
  }
]
```

**Note**: Only searches `.md` files.

---

## Registry Tools

### `list_books`

List all registered books from registry.yaml.

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
    "title": "AI-Native Python Development",
    "storage_backend": "fs",
    "created_at": "2025-01-01T00:00:00Z",
    "status": "active"
  },
  {
    "book_id": "generative-ai-fundamentals",
    "title": "Generative AI Fundamentals",
    "storage_backend": "s3",
    "created_at": "2025-02-01T00:00:00Z",
    "status": "active"
  }
]
```

**Book Status Values**: `active`, `archived`, `migrating`

**Registry Format** (`registry.yaml`):
```yaml
books:
  - book_id: ai-native-python
    title: AI-Native Python Development
    storage_backend: fs
    created_at: "2025-01-01T00:00:00Z"
    status: active
```

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

**Archive Contents**:
- All lessons (`lessons/**/*.md`)
- All summaries (`chapters/**/.summary.md`)
- All assets (`assets/**/*`)
- Book metadata (`book.yaml`)

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
