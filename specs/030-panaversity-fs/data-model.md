# Data Model: PanaversityFS

**Feature**: 030-panaversity-fs
**Date**: 2025-11-24

## Purpose

This document defines all entities, schemas, and relationships in PanaversityFS storage system.

---

## Entity Overview

```
Registry (registry.yaml)
  ├── Book[] (books/{book-id}/book.yaml)
  │   ├── Lesson[] (lessons/{part}/{chapter}/{lesson}.md)
  │   ├── Asset[] (assets/{type}/{filename})
  │   ├── Summary[] (chapters/{chapter-id}/.summary.md)
  │   └── Chapter Metadata[] (chapters/{chapter-id}/metadata.yaml)
  └── Audit Entry[] (.audit/{YYYY-MM-DD}.jsonl)
```

**Cardinality**:
- 1 Registry → 50 Books (per spec success criteria SC-008)
- 1 Book → 500 Lessons (per spec success criteria SC-009)
- 1 Book → Unlimited Assets (constrained by storage backend capacity)
- 1 Chapter → 0-1 Summary (optional, created by external agents)
- All Operations → 1 Audit Entry (immutable log)

---

## Entity 1: Registry

### Description
Central catalog of all books in PanaversityFS. Single source of truth for book discovery and storage backend configuration.

### Schema

**File**: `registry.yaml`

```yaml
version: "1.0"
created_at: "2025-11-24T00:00:00Z"
last_updated: "2025-11-24T12:34:56Z"

books:
  - book_id: "ai-native-software-development"
    title: "AI Native Software Development"
    storage_backend: "r2"
    created_at: "2025-11-24T00:00:00Z"
    status: "active"  # active | archived | migrating

  - book_id: "generative-ai-fundamentals"
    title: "Generative AI Fundamentals"
    storage_backend: "s3"
    created_at: "2025-11-25T00:00:00Z"
    status: "active"
```

### TypeScript Interface

```typescript
interface Registry {
  version: string;
  created_at: string; // ISO 8601
  last_updated: string; // ISO 8601
  books: BookRegistryEntry[];
}

interface BookRegistryEntry {
  book_id: string;       // Unique identifier (kebab-case)
  title: string;         // Display name
  storage_backend: 'local' | 'r2' | 'supabase' | 's3';
  created_at: string;    // ISO 8601
  status: 'active' | 'archived' | 'migrating';
}
```

### Validation Rules

- `book_id`: Must be unique, lowercase, kebab-case, alphanumeric + hyphens only
- `storage_backend`: Must be one of supported backends (per FR-002)
- `status`:
  - `active`: Book available for operations
  - `archived`: Read-only, no modifications allowed
  - `migrating`: Temporary state during storage backend migration
- Registry file must be atomically updated (read-modify-write with file hash check)

### Operations

- **Read**: `list_books` MCP tool
- **Update**: `add_book_to_registry` MCP tool (append new book, update `last_updated`)
- **No Delete**: Books cannot be removed from registry (set status to `archived` instead)

---

## Entity 2: Book

### Description
Container for all content related to a single educational book. Each book has isolated storage configuration and directory structure.

### Schema

**File**: `books/{book_id}/book.yaml`

```yaml
book_id: "ai-native-software-development"
title: "AI Native Software Development"
author: "Panaversity"
version: "2.0.0"
storage_backend: "r2"

content_structure:
  parts:
    - part_id: "part-1"
      title: "Foundations"
      chapters:
        - chapter_id: "chapter-01"
          title: "Introduction to AI-Native Development"
          lesson_count: 9

        - chapter_id: "chapter-02"
          title: "Setting Up Your Environment"
          lesson_count: 7

    - part_id: "part-2"
      title: "Python Fundamentals"
      chapters:
        - chapter_id: "chapter-03"
          title: "Variables and Data Types"
          lesson_count: 8
```

### TypeScript Interface

```typescript
interface Book {
  book_id: string;
  title: string;
  author: string;
  version: string; // Semantic versioning
  storage_backend: 'local' | 'r2' | 'supabase' | 's3';
  content_structure: ContentStructure;
}

interface ContentStructure {
  parts: Part[];
}

interface Part {
  part_id: string;   // e.g., "part-1"
  title: string;
  chapters: ChapterMeta[];
}

interface ChapterMeta {
  chapter_id: string; // e.g., "chapter-01"
  title: string;
  lesson_count: number;
}
```

### Validation Rules

- `book_id`: Must match parent directory name
- `storage_backend`: Must match registry entry
- `content_structure.parts`: At least 1 part required
- `chapter_id`: Unique within book, format `chapter-{NN}` (zero-padded)
- `lesson_count`: Must match actual lesson files in directory (validated by `list_content` tool)

### Operations

- **Read**: Included in `list_books` response, read by all content operations
- **Update**: Modified when chapters added/removed (via admin tools, not MCP)
- **Create**: Automatically created by `add_book_to_registry` tool

---

## Entity 3: Lesson

### Description
Individual markdown file teaching a specific concept. Core content unit in PanaversityFS.

### File Path Pattern

```
books/{book_id}/lessons/{part_id}/{chapter_id}/{lesson_id}.md
```

**Example**: `books/ai-native-software-development/lessons/part-1/chapter-01/lesson-01.md`

### Markdown Structure

```markdown
---
title: "Introduction to AI Agents"
lesson_id: "lesson-01"
chapter_id: "chapter-01"
part_id: "part-1"
learning_objectives:
  - "Understand what AI agents are"
  - "Recognize different agent types"
proficiency_level: "A2"
estimated_time_minutes: 45
last_updated: "2025-11-24T12:00:00Z"
author: "content-implementer-agent"
---

# Introduction to AI Agents

[Lesson content in markdown...]
```

### TypeScript Interface

```typescript
interface Lesson {
  // Frontmatter metadata
  metadata: LessonMetadata;

  // Content
  markdown: string;

  // Storage metadata (not in file, added by read operation)
  storage_info: StorageInfo;
}

interface LessonMetadata {
  title: string;
  lesson_id: string;
  chapter_id: string;
  part_id: string;
  learning_objectives: string[];
  proficiency_level: 'A1' | 'A2' | 'B1' | 'B2' | 'C1' | 'C2';
  estimated_time_minutes: number;
  last_updated: string; // ISO 8601
  author: string;       // Agent ID or human username
}

interface StorageInfo {
  file_size: number;        // Bytes
  last_modified: string;    // ISO 8601 from storage backend
  storage_backend: string;  // Actual backend storing this file
  file_hash_sha256: string; // For conflict detection
}
```

### Validation Rules

- **Frontmatter**: Must be valid YAML between `---` delimiters
- `lesson_id`: Must match filename (e.g., `lesson-01.md` → `lesson_id: "lesson-01"`)
- `chapter_id`, `part_id`: Must match directory structure
- `file_hash_sha256`: Computed on read, checked on update (conflict detection per FR-008)

### Operations

- **Create**: `add_content` MCP tool (writes markdown file, logs audit entry)
- **Read**: `read_content` MCP tool (returns markdown + metadata + storage_info)
- **Update**: `update_content` MCP tool (requires file_hash for conflict detection)
- **Delete**: `delete_content` MCP tool (removes file, logs audit entry)
- **List**: `glob_search` with pattern `lessons/**/*.md`
- **Search**: `grep_search` with pattern across lesson content

---

## Entity 4: Asset

### Description
Binary file (slides, images, videos, audio) associated with educational content. Stored with categorization by type.

### File Path Pattern

```
books/{book_id}/assets/{asset_type}/{filename}
```

**Asset Types**:
- `slides`: PDF slide decks
- `images`: PNG, JPG, SVG files
- `videos`: MP4 files
- `audio`: MP3, WAV files

**Example**: `books/ai-native-software-development/assets/slides/chapter-01-intro.pdf`

### TypeScript Interface

```typescript
interface Asset {
  // Identifiers
  book_id: string;
  asset_type: 'slides' | 'images' | 'videos' | 'audio';
  filename: string;

  // Storage metadata
  cdn_url: string;          // Public access URL
  file_size: number;        // Bytes
  mime_type: string;        // e.g., "application/pdf"
  upload_timestamp: string; // ISO 8601
  uploaded_by_agent_id: string;

  // Storage backend info
  storage_backend: string;
  storage_path: string;     // Full path in storage backend
  file_hash_sha256: string;
}
```

### CDN URL Pattern

```
https://cdn.panaversity.com/books/{book_id}/assets/{asset_type}/{filename}
```

**Example**: `https://cdn.panaversity.com/books/ai-native-software-development/assets/slides/chapter-01-intro.pdf`

### Validation Rules

- **File Size Limits** (per FR-010):
  - Direct upload: Max 10MB
  - Presigned URL: 10MB - 100MB
  - Rejected: >100MB (error message suggests chunked upload)
- **MIME Type Validation**:
  - `slides`: `application/pdf`
  - `images`: `image/png`, `image/jpeg`, `image/svg+xml`
  - `videos`: `video/mp4`
  - `audio`: `audio/mpeg`, `audio/wav`
- **Filename**: Must be URL-safe (alphanumeric, hyphens, underscores, dots)
- **Immutability**: Assets are write-once (no in-place updates). To update, delete old and upload new with different filename.

### Operations

- **Upload**: `add_asset` MCP tool (direct upload <10MB or presigned URL ≥10MB)
- **Read Metadata**: `get_asset` MCP tool (returns Asset interface, NOT binary data)
- **List**: `list_assets` MCP tool (filter by asset_type optional)
- **Download Binary**: Use `cdn_url` directly (public access via CDN)
- **Delete**: (Not in MVP scope, future enhancement)

---

## Entity 5: Summary

### Description
Pre-generated chapter overview markdown file created by external agents. Stored as hidden file (`.summary.md` prefix) alongside chapter content.

### File Path Pattern

```
books/{book_id}/chapters/{chapter_id}/.summary.md
```

**Example**: `books/ai-native-software-development/chapters/chapter-01/.summary.md`

### Markdown Structure

```markdown
---
chapter_id: "chapter-01"
generated_at: "2025-11-24T14:00:00Z"
generated_by: "summary-agent"
lesson_count: 9
token_count: 1500
---

# Chapter 01 Summary: Introduction to AI-Native Development

[Summary content generated by external agent...]

## Key Concepts Covered
- AI agent architectures
- MCP protocol fundamentals
- Specification-driven development

## Learning Progression
[Agent-generated synthesis of lessons 01-09]
```

### TypeScript Interface

```typescript
interface Summary {
  // Frontmatter metadata
  metadata: SummaryMetadata;

  // Content
  markdown: string;

  // Storage metadata
  storage_info: StorageInfo; // Same as Lesson
}

interface SummaryMetadata {
  chapter_id: string;
  generated_at: string;     // ISO 8601
  generated_by: string;     // Agent ID
  lesson_count: number;     // Lessons summarized
  token_count: number;      // Approximate content length
}
```

### Validation Rules

- **Filename**: Must be `.summary.md` (hidden file convention)
- **Chapter Association**: `chapter_id` in metadata must match parent directory
- **Optional**: Chapters may not have summaries (external agent responsibility)
- **Regeneration**: Summaries can be overwritten when lessons change (via `update_summary` tool)

### Operations

- **Create**: `add_summary` MCP tool (writes markdown file)
- **Read**: `get_summary` MCP tool (returns markdown + metadata)
- **Update**: `update_summary` MCP tool (overwrites existing file)
- **List**: `list_summaries` MCP tool (filter by chapter_id optional)
- **Auto-Trigger** (Not in MVP): `add_content` / `update_content` could trigger summary regeneration workflow

---

## Entity 6: Audit Entry

### Description
Immutable JSONL record of every operation performed on PanaversityFS. Used for debugging, compliance, and agent behavior analysis.

### File Path Pattern

```
.audit/{YYYY-MM-DD}.jsonl
```

**Example**: `.audit/2025-11-24.jsonl`

### JSONL Record Schema

```json
{
  "timestamp": "2025-11-24T12:34:56.789Z",
  "agent_id": "content-implementer-haiku",
  "operation": "update_content",
  "path": "books/ai-native-software-development/lessons/part-1/chapter-01/lesson-01.md",
  "status": "success",
  "execution_time_ms": 145,
  "file_hash_before": "abc123...",
  "file_hash_after": "def456...",
  "error_message": null
}
```

### TypeScript Interface

```typescript
interface AuditEntry {
  // Temporal info
  timestamp: string;        // ISO 8601 with milliseconds
  agent_id: string;         // OAuth user ID or agent identifier

  // Operation details
  operation: Operation;
  path: string;             // Resource path (lesson, asset, summary)
  status: 'success' | 'error' | 'conflict';

  // Performance metrics
  execution_time_ms: number;

  // Optional context
  file_hash_before?: string;  // For update operations
  file_hash_after?: string;   // For create/update operations
  error_message?: string;     // If status === 'error'
  error_stack?: string;       // Full stack trace (sent to Sentry, not in audit log)
}

type Operation =
  | 'add_content'
  | 'update_content'
  | 'read_content'
  | 'delete_content'
  | 'add_asset'
  | 'get_asset'
  | 'list_assets'
  | 'add_summary'
  | 'update_summary'
  | 'get_summary'
  | 'list_summaries'
  | 'get_book_archive'
  | 'list_books'
  | 'add_book_to_registry'
  | 'glob_search'
  | 'grep_search';
```

### Validation Rules

- **Immutability**: Once written, audit entries cannot be modified (append-only log)
- **Daily Rotation**: New file created at 00:01 UTC each day
- **Atomic Append**: Each entry written as single line (no partial writes)
- **Eventual Consistency**: Race conditions acceptable per FR-018 (concurrent writes may interleave)

### Query Filters (per FR-020)

```typescript
interface AuditFilters {
  date_range: string[];     // Array of dates: ["2025-11-24", "2025-11-25"]
  agent_id?: string;        // Filter by specific agent
  operation_type?: Operation; // Filter by operation
  status?: 'success' | 'error' | 'conflict';
}
```

### Operations

- **Write**: Automatic on every MCP tool invocation (5-20ms latency per research.md)
- **Query**: `get_audit_log` MCP tool (scans JSONL files, filters in-memory)
- **Pagination**: Max 100 entries per response with continuation token (per FR-020)
- **Rotation**: Cron job creates new daily file at 00:01 UTC
- **No Delete**: Audit logs retained indefinitely (manual archival documented in ops guide)

---

## Relationships

### Book → Lessons (1:N)

- **Cardinality**: 1 Book contains 0-500 Lessons
- **Referential Integrity**: Lesson metadata (`book_id`, `part_id`, `chapter_id`) must match book directory structure
- **Cascade Delete**: Deleting book deletes all lessons (admin operation, not exposed via MCP)
- **Orphan Detection**: Startup check validates all lessons reference valid book (per spec edge case handling)

### Book → Assets (1:N)

- **Cardinality**: 1 Book contains 0-∞ Assets
- **Referential Integrity**: Asset `book_id` must match registry entry
- **Cascade Delete**: Deleting book deletes all assets
- **CDN URL Generation**: Deterministic from `book_id`, `asset_type`, `filename`

### Chapter → Summary (1:0..1)

- **Cardinality**: 1 Chapter may have 0 or 1 Summary
- **Optional**: Summaries created by external agents, not required for chapter validity
- **Regeneration**: `update_summary` overwrites existing summary (idempotent)
- **Orphan Detection**: Summaries without corresponding chapter logged as warning (not error)

### Operation → Audit Entry (1:1)

- **Cardinality**: Every operation produces exactly 1 audit entry
- **Write Guarantee**: Even if operation fails, audit entry written with `status: 'error'`
- **Traceability**: `agent_id` + `timestamp` + `operation` + `path` uniquely identify operation

---

## Storage Backend Mapping

### Directory Structure

```
panaversity-storage/
├── registry.yaml                       # Registry entity
├── .audit/                             # Audit entries by date
│   ├── 2025-11-24.jsonl
│   └── 2025-11-25.jsonl
├── .temp/                              # Temporary archives
│   └── ai-native-software-development-1732454321.zip
└── books/
    ├── ai-native-software-development/
    │   ├── book.yaml                  # Book entity
    │   ├── chapters/
    │   │   ├── chapter-01/
    │   │   │   └── .summary.md        # Summary entity
    │   │   └── chapter-02/
    │   │       └── .summary.md
    │   ├── lessons/
    │   │   ├── part-1/
    │   │   │   ├── chapter-01/
    │   │   │   │   ├── lesson-01.md   # Lesson entity
    │   │   │   │   ├── lesson-02.md
    │   │   │   │   └── ...
    │   │   │   └── chapter-02/
    │   │   │       └── ...
    │   │   └── part-2/
    │   │       └── ...
    │   └── assets/
    │       ├── slides/
    │       │   └── chapter-01-intro.pdf # Asset entity
    │       ├── images/
    │       │   └── architecture.png
    │       ├── videos/
    │       │   └── demo.mp4
    │       └── audio/
    │           └── lecture.mp3
    └── generative-ai-fundamentals/
        └── [same structure]
```

### Backend-Specific Considerations

**Local Filesystem** (`storage_backend: "local"`):
- Standard POSIX filesystem operations
- No CDN (files served via relative paths in development)

**Cloudflare R2** (`storage_backend: "r2"`):
- Object keys map to paths (e.g., `books/ai-native-software-development/book.yaml`)
- Public bucket for assets (CDN URL = R2 public URL)
- Presigned URLs for archives

**AWS S3** (`storage_backend: "s3"`):
- Bucket per book or single bucket with prefixes
- CloudFront CDN for asset URLs
- S3 presigned URLs for archives

**Supabase Storage** (`storage_backend: "supabase"`):
- Bucket per book
- Supabase CDN for public assets
- Presigned URLs via Supabase API

---

## Migration & Versioning

### Schema Version

Current data model version: **1.0**

All YAML files include `version` field for forward compatibility:

```yaml
version: "1.0"
```

### Migration Path (Git → PanaversityFS)

**Step 1**: Read existing Git repository structure
**Step 2**: Map to PanaversityFS directory layout
**Step 3**: Create registry entry
**Step 4**: Initialize book.yaml
**Step 5**: Upload lessons preserving directory structure
**Step 6**: Upload assets to categorized subdirectories
**Step 7**: Generate initial audit log entry

**Tooling**: CLI script `panaversity-fs migrate ./repo --to r2 --book-id ai-native-software-development`

### Data Model Evolution

**Breaking Changes** (require migration):
- Adding required fields to YAML schemas
- Changing directory structure
- Renaming entity keys

**Non-Breaking Changes** (backward compatible):
- Adding optional fields
- Adding new asset types
- Extending audit entry with optional fields

---

## Appendix: Example Queries

### Query 1: Find all lessons in Chapter 01

```typescript
const pattern = 'books/ai-native-software-development/lessons/*/chapter-01/*.md';
const lessons = await glob_search({ book_id, pattern });
```

### Query 2: Get audit log for specific agent

```typescript
const filters: AuditFilters = {
  date_range: ['2025-11-24'],
  agent_id: 'content-implementer-haiku'
};
const entries = await get_audit_log(filters);
```

### Query 3: List all slide assets

```typescript
const assets = await list_assets({
  book_id: 'ai-native-software-development',
  asset_type: 'slides'
});
```

### Query 4: Search for "OpenDAL" across all lessons

```typescript
const matches = await grep_search({
  book_id: 'ai-native-software-development',
  pattern: 'OpenDAL',
  all_books: false
});
// Returns: [{ file: "lessons/part-4/chapter-12/lesson-03.md", line: 42, content: "..." }]
```
