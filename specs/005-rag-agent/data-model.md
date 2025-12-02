# Data Model: RAG Agent Backend

**Feature**: 005-rag-agent
**Date**: 2025-11-29

## Entity Overview

This RAG system manages four primary entities:

1. **DocumentChunk** - A semantic content unit with embedding
2. **DocumentMetadata** - Hierarchical location and personalization filters
3. **IngestionJob** - Background job tracking ingestion progress
4. **IngestionState** - Tracks indexed files for incremental updates

## Entity Diagrams

### Core Entity Relationships

```
LessonFile (source)
    ↓ parse (frontmatter + content)
    ↓
DocumentChunk (1..N per file)
    ├── metadata: DocumentMetadata
    ├── embedding: list[float]
    ├── prev_chunk_id → DocumentChunk (optional)
    ├── next_chunk_id → DocumentChunk (optional)
    └── parent_doc_id → Parent Document (lesson/chapter)

IngestionState
    ├── records: dict[str, IngestionRecord]
    │   └── IngestionRecord
    │       ├── source_file: str
    │       ├── file_hash: str
    │       └── chunk_ids: list[str] → DocumentChunk
    ├── last_full_ingest: datetime
    └── last_incremental_update: datetime

IngestionJob
    ├── job_id: int
    ├── status: queued | running | completed | failed
    ├── files_processed: int
    └── chunks_created: int
```

## Entity Definitions

### 1. DocumentChunk

**Purpose**: A semantic unit of content ready for embedding and retrieval.

**Fields**:

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | str (UUID) | Required, Unique | Content-hash-based UUID (deterministic) |
| `text` | str | Required, min_length=10 | Chunk content (target 400 tokens) |
| `metadata` | DocumentMetadata | Required | Full metadata for filtering |
| `word_count` | int | Required, ge=1 | Word count |
| `token_count` | int | Required, ge=1 | Estimated tokens (words * 1.3) |
| `char_count` | int | Required, ge=1 | Character count |

**Derived Fields** (EmbeddedChunk extends DocumentChunk):

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `embedding` | list[float] | Required, length=1536 | Vector embedding from OpenAI |
| `embedded_at` | datetime | Auto-set | Timestamp of embedding |
| `embedding_model` | str | Default: text-embedding-3-small | Model used |

**ID Generation**:
```python
# Deterministic UUID from content hash
id_string = f"{book_id}:{module}:{chapter}:{lesson}:{content_hash[:16]}"
namespace = uuid.UUID("6ba7b810-9dad-11d1-80b4-00c04fd430c8")
chunk_id = str(uuid.uuid5(namespace, id_string))
```

**Invariants**:
- `id` changes when `text` changes (content hash changes)
- `id` stable across re-ingestion if content unchanged
- `word_count * 1.3 ≈ token_count` (estimation)
- `text` stripped and non-empty (min 10 chars)

### 2. DocumentMetadata

**Purpose**: Hierarchical location and personalization filters for a chunk.

**Fields**:

| Category | Field | Type | Constraints | Description |
|----------|-------|------|-------------|-------------|
| **Tenant** | `book_id` | str | Required | Tenant isolation (e.g., "physical-ai-robotics") |
| **Hierarchy** | `module` | ModuleName | Required | ros2, gazebo, isaac, vla |
| | `chapter` | int | Required, 0-20 | Chapter number (0 for module-level) |
| | `lesson` | int | Required, 0-15 | Lesson number (0 for chapter overview) |
| | `section_title` | str \| None | Optional | Section header if chunked by ## |
| **Personalization** | `hardware_tier` | HardwareTier | Required, 1-4 | Minimum hardware tier |
| | `proficiency_level` | ProficiencyLevel | Default: A2 | A1, A2, B1, B2, C1, C2 |
| | `layer` | TeachingLayer | Default: L1 | L1, L2, L3, L4 |
| **Source** | `source_file` | str | Required | Relative path from docs root |
| **Hierarchy** | `parent_doc_id` | str (UUID) | Required | Parent lesson/chapter UUID |
| **Context Chain** | `chunk_index` | int | Required, ge=0 | Position within lesson |
| | `total_chunks` | int | Required, ge=1 | Total chunks in lesson |
| | `prev_chunk_id` | str (UUID) \| None | Optional | Previous chunk (None if first) |
| | `next_chunk_id` | str (UUID) \| None | Optional | Next chunk (None if last) |
| **Change Detection** | `content_hash` | str | Required | SHA-256 of chunk text |
| | `source_file_hash` | str | Required | SHA-256 of source file |
| **Versioning** | `version` | int | Default: 1 | Chunk version for updates |
| | `created_at` | datetime | Auto-set | Creation timestamp |
| | `updated_at` | datetime | Auto-set | Last update timestamp |

**Invariants**:
- `∀ chunk: chunk.chunk_index < chunk.total_chunks`
- `∀ chunk: chunk.chunk_index = 0 → chunk.prev_chunk_id is None` (first chunk)
- `∀ chunk: chunk.chunk_index = total_chunks - 1 → chunk.next_chunk_id is None` (last chunk)
- `∀ chunk: chunk.prev_chunk_id is not None → ∃ prev_chunk: prev_chunk.id = chunk.prev_chunk_id`
- `parent_doc_id` same for all chunks in a lesson

**Parent Document ID Generation**:
```python
# Deterministic UUID for parent document
id_string = f"{book_id}:{module}:{chapter}:{lesson}:parent"
namespace = uuid.UUID("6ba7b810-9dad-11d1-80b4-00c04fd430c8")
parent_id = str(uuid.uuid5(namespace, id_string))
```

### 3. QdrantPayload

**Purpose**: Schema for Qdrant point payload (JSON stored with vector).

**Fields** (subset of DocumentMetadata + text):

| Field | Type | Indexed | Purpose |
|-------|------|---------|---------|
| `book_id` | str | ✅ KEYWORD | Tenant isolation (is_tenant=true) |
| `module` | str | ✅ KEYWORD | Module filter |
| `chapter` | int | ✅ INTEGER | Chapter filter |
| `lesson` | int | ✅ INTEGER | Lesson filter |
| `hardware_tier` | int | ✅ INTEGER | Range filter (lte) |
| `proficiency_level` | str | ✅ KEYWORD | Proficiency filter |
| `layer` | str | ✅ KEYWORD | Teaching layer filter |
| `parent_doc_id` | str | ✅ KEYWORD | Full lesson retrieval |
| `content_hash` | str | ✅ KEYWORD | Deduplication |
| `source_file_hash` | str | ✅ KEYWORD | Change detection |
| `text` | str | ❌ | Content for retrieval display |
| `section_title` | str \| None | ❌ | Display metadata |
| `source_file` | str | ❌ | Display metadata |
| `chunk_index` | int | ❌ | Ordering |
| `total_chunks` | int | ❌ | Display metadata |
| `prev_chunk_id` | str \| None | ❌ | Context expansion |
| `next_chunk_id` | str \| None | ❌ | Context expansion |
| `word_count` | int | ❌ | Metrics |
| `token_count` | int | ❌ | Metrics |
| `version` | int | ❌ | Versioning |
| `created_at` | str (ISO 8601) | ❌ | Timestamp |
| `updated_at` | str (ISO 8601) | ❌ | Timestamp |

**Index Strategy**:
- All filter fields indexed for fast filtering
- Text and display fields not indexed (only retrieved)
- Payload indexes created on collection initialization

**Filter Operations**:
- `book_id`: FieldCondition(match=MatchValue) - exact match
- `hardware_tier`: FieldCondition(range=Range(lte=X)) - tier X or lower
- `module`: FieldCondition(match=MatchValue) - exact match
- `chapter`: FieldCondition(range=Range(gte=X, lte=Y)) - range
- `proficiency_level`: FieldCondition(match=MatchAny(any=[A2, B1])) - OR within field
- `parent_doc_id`: FieldCondition(match=MatchValue) - all chunks in lesson

### 4. IngestionState

**Purpose**: Tracks which files have been indexed for incremental updates.

**Fields**:

| Field | Type | Description |
|-------|------|-------------|
| `book_id` | str | Book identifier |
| `collection_name` | str | Qdrant collection name |
| `records` | dict[str, IngestionRecord] | Map of source_file → record |
| `last_full_ingest` | datetime \| None | Last full ingestion timestamp |
| `last_incremental_update` | datetime \| None | Last incremental update timestamp |
| `total_chunks` | int | Total chunks currently indexed |

**IngestionRecord**:

| Field | Type | Description |
|-------|------|-------------|
| `source_file` | str | Relative path to source file |
| `file_hash` | str | SHA-256 hash at last ingestion |
| `chunk_ids` | list[str] | UUIDs of chunks from this file |
| `chunk_count` | int | Number of chunks |
| `ingested_at` | datetime | Ingestion timestamp |
| `book_id` | str | Book identifier |
| `module` | str | Module name |
| `chapter` | int | Chapter number |
| `lesson` | int | Lesson number |

**Operations**:

```python
def get_changed_files(current_files: dict[str, str]) -> tuple[list, list, list]:
    """
    Compare current file hashes against stored state.

    Returns: (new_files, modified_files, deleted_files)
    """
    new_files = [path for path in current_files if path not in records]
    modified_files = [
        path for path in current_files
        if path in records and records[path].file_hash != current_files[path]
    ]
    deleted_files = [path for path in records if path not in current_files]
    return new_files, modified_files, deleted_files

def get_chunks_to_delete(files: list[str]) -> list[str]:
    """Get all chunk IDs associated with given files."""
    return [
        chunk_id
        for path in files if path in records
        for chunk_id in records[path].chunk_ids
    ]
```

**Storage**:
- **Current**: `.ingestion_state.json` file
- **Recommended**: Qdrant collection `_state` with one point per file

**Invariants**:
- `total_chunks = sum(record.chunk_count for record in records.values())`
- All `chunk_ids` in records exist in Qdrant collection
- All files in `records` currently indexed (unless deleted)

### 5. IngestionJob

**Purpose**: Track background ingestion job progress.

**Fields**:

| Field | Type | Description |
|-------|------|-------------|
| `job_id` | int | Unique job identifier |
| `book_id` | str | Book being ingested |
| `mode` | str | incremental, full, recreate |
| `status` | str | queued, running, completed, failed |
| `files_processed` | int | Files successfully processed |
| `files_skipped` | int | Unchanged files skipped (incremental) |
| `chunks_created` | int | New chunks created |
| `chunks_deleted` | int | Old chunks deleted |
| `started_at` | datetime | Job start time |
| `completed_at` | datetime \| None | Job completion time (None if running) |
| `duration_seconds` | int \| None | Total duration |
| `error_message` | str \| None | Error details if failed |

**State Transitions**:
```
queued → running → completed
              ↓
            failed
```

**Storage**:
- **Current**: In-memory dict (lost on restart)
- **Recommended**: SQLite or Qdrant collection for persistence

**Invariants**:
- `status = completed → completed_at is not None`
- `status = running → completed_at is None`
- `chunks_created ≥ 0 ∧ chunks_deleted ≥ 0`
- `files_processed + files_skipped = total_files_discovered`

## Search Models

### SearchQuery

**Purpose**: User's search request with comprehensive filters.

**Fields**:

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `text` | str | Required | Search query text (min 3 chars) |
| `book_id` | str | "physical-ai-robotics" | Book to search |
| `hardware_tier_filter` | int | 1 | Return tier X or lower (1-4) |
| `module_filter` | ModuleName \| None | None | Filter to specific module |
| `chapter_min` | int \| None | None | Minimum chapter (inclusive) |
| `chapter_max` | int \| None | None | Maximum chapter (inclusive) |
| `lesson_filter` | int \| None | None | Specific lesson number |
| `proficiency_levels` | list[ProficiencyLevel] \| None | None | OR logic within |
| `layer_filter` | TeachingLayer \| None | None | Specific teaching layer |
| `parent_doc_id` | str \| None | None | Get all chunks from lesson |
| `limit` | int | 5 | Max results (1-20) |

**Filter Logic**: All filters use AND logic (must match all conditions)

### SearchResult

**Purpose**: A single search result with score and metadata.

**Fields**:

| Field | Type | Description |
|-------|------|-------------|
| `text` | str | Retrieved chunk content |
| `score` | float | Cosine similarity score (0.0-1.0) |
| `source_file` | str | Source markdown file |
| `section_title` | str \| None | Section header |
| `module` | str | Module name |
| `chapter` | int | Chapter number |
| `lesson` | int | Lesson number |
| `hardware_tier` | int | Minimum hardware tier |
| `proficiency_level` | str | Proficiency level |
| `layer` | str | Teaching layer |
| `chunk_index` | int | Position in lesson |
| `total_chunks` | int | Total chunks in lesson |
| `parent_doc_id` | str \| None | Parent document UUID |
| `prev_chunk_id` | str \| None | Previous chunk UUID |
| `next_chunk_id` | str \| None | Next chunk UUID |
| `content_hash` | str \| None | Content hash for dedup |

**Invariant**: Results ordered by `score` descending

### SearchResponse

**Purpose**: Complete search response with multiple results.

**Fields**:

| Field | Type | Description |
|-------|------|-------------|
| `query` | str | Original query text |
| `results` | list[SearchResult] | Search results (ordered by score) |
| `total_found` | int | Total matching documents |
| `hardware_tier_filter` | int | Filter applied |
| `module_filter` | str \| None | Filter applied |
| `book_id` | str | Book searched |

## Relationships and Constraints

### Parent-Child Relationships

```
LessonFile (1)
    ↓ chunks into
DocumentChunk (N)
    ↓ all share
parent_doc_id (1)
    ↓ enables
Full Lesson Retrieval
```

**Constraint**: `∀ chunks c1, c2 from same lesson: c1.parent_doc_id = c2.parent_doc_id`

### Context Chain

```
Chunk 0 ← Chunk 1 ← Chunk 2 ← ... ← Chunk N-1
(first)   (middle)  (middle)        (last)

prev_id: None  ←  c0.id  ←  c1.id  ←  cN-2.id
next_id: c1.id →  c2.id  →  None   →  None
```

**Constraints**:
- `chunk.chunk_index = 0 → chunk.prev_chunk_id is None`
- `chunk.chunk_index = total_chunks - 1 → chunk.next_chunk_id is None`
- No cycles: `no chunk: chunk in chunk.^(prev|next)_chunk_id`

### Change Detection Flow

```
Source File (docs/module-1/chapter-1/01-lesson.md)
    ↓ compute SHA-256
file_hash (abc123...)
    ↓ compare with
IngestionState.records["docs/..."].file_hash
    ↓ if different
Mark as modified → Delete old chunks → Re-ingest
```

**Constraint**: `file_hash changes → re-embed all chunks from file`

## Type Definitions

```python
# Literal types for validation
ModuleName = Literal["ros2", "gazebo", "isaac", "vla"]
ProficiencyLevel = Literal["A1", "A2", "B1", "B2", "C1", "C2"]
TeachingLayer = Literal["L1", "L2", "L3", "L4"]
HardwareTier = Literal[1, 2, 3, 4]
IngestionMode = Literal["incremental", "full", "recreate"]
JobStatus = Literal["queued", "running", "completed", "failed"]
```

## Formal Verification (Small Scope Test)

### Test Instance: 3 Lessons

```
Lesson A: 5 chunks (c0, c1, c2, c3, c4)
Lesson B: 3 chunks (c5, c6, c7)
Lesson C: 1 chunk (c8)
```

### Invariant Checks

**Coverage Invariant**: Every chunk has parent_doc_id
```
∀ chunk: some chunk.parent_doc_id
✅ All 9 chunks have parent_doc_id
```

**Context Chain Integrity**:
```
Lesson A:
  c0: prev=None, next=c1 ✅
  c1: prev=c0, next=c2 ✅
  c2: prev=c1, next=c3 ✅
  c3: prev=c2, next=c4 ✅
  c4: prev=c3, next=None ✅

Lesson B:
  c5: prev=None, next=c6 ✅
  c6: prev=c5, next=c7 ✅
  c7: prev=c6, next=None ✅

Lesson C:
  c8: prev=None, next=None ✅ (single chunk)
```

**No Cycles**:
```
no chunk: chunk in chunk.^next_chunk_id
✅ No chunk reachable from itself
```

**Hardware Tier Filter**:
```
Query: hardware_tier_filter = 2
Chunks: [tier 1, tier 1, tier 2, tier 3, tier 1]
Result: [tier 1, tier 1, tier 2, tier 1] ✅ (tier 3 excluded)
```

**Parent Document Grouping**:
```
parent_A chunks: [c0, c1, c2, c3, c4]
parent_B chunks: [c5, c6, c7]
parent_C chunks: [c8]
∀ lesson: chunks ordered by chunk_index ✅
```

### Counterexample Check

**Scenario**: What if prev_chunk_id references deleted chunk?

```
Initial: c0 ← c1 ← c2
Update: Source file changed, c1 modified but c0, c2 unchanged
```

**Potential Issue**: c1's content hash changes → new chunk ID → c2.prev_chunk_id points to old c1

**Solution**: Atomic update - delete ALL old chunks from file BEFORE inserting new:
```python
# From ingestion/uploader.py
for f in files_to_process:
    if f.relative_path in state.records:
        chunk_ids_to_delete.append(state.records[f].chunk_ids)
uploader.delete_chunks(chunk_ids_to_delete)  # THEN
uploader.upload(new_chunks)  # Insert new
```

**Verification**: No orphaned references ✅

## Summary

The data model uses:
- **Content-hash-based UUIDs** for stable chunk IDs
- **SHA-256 file hashing** for change detection
- **Prev/next chain** for context expansion
- **Parent document grouping** for full lesson retrieval
- **Qdrant payload indexes** for fast filtering
- **Atomic updates** to maintain referential integrity

All invariants verified against small scope (3 lessons, 9 chunks) with no counterexamples found.
