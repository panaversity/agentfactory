# Research: RAG Agent Backend

**Feature**: 005-rag-agent
**Date**: 2025-11-29
**Research Phase**: Technology decisions and architecture patterns

## Technology Stack (Pre-Selected)

| Component | Technology | Version | Rationale |
|-----------|------------|---------|-----------|
| **Vector Database** | Qdrant Cloud | Latest | - Hosted solution (no ops overhead)<br>- Excellent filtering capabilities<br>- Payload indexing for multi-dimensional filters |
| **Embeddings** | OpenAI text-embedding-3-small | Latest | - 1536 dimensions (optimal for retrieval)<br>- Cost-effective ($0.02/1M tokens)<br>- Production-proven quality |
| **API Framework** | FastAPI | 0.104+ | - Async support (non-blocking I/O)<br>- Automatic OpenAPI docs<br>- Pydantic integration |
| **Type Safety** | Pydantic | 2.0+ | - Runtime validation<br>- Settings management<br>- Serialization |
| **HTTP Client** | httpx | Latest | - Async/await support<br>- Used by OpenAI SDK |
| **Deployment** | Google Cloud Run | N/A | - Serverless (auto-scaling)<br>- Pay-per-request<br>- Docker-based |

## Existing Implementation Analysis

### What Already Exists (backend/ directory)

The project has a **working RAG implementation** with the following components:

#### 1. Ingestion Pipeline (Complete)

**Location**: `backend/ingestion/`

| Component | File | Status | Notes |
|-----------|------|--------|-------|
| **Crawler** | `crawler.py` | ✅ Complete | - Discovers markdown files in docs/<br>- Extracts module/chapter/lesson from path<br>- Pattern: `module-N-*/chapter-N-*/*.md` |
| **Parser** | `parser.py` | ✅ Complete | - Parses YAML frontmatter<br>- Normalizes proficiency/layer/tier<br>- Computes file hash for change detection |
| **Chunker** | `chunker.py` | ✅ Complete | - Semantic chunking by ## headers<br>- 400-512 token target<br>- 15% overlap between chunks<br>- Prev/next relationship tracking |
| **Embedder** | `embedder.py` | ✅ Complete | - OpenAI embedding API integration<br>- Batch processing (20 chunks/batch)<br>- Progress display with rich |
| **Uploader** | `uploader.py` | ✅ Complete | - Qdrant Cloud integration<br>- Incremental update support<br>- State persistence (.ingestion_state.json) |

**Key Design Patterns**:
- **Content-hash-based UUIDs**: Chunk IDs deterministic from content (not position)
- **Incremental updates**: SHA-256 file hashing detects changed files
- **Atomic operations**: Delete old chunks before inserting new
- **State tracking**: JSON file tracks indexed files and their hashes

#### 2. Data Models (Complete)

**Location**: `backend/models/`

| Model | File | Purpose |
|-------|------|---------|
| `DocumentChunk` | `document.py` | Core chunk with text, metadata, metrics |
| `DocumentMetadata` | `document.py` | Hierarchical location + personalization filters |
| `EmbeddedChunk` | `document.py` | Chunk + embedding vector |
| `QdrantPayload` | `document.py` | Qdrant point payload schema |
| `IngestionState` | `document.py` | Tracks indexed files for incremental updates |
| `SearchQuery` | `search.py` | Request with comprehensive filters |
| `SearchResult` | `search.py` | Result with metadata |
| `SearchResponse` | `search.py` | Response with results list |

**Key Fields**:
- **Tenant isolation**: `book_id` field
- **Hardware filtering**: `hardware_tier` (1-4)
- **Pedagogical filters**: `proficiency_level` (A1-C2), `layer` (L1-L4)
- **Context expansion**: `prev_chunk_id`, `next_chunk_id`, `parent_doc_id`

#### 3. Retrieval Service (Complete)

**Location**: `backend/retrieval/search.py`

**Capabilities**:
- Semantic search with query embedding
- Multi-dimensional filtering (book_id, hardware_tier, module, chapter, lesson, proficiency, layer)
- Context expansion (walk prev/next chain)
- Full lesson retrieval (all chunks by parent_doc_id)

**Filter Strategy**:
- All filters use AND logic
- `hardware_tier`: Range(lte) - returns tier X or lower
- `proficiency_levels`: MatchAny - OR within field
- `chapter_range`: Range(gte, lte)

#### 4. API Endpoints (Partial)

**Location**: `backend/api/`

**Existing**:
- ✅ `POST /search` - Semantic search
- ✅ `GET /health` - Health check
- ✅ `GET /info` - Collection statistics
- ✅ `GET /context/{chunk_id}` - Context expansion
- ✅ `GET /lesson/{parent_doc_id}` - Full lesson retrieval
- ✅ `POST /ingestion/trigger` - Manual ingestion trigger
- ✅ `GET /ingestion/status/{job_id}` - Job status
- ✅ `GET /ingestion/changes` - Change detection preview
- ✅ `POST /ingestion/webhook/github` - GitHub webhook

**Implementation Notes**:
- FastAPI with async lifespan for client initialization
- CORS configured (needs production tightening)
- In-memory job tracking (should migrate to DB for production)
- Admin API key protection for ingestion endpoints
- Background tasks for long-running ingestion

#### 5. Configuration (Complete)

**Location**: `backend/config.py`

Uses `pydantic-settings` for environment variable management:
- Qdrant credentials
- OpenAI API key
- Book configuration (book_id, docs_path)
- Collection settings (name, embedding model, dimensions)
- Chunking parameters (max/min words)
- Database URL (for future use)
- Admin API key

### What Needs to Be Created

#### 1. State Management Enhancement

**Current**: `.ingestion_state.json` file (works but not production-ready)

**Spec Requirement**: FR-010 supports three ingestion modes (incremental, full, recreate)

**Gap**: Need better state tracking for:
- Multiple concurrent jobs
- Job history
- Error recovery

**Solution**: Migrate to Qdrant-native state tracking (store state in separate collection)

#### 2. API Route Organization

**Current**: Mixed in `api/main.py` and `api/ingestion.py`

**Spec Requirement**: Clean separation by domain

**Gap**: Need explicit route files for:
- `api/routes/search.py` - Search endpoints
- `api/routes/context.py` - Context expansion
- `api/routes/lesson.py` - Full lesson retrieval
- `api/routes/health.py` - Health/info
- `api/routes/ingestion.py` - Already exists but needs refinement

#### 3. Testing Infrastructure

**Current**: None

**Spec Requirement**: Automated tests for Eval-001 through Eval-005

**Gap**: Need:
- Unit tests for each component
- Integration tests for pipeline
- Contract tests for API endpoints
- Test fixtures with sample data

#### 4. Deployment Configuration

**Current**: None

**Spec Requirement**: Google Cloud Run deployment

**Gap**: Need:
- Dockerfile
- Cloud Run configuration
- Environment variable setup
- Health check configuration

## Architecture Patterns Extracted

### Pattern 1: Content-Hash-Based IDs

**Source**: `models/document.py:generate_id()`

```python
# Deterministic UUID from content hash
id_string = f"{book_id}:{module}:{chapter}:{lesson}:{content_hash[:16]}"
namespace = uuid.UUID("6ba7b810-9dad-11d1-80b4-00c04fd430c8")
return str(uuid.uuid5(namespace, id_string))
```

**Benefits**:
- Same content → same ID (idempotent)
- Changed content → new ID (forces re-embedding)
- Position changes don't affect ID (stable references)

### Pattern 2: Incremental Update Flow

**Source**: `ingestion/uploader.py`

```
1. Crawl docs → compute file hashes
2. Load state → compare hashes
3. Identify: new_files, modified_files, deleted_files
4. Delete old chunks (by chunk IDs from state)
5. Parse + chunk + embed + upload new/modified
6. Update state with new file hashes and chunk IDs
```

### Pattern 3: Rich Progress Display

**Source**: `ingestion/embedder.py`, `ingestion/uploader.py`

All long-running operations use `rich.progress`:
- Spinner column
- Text description
- Progress bar
- Task progress percentage

### Pattern 4: Settings Management

**Source**: `config.py`

```python
class Settings(BaseSettings):
    qdrant_url: str = Field(..., description="...")

    model_config = {
        "env_file": ".env",
        "env_file_encoding": "utf-8",
        "extra": "ignore"
    }

@lru_cache
def get_settings() -> Settings:
    return Settings()
```

**Benefits**:
- Cached settings (single read)
- Type-safe with validation
- Auto-loads from .env

### Pattern 5: Qdrant Payload Indexing

**Source**: `ingestion/uploader.py:_create_indexes()`

All filterable fields get payload indexes:
- `book_id` (KEYWORD) - tenant isolation
- `module` (KEYWORD)
- `hardware_tier` (INTEGER)
- `chapter`, `lesson` (INTEGER)
- `proficiency_level`, `layer` (KEYWORD)
- `parent_doc_id`, `content_hash`, `source_file_hash` (KEYWORD)

**Performance Impact**: Enables fast filtered retrieval without full collection scan

## Research Artifacts Referenced

### Chunking Strategy

**Sources**:
- NVIDIA benchmark: 400-512 tokens optimal
- Weaviate best practices: 10-20% overlap
- Unstructured.io: Semantic boundaries over fixed character counts

**Implementation**: `ingestion/chunker.py`
- Split on ## headers (semantic boundaries)
- Target 400 tokens (~308 words at 1.3 tokens/word)
- 15% overlap between consecutive chunks
- Merge small chunks, split large sections

### Change Detection

**Source**: Particula.tech blog on RAG updates

**Implementation**: SHA-256 hashing
- File hash: Detects if file changed
- Content hash: Detects if chunk changed
- UUID from content hash: Stable IDs across re-ingestion

### Qdrant Best Practices

**Source**: Qdrant documentation

**Implementation**:
- Use `upsert` (idempotent)
- Batch operations (100 points/batch)
- Payload indexes for all filters
- `wait=True` for consistency

## Gaps Between Spec and Implementation

### 1. Edge Case Handling

**Spec FR-016a/FR-016b**: Context expansion must handle first/last chunk gracefully

**Current**: Implementation returns `None` for missing prev/next IDs ✅

**Status**: Already handled correctly

### 2. State Persistence

**Spec**: Production-ready state tracking

**Current**: `.ingestion_state.json` file works but isn't ideal for serverless

**Gap**: Consider Qdrant-native state (separate collection) or SQLite for multi-instance consistency

### 3. Job Tracking

**Spec FR-023**: GET /ingestion/status/{job_id}

**Current**: In-memory `_jobs` dict (lost on restart)

**Gap**: Need persistent job storage (SQLite or separate Qdrant collection)

### 4. GitHub Webhook Signature

**Spec FR-024**: GitHub webhook with signature verification

**Current**: Implemented but uses `admin_api_key` as secret

**Gap**: Should use separate `GITHUB_WEBHOOK_SECRET` environment variable

### 5. Rate Limiting

**Spec Edge Case**: "What happens when embedding API rate limit is reached?"

**Current**: No retry logic

**Gap**: Need exponential backoff for OpenAI API calls

## Dependencies Analysis

### Production Dependencies

From `backend/requirements.txt`:

```
fastapi>=0.104.0
uvicorn[standard]>=0.24.0
pydantic>=2.0.0
pydantic-settings>=2.0.0
qdrant-client>=1.7.0
openai>=1.0.0
python-frontmatter>=1.0.0
python-multipart>=0.0.6
rich>=13.0.0
```

**Analysis**: All necessary dependencies present

### Additional Needs

For production deployment:
- `pytest>=7.4.0` (testing)
- `pytest-asyncio>=0.21.0` (async tests)
- `httpx>=0.25.0` (test client)
- `gunicorn` (production WSGI server - though uvicorn is sufficient)

## Performance Baseline

**From Spec Success Criteria**:
- SC-001: Search latency under 500ms (p95)
- SC-003: Full ingestion of 500+ chunks in under 5 minutes
- SC-007: Handle 100 concurrent search requests

**Current Performance** (estimated from code):
- Embedding: 20 chunks/batch → ~30 batches for 500 chunks → ~60 seconds
- Upload: 100 points/batch → ~5 batches → ~10 seconds
- **Total ingestion**: ~2-3 minutes ✅ (within spec)

**Search**: Qdrant Cloud with payload indexes → typically under 100ms ✅

## Recommendations

### 1. Use Existing Implementation as Foundation

**Rationale**: The current backend is **production-quality**:
- Clean architecture (separation of concerns)
- Type-safe with Pydantic
- Rich progress display
- Incremental updates working
- Comprehensive filtering

**Action**: Enhance and organize, don't rebuild

### 2. Organize API Routes

**Current**: Mixed in `main.py`

**Recommended Structure**:
```
api/
├── main.py              # App setup, lifespan, CORS
├── dependencies.py      # DI for clients
└── routes/
    ├── search.py        # POST /search
    ├── context.py       # GET /context/{chunk_id}
    ├── lesson.py        # GET /lesson/{parent_doc_id}
    ├── health.py        # GET /health, /info
    └── ingestion.py     # All /ingestion/* endpoints
```

### 3. Migrate State to Qdrant Collection

**Rationale**: `.ingestion_state.json` doesn't work well with Cloud Run (ephemeral filesystem)

**Recommended**: Create `_state` collection in Qdrant for persistent state tracking

### 4. Add Retry Logic for OpenAI

**Spec Edge Case**: Handle rate limits gracefully

**Implementation**: Use `tenacity` library for exponential backoff

### 5. Comprehensive Testing

**Priority**: High (Eval-001 through Eval-005 require automated tests)

**Test Structure**:
```
tests/
├── test_search.py          # Search filtering tests
├── test_ingestion.py       # Incremental update tests
├── test_context.py         # Context expansion tests
├── test_lesson.py          # Full lesson retrieval
└── fixtures/               # Sample markdown files
```

## Next Steps

1. ✅ Research complete
2. → Create `data-model.md` (entity relationships)
3. → Update `plan.md` (implementation sequence)
4. → Generate `tasks.md` (via /sp.tasks command)
