# Implementation Plan: RAG Agent Backend

**Branch**: `005-rag-agent` | **Date**: 2025-11-29 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/005-rag-agent/spec.md`

## Summary

Production-grade RAG (Retrieval-Augmented Generation) backend for the RoboLearn educational platform. The system ingests markdown documentation from the book repository, creates semantic embeddings, stores them in Qdrant vector database, and provides a comprehensive search API with hardware-tier-aware personalization and context expansion capabilities.

**Key Insight**: A working implementation already exists in `backend/`. This plan focuses on **organizing, enhancing, and testing** the existing code rather than building from scratch.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: FastAPI (0.104+), qdrant-client (1.14+), openai (1.0+), pydantic (2.0+), python-frontmatter
**Storage**:
- Vector DB: Qdrant Cloud (hosted)
- State: `.ingestion_state.json` (migrate to Qdrant collection recommended)
- Job tracking: In-memory dict (migrate to SQLite recommended)

**Testing**: pytest with pytest-asyncio for async endpoint testing
**Target Platform**: Google Cloud Run (serverless, Docker-based)
**Project Type**: single (backend API)
**Performance Goals**:
- Search latency: <500ms p95
- Ingestion throughput: ~500 chunks/minute
- Concurrent searches: 100+ without degradation

**Constraints**:
- Stateless API (Cloud Run ephemeral filesystem)
- Background job processing (FastAPI BackgroundTasks)
- OpenAI API rate limits (handle gracefully)

**Scale/Scope**:
- ~500+ chunks initially (Module 1 ROS 2)
- ~2000+ chunks eventually (all 4 modules)
- 100+ concurrent search requests
- 3 ingestion modes: incremental, full, recreate

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Platform Principles Assessment

| Principle | Status | Notes |
|-----------|--------|-------|
| **Specification Primacy** | ✅ PASS | Spec created first (spec.md), implementation follows |
| **Progressive Complexity** | ✅ PASS | Platform work, not content - complexity appropriate for engineering task |
| **Factual Accuracy** | ⚠️ VERIFY | Need to verify against Qdrant 1.14+ and OpenAI latest docs during implementation |
| **Intelligence Accumulation** | ✅ PASS | Skills harvested: `rag-ingestion-pipeline`, `rag-retrieval-service` |
| **Anti-Convergence** | N/A | Platform engineering, not pedagogical content |
| **Minimal Content** | ✅ PASS | API endpoints serve specific retrieval needs, no bloat |
| **Formal Verification** | ✅ REQUIRED | 5+ entities (DocumentChunk, Metadata, State, Job, Search) → small scope test in data-model.md |

### Formal Verification Applied

**Trigger**: 5+ interacting entities, safety-critical data handling

**Invariants Identified**:
1. **Coverage**: `∀ chunk: some chunk.parent_doc_id` (every chunk has parent)
2. **No cycles**: `no chunk: chunk in chunk.^next_chunk_id` (context chain acyclic)
3. **Context integrity**: First chunk has `prev_chunk_id = None`, last chunk has `next_chunk_id = None`
4. **Hardware tier filter**: `∀ query with tier=X: ∀ result: result.tier <= X` (personalization correct)

**Small Scope Test**: 3 lessons, 9 chunks tested in data-model.md - no counterexamples found

**Conclusion**: Design verified for referential integrity, ready for implementation

## Project Structure

### Documentation (this feature)

```text
specs/005-rag-agent/
├── spec.md              # Feature specification (already exists)
├── plan.md              # This file (implementation plan)
├── research.md          # Technology decisions and existing implementation analysis
├── data-model.md        # Entity definitions and relationships
└── tasks.md             # WILL BE CREATED by /sp.tasks command (NOT by /sp.plan)
```

### Source Code (repository root)

**Existing Implementation**: `backend/` directory already contains working RAG system

**Current Structure** (what exists):
```text
backend/
├── __init__.py
├── config.py                    # ✅ Settings management (pydantic-settings)
├── .env.example                 # ✅ Environment template
├── requirements.txt             # ✅ Dependencies
├── pyproject.toml              # ✅ Project metadata
│
├── models/                      # ✅ Data models (complete)
│   ├── __init__.py
│   ├── document.py              # DocumentChunk, Metadata, IngestionState
│   └── search.py                # SearchQuery, SearchResult, SearchResponse
│
├── ingestion/                   # ✅ Ingestion pipeline (complete)
│   ├── __init__.py
│   ├── crawler.py               # Discovers markdown files
│   ├── parser.py                # Parses frontmatter
│   ├── chunker.py               # Semantic chunking (## headers, 400 tokens)
│   ├── embedder.py              # OpenAI embeddings
│   ├── uploader.py              # Qdrant upload + state management
│   └── state.py                 # State tracking utilities (TO BE CREATED)
│
├── retrieval/                   # ✅ Search service (complete)
│   ├── __init__.py
│   └── search.py                # RoboLearnSearch class
│
├── api/                         # ⚠️ Needs organization
│   ├── __init__.py
│   ├── main.py                  # FastAPI app, CORS, lifespan
│   ├── ingestion.py             # Ingestion routes
│   ├── dependencies.py          # TO BE CREATED (DI for clients)
│   └── routes/                  # TO BE CREATED (organized routes)
│       ├── __init__.py
│       ├── search.py            # POST /search
│       ├── context.py           # GET /context/{chunk_id}
│       ├── lesson.py            # GET /lesson/{parent_doc_id}
│       ├── health.py            # GET /health, /info
│       └── ingestion.py         # All /ingestion/* endpoints
│
├── scripts/                     # ✅ CLI tools (complete)
│   ├── ingest.py                # Manual ingestion script
│   └── search_test.py           # Search testing script
│
├── db/                          # ⚠️ Database models (future)
│   ├── __init__.py
│   └── models.py                # SQLAlchemy models (for job tracking)
│
└── tests/                       # ❌ TO BE CREATED
    ├── __init__.py
    ├── conftest.py              # Pytest fixtures
    ├── test_search.py           # Search filtering tests (Eval-001, Eval-002)
    ├── test_ingestion.py        # Incremental update tests (Eval-003, Eval-004)
    ├── test_context.py          # Context expansion tests (Eval-005)
    └── fixtures/                # Sample markdown files for testing
        ├── sample_lesson.md
        └── sample_module/
```

**Deployment** (to be created):
```text
backend/
├── Dockerfile                   # TO BE CREATED (Cloud Run container)
├── .dockerignore               # TO BE CREATED
└── cloudbuild.yaml             # TO BE CREATED (Cloud Build config)
```

**Structure Decision**:

The existing `backend/` structure is **production-quality** with proper separation of concerns:
- **Models**: Type-safe data definitions (Pydantic)
- **Ingestion**: Pipeline components (crawler → parser → chunker → embedder → uploader)
- **Retrieval**: Search service with filtering
- **API**: FastAPI endpoints (needs route organization)

**Changes Needed**:
1. Organize API routes into separate files (currently mixed in main.py)
2. Create comprehensive test suite (tests/ directory)
3. Add deployment configuration (Dockerfile, Cloud Run)
4. Enhance state management (migrate from JSON file to Qdrant collection recommended)

## Component Breakdown

**Implementation Sequence** (dependency order):

### Phase 0: Foundation (Already Complete)
- ✅ Models (document.py, search.py)
- ✅ Config (config.py with pydantic-settings)
- ✅ Ingestion Pipeline (crawler → parser → chunker → embedder → uploader)
- ✅ Retrieval Service (search.py)
- ✅ Basic API (main.py, ingestion.py)

### Phase 1: API Organization (Refactoring)

**Priority**: P2 (organizational improvement)

**Components**:

1. **api/dependencies.py** - Dependency injection
   - `get_search_client()` - Returns RoboLearnSearch instance
   - `get_uploader()` - Returns QdrantUploader instance
   - `verify_admin_key()` - Admin API key validation
   - `verify_github_signature()` - GitHub webhook validation

2. **api/routes/search.py** - Search endpoints
   - `POST /search` - Semantic search with filters
   - Extract from current `main.py`

3. **api/routes/context.py** - Context expansion
   - `GET /context/{chunk_id}` - Get chunk with prev/next
   - Extract from current `main.py`

4. **api/routes/lesson.py** - Full lesson retrieval
   - `GET /lesson/{parent_doc_id}` - All chunks for lesson
   - Extract from current `main.py`

5. **api/routes/health.py** - Health and info
   - `GET /health` - Health check with Qdrant status
   - `GET /info` - Collection statistics
   - Extract from current `main.py`

6. **api/routes/ingestion.py** - Ingestion endpoints
   - Already exists in `api/ingestion.py`
   - Refactor to use dependencies.py

7. **api/main.py** - App setup
   - FastAPI app creation
   - CORS middleware
   - Lifespan management
   - Router registration
   - Simplified after extracting routes

**Dependencies**: None (refactoring existing code)

**Validation**: All existing endpoints still work after refactoring

### Phase 2: State Management Enhancement (Optional Improvement)

**Priority**: P3 (nice-to-have for production)

**Current**: `.ingestion_state.json` file

**Issue**: Cloud Run has ephemeral filesystem, state lost on container restart

**Recommended**: Qdrant-native state tracking

**Component**: `ingestion/state.py`

**Implementation**:
```python
class QdrantStateTracker:
    """Store ingestion state in Qdrant collection."""

    def __init__(self, client, collection_name="_state"):
        self.client = client
        self.collection = collection_name

    def save_record(self, record: IngestionRecord):
        """Save file ingestion record as Qdrant point."""
        # Point ID = hash of source_file
        # Payload = IngestionRecord.model_dump()
        # Vector = zero vector (not used for retrieval)

    def load_state(self) -> IngestionState:
        """Load all records from collection."""
        # Scroll collection, reconstruct IngestionState

    def get_record(self, source_file: str) -> IngestionRecord | None:
        """Get specific file record."""
        # Retrieve by point ID
```

**Benefits**:
- State persists across Cloud Run restarts
- Multiple instances can share state
- No file I/O on ephemeral filesystem

**Alternative**: SQLite with persistent volume (more complex)

**Decision**: Implement if time permits, otherwise keep JSON file for MVP

### Phase 3: Testing Infrastructure (High Priority)

**Priority**: P1 (required for Eval-001 through Eval-005)

**Components**:

1. **tests/conftest.py** - Pytest fixtures
   - `qdrant_client` - Test Qdrant client (isolated collection)
   - `sample_chunks` - Sample DocumentChunk instances
   - `search_client` - RoboLearnSearch instance
   - `api_client` - FastAPI TestClient

2. **tests/test_search.py** - Search filtering tests
   - **Eval-001**: Top-5 semantic relevance for 20 test queries
   - **Eval-002**: Hardware tier filtering accuracy (100% no Tier 3+ for Tier 1 queries)
   - Test cases:
     - `test_hardware_tier_filtering()` - Verify tier X returns only tier X or lower
     - `test_semantic_relevance()` - Top-5 results contain expected content
     - `test_module_filter()` - Module filter works correctly
     - `test_chapter_range_filter()` - Chapter range works
     - `test_proficiency_filter()` - Proficiency OR logic works

3. **tests/test_ingestion.py** - Incremental update tests
   - **Eval-003**: Incremental ingestion only processes changed files (0 unchanged re-embedded)
   - Test cases:
     - `test_incremental_skips_unchanged()` - Unchanged files not re-embedded
     - `test_incremental_processes_modified()` - Modified files re-embedded
     - `test_incremental_deletes_removed()` - Deleted files removed from index
     - `test_atomic_update()` - Old chunks deleted before new inserted

4. **tests/test_context.py** - Context expansion tests
   - **Eval-005**: Context expansion returns prev/next chunks correctly
   - Test cases:
     - `test_context_expansion()` - Walk prev/next chain
     - `test_first_chunk_no_prev()` - First chunk has prev=None
     - `test_last_chunk_no_next()` - Last chunk has next=None
     - `test_full_lesson_retrieval()` - All chunks returned in order

5. **tests/test_lesson.py** - Full lesson retrieval
   - Test cases:
     - `test_lesson_all_chunks()` - All chunks for lesson returned
     - `test_lesson_ordered()` - Chunks ordered by chunk_index
     - `test_lesson_not_found()` - 404 for invalid parent_doc_id

6. **tests/fixtures/** - Sample data
   - `sample_lesson.md` - Markdown file with frontmatter
   - `sample_module/` - Module directory structure
   - Pre-computed expected embeddings (mocked)

**Dependencies**: Phase 1 (organized API routes)

**Validation**: All tests pass, coverage > 80%

### Phase 4: Deployment Configuration (Cloud Run)

**Priority**: P1 (required for production)

**Components**:

1. **Dockerfile** - Multi-stage Docker build
   ```dockerfile
   FROM python:3.11-slim as builder
   # Install dependencies

   FROM python:3.11-slim
   # Copy app
   CMD ["uvicorn", "api.main:app", "--host", "0.0.0.0", "--port", "8080"]
   ```

2. **.dockerignore** - Exclude unnecessary files
   - venv/, __pycache__/, .env, tests/

3. **cloudbuild.yaml** - Cloud Build configuration
   - Build Docker image
   - Push to Artifact Registry
   - Deploy to Cloud Run

4. **Environment Variables** - Cloud Run secrets
   - QDRANT_URL
   - QDRANT_API_KEY
   - OPENAI_API_KEY
   - ADMIN_API_KEY
   - GITHUB_WEBHOOK_SECRET (optional)

**Dependencies**: None

**Validation**: `docker build` succeeds, container runs locally

### Phase 5: Performance Optimization (Optional)

**Priority**: P3 (only if performance issues)

**Potential Optimizations**:

1. **Caching** - Redis for frequently searched queries
2. **Batch Endpoints** - Bulk search for chatbot context
3. **Connection Pooling** - Reuse Qdrant connections
4. **Rate Limiting** - Prevent abuse (e.g., slowapi)

**Decision**: Implement only if SC-001 (500ms p95) or SC-007 (100 concurrent) not met

### Phase 6: Monitoring and Observability (Production)

**Priority**: P2 (important for production)

**Components**:

1. **Logging** - Structured logging with Python `logging`
   - Request/response logging
   - Error logging with tracebacks
   - Ingestion job progress logging

2. **Metrics** - Cloud Run metrics
   - Request latency (p50, p95, p99)
   - Request count by endpoint
   - Error rate
   - Ingestion job duration

3. **Alerts** - Cloud Monitoring alerts
   - High error rate (> 5%)
   - High latency (> 1s p95)
   - Qdrant connection failures

**Dependencies**: Phase 4 (Cloud Run deployment)

**Validation**: Logs visible in Cloud Logging, metrics in Cloud Monitoring

## Implementation Order Summary

**Week 1** (MVP):
1. Phase 1: API organization (2-3 days)
2. Phase 3: Testing infrastructure (3-4 days)
3. Phase 4: Deployment configuration (1 day)

**Week 2** (Production-ready):
4. Phase 2: State management enhancement (2 days - optional)
5. Phase 6: Monitoring and observability (2-3 days)
6. Phase 5: Performance optimization (if needed)

**Critical Path**: Phase 1 → Phase 3 → Phase 4 (deployment)

**Optional Enhancements**: Phase 2 (state), Phase 5 (performance), Phase 6 (monitoring)

## Complexity Tracking

> No constitutional violations detected. All complexity justified by production requirements.
