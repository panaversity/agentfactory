# RAG Retrieval Service Skill

**Version**: 1.0.0
**Category**: Engineering / Platform Infrastructure
**Author**: Harvested from RoboLearn backend implementation (2025-11-29)

## Persona

You are a **RAG Search Engineer** specializing in production-grade semantic retrieval systems. You think about search the way a search engineer thinks about ranking and relevance - balancing semantic similarity with metadata filtering, supporting personalization, and enabling context expansion for complete answers.

**Your distinctive capability**: Designing retrieval APIs that combine vector similarity search with comprehensive Qdrant filtering, context expansion via chunk relationships, and personalization based on user profiles.

## When to Use This Skill

Use this skill when:
- Building semantic search APIs for educational or technical content
- Implementing personalized retrieval (filter by hardware tier, proficiency level)
- Designing context expansion features (get surrounding chunks)
- Creating comprehensive Qdrant filter strategies
- Building FastAPI search endpoints with proper validation

## Core Questions (Reasoning Activation)

Before implementing any retrieval service, ask:

### 1. Filter Strategy Design
- What filters are REQUIRED for every query? (book_id for tenant isolation, hardware_tier)
- What filters are OPTIONAL for user refinement? (module, chapter, lesson, proficiency)
- Which filters use exact match vs range vs MatchAny?

### 2. Personalization Requirements
- How does user hardware tier affect results? (tier X returns tier X or lower)
- How does proficiency level filter content? (A2 student shouldn't see C2 content)
- What other user context affects search?

### 3. Context Expansion Needs
- Can users expand a result to see surrounding content?
- Do you need to retrieve full lessons (all chunks from parent_doc_id)?
- How do prev_chunk_id/next_chunk_id enable walking the context chain?

### 4. API Design
- What's the appropriate response structure? (results, metadata, context)
- How to handle "no results" gracefully?
- What limits and pagination are needed?

## Qdrant Filter Architecture

```
Query Filter Strategy (all AND logic via must=[]):

┌─────────────────────────────────────────────────────────────────┐
│                       REQUIRED FILTERS                          │
├─────────────────────────────────────────────────────────────────┤
│  book_id         │ MatchValue   │ Tenant isolation              │
│  hardware_tier   │ Range(lte)   │ User's tier or lower          │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                       OPTIONAL FILTERS                          │
├─────────────────────────────────────────────────────────────────┤
│  module          │ MatchValue   │ Exact: ros2, gazebo, etc      │
│  chapter_min/max │ Range        │ Chapter range filter          │
│  lesson          │ MatchValue   │ Exact lesson number           │
│  proficiency     │ MatchAny     │ OR within: [A2, B1]           │
│  layer           │ MatchValue   │ Teaching layer: L1-L4         │
│  parent_doc_id   │ MatchValue   │ All chunks from specific doc  │
└─────────────────────────────────────────────────────────────────┘
```

## Implementation Patterns

### Pattern 1: Comprehensive Search Query Model

```python
"""
Pydantic models for type-safe search with validation.
"""

ModuleName = Literal["ros2", "gazebo", "isaac", "vla"]
ProficiencyLevel = Literal["A1", "A2", "B1", "B2", "C1", "C2"]
TeachingLayer = Literal["L1", "L2", "L3", "L4"]

class SearchQuery(BaseModel):
    """
    User's search request with comprehensive filters.

    Qdrant Filter Strategy:
    - All filters use 'must' (AND logic)
    - hardware_tier uses Range(lte) for "tier X or lower"
    - proficiency_levels uses MatchAny for OR within field
    - chapter uses Range(gte, lte) for ranges
    """

    text: str = Field(..., min_length=3, description="Search query text")

    # Required: Book tenant
    book_id: str = Field(default="physical-ai-robotics")

    # Hardware personalization (Range filter: lte)
    hardware_tier_filter: int = Field(default=1, ge=1, le=4)

    # Content location filters
    module_filter: Optional[ModuleName] = None
    chapter_min: Optional[int] = Field(None, ge=0, le=20)
    chapter_max: Optional[int] = Field(None, ge=0, le=20)
    lesson_filter: Optional[int] = Field(None, ge=0, le=15)

    # Pedagogical filters
    proficiency_levels: Optional[list[ProficiencyLevel]] = None  # OR logic
    layer_filter: Optional[TeachingLayer] = None

    # Context expansion
    parent_doc_id: Optional[str] = None

    # Pagination
    limit: int = Field(default=5, ge=1, le=20)
```

### Pattern 2: Search Response with Full Context

```python
class SearchResult(BaseModel):
    """Single search result with metadata for context expansion."""

    # Content
    text: str
    score: float = Field(..., ge=0.0, le=1.0)

    # Location
    source_file: str
    section_title: Optional[str]
    module: str
    chapter: int
    lesson: int

    # Personalization context
    hardware_tier: int
    proficiency_level: str
    layer: str

    # Context expansion metadata
    chunk_index: int
    total_chunks: int
    parent_doc_id: Optional[str]
    prev_chunk_id: Optional[str]  # For walking backward
    next_chunk_id: Optional[str]  # For walking forward
    content_hash: Optional[str]


class SearchResponse(BaseModel):
    """Complete search response."""

    query: str
    results: list[SearchResult]
    total_found: int

    # Filter context (for UI display)
    hardware_tier_filter: int
    module_filter: Optional[str]
    book_id: str
```

### Pattern 3: Comprehensive Qdrant Filter Builder

```python
"""
Build Qdrant filters from SearchQuery with all filter types.
"""

from qdrant_client.models import Filter, FieldCondition, MatchValue, MatchAny, Range

class RoboLearnSearch:
    def search(self, query: SearchQuery) -> SearchResponse:
        # Generate query embedding
        query_vector = self.embedder.embed_single(query.text)

        # Build filter conditions (all must=AND)
        conditions = []

        # Required: Book filter (tenant isolation)
        conditions.append(
            FieldCondition(
                key="book_id",
                match=MatchValue(value=query.book_id),
            )
        )

        # Required: Hardware tier filter (tier X or lower)
        conditions.append(
            FieldCondition(
                key="hardware_tier",
                range=Range(lte=query.hardware_tier_filter),
            )
        )

        # Optional: Module filter (exact match)
        if query.module_filter:
            conditions.append(
                FieldCondition(
                    key="module",
                    match=MatchValue(value=query.module_filter),
                )
            )

        # Optional: Chapter range filter
        if query.chapter_min is not None or query.chapter_max is not None:
            chapter_range = Range()
            if query.chapter_min is not None:
                chapter_range.gte = query.chapter_min
            if query.chapter_max is not None:
                chapter_range.lte = query.chapter_max
            conditions.append(
                FieldCondition(key="chapter", range=chapter_range)
            )

        # Optional: Lesson filter (exact match)
        if query.lesson_filter is not None:
            conditions.append(
                FieldCondition(
                    key="lesson",
                    match=MatchValue(value=query.lesson_filter),
                )
            )

        # Optional: Proficiency levels (OR logic within)
        if query.proficiency_levels:
            conditions.append(
                FieldCondition(
                    key="proficiency_level",
                    match=MatchAny(any=query.proficiency_levels),
                )
            )

        # Optional: Teaching layer filter
        if query.layer_filter:
            conditions.append(
                FieldCondition(
                    key="layer",
                    match=MatchValue(value=query.layer_filter),
                )
            )

        # Optional: Parent document filter (context expansion)
        if query.parent_doc_id:
            conditions.append(
                FieldCondition(
                    key="parent_doc_id",
                    match=MatchValue(value=query.parent_doc_id),
                )
            )

        # Execute search
        results = self.client.query_points(
            collection_name=self.collection_name,
            query=query_vector,
            query_filter=Filter(must=conditions),
            limit=query.limit,
            with_payload=True,
        )

        return self._convert_to_response(results, query)
```

### Pattern 4: Direct Chunk Retrieval by ID

```python
"""
Retrieve specific chunks for context expansion.
"""

def get_chunk_by_id(self, chunk_id: str) -> Optional[SearchResult]:
    """
    Retrieve a specific chunk by its UUID.
    Used for context expansion (fetching prev/next chunks).
    """
    try:
        points = self.client.retrieve(
            collection_name=self.collection_name,
            ids=[chunk_id],
            with_payload=True,
            with_vectors=False,
        )

        if not points:
            return None

        point = points[0]
        payload = point.payload

        return SearchResult(
            text=payload.get("text", ""),
            score=1.0,  # Direct retrieval, no similarity score
            source_file=payload.get("source_file", ""),
            section_title=payload.get("section_title"),
            module=payload.get("module", ""),
            chapter=payload.get("chapter", 0),
            lesson=payload.get("lesson", 0),
            hardware_tier=payload.get("hardware_tier", 1),
            proficiency_level=payload.get("proficiency_level", "A2"),
            layer=payload.get("layer", "L1"),
            chunk_index=payload.get("chunk_index", 0),
            total_chunks=payload.get("total_chunks", 1),
            parent_doc_id=payload.get("parent_doc_id"),
            prev_chunk_id=payload.get("prev_chunk_id"),
            next_chunk_id=payload.get("next_chunk_id"),
            content_hash=payload.get("content_hash"),
        )
    except Exception:
        return None
```

### Pattern 5: Context Expansion (Walk Chunk Chain)

```python
"""
Expand context by walking prev/next chunk relationships.
"""

def expand_context(
    self,
    chunk_id: str,
    prev_count: int = 1,
    next_count: int = 1,
) -> list[SearchResult]:
    """
    Get a chunk and its neighboring chunks for context expansion.

    Walks the prev_chunk_id/next_chunk_id chain stored in payloads.

    Returns:
        List of chunks in order [prev..., current, next...]
    """
    current = self.get_chunk_by_id(chunk_id)
    if not current:
        return []

    # Walk backwards
    prev_chunks = []
    prev_id = current.prev_chunk_id
    for _ in range(prev_count):
        if not prev_id:
            break
        prev_chunk = self.get_chunk_by_id(prev_id)
        if not prev_chunk:
            break
        prev_chunks.insert(0, prev_chunk)  # Prepend to maintain order
        prev_id = prev_chunk.prev_chunk_id

    # Walk forwards
    next_chunks = []
    next_id = current.next_chunk_id
    for _ in range(next_count):
        if not next_id:
            break
        next_chunk = self.get_chunk_by_id(next_id)
        if not next_chunk:
            break
        next_chunks.append(next_chunk)
        next_id = next_chunk.next_chunk_id

    return prev_chunks + [current] + next_chunks
```

### Pattern 6: Full Lesson Retrieval

```python
"""
Get all chunks for a lesson, ordered by chunk_index.
"""

def get_lesson_chunks(
    self,
    parent_doc_id: str,
    book_id: str = "physical-ai-robotics",
) -> list[SearchResult]:
    """
    Get all chunks for a lesson/document, ordered by chunk_index.
    Useful for showing full lesson context.
    """
    results = self.client.scroll(
        collection_name=self.collection_name,
        scroll_filter=Filter(
            must=[
                FieldCondition(
                    key="book_id",
                    match=MatchValue(value=book_id),
                ),
                FieldCondition(
                    key="parent_doc_id",
                    match=MatchValue(value=parent_doc_id),
                ),
            ]
        ),
        limit=100,
        with_payload=True,
        with_vectors=False,
    )

    points, _ = results
    chunks = [self._point_to_result(p) for p in points]

    # Sort by chunk_index for reading order
    chunks.sort(key=lambda c: c.chunk_index)
    return chunks
```

### Pattern 7: FastAPI Search Endpoints

```python
"""
Production FastAPI endpoints for semantic search.
"""

from fastapi import FastAPI, HTTPException
from contextlib import asynccontextmanager

# Global search client (initialized on startup)
search_client: Optional[RoboLearnSearch] = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Initialize search client on startup."""
    global search_client
    search_client = RoboLearnSearch()
    yield
    search_client = None

app = FastAPI(lifespan=lifespan)

@app.post("/search", response_model=SearchResponse)
async def search(request: SearchRequest):
    """
    Semantic search with comprehensive Qdrant filters.

    Filter Strategy (all AND logic):
    - hardware_tier: Range lte (tier X or lower)
    - module: Exact match
    - chapter_min/max: Range filter
    - lesson: Exact match
    - proficiency_levels: MatchAny (OR within)
    - layer: Exact match
    - parent_doc_id: Get all chunks from lesson
    """
    if search_client is None:
        raise HTTPException(status_code=503, detail="Search service not initialized")

    query = SearchQuery(
        text=request.query,
        book_id=settings.book_id,
        hardware_tier_filter=request.hardware_tier,
        module_filter=request.module,
        chapter_min=request.chapter_min,
        chapter_max=request.chapter_max,
        lesson_filter=request.lesson,
        proficiency_levels=request.proficiency_levels,
        layer_filter=request.layer,
        parent_doc_id=request.parent_doc_id,
        limit=request.limit,
    )

    return search_client.search(query)


@app.get("/context/{chunk_id}")
async def get_context(chunk_id: str, prev: int = 1, next: int = 1):
    """
    Get a chunk with surrounding context.
    Walks prev_chunk_id/next_chunk_id chain for context expansion.
    """
    chunks = search_client.expand_context(chunk_id, prev_count=prev, next_count=next)
    if not chunks:
        raise HTTPException(status_code=404, detail="Chunk not found")
    return {"chunks": [c.model_dump() for c in chunks]}


@app.get("/lesson/{parent_doc_id}")
async def get_lesson(parent_doc_id: str):
    """
    Get all chunks for a lesson/document.
    Returns chunks ordered by chunk_index for full lesson view.
    """
    chunks = search_client.get_lesson_chunks(parent_doc_id)
    if not chunks:
        raise HTTPException(status_code=404, detail="Lesson not found")
    return {
        "parent_doc_id": parent_doc_id,
        "total_chunks": len(chunks),
        "chunks": [c.model_dump() for c in chunks],
    }
```

## Filter Type Reference

| Filter Field | Qdrant Type | Use Case |
|-------------|-------------|----------|
| `book_id` | MatchValue | Tenant isolation (exact) |
| `hardware_tier` | Range(lte) | Personalization (tier X or lower) |
| `module` | MatchValue | Content location (exact) |
| `chapter` | Range(gte/lte) | Content location (range) |
| `lesson` | MatchValue | Content location (exact) |
| `proficiency_level` | MatchAny | Pedagogical (OR within) |
| `layer` | MatchValue | Pedagogical (exact) |
| `parent_doc_id` | MatchValue | Context expansion (exact) |

## Anti-Patterns to Avoid

1. **Missing tenant filter** - Always include book_id for multitenancy
2. **Hardcoded hardware tier** - Use lte filter for "tier X or lower" semantic
3. **Ignoring chunk relationships** - Store prev/next for context expansion
4. **Single result endpoint only** - Provide context expansion and lesson retrieval
5. **No proficiency filtering** - Use MatchAny for OR logic on proficiency levels
6. **Missing payload indexing** - All filter fields need Qdrant indexes
7. **Sync initialization** - Use lifespan context manager for async startup

## Required Dependencies

```txt
qdrant-client>=1.14.0
openai>=1.0.0
pydantic>=2.0.0
pydantic-settings>=2.0.0
fastapi>=0.104.0
```

## Success Metrics

- **Query latency**: under 100ms for filtered search
- **Relevance**: Top-3 results contain answer 80%+ of the time
- **Context expansion**: Successfully walk chunk chains
- **Filter accuracy**: 100% compliance with hardware tier constraints
