# RAG Ingestion Pipeline Skill

**Version**: 1.0.0
**Category**: Engineering / Platform Infrastructure
**Author**: Harvested from RoboLearn backend implementation (2025-11-29)

## Persona

You are a **RAG Infrastructure Engineer** specializing in production-grade document ingestion pipelines. You think about content processing the way a data engineer thinks about ETL pipelines - ensuring data integrity, change detection, efficient batch processing, and semantic chunking that preserves meaning.

**Your distinctive capability**: Designing ingestion pipelines that support incremental updates without full re-indexing, using content hashing for change detection and optimal chunk sizing for retrieval quality.

## When to Use This Skill

Use this skill when:
- Building a RAG system that needs to ingest documents (MD, HTML, PDF)
- Implementing incremental update capability (only re-embed changed content)
- Designing semantic chunking strategies for educational or technical content
- Setting up vector database ingestion with proper payload indexing
- Creating production-grade ingestion APIs with background job processing

## Core Questions (Reasoning Activation)

Before implementing any ingestion pipeline, ask:

### 1. Content Structure Analysis
- What is the document hierarchy? (book -> module -> chapter -> lesson)
- What semantic boundaries exist? (## headers, sections, paragraphs)
- What metadata needs to be preserved per chunk? (hardware tier, proficiency level, etc.)

### 2. Change Detection Strategy
- How will you detect new/modified/deleted files?
- Where will you store ingestion state? (file-based, Qdrant-native, database)
- What hash algorithm for content fingerprinting? (SHA-256 recommended)

### 3. Chunking Parameters
- What's the target token size? (400-512 tokens optimal for factoid queries)
- What overlap percentage between chunks? (10-20% for context continuity)
- How to handle sections larger than max tokens?

### 4. Embedding Strategy
- Which embedding model? (text-embedding-3-small: 1536 dims, cost-effective)
- What batch size for API calls? (~20 for OpenAI embeddings)
- How to handle rate limits?

### 5. Vector Database Configuration
- What payload fields need indexing for filtered retrieval?
- What's the multitenancy strategy? (book_id as tenant field)
- How to track chunk relationships? (prev/next for context expansion)

## Production Architecture

```
docs/
  ├── module-1-ros2/
  │   ├── chapter-1-physical-ai/
  │   │   ├── 01-lesson.md
  │   │   └── 02-lesson.md
  │   └── README.md
  └── ...

Pipeline Stages:
┌──────────┐    ┌────────┐    ┌─────────┐    ┌──────────┐    ┌──────────┐
│ Crawler  │ -> │ Parser │ -> │ Chunker │ -> │ Embedder │ -> │ Uploader │
└──────────┘    └────────┘    └─────────┘    └──────────┘    └──────────┘
     │              │              │              │              │
     v              v              v              v              v
 Discovers      Extracts       Splits by     Generates      Upserts to
 MD files      frontmatter    semantic      vectors        Qdrant
              + computes      boundaries    (batched)      (batched)
              file hash       + overlap
```

## Implementation Patterns

### Pattern 1: Document Crawler

```python
"""
Discovers markdown files with path-based metadata extraction.
Convention: docs/module-{N}-{name}/chapter-{N}-{name}/{NN}-lesson.md
"""

@dataclass
class DiscoveredFile:
    absolute_path: Path
    relative_path: str
    module: str          # ros2, gazebo, isaac, vla
    module_number: int   # 1, 2, 3, 4
    chapter: int
    lesson: int | None
    is_readme: bool

class DocsCrawler:
    MODULE_MAP = {
        "module-1-ros2": ("ros2", 1),
        "module-2-simulation": ("gazebo", 2),
        # ...
    }

    CHAPTER_PATTERN = re.compile(r"chapter-(\d+)-")
    LESSON_PATTERN = re.compile(r"^(\d+)-.*\.md$")

    def discover(self, include_readmes: bool = True) -> Iterator[DiscoveredFile]:
        """Walk directory tree extracting metadata from paths."""
```

### Pattern 2: Frontmatter Parser with Hash

```python
"""
Extracts frontmatter + computes file hash for change detection.
"""

def compute_file_hash(file_path: str) -> str:
    """SHA-256 of file content for incremental updates."""
    with open(file_path, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()

class LessonParser:
    DEFAULTS = {
        "proficiency_level": "A2",
        "layer": "L1",
        "hardware_tier": 1,
    }

    def parse(self, discovered: DiscoveredFile) -> LessonFile:
        file_hash = compute_file_hash(str(discovered.absolute_path))
        post = frontmatter.load(discovered.absolute_path)

        return LessonFile(
            file_path=str(discovered.absolute_path),
            relative_path=discovered.relative_path,
            file_hash=file_hash,
            title=post.get("title", discovered.filename),
            proficiency_level=self._normalize_proficiency(post.get("proficiency_level")),
            # ... other metadata
            raw_content=post.content,
        )
```

### Pattern 3: Semantic Chunker with Overlap

```python
"""
Production-grade chunking:
- Split on ## headers (semantic boundaries)
- Target 400 tokens per chunk (NVIDIA benchmark optimal)
- 15% overlap for context continuity
- Track prev/next relationships
"""

class SectionChunker:
    SECTION_PATTERN = re.compile(r"(?=^## )", re.MULTILINE)
    TOKENS_PER_WORD = 1.3  # English text approximation

    def __init__(
        self,
        target_tokens: int = 400,
        max_tokens: int = 512,
        min_tokens: int = 100,
        overlap_percent: float = 0.15,
    ):
        self.target_words = int(target_tokens / self.TOKENS_PER_WORD)
        self.overlap_words = int(self.target_words * overlap_percent)

    def chunk(self, lesson: LessonFile) -> Iterator[DocumentChunk]:
        # 1. Split by ## headers (semantic boundaries)
        sections = self._split_by_sections(lesson.raw_content)

        # 2. Create overlapping chunks
        raw_chunks = self._create_overlapping_chunks(sections)

        # 3. Generate deterministic IDs from content hash
        for idx, (title, text) in enumerate(raw_chunks):
            content_hash = compute_content_hash(text)
            chunk_id = self._generate_id(lesson, content_hash)

            yield DocumentChunk(
                id=chunk_id,
                text=text,
                metadata=DocumentMetadata(
                    chunk_index=idx,
                    total_chunks=len(raw_chunks),
                    prev_chunk_id=prev_id,  # For context expansion
                    next_chunk_id=next_id,
                    content_hash=content_hash,
                    source_file_hash=lesson.file_hash,
                    # ... other metadata
                ),
            )
```

### Pattern 4: Batched Embeddings

```python
"""
Efficient embedding with batching and progress tracking.
"""

class OpenAIEmbedder:
    def __init__(
        self,
        model: str = "text-embedding-3-small",
        batch_size: int = 20,  # OpenAI recommendation
    ):
        self.client = OpenAI()
        self.model = model
        self.batch_size = batch_size

    def embed_chunks(self, chunks: list[DocumentChunk]) -> list[EmbeddedChunk]:
        embedded = []
        for batch_start in range(0, len(chunks), self.batch_size):
            batch = chunks[batch_start:batch_start + self.batch_size]
            texts = [c.text for c in batch]

            response = self.client.embeddings.create(
                input=texts,
                model=self.model,
            )

            for chunk, emb_data in zip(batch, response.data):
                embedded.append(EmbeddedChunk(
                    **chunk.model_dump(),
                    embedding=emb_data.embedding,
                ))
        return embedded
```

### Pattern 5: Qdrant Uploader with State Management

```python
"""
Production uploader with:
- Payload indexing for filtered retrieval
- Incremental update support
- State persistence
"""

class QdrantUploader:
    def __init__(self, collection_name: str):
        self.client = QdrantClient(url=url, api_key=api_key)
        self.collection_name = collection_name

    def ensure_collection(self, recreate: bool = False):
        """Create collection with proper vector config and indexes."""
        self.client.create_collection(
            collection_name=self.collection_name,
            vectors_config=VectorParams(
                size=1536,  # text-embedding-3-small
                distance=Distance.COSINE,
            ),
        )

        # Create payload indexes for efficient filtering
        indexes = [
            ("book_id", PayloadSchemaType.KEYWORD),
            ("module", PayloadSchemaType.KEYWORD),
            ("hardware_tier", PayloadSchemaType.INTEGER),
            ("chapter", PayloadSchemaType.INTEGER),
            ("lesson", PayloadSchemaType.INTEGER),
            ("proficiency_level", PayloadSchemaType.KEYWORD),
            ("layer", PayloadSchemaType.KEYWORD),
            ("parent_doc_id", PayloadSchemaType.KEYWORD),
            ("content_hash", PayloadSchemaType.KEYWORD),
            ("source_file_hash", PayloadSchemaType.KEYWORD),
        ]

        for field_name, field_type in indexes:
            self.client.create_payload_index(
                collection_name=self.collection_name,
                field_name=field_name,
                field_schema=field_type,
            )

    def upload(self, chunks: list[EmbeddedChunk]):
        """Batch upsert with full payload."""
        points = [
            PointStruct(
                id=chunk.id,
                vector=chunk.embedding,
                payload=QdrantPayload.from_chunk(chunk).model_dump(),
            )
            for chunk in chunks
        ]

        for batch in batched(points, self.batch_size):
            self.client.upsert(
                collection_name=self.collection_name,
                wait=True,
                points=list(batch),
            )
```

### Pattern 6: Qdrant-Native State Tracking

```python
"""
Modern approach: Query Qdrant payloads directly for change detection.
No external state database needed.
"""

class QdrantStateTracker:
    def get_indexed_files(self) -> dict[str, IndexedFileInfo]:
        """Query Qdrant payloads to find what's indexed."""
        indexed = {}
        offset = None

        while True:
            points, next_offset = self.client.scroll(
                collection_name=self.collection_name,
                scroll_filter=Filter(must=[
                    FieldCondition(key="book_id", match=MatchValue(value=self.book_id))
                ]),
                limit=100,
                offset=offset,
                with_payload=["source_file", "source_file_hash"],
                with_vectors=False,
            )

            for point in points:
                source_file = point.payload.get("source_file")
                if source_file not in indexed:
                    indexed[source_file] = IndexedFileInfo(
                        source_file=source_file,
                        file_hash=point.payload.get("source_file_hash"),
                        chunk_count=0,
                        chunk_ids=[],
                    )
                indexed[source_file].chunk_count += 1
                indexed[source_file].chunk_ids.append(str(point.id))

            if next_offset is None:
                break
            offset = next_offset

        return indexed

    def detect_changes(self, current_files: dict[str, str]) -> tuple[list, list, list]:
        """Compare filesystem against Qdrant index."""
        indexed = self.get_indexed_files()

        new_files = [p for p in current_files if p not in indexed]
        deleted_files = [p for p in indexed if p not in current_files]
        modified_files = [
            p for p in current_files & indexed
            if current_files[p] != indexed[p].file_hash
        ]

        return new_files, modified_files, deleted_files
```

### Pattern 7: Production Ingestion API

```python
"""
FastAPI endpoints for production ingestion with background jobs.
"""

@router.post("/trigger")
async def trigger_ingestion(
    request: TriggerRequest,
    background_tasks: BackgroundTasks,
):
    """Trigger ingestion job (incremental/full/recreate)."""
    job_id = create_job(request.mode, request.book_id)

    background_tasks.add_task(
        run_ingestion_job,
        job_id,
        request.mode,
        request.book_id,
        request.docs_path,
    )

    return {"job_id": job_id, "status": "queued"}

@router.get("/status/{job_id}")
async def get_job_status(job_id: int):
    """Check ingestion job progress."""
    return get_job(job_id)

@router.post("/webhook/github")
async def github_webhook(background_tasks: BackgroundTasks):
    """Auto-trigger on GitHub push events."""
    # Verify signature, trigger incremental ingestion
```

## Payload Schema

```python
class QdrantPayload(BaseModel):
    """Complete payload for filtered retrieval and context expansion."""

    # Tenant isolation
    book_id: str

    # Hierarchical filters (all indexed)
    module: str
    chapter: int
    lesson: int
    hardware_tier: int
    proficiency_level: str
    layer: str

    # Content for display
    text: str
    section_title: Optional[str]
    source_file: str

    # Context expansion
    parent_doc_id: str
    chunk_index: int
    total_chunks: int
    prev_chunk_id: Optional[str]
    next_chunk_id: Optional[str]

    # Change detection
    content_hash: str
    source_file_hash: str
    version: int

    # Metrics
    word_count: int
    token_count: int
```

## Anti-Patterns to Avoid

1. **Fixed character chunking** - Use semantic boundaries (headers) instead
2. **Position-based IDs** - Use content hash for stable IDs across re-indexing
3. **No overlap** - Always include 10-20% overlap for context continuity
4. **Full re-index on every change** - Implement incremental updates
5. **Missing payload indexes** - Always index fields you filter by
6. **Synchronous embedding** - Use batching and background jobs for production
7. **External state database** - Consider Qdrant-native state tracking first

## Required Dependencies

```txt
qdrant-client>=1.14.0
openai>=1.0.0
pydantic>=2.0.0
pydantic-settings>=2.0.0
python-frontmatter>=1.0.0
rich>=13.0.0
fastapi>=0.104.0
```

## Success Metrics

- **Ingestion speed**: ~500 chunks/minute with batching
- **Change detection accuracy**: 100% (content hash based)
- **Re-indexing time**: Only changed files processed
- **Storage efficiency**: No duplicate embeddings for unchanged content
