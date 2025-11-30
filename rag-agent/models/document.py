"""
Document models for RoboLearn RAG ingestion pipeline.

Production-grade RAG with:
- Content hashing for change detection (SHA-256)
- Chunk overlap for context continuity (10-20%)
- Parent-child document relationships
- Hierarchical retrieval support
- Incremental update capability

References:
- https://unstructured.io/blog/chunking-for-rag-best-practices
- https://weaviate.io/blog/chunking-strategies-for-rag
- https://particula.tech/blog/update-rag-knowledge-without-rebuilding
"""

from pydantic import BaseModel, Field, field_validator, computed_field
from typing import Literal, Optional
from datetime import datetime
import uuid
import hashlib


# === Type Definitions ===

ModuleName = Literal["ros2", "gazebo", "isaac", "vla"]
ProficiencyLevel = Literal["A1", "A2", "B1", "B2", "C1", "C2"]
TeachingLayer = Literal["L1", "L2", "L3", "L4"]
HardwareTier = Literal[1, 2, 3, 4]


# === Content Hashing Utilities ===

def compute_content_hash(content: str) -> str:
    """
    Compute SHA-256 hash of content for change detection.
    Used to determine if a chunk needs re-embedding.
    """
    return hashlib.sha256(content.encode('utf-8')).hexdigest()


def compute_file_hash(file_path: str) -> str:
    """
    Compute SHA-256 hash of entire file for change detection.
    Used to skip unchanged files during incremental updates.
    """
    with open(file_path, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


# === Document Models ===

class DocumentMetadata(BaseModel):
    """
    Metadata extracted from lesson frontmatter and file path.

    Production features:
    - Content hash for change detection
    - Parent document reference for hierarchical retrieval
    - Chunk relationships (prev/next) for context
    """

    # Platform-level tenant (for multi-book support)
    book_id: str = Field(..., description="Book identifier, e.g., 'physical-ai-robotics'")

    # Hierarchical structure (Module → Chapter → Lesson → Section)
    module: ModuleName = Field(..., description="Module: ros2, gazebo, isaac, vla")
    chapter: int = Field(..., ge=0, le=20, description="Chapter number (0 for module-level)")
    lesson: int = Field(..., ge=0, le=15, description="Lesson number (0 for chapter/module overview)")
    section_title: Optional[str] = Field(None, description="Section header if chunked by ##")

    # Personalization filters
    hardware_tier: HardwareTier = Field(..., description="Minimum hardware tier: 1-4")
    proficiency_level: ProficiencyLevel = Field(default="A2", description="CEFR level")
    layer: TeachingLayer = Field(default="L1", description="Teaching layer: L1-L4")

    # Source tracking
    source_file: str = Field(..., description="Relative path to source MD file")

    # Lesson-level metadata (from frontmatter)
    lesson_title: Optional[str] = Field(None, description="Lesson title from frontmatter")
    lesson_description: Optional[str] = Field(None, description="Lesson description from frontmatter")
    doc_id: Optional[str] = Field(None, description="Docusaurus doc ID from frontmatter (for URL generation)")

    # Parent document reference (for hierarchical retrieval)
    parent_doc_id: str = Field(..., description="Parent lesson/chapter document ID")

    # Chunk position and relationships
    chunk_index: int = Field(..., ge=0, description="Position within lesson")
    total_chunks: int = Field(..., ge=1, description="Total chunks in lesson")
    prev_chunk_id: Optional[str] = Field(None, description="Previous chunk UUID for context chain")
    next_chunk_id: Optional[str] = Field(None, description="Next chunk UUID for context chain")

    # Change detection
    content_hash: str = Field(..., description="SHA-256 hash of chunk content")
    source_file_hash: str = Field(..., description="SHA-256 hash of source file")

    # Versioning
    version: int = Field(default=1, description="Chunk version for updates")
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)


class DocumentChunk(BaseModel):
    """
    A content chunk ready for embedding.

    Production features:
    - Deterministic UUID from content (not position) for stable IDs
    - Content hash for change detection
    - Token count for precise sizing
    """

    id: str = Field(..., description="Unique chunk UUID")
    text: str = Field(..., min_length=10, description="Chunk content")
    metadata: DocumentMetadata

    # Precise metrics
    word_count: int = Field(..., ge=1, description="Word count")
    token_count: int = Field(..., ge=1, description="Approximate token count (words * 1.3)")
    char_count: int = Field(..., ge=1, description="Character count")

    @field_validator("text")
    @classmethod
    def validate_text_not_empty(cls, v: str) -> str:
        """Ensure text has meaningful content."""
        stripped = v.strip()
        if len(stripped) < 10:
            raise ValueError("Chunk text must have at least 10 characters")
        return stripped

    @classmethod
    def generate_id(
        cls,
        book_id: str,
        module: str,
        chapter: int,
        lesson: int,
        content_hash: str,
    ) -> str:
        """
        Generate unique chunk ID as UUID based on CONTENT hash.

        This ensures:
        1. Same content = same ID (idempotent)
        2. Changed content = new ID (forces re-embedding)
        3. Position changes don't affect ID (stable references)
        """
        # Include content hash so ID changes when content changes
        id_string = f"{book_id}:{module}:{chapter}:{lesson}:{content_hash[:16]}"
        namespace = uuid.UUID("6ba7b810-9dad-11d1-80b4-00c04fd430c8")
        return str(uuid.uuid5(namespace, id_string))

    @classmethod
    def generate_parent_doc_id(
        cls,
        book_id: str,
        module: str,
        chapter: int,
        lesson: int,
    ) -> str:
        """Generate parent document ID for hierarchical retrieval."""
        id_string = f"{book_id}:{module}:{chapter}:{lesson}:parent"
        namespace = uuid.UUID("6ba7b810-9dad-11d1-80b4-00c04fd430c8")
        return str(uuid.uuid5(namespace, id_string))


class EmbeddedChunk(DocumentChunk):
    """
    Document chunk with its vector embedding.
    """

    embedding: list[float] = Field(..., min_length=1536, max_length=1536)
    embedded_at: datetime = Field(default_factory=datetime.utcnow)
    embedding_model: str = Field(default="text-embedding-3-small")


class LessonFile(BaseModel):
    """
    Represents a parsed lesson file before chunking.
    """

    # File info
    file_path: str = Field(..., description="Absolute path to MD file")
    relative_path: str = Field(..., description="Relative path from docs root")
    file_hash: str = Field(..., description="SHA-256 of file content for change detection")

    # Parsed frontmatter
    title: str = Field(..., description="Lesson title from frontmatter")
    description: Optional[str] = Field(None, description="Lesson description from frontmatter")
    doc_id: Optional[str] = Field(None, description="Docusaurus doc ID from frontmatter (for URL generation)")
    module: ModuleName
    chapter: int
    lesson: int
    hardware_tier: HardwareTier = Field(default=1)
    proficiency_level: ProficiencyLevel = Field(default="A2")
    layer: TeachingLayer = Field(default="L1")

    # Content
    raw_content: str = Field(..., description="Full markdown content without frontmatter")
    sections: list[tuple[str, str]] = Field(
        default_factory=list,
        description="List of (section_title, section_content) tuples"
    )


# === Ingestion State Tracking ===

class IngestionRecord(BaseModel):
    """
    Tracks ingestion state for incremental updates.
    Stored in a local JSON file or database.
    """

    source_file: str = Field(..., description="Relative path to source file")
    file_hash: str = Field(..., description="SHA-256 of file at last ingestion")
    chunk_ids: list[str] = Field(default_factory=list, description="UUIDs of chunks from this file")
    chunk_count: int = Field(..., ge=0)
    ingested_at: datetime = Field(default_factory=datetime.utcnow)
    book_id: str
    module: str
    chapter: int
    lesson: int


class IngestionState(BaseModel):
    """
    Complete ingestion state for a book.
    Enables incremental updates by tracking what's already indexed.
    """

    book_id: str
    collection_name: str
    records: dict[str, IngestionRecord] = Field(
        default_factory=dict,
        description="Map of source_file -> IngestionRecord"
    )
    last_full_ingest: Optional[datetime] = None
    last_incremental_update: Optional[datetime] = None
    total_chunks: int = Field(default=0)

    def get_changed_files(self, current_files: dict[str, str]) -> tuple[list[str], list[str], list[str]]:
        """
        Compare current file hashes against stored state.

        Args:
            current_files: Dict of {relative_path: file_hash}

        Returns:
            Tuple of (new_files, modified_files, deleted_files)
        """
        new_files = []
        modified_files = []
        deleted_files = []

        # Find new and modified
        for path, current_hash in current_files.items():
            if path not in self.records:
                new_files.append(path)
            elif self.records[path].file_hash != current_hash:
                modified_files.append(path)

        # Find deleted
        for path in self.records:
            if path not in current_files:
                deleted_files.append(path)

        return new_files, modified_files, deleted_files

    def get_chunks_to_delete(self, files: list[str]) -> list[str]:
        """Get all chunk IDs associated with given files."""
        chunk_ids = []
        for path in files:
            if path in self.records:
                chunk_ids.extend(self.records[path].chunk_ids)
        return chunk_ids


# === Qdrant Payload Schema ===

class QdrantPayload(BaseModel):
    """
    Schema for Qdrant point payload.
    Production-grade with full metadata for filtering and retrieval.
    """

    # Tenant field (is_tenant=true in Qdrant)
    book_id: str

    # Hierarchical structure (all indexed for filtering)
    module: str
    chapter: int
    lesson: int
    hardware_tier: int
    proficiency_level: str
    layer: str

    # Content (for retrieval display)
    text: str
    section_title: Optional[str]
    source_file: str
    lesson_title: Optional[str]
    lesson_description: Optional[str]
    doc_id: Optional[str]  # Docusaurus doc ID for URL generation

    # Parent document reference (for hierarchical retrieval)
    parent_doc_id: str

    # Chunk relationships
    chunk_index: int
    total_chunks: int
    prev_chunk_id: Optional[str]
    next_chunk_id: Optional[str]

    # Metrics
    word_count: int
    token_count: int

    # Change detection & versioning
    content_hash: str
    source_file_hash: str
    version: int

    # Timestamps
    created_at: str
    updated_at: str

    @classmethod
    def from_chunk(cls, chunk: DocumentChunk) -> "QdrantPayload":
        """Convert DocumentChunk to Qdrant payload."""
        return cls(
            book_id=chunk.metadata.book_id,
            module=chunk.metadata.module,
            chapter=chunk.metadata.chapter,
            lesson=chunk.metadata.lesson,
            hardware_tier=chunk.metadata.hardware_tier,
            proficiency_level=chunk.metadata.proficiency_level,
            layer=chunk.metadata.layer,
            text=chunk.text,
            section_title=chunk.metadata.section_title,
            source_file=chunk.metadata.source_file,
            lesson_title=chunk.metadata.lesson_title,
            lesson_description=chunk.metadata.lesson_description,
            doc_id=chunk.metadata.doc_id,
            parent_doc_id=chunk.metadata.parent_doc_id,
            chunk_index=chunk.metadata.chunk_index,
            total_chunks=chunk.metadata.total_chunks,
            prev_chunk_id=chunk.metadata.prev_chunk_id,
            next_chunk_id=chunk.metadata.next_chunk_id,
            word_count=chunk.word_count,
            token_count=chunk.token_count,
            content_hash=chunk.metadata.content_hash,
            source_file_hash=chunk.metadata.source_file_hash,
            version=chunk.metadata.version,
            created_at=chunk.metadata.created_at.isoformat(),
            updated_at=chunk.metadata.updated_at.isoformat(),
        )
