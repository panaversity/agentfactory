"""
Search models for RoboLearn RAG retrieval.

These models define the query and response structures for semantic search.
"""

from pydantic import BaseModel, Field, computed_field
from typing import Optional, Literal


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
    - chapter_range uses Range(gte, lte) for ranges

    Invariant: Results respect all filters simultaneously
    """

    text: str = Field(..., min_length=3, description="Search query text")

    # Required: Book tenant
    book_id: str = Field(
        default="physical-ai-robotics",
        description="Book to search within"
    )

    # Hardware personalization (Range filter: lte) - Optional
    hardware_tier_filter: Optional[int] = Field(
        default=None,
        ge=1,
        le=4,
        description="Optional: Only return content for this tier or lower (None = all tiers)"
    )

    # Content location filters
    module_filter: Optional[ModuleName] = Field(
        None,
        description="Filter to specific module (ros2, gazebo, isaac, vla)"
    )
    chapter_min: Optional[int] = Field(
        None,
        ge=0,
        le=20,
        description="Minimum chapter number (inclusive)"
    )
    chapter_max: Optional[int] = Field(
        None,
        ge=0,
        le=20,
        description="Maximum chapter number (inclusive)"
    )
    lesson_filter: Optional[int] = Field(
        None,
        ge=0,
        le=15,
        description="Filter to specific lesson number"
    )

    # Pedagogical filters
    proficiency_levels: Optional[list[ProficiencyLevel]] = Field(
        None,
        description="Filter to specific proficiency levels (OR logic)"
    )
    layer_filter: Optional[TeachingLayer] = Field(
        None,
        description="Filter to specific teaching layer (L1-L4)"
    )

    # Context expansion
    parent_doc_id: Optional[str] = Field(
        None,
        description="Get all chunks from specific parent document"
    )

    # Pagination
    limit: int = Field(default=5, ge=1, le=20, description="Max results to return")


class SearchResult(BaseModel):
    """
    A single search result with score and metadata.

    Invariant: Results are ordered by score (descending)
    """

    # Content
    text: str = Field(..., description="Retrieved chunk content")
    score: float = Field(..., ge=0.0, le=1.0, description="Cosine similarity score")

    # Location
    source_file: str
    section_title: Optional[str]
    module: str
    chapter: int
    lesson: int

    # Lesson-level metadata (for better context)
    lesson_title: Optional[str] = Field(None, description="Lesson title from frontmatter")
    lesson_description: Optional[str] = Field(None, description="Lesson description from frontmatter")

    # Personalization context
    hardware_tier: int
    proficiency_level: str
    layer: str

    # Production metadata for context expansion
    chunk_index: int = Field(default=0, description="Position in parent document")
    total_chunks: int = Field(default=1, description="Total chunks in parent document")
    parent_doc_id: Optional[str] = Field(None, description="Parent document reference")
    prev_chunk_id: Optional[str] = Field(None, description="Previous chunk for context expansion")
    next_chunk_id: Optional[str] = Field(None, description="Next chunk for context expansion")
    content_hash: Optional[str] = Field(None, description="Content hash for deduplication")

    # Citation and navigation (computed from source_file)
    citation: Optional[str] = Field(None, description="Formatted citation string")
    lesson_url: Optional[str] = Field(None, description="URL to navigate to this lesson")


class SearchResponse(BaseModel):
    """
    Complete search response with multiple results.
    """

    query: str = Field(..., description="Original query text")
    results: list[SearchResult] = Field(default_factory=list)
    total_found: int = Field(..., ge=0, description="Total matching documents")

    # Filter context (for UI display)
    hardware_tier_filter: Optional[int] = Field(None, description="Hardware tier filter applied (None = all tiers)")
    module_filter: Optional[str]
    book_id: str
