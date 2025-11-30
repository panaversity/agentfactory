"""
Semantic search for RoboLearn RAG.

Queries Qdrant with hardware-tier filtering and returns relevant content.
"""

from typing import Optional

from qdrant_client import QdrantClient
from qdrant_client.models import Filter, FieldCondition, MatchValue, MatchAny, Range

from models.search import SearchQuery, SearchResult, SearchResponse
from rag.ingestion.embedder import OpenAIEmbedder
from core.config import get_settings
import logging

logger = logging.getLogger(__name__)


class RoboLearnSearch:
    """
    Semantic search over RoboLearn content.

    Supports:
    - Hardware tier filtering (only return content for user's tier or lower)
    - Module filtering (search within specific module)
    - Book filtering (multitenancy)
    """

    def __init__(
        self,
        qdrant_url: str = None,
        qdrant_api_key: str = None,
        openai_api_key: str = None,
        collection_name: str = None,
    ):
        """
        Initialize search client.

        Args:
            qdrant_url: Qdrant Cloud URL
            qdrant_api_key: Qdrant API key
            openai_api_key: OpenAI API key for query embedding
            collection_name: Collection name
        """
        settings = get_settings()

        self.client = QdrantClient(
            url=qdrant_url or settings.qdrant_url,
            api_key=qdrant_api_key or settings.qdrant_api_key,
        )
        self.embedder = OpenAIEmbedder(api_key=openai_api_key)
        self.collection_name = collection_name or settings.collection_name

    def search(self, query: SearchQuery) -> SearchResponse:
        """
        Perform semantic search with comprehensive filters.

        Filter Strategy (all AND logic):
        - book_id: exact match (tenant isolation)
        - hardware_tier: range lte (tier X or lower)
        - module: exact match
        - chapter: range gte/lte
        - lesson: exact match
        - proficiency_levels: MatchAny (OR within field)
        - layer: exact match
        - parent_doc_id: exact match (context expansion)

        Args:
            query: SearchQuery with text and filters

        Returns:
            SearchResponse with results
        """
        logger.info(
            f"Search: query='{query.text[:50]}...' "
            f"tier={query.hardware_tier_filter} "
            f"module={query.module_filter} "
            f"limit={query.limit}"
        )

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

        # Optional: Hardware tier filter (tier X or lower)
        if query.hardware_tier_filter is not None:
            conditions.append(
                FieldCondition(
                    key="hardware_tier",
                    range=Range(lte=query.hardware_tier_filter),
                )
            )

        # Optional: Module filter
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
                FieldCondition(
                    key="chapter",
                    range=chapter_range,
                )
            )

        # Optional: Lesson filter
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

        # Optional: Parent document filter (for context expansion)
        if query.parent_doc_id:
            conditions.append(
                FieldCondition(
                    key="parent_doc_id",
                    match=MatchValue(value=query.parent_doc_id),
                )
            )

        # Execute search
        try:
            results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                query_filter=Filter(must=conditions),
                limit=query.limit,
                with_payload=True,
            )

            logger.info(f"Search completed: found {len(results.points)} results")
        except Exception as e:
            logger.error(f"Search error: {e}", exc_info=True)
            raise

        # Convert to response
        settings = get_settings()
        search_results = []
        for idx, point in enumerate(results.points):
            payload = point.payload
            
            # Build citation
            citation_parts = []
            lesson_title = payload.get("lesson_title")
            if lesson_title:
                citation_parts.append(lesson_title)
            else:
                chapter = payload.get("chapter", 0)
                lesson = payload.get("lesson", 0)
                if lesson > 0:
                    citation_parts.append(f"Lesson {chapter}.{lesson}")
                elif chapter > 0:
                    citation_parts.append(f"Chapter {chapter}")
            
            section_title = payload.get("section_title")
            if section_title:
                citation_parts.append(f"Section: {section_title}")
            
            module = payload.get("module", "")
            if module:
                citation_parts.append(f"Module {module.upper()}")
            
            citation = " | ".join(citation_parts) if citation_parts else None
            
            # Build lesson URL
            # Priority: Use doc_id from frontmatter if available, otherwise generate from source_file
            doc_id = payload.get("doc_id")
            source_file = payload.get("source_file", "")
            
            if doc_id:
                # Use Docusaurus doc ID directly (most accurate)
                # Example: "lesson-1-3-humanoid-revolution" → "/docs/module-1-ros2/chapter-1-physical-ai/lesson-1-3-humanoid-revolution"
                # Extract module and chapter from source_file path
                if source_file:
                    path_parts = source_file.split("/")
                    if len(path_parts) >= 2:
                        # module-1-ros2/chapter-1-physical-ai/03-humanoid-revolution.md
                        module_part = path_parts[0]  # module-1-ros2
                        if len(path_parts) >= 2 and not path_parts[1].endswith(".md"):
                            # Has chapter directory
                            chapter_part = path_parts[1]  # chapter-1-physical-ai
                            lesson_url = f"{settings.frontend_base_url}/docs/{module_part}/{chapter_part}/{doc_id}"
                        else:
                            # Module-level (no chapter)
                            lesson_url = f"{settings.frontend_base_url}/docs/{module_part}/{doc_id}"
                    else:
                        lesson_url = f"{settings.frontend_base_url}/docs/{doc_id}"
                else:
                    lesson_url = f"{settings.frontend_base_url}/docs/{doc_id}"
            elif source_file:
                # Fallback: Generate from source_file path
                # Examples:
                # - "module-1-ros2/README.md" → "/docs/module-1-ros2"
                # - "module-1-ros2/chapter-2-robot-system/README.md" → "/docs/module-1-ros2/chapter-2-robot-system"
                path = source_file.replace(".md", "")
                # Remove README from path (Docusaurus doesn't include README in URL)
                if path.endswith("/README"):
                    path = path[:-7]  # Remove "/README"
                elif path.endswith("README"):
                    # Handle case where path is just "module-1-ros2/README"
                    parts = path.split("/")
                    if len(parts) > 1 and parts[-1] == "README":
                        path = "/".join(parts[:-1])
                
                lesson_url = f"{settings.frontend_base_url}/docs/{path}"
            else:
                lesson_url = None

            search_results.append(SearchResult(
                text=payload.get("text", ""),
                score=point.score,
                source_file=source_file,
                section_title=section_title,
                module=module,
                chapter=payload.get("chapter", 0),
                lesson=payload.get("lesson", 0),
                # Lesson metadata (from frontmatter)
                lesson_title=lesson_title,
                lesson_description=payload.get("lesson_description"),
                hardware_tier=payload.get("hardware_tier", 1),
                proficiency_level=payload.get("proficiency_level", "A2"),
                layer=payload.get("layer", "L1"),
                # Production metadata
                chunk_index=payload.get("chunk_index", 0),
                total_chunks=payload.get("total_chunks", 1),
                parent_doc_id=payload.get("parent_doc_id"),
                prev_chunk_id=payload.get("prev_chunk_id"),
                next_chunk_id=payload.get("next_chunk_id"),
                content_hash=payload.get("content_hash"),
                # Citation and navigation
                citation=citation,
                lesson_url=lesson_url,
            ))

        return SearchResponse(
            query=query.text,
            results=search_results,
            total_found=len(search_results),
            hardware_tier_filter=query.hardware_tier_filter,  # Can be None
            module_filter=query.module_filter,
            book_id=query.book_id,
        )

    def search_simple(
        self,
        text: str,
        hardware_tier: int = 1,
        module: Optional[str] = None,
        book_id: str = "physical-ai-robotics",
        limit: int = 5,
    ) -> SearchResponse:
        """
        Simplified search interface.

        Args:
            text: Search query text
            hardware_tier: User's hardware tier (1-4)
            module: Optional module filter
            book_id: Book to search
            limit: Max results

        Returns:
            SearchResponse
        """
        query = SearchQuery(
            text=text,
            hardware_tier_filter=hardware_tier,
            module_filter=module,
            book_id=book_id,
            limit=limit,
        )
        return self.search(query)

    def get_chunk_by_id(self, chunk_id: str) -> Optional[SearchResult]:
        """
        Retrieve a specific chunk by its UUID.

        Used for context expansion (fetching prev/next chunks).

        Args:
            chunk_id: UUID of the chunk

        Returns:
            SearchResult or None if not found
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

    def expand_context(
        self,
        chunk_id: str,
        prev_count: int = 1,
        next_count: int = 1,
    ) -> list[SearchResult]:
        """
        Get a chunk and its neighboring chunks for context expansion.

        Walks the prev_chunk_id/next_chunk_id chain.

        Args:
            chunk_id: Starting chunk UUID
            prev_count: Number of previous chunks to include
            next_count: Number of next chunks to include

        Returns:
            List of chunks in order [prev..., current, next...]
        """
        result = []
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
            prev_chunks.insert(0, prev_chunk)
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

    def get_lesson_chunks(
        self,
        parent_doc_id: str,
        book_id: str = "physical-ai-robotics",
    ) -> list[SearchResult]:
        """
        Get all chunks for a lesson/document, ordered by chunk_index.

        Useful for showing full lesson context.

        Args:
            parent_doc_id: Parent document UUID
            book_id: Book identifier

        Returns:
            List of all chunks in order
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

        chunks = []
        for point in points:
            payload = point.payload
            chunks.append(SearchResult(
                text=payload.get("text", ""),
                score=1.0,
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
            ))

        # Sort by chunk_index
        chunks.sort(key=lambda c: c.chunk_index)
        return chunks
