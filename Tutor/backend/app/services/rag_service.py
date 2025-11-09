"""
RAG Service - Retrieval-Augmented Generation

This service provides semantic search over book content using:
- Gemini embeddings (gemini-embedding-001)
- ChromaDB vector storage
- Multi-level scoping (lesson, chapter, book)
"""

import time
from typing import List, Dict, Any, Literal, Optional
from pydantic import BaseModel, Field

from app.services.embedding_service import EmbeddingService
from app.services.vector_store import VectorStore


# Pydantic models matching the API contract
class RAGSearchRequest(BaseModel):
    """Request for RAG search."""
    query: str = Field(..., min_length=1, max_length=500)
    scope: Literal['current_lesson', 'current_chapter', 'entire_book'] = 'current_lesson'
    n_results: int = Field(default=5, ge=1, le=20)

    # Context (required for lesson/chapter scope)
    current_chapter: Optional[str] = None
    current_lesson: Optional[str] = None


class RAGSearchResult(BaseModel):
    """Single RAG search result."""
    chunk_id: str
    content: str
    score: float = Field(..., ge=0.0, le=1.0, description="Similarity score")
    metadata: Dict[str, Any]


class RAGSearchResponse(BaseModel):
    """Response from RAG search."""
    query: str
    scope: str
    results: List[RAGSearchResult]
    total_results: int
    search_time_ms: int


class RAGService:
    """
    RAG service for semantic search over book content.

    Features:
    - Multi-level scoping (lesson, chapter, entire book)
    - Gemini embeddings for queries
    - ChromaDB for vector search
    - Metadata filtering
    """

    def __init__(
        self,
        embedding_service: Optional[EmbeddingService] = None,
        vector_store: Optional[VectorStore] = None
    ):
        """
        Initialize RAG service.

        Args:
            embedding_service: Service for generating embeddings
            vector_store: Vector store for searching
        """
        self.embedding_service = embedding_service or EmbeddingService()
        self.vector_store = vector_store or VectorStore()

    async def search(self, request: RAGSearchRequest) -> RAGSearchResponse:
        """
        Search book content using RAG.

        Args:
            request: RAG search request

        Returns:
            RAG search response with results

        Raises:
            ValueError: If required context fields are missing
        """
        # Validate context requirements
        if request.scope == 'current_lesson':
            if not request.current_chapter or not request.current_lesson:
                raise ValueError(
                    "current_chapter and current_lesson are required when scope is 'current_lesson'"
                )
        elif request.scope == 'current_chapter':
            if not request.current_chapter:
                raise ValueError(
                    "current_chapter is required when scope is 'current_chapter'"
                )

        # Start timing
        start_time = time.time()

        # Generate query embedding
        query_embedding = self.embedding_service.embed_query(request.query)

        # Build metadata filter based on scope
        where_filter = self._build_filter(
            request.scope,
            request.current_chapter,
            request.current_lesson
        )

        # Execute vector search
        search_results = self.vector_store.search(
            query_embedding=query_embedding,
            n_results=request.n_results,
            where=where_filter
        )

        # Calculate search time
        search_time_ms = int((time.time() - start_time) * 1000)

        # Format results
        results = self._format_results(search_results)

        return RAGSearchResponse(
            query=request.query,
            scope=request.scope,
            results=results,
            total_results=len(results),
            search_time_ms=search_time_ms
        )

    def search_sync(self, request: RAGSearchRequest) -> RAGSearchResponse:
        """
        Synchronous version of search (for non-async contexts).

        Args:
            request: RAG search request

        Returns:
            RAG search response with results
        """
        # Same as async version but without await
        # Validate context requirements
        if request.scope == 'current_lesson':
            if not request.current_chapter or not request.current_lesson:
                raise ValueError(
                    "current_chapter and current_lesson are required when scope is 'current_lesson'"
                )
        elif request.scope == 'current_chapter':
            if not request.current_chapter:
                raise ValueError(
                    "current_chapter is required when scope is 'current_chapter'"
                )

        # Start timing
        start_time = time.time()

        # Generate query embedding
        query_embedding = self.embedding_service.embed_query(request.query)

        # Build metadata filter based on scope
        where_filter = self._build_filter(
            request.scope,
            request.current_chapter,
            request.current_lesson
        )

        # Execute vector search
        search_results = self.vector_store.search(
            query_embedding=query_embedding,
            n_results=request.n_results,
            where=where_filter
        )

        # Calculate search time
        search_time_ms = int((time.time() - start_time) * 1000)

        # Format results
        results = self._format_results(search_results)

        return RAGSearchResponse(
            query=request.query,
            scope=request.scope,
            results=results,
            total_results=len(results),
            search_time_ms=search_time_ms
        )

    def _build_filter(
        self,
        scope: str,
        current_chapter: Optional[str],
        current_lesson: Optional[str]
    ) -> Optional[Dict[str, Any]]:
        """
        Build metadata filter based on scope.

        Args:
            scope: Search scope (current_lesson, current_chapter, entire_book)
            current_chapter: Current chapter ID
            current_lesson: Current lesson ID

        Returns:
            Metadata filter for ChromaDB or None for entire_book
        """
        if scope == 'current_lesson' and current_lesson:
            return {"lesson": current_lesson}
        elif scope == 'current_chapter' and current_chapter:
            return {"chapter": current_chapter}
        else:
            # entire_book: no filter
            return None

    def _format_results(self, search_results: Dict[str, Any]) -> List[RAGSearchResult]:
        """
        Format ChromaDB search results into RAGSearchResult objects.

        Args:
            search_results: Raw results from ChromaDB

        Returns:
            List of formatted search results
        """
        results = []

        # ChromaDB returns nested lists: results['ids'][0], results['documents'][0], etc.
        if not search_results['ids'] or not search_results['ids'][0]:
            return results

        for i in range(len(search_results['ids'][0])):
            # Convert distance to similarity score (1.0 - distance for cosine)
            # ChromaDB returns distance, we want similarity
            distance = search_results['distances'][0][i]
            score = 1.0 - distance  # Cosine similarity

            # Ensure score is in [0, 1]
            score = max(0.0, min(1.0, score))

            results.append(RAGSearchResult(
                chunk_id=search_results['ids'][0][i],
                content=search_results['documents'][0][i],
                score=score,
                metadata=search_results['metadatas'][0][i]
            ))

        return results


# Test function
if __name__ == "__main__":
    import asyncio

    async def test_rag():
        # Initialize service
        rag_service = RAGService()

        # Note: This test requires the vector store to be populated
        # Run ingest_book.py first to populate the database

        # Test search
        request = RAGSearchRequest(
            query="What is Python?",
            scope="entire_book",
            n_results=5
        )

        try:
            response = await rag_service.search(request)
            print(f"Query: {response.query}")
            print(f"Scope: {response.scope}")
            print(f"Found {response.total_results} results in {response.search_time_ms}ms")
            print("\nResults:")
            for i, result in enumerate(response.results, 1):
                print(f"\n[{i}] Score: {result.score:.3f}")
                print(f"Content: {result.content[:200]}...")
                print(f"Metadata: {result.metadata}")

        except Exception as e:
            print(f"Error: {e}")
            print("\nNote: Make sure to run ingest_book.py first to populate the vector store")

    # Run test
    asyncio.run(test_rag())
