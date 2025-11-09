"""
Tests for RAG Service

These tests verify the RAG search functionality.
"""

import pytest
from app.services.rag_service import RAGService, RAGSearchRequest


@pytest.mark.asyncio
async def test_rag_search_entire_book():
    """Test RAG search with entire book scope."""
    # Create RAG service
    rag_service = RAGService()

    # Create search request
    request = RAGSearchRequest(
        query="What is Python?",
        scope="entire_book",
        n_results=5
    )

    # Execute search
    response = await rag_service.search(request)

    # Verify response
    assert response.query == "What is Python?"
    assert response.scope == "entire_book"
    assert response.total_results >= 0
    assert response.search_time_ms >= 0


@pytest.mark.asyncio
async def test_rag_search_with_chapter_scope():
    """Test RAG search with chapter scope."""
    rag_service = RAGService()

    request = RAGSearchRequest(
        query="async programming",
        scope="current_chapter",
        n_results=5,
        current_chapter="04-part-4-python-fundamentals"
    )

    response = await rag_service.search(request)

    assert response.query == "async programming"
    assert response.scope == "current_chapter"
    assert response.total_results >= 0


@pytest.mark.asyncio
async def test_rag_search_missing_context():
    """Test that RAG search raises error when context is missing."""
    rag_service = RAGService()

    # Missing current_chapter for current_chapter scope
    request = RAGSearchRequest(
        query="test",
        scope="current_chapter",
        n_results=5
    )

    with pytest.raises(ValueError):
        await rag_service.search(request)


def test_rag_search_sync():
    """Test synchronous RAG search."""
    rag_service = RAGService()

    request = RAGSearchRequest(
        query="What is AI-driven development?",
        scope="entire_book",
        n_results=3
    )

    response = rag_service.search_sync(request)

    assert response.query == "What is AI-driven development?"
    assert response.total_results >= 0
    assert isinstance(response.results, list)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
