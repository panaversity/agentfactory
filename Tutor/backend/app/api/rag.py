"""
RAG API Router

Provides the /api/rag/search endpoint for semantic search over book content.
"""

from fastapi import APIRouter, HTTPException, status
from typing import Dict, Any

from app.services.rag_service import RAGService, RAGSearchRequest, RAGSearchResponse


# Create router
router = APIRouter(prefix="/api/rag", tags=["RAG"])

# Initialize RAG service (singleton)
_rag_service = None


def get_rag_service() -> RAGService:
    """Get or create the RAG service."""
    global _rag_service
    if _rag_service is None:
        _rag_service = RAGService()
    return _rag_service


@router.post("/search", response_model=RAGSearchResponse)
async def search_rag(request: RAGSearchRequest) -> RAGSearchResponse:
    """
    Search book content using RAG.

    This endpoint performs semantic search over the book content using:
    - Gemini embeddings (gemini-embedding-001)
    - ChromaDB vector search
    - Multi-level scoping (lesson, chapter, entire book)

    Args:
        request: RAG search request

    Returns:
        Search results with metadata and similarity scores

    Raises:
        400: Invalid request (missing required context fields)
        500: Search error
    """
    try:
        # Get RAG service
        rag_service = get_rag_service()

        # Execute search
        response = await rag_service.search(request)

        return response

    except ValueError as e:
        # Validation errors (missing context fields, etc.)
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={
                "error": "Invalid request",
                "details": [
                    {
                        "code": "VALIDATION_ERROR",
                        "message": str(e)
                    }
                ]
            }
        )

    except Exception as e:
        # Search errors
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "Search failed",
                "details": [
                    {
                        "code": "SEARCH_ERROR",
                        "message": "An error occurred during search. Please try again."
                    }
                ]
            }
        )


@router.get("/health")
async def health_check() -> Dict[str, Any]:
    """
    Health check endpoint for RAG service.

    Returns:
        Service status and statistics
    """
    try:
        rag_service = get_rag_service()
        stats = rag_service.vector_store.get_stats()

        return {
            "status": "healthy",
            "vector_store": {
                "collection": stats["name"],
                "count": stats["count"],
                "metadata": stats["metadata"]
            }
        }

    except Exception as e:
        return {
            "status": "unhealthy",
            "error": str(e)
        }
