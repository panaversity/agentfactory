"""Search API routes."""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from typing import Optional

from models.search import SearchQuery, SearchResponse
from rag.search import RoboLearnSearch
from core.config import get_settings
import logging

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/search", tags=["search"])


class SearchRequest(BaseModel):
    """Search request with optional filters."""
    query: str = Field(..., min_length=3, description="Search query text")
    hardware_tier: Optional[int] = Field(None, ge=1, le=4, description="Optional hardware tier filter")
    module: Optional[str] = Field(None, description="Filter by module: ros2, gazebo, isaac, vla")
    chapter_min: Optional[int] = Field(None, ge=0, le=20, description="Minimum chapter number (inclusive)")
    chapter_max: Optional[int] = Field(None, ge=0, le=20, description="Maximum chapter number (inclusive)")
    lesson: Optional[int] = Field(None, ge=0, le=15, description="Specific lesson number")
    limit: int = Field(default=5, ge=1, le=20, description="Max results")


@router.post("", response_model=SearchResponse)
async def search(request: SearchRequest):
    """Semantic search with optional filters."""
    try:
        from rag.tools import get_search_client
        client = get_search_client()
        if client is None:
            raise HTTPException(
                status_code=503,
                detail="Search service unavailable (Qdrant connection failed)"
            )
        
        settings = get_settings()
        query = SearchQuery(
            text=request.query,
            book_id=settings.book_id,
            hardware_tier_filter=request.hardware_tier,
            module_filter=request.module,
            chapter_min=request.chapter_min,
            chapter_max=request.chapter_max,
            lesson_filter=request.lesson,
            limit=request.limit,
        )
        return client.search(query)
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Search error: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))

