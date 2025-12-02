"""Health check API routes."""

from fastapi import APIRouter
from pydantic import BaseModel
from rag.ingestion.uploader import QdrantUploader
import logging

logger = logging.getLogger(__name__)

router = APIRouter(tags=["health"])


class HealthResponse(BaseModel):
    """Health check response."""
    status: str
    qdrant: str
    collection: str


@router.get("/health")
async def health():
    """Health check endpoint."""
    try:
        uploader = QdrantUploader()
        info = uploader.get_collection_info()
        
        if "error" in info:
            return HealthResponse(
                status="degraded",
                qdrant="error",
                collection=info.get("error", "unknown"),
            )
        
        return HealthResponse(
            status="healthy",
            qdrant="connected",
            collection=info.get("status", "unknown"),
        )
    except Exception as e:
        logger.error(f"Health check error: {e}", exc_info=True)
        return HealthResponse(
            status="unhealthy",
            qdrant="disconnected",
            collection=str(e),
        )

