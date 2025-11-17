"""
T043-T050: Personalization endpoint router with SSE streaming.

Provides GET /personalize endpoint that streams AI-personalized content
based on user proficiency levels using Server-Sent Events (SSE).

Pattern mirrors summarize.py but adds programmingLevel and aiLevel parameters.
"""

import json
import logging
from fastapi import APIRouter, Query, HTTPException
from fastapi.responses import StreamingResponse
from api.src.services import openai_agent

router = APIRouter()
logger = logging.getLogger(__name__)


# T044: GET /personalize endpoint with proficiency query params
@router.get("/personalize")
async def personalize_content(
    pageId: str = Query(..., description="Unique identifier for the content page"),
    content: str = Query(..., description="Full page content text to personalize"),
    token: str = Query(..., description="Authentication token (dummy_token prefix)"),
    programmingLevel: str = Query(..., description="User's programming proficiency (Novice/Beginner/Intermediate/Expert)"),
    aiLevel: str = Query(..., description="User's AI knowledge proficiency (Novice/Beginner/Intermediate/Expert)"),
):
    """
    Stream AI-personalized content tailored to user's proficiency levels.
    
    Uses Server-Sent Events (SSE) to stream personalized content as it's generated,
    providing real-time feedback to the user.
    
    Query Parameters:
        pageId: Unique identifier for the content page
        content: Full page content text to personalize
        token: Authentication token (must start with 'dummy_token')
        programmingLevel: Programming proficiency (Novice/Beginner/Intermediate/Expert)
        aiLevel: AI knowledge proficiency (Novice/Beginner/Intermediate/Expert)
    
    Returns:
        StreamingResponse: SSE stream of personalized content chunks
        
    Raises:
        HTTPException 401: If token is missing or invalid
        HTTPException 400: If content is too short or proficiency levels are invalid
    """
    logger.info(f"Personalization request for pageId: {pageId}, Programming: {programmingLevel}, AI: {aiLevel}")
    
    # T045: Token validation
    if not token:
        logger.warning("Personalization request missing token")
        raise HTTPException(status_code=401, detail="Authentication token required")
    
    if not token.startswith("dummy_token"):
        logger.warning(f"Invalid token format: {token[:20]}...")
        raise HTTPException(status_code=401, detail="Invalid authentication token")
    
    # T046: Content validation (minimum 100 characters)
    if len(content) < 100:
        logger.warning(f"Content too short: {len(content)} characters")
        raise HTTPException(
            status_code=400,
            detail="Content must be at least 100 characters for personalization"
        )
    
    # T047: Validate proficiency levels
    valid_levels = ["Novice", "Beginner", "Intermediate", "Expert"]
    
    if programmingLevel not in valid_levels:
        logger.warning(f"Invalid programming level: {programmingLevel}")
        raise HTTPException(
            status_code=400,
            detail=f"Invalid programmingLevel. Must be one of: {', '.join(valid_levels)}"
        )
    
    if aiLevel not in valid_levels:
        logger.warning(f"Invalid AI level: {aiLevel}")
        raise HTTPException(
            status_code=400,
            detail=f"Invalid aiLevel. Must be one of: {', '.join(valid_levels)}"
        )
    
    # T048: Define event stream generator
    async def event_stream():
        """
        Generate Server-Sent Events stream for personalized content.
        
        Yields SSE-formatted events containing personalized content chunks.
        Follows SSE format: data: {JSON}\n\n
        """
        try:
            # T049: Stream chunks from generate_personalized_content()
            async for chunk in openai_agent.generate_personalized_content(
                content=content,
                page_id=pageId,
                programming_level=programmingLevel,
                ai_proficiency=aiLevel
            ):
                event_data = json.dumps({
                    "chunk": chunk,
                    "done": False
                })
                yield f"data: {event_data}\n\n"
            
            # Send completion event
            completion_data = json.dumps({
                "chunk": "",
                "done": True
            })
            yield f"data: {completion_data}\n\n"
            
            logger.info(f"Personalization stream completed for pageId: {pageId}")
            
        except Exception as e:
            logger.error(f"Error during personalization stream for pageId {pageId}: {str(e)}")
            # Send error event
            error_data = json.dumps({
                "chunk": "",
                "done": True,
                "error": str(e)
            })
            yield f"data: {error_data}\n\n"
    
    # Return SSE streaming response
    return StreamingResponse(
        event_stream(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no",  # Disable nginx buffering
        }
    )
