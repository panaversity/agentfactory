"""
RoboLearn Backend - Modular API for RAG, ChatKit, and Agents

Structure:
- /api/* - API routes (search, health, etc.)
- /rag/* - RAG retrieval and ingestion
- /chatkit/* - ChatKit server (agent works within ChatKit)
- /core/* - Shared utilities and config
"""

import json
import logging
from fastapi import FastAPI, Request, HTTPException, Header
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import Response, StreamingResponse
from chatkit.server import StreamingResult

# Import API routes
from api import search, health
from lifespan import lifespan
from chatkit_store import RequestContext

# Simple logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


app = FastAPI(
    title="RoboLearn Backend",
    description="RAG search, ChatKit, and Agent integration for educational content",
    version="1.0.0",
    lifespan=lifespan,
)

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Register API routes
app.include_router(search.router)
app.include_router(health.router)


def get_request_context(
    user_id: str = Header(..., alias="X-User-ID"),
    request_id: str | None = Header(None, alias="X-Request-ID"),
) -> RequestContext:
    """
    Extract request context from headers.

    In production, you should validate authentication here.
    """
    return RequestContext(
        user_id=user_id,
        request_id=request_id,
    )


@app.post("/chatkit")
async def chatkit_endpoint(request: Request):
    """
    Main ChatKit endpoint for conversational interaction.

    Requires X-User-ID header for user identification.
    """
    payload = await request.body()
    logger.debug(f"chatkit_endpoint received payload: {len(payload)} bytes")
    
    # Get server from app state
    chatkit_server = getattr(request.app.state, "chatkit_server", None)
    if not chatkit_server:
        raise HTTPException(
            status_code=503,
            detail="ChatKit server not initialized. Check database configuration.",
        )

    # Extract user ID from header
    user_id = request.headers.get("X-User-ID")
    if not user_id:
        raise HTTPException(status_code=401, detail="Missing X-User-ID header")

    try:
        # Process ChatKit request
        payload = await request.body()
        # decode payload to dict from bytes and add in metadata
        payload_dict = json.loads(payload)

        # Extract metadata from the correct location in ChatKit request
        metadata = {}
        if "params" in payload_dict and "input" in payload_dict["params"]:
            metadata = payload_dict["params"]["input"].get("metadata", {})

        logger.debug(f"Extracted metadata: {metadata}")

        # Create request context
        context = RequestContext(
            user_id=user_id,
            request_id=request.headers.get("X-Request-ID"),
            metadata=metadata,
        )
        result = await chatkit_server.process(payload, context)

        # Return appropriate response type
        if isinstance(result, StreamingResult):
            return StreamingResponse(
                result,
                media_type="text/event-stream",
                headers={
                    "Cache-Control": "no-cache",
                    "Connection": "keep-alive",
                    "X-Accel-Buffering": "no",
                },
            )
        else:
            return Response(
                content=result.json,
                media_type="application/json",
            )
    except Exception as e:
        logger.exception(f"Error processing ChatKit request: {e}")
        raise HTTPException(
            status_code=500, detail=f"Error processing request: {str(e)}"
        )


@app.get("/")
async def root():
    """Root endpoint."""
    chatkit_status = "✅ Active" if hasattr(app.state, "chatkit_server") and app.state.chatkit_server else "⏳ Not initialized"
    return {
        "service": "RoboLearn Backend",
        "version": "1.0.0",
        "modules": {
            "rag": "✅ Active",
            "chatkit": chatkit_status,
        },
        "endpoints": {
            "search": "POST /search",
            "health": "GET /health",
            "chatkit": "POST /chatkit",
        },
        "tool": {
            "name": "search_robolearn_content",
            "import": "from rag.tools import search_tool, SEARCH_TOOL_SCHEMA",
            "note": "Integrated with ChatKit agent",
        },
    }

