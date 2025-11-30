"""
RoboLearn Backend - Modular API for RAG, ChatKit, and Agents

Structure:
- /api/* - API routes (search, health, etc.)
- /rag/* - RAG retrieval and ingestion
- /chatkit/* - ChatKit server (agent works within ChatKit)
- /core/* - Shared utilities and config

To integrate your ChatKit code:
1. Copy your chatkit/ folder here
2. Register router in this app
3. Add search_tool from rag.tools to your agent (within ChatKit)
"""

import logging
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

# Import API routes
from api import search, health

# Simple logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Lifespan manager - resilient to Qdrant failures."""
    logger.info("Starting RoboLearn Backend")
    
    # Initialize RAG search client (non-blocking, fails gracefully)
    try:
        from rag.tools import get_search_client
        client = get_search_client()
        if client is not None:
            logger.info("RAG search client initialized")
        else:
            logger.warning("RAG search client unavailable (Qdrant connection failed)")
    except Exception as e:
        logger.warning(f"RAG initialization failed (non-critical): {e}")
    
    yield
    
    logger.info("Shutting down RoboLearn Backend")


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

# Register ChatKit router (agent works within ChatKit)
_chatkit_registered = False
try:
    from chatkit import router as chatkit_router
    app.include_router(chatkit_router)
    _chatkit_registered = True
    logger.info("ChatKit router registered")
except ImportError:
    logger.info("ChatKit router not found (expected if not yet integrated)")
except Exception as e:
    logger.warning(f"Failed to register ChatKit router: {e}")


@app.get("/")
async def root():
    """Root endpoint."""
    chatkit_status = "✅ Active" if _chatkit_registered else "⏳ Not integrated"
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
        },
        "tool": {
            "name": "search_robolearn_content",
            "import": "from rag.tools import search_tool, SEARCH_TOOL_SCHEMA",
            "note": "Add to your agent within ChatKit",
        },
    }

