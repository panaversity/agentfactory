"""
TutorGPT FastAPI Application

Main FastAPI application for TutorGPT MVP.
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.api.rag import router as rag_router


# Create FastAPI app
app = FastAPI(
    title="TutorGPT API",
    description="AI-Native Software Development Tutor with RAG",
    version="0.1.0"
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://localhost:8000"],  # Docusaurus dev server
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(rag_router)


@app.get("/")
async def root():
    """Root endpoint."""
    return {
        "name": "TutorGPT API",
        "version": "0.1.0",
        "description": "AI-Native Software Development Tutor",
        "endpoints": {
            "rag_search": "/api/rag/search",
            "rag_health": "/api/rag/health",
            "docs": "/docs"
        }
    }


@app.get("/health")
async def health():
    """Health check endpoint."""
    return {"status": "healthy"}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
