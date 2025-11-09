"""
TutorGPT FastAPI Application

Main FastAPI application for TutorGPT MVP.
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.api.rag import router as rag_router
from app.api.auth import router as auth_router
from app.api.profile import router as profile_router
from app.api.chat import router as chat_router
from app.database import init_db


# Create FastAPI app
app = FastAPI(
    title="TutorGPT API",
    description="AI-Native Software Development Tutor with RAG and User Authentication",
    version="0.2.0"
)


@app.on_event("startup")
async def startup_event():
    """Initialize database on startup."""
    init_db()
    print("✅ Database initialized")

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://localhost:8000"],  # Docusaurus dev server
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(auth_router)
app.include_router(profile_router)
app.include_router(chat_router)
app.include_router(rag_router)  # Optional - for advanced use


@app.get("/")
async def root():
    """Root endpoint."""
    return {
        "name": "TutorGPT API",
        "version": "0.2.0",
        "description": "AI-Native Software Development Tutor with RAG and User Authentication",
        "endpoints": {
            "auth_signup": "/api/auth/signup",
            "auth_login": "/api/auth/login",
            "auth_me": "/api/auth/me",
            "profile_get": "/api/profile",
            "profile_update": "/api/profile",
            "chat_message": "/api/chat/message",
            "chat_greeting": "/api/chat/greeting",
            "chat_status": "/api/chat/status",
            "rag_search": "/api/rag/search (optional - advanced)",
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
