"""
FastAPI application entry point for Content Summarization API
"""
import logging
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
import os

# Load environment variables
load_dotenv()

# Configure logging
log_level = os.getenv("LOG_LEVEL", "INFO")
logging.basicConfig(
    level=getattr(logging, log_level),
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# Initialize FastAPI app
app = FastAPI(
    title="Content Summarization API",
    description="API for generating AI-powered summaries of content pages with streaming support",
    version="1.0.0"
)

# Configure CORS
cors_origins = os.getenv("CORS_ORIGINS", "http://localhost:3000").split(",")
app.add_middleware(
    CORSMiddleware,
    allow_origins=cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {"status": "healthy", "service": "content-summarization-api"}

@app.get("/")
async def root():
    """Root endpoint"""
    return {
        "message": "Content Summarization API",
        "version": "1.0.0",
        "docs": "/docs"
    }

# TODO: Register routers (will be added in Phase 5)
# from api.src.routers import summarize, auth
# app.include_router(summarize.router, prefix="/api/v1", tags=["summarization"])
# app.include_router(auth.router, prefix="/api/v1/auth", tags=["authentication"])

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
