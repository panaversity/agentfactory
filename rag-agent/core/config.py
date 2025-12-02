"""
Configuration management for RoboLearn backend.
Loads settings from environment variables with validation.
"""

from pydantic_settings import BaseSettings
from pydantic import Field
from functools import lru_cache


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Qdrant Cloud
    qdrant_url: str = Field(..., description="Qdrant Cloud cluster URL")
    qdrant_api_key: str = Field(..., description="Qdrant API key")

    # OpenAI
    openai_api_key: str = Field(..., description="OpenAI API key")

    # Book Configuration
    book_id: str = Field(default="physical-ai-robotics", description="Current book identifier")
    docs_path: str = Field(default="../robolearn-interface/docs", description="Path to docs folder")

    # Collection Configuration
    collection_name: str = Field(default="robolearn_platform", description="Qdrant collection name")
    embedding_model: str = Field(default="text-embedding-3-small", description="OpenAI embedding model")
    embedding_dimensions: int = Field(default=1536, description="Embedding vector dimensions")

    # Chunking Configuration
    max_chunk_words: int = Field(default=500, description="Maximum words per chunk")
    min_chunk_words: int = Field(default=50, description="Minimum words per chunk")

    # Batch Configuration
    batch_size: int = Field(default=100, description="Batch size for Qdrant upserts")

    # Database (Neon Postgres) - Required for ChatKit
    database_url: str = Field(
        default="",
        description="Postgres connection URL (postgresql+asyncpg://user:pass@host:port/db). Required for ChatKit."
    )

    # Admin API
    admin_api_key: str = Field(default="", description="API key for admin endpoints")

    # GitHub Webhook
    github_webhook_secret: str = Field(default="", description="GitHub webhook secret for signature verification")

    # Frontend URL (for generating lesson links)
    frontend_base_url: str = Field(
        default="https://mjunaidca.github.io/robolearn",
        description="Base URL for frontend (for generating lesson navigation links)"
    )

    model_config = {
        "env_file": ".env",
        "env_file_encoding": "utf-8",
        "extra": "ignore"
    }


@lru_cache
def get_settings() -> Settings:
    """Get cached settings instance."""
    return Settings()
