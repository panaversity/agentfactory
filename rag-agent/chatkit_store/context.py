"""Request context for ChatKit store operations."""

from typing import Any
from pydantic import BaseModel, Field


class RequestContext(BaseModel):
    """
    Request context passed to all store operations.
    
    Provides user isolation and optional metadata for access control,
    logging, and tracing across all store operations.
    """
    
    user_id: str | None = Field(
        ...,
        description="Unique identifier for the user making the request",
        min_length=1
    )
    
    organization_id: str | None = Field(
        default=None,
        description="Optional organization ID for multi-org tenancy"
    )
    
    request_id: str | None = Field(
        default=None,
        description="Optional request ID for tracing and logging"
    )
    
    metadata: dict[str, Any] = Field(
        default_factory=dict,
        description="Additional context metadata"
    )
    
    class Config:
        """Pydantic configuration."""
        frozen = False  # Allow mutation for adding trace info
        json_encoders = {
            # Add custom encoders if needed
        }

