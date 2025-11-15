"""
Pydantic schemas for API request/response models
"""
from pydantic import BaseModel, Field
from typing import Optional


class SummaryChunk(BaseModel):
    """Streaming summary response chunk"""
    chunk: str = Field(..., description="Partial summary text chunk")
    done: bool = Field(..., description="Whether streaming is complete")
    error: Optional[str] = Field(None, description="Error message if generation failed")


class ErrorResponse(BaseModel):
    """Standard error response"""
    error: str = Field(..., description="Error type")
    message: str = Field(..., description="Human-readable error message")


class AuthResponse(BaseModel):
    """Authentication response"""
    token: str = Field(..., description="Authentication token")
    expires: str = Field(..., description="Token expiration (e.g., 'session')")
    user: dict = Field(..., description="User information")


class SummaryRequest(BaseModel):
    """Summary generation request"""
    page_id: str = Field(..., description="Unique identifier for the content page")
    content: str = Field(..., description="Full page content text", max_length=50000)
    
    class Config:
        json_schema_extra = {
            "example": {
                "page_id": "01-introducing-ai-driven-development/01-ai-development-revolution/06-autonomous-agent-era",
                "content": "This chapter explores the autonomous agent era..."
            }
        }
