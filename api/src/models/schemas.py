"""
Pydantic schemas for API request/response models
"""
from pydantic import BaseModel, Field, EmailStr
from typing import Optional
from enum import Enum


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


# Personalization schemas (T004-T007)

class ProficiencyLevel(str, Enum):
    """User proficiency levels for programming and AI knowledge"""
    NOVICE = "Novice"
    BEGINNER = "Beginner"
    INTERMEDIATE = "Intermediate"
    EXPERT = "Expert"


class UserProfile(BaseModel):
    """User profile with proficiency levels"""
    name: str = Field(..., min_length=1, max_length=100, description="User's display name")
    email: EmailStr = Field(..., description="Email address (for future SSO)")
    programming_experience: ProficiencyLevel = Field(..., description="Programming proficiency level")
    ai_proficiency: ProficiencyLevel = Field(..., description="AI knowledge proficiency level")


class ProfileLoginRequest(BaseModel):
    """Login request with user profiling"""
    name: str = Field(..., min_length=1, max_length=100, description="User's display name")
    email: EmailStr = Field(..., description="Email address")
    programming_experience: ProficiencyLevel = Field(..., description="Programming proficiency level", alias="programmingExperience")
    ai_proficiency: ProficiencyLevel = Field(..., description="AI proficiency level", alias="aiProficiency")
    
    class Config:
        populate_by_name = True
        json_schema_extra = {
            "example": {
                "name": "Jane Doe",
                "email": "jane@example.com",
                "programmingExperience": "Intermediate",
                "aiProficiency": "Beginner"
            }
        }


class AuthWithProfileResponse(BaseModel):
    """Authentication response with user profile"""
    token: str = Field(..., description="Dummy authentication token")
    expires: str = Field(..., description="Token expiration (e.g., 'session')")
    user: UserProfile = Field(..., description="User profile information")


class PersonalizationRequest(BaseModel):
    """Personalization generation request (query params)"""
    page_id: str = Field(..., description="Unique identifier for the content page")
    content: str = Field(..., min_length=100, max_length=50000, description="Full page content text")
    token: str = Field(..., min_length=1, description="Authentication token")
    programming_level: ProficiencyLevel = Field(..., description="User's programming proficiency")
    ai_level: ProficiencyLevel = Field(..., description="User's AI proficiency")
    
    class Config:
        json_schema_extra = {
            "example": {
                "page_id": "intro-to-ai-agents",
                "content": "AI agents are autonomous systems that can...",
                "token": "dummy_token_abc123",
                "programming_level": "Intermediate",
                "ai_level": "Beginner"
            }
        }
