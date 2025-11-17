"""
Authentication API endpoints (dummy implementation)
"""
import logging
import uuid
from fastapi import APIRouter, HTTPException
from src.models.schemas import AuthResponse, ProfileLoginRequest, AuthWithProfileResponse, ErrorResponse, UserProfile

logger = logging.getLogger(__name__)

router = APIRouter()


@router.post("/dummy-login", response_model=AuthResponse)
async def dummy_login():
    """
    Dummy login endpoint for temporary authentication.
    
    Always succeeds and returns a dummy token.
    Future: Replace with SSO-based authentication.
    """
    logger.info("Dummy login request received")
    
    return AuthResponse(
        token="dummy_token_12345",
        expires="session",
        user={
            "id": "dummy_user",
            "name": "Anonymous User"
        }
    )


@router.get("/verify")
async def verify_token(
    token: str = None
):
    """
    Verify authentication token.
    
    For dummy implementation, accepts any token.
    Future: Implement proper JWT validation.
    """
    if not token:
        raise HTTPException(status_code=401, detail="Token required")
    
    # Dummy implementation: accept any token
    logger.info(f"Token verification request (dummy): {token[:20]}...")
    
    return {
        "valid": True,
        "user": {
            "id": "dummy_user",
            "name": "Anonymous User"
        }
    }


@router.post("/dummy-login-with-profile", response_model=AuthWithProfileResponse, responses={
    400: {"model": ErrorResponse, "description": "Validation Error"}
})
async def dummy_login_with_profile(request: ProfileLoginRequest):
    """
    Dummy login endpoint with user profiling.
    
    Accepts user profile (name, email, programming experience, AI proficiency)
    and returns a dummy authentication token with the profile.
    
    Future: Replace with SSO-based authentication (Clerk).
    """
    logger.info(f"Dummy login with profile request: {request.name} ({request.email})")
    
    # T014: Generate dummy token with UUID
    token = f"dummy_token_{uuid.uuid4().hex[:16]}"
    
    # T015: Return AuthWithProfileResponse
    user_profile = UserProfile(
        name=request.name,
        email=request.email,
        programming_experience=request.programming_experience,
        ai_proficiency=request.ai_proficiency
    )
    
    return AuthWithProfileResponse(
        token=token,
        expires="session",
        user=user_profile
    )
