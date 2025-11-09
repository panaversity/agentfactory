"""
Chat API endpoints.

This module provides:
- Chat with personalized TutorGPT agent
- Real-time Q&A with book content
- Session management and conversation history
- Automatic profile updates (questions asked, time spent, etc.)
"""

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from datetime import datetime
from pydantic import BaseModel, Field
from typing import Optional

from app.database import get_db_session
from app.models.user import User, StudentProfile
from app.api.dependencies import get_current_user
from app.agent.tutor_agent import create_tutor_agent

router = APIRouter(prefix="/api/chat", tags=["chat"])


# Request/Response schemas
class ChatRequest(BaseModel):
    """Chat message request."""
    message: str = Field(..., min_length=1, max_length=2000, description="Student's question or message")
    session_id: Optional[str] = Field(None, description="Session ID for conversation continuity")
    current_chapter: Optional[str] = Field(None, description="Current chapter (if on specific page)")
    current_lesson: Optional[str] = Field(None, description="Current lesson (if on specific page)")


class ChatResponse(BaseModel):
    """Chat message response."""
    response: str = Field(..., description="Agent's response")
    session_id: str = Field(..., description="Session ID for this conversation")
    user_name: str = Field(..., description="Student's name")
    student_level: str = Field(..., description="Student's current level")


class GreetingResponse(BaseModel):
    """Personalized greeting response."""
    greeting: str = Field(..., description="Personalized greeting message")
    user_name: str = Field(..., description="Student's name")
    student_level: str = Field(..., description="Student's level")
    completed_lessons_count: int = Field(..., description="Number of completed lessons")


@router.post("/message", response_model=ChatResponse)
async def send_message(
    request: ChatRequest,
    db: Session = Depends(get_db_session),
    user: User = Depends(get_current_user)
):
    """
    Send a message to the personalized TutorGPT agent.

    This endpoint:
    1. Gets the student's profile from database
    2. Creates a personalized agent with their learning preferences
    3. Sends the message to the agent
    4. Updates student stats (questions asked, last active time)
    5. Returns the agent's response

    Args:
        request: Chat message and optional context
        db: Database session
        user: Current authenticated user

    Returns:
        ChatResponse with agent's answer

    Example:
        POST /api/chat/message
        Headers: Authorization: Bearer <token>
        Body: {"message": "What is Python?", "current_chapter": "04-python"}
    """
    # Get student profile
    profile = db.query(StudentProfile).filter(
        StudentProfile.user_id == user.id
    ).first()

    if not profile:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Student profile not found"
        )

    # Use request context or fall back to profile context
    chapter = request.current_chapter or profile.current_chapter
    lesson = request.current_lesson or profile.current_lesson

    # Create personalized agent
    agent = create_tutor_agent(
        current_chapter=chapter,
        current_lesson=lesson,
        student_level=profile.level,
        student_name=user.name,
        learning_style=profile.learning_style,
        completed_lessons=profile.completed_lessons or [],
        difficulty_topics=profile.difficulty_topics or []
    )

    # Generate session ID if not provided
    session_id = request.session_id or f"{user.id}_session_{datetime.utcnow().timestamp()}"

    # Get response from agent
    try:
        agent_response = await agent.teach(
            student_message=request.message,
            session_id=session_id
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Agent error: {str(e)}"
        )

    # Update student stats
    profile.total_questions_asked = (profile.total_questions_asked or 0) + 1
    profile.last_active_at = datetime.utcnow()

    # Update current context if provided
    if request.current_chapter:
        profile.current_chapter = request.current_chapter
    if request.current_lesson:
        profile.current_lesson = request.current_lesson

    try:
        db.commit()
    except Exception as e:
        db.rollback()
        # Don't fail the request if stats update fails
        print(f"Warning: Failed to update profile stats: {e}")

    return ChatResponse(
        response=agent_response,
        session_id=session_id,
        user_name=user.name,
        student_level=profile.level
    )


@router.get("/greeting", response_model=GreetingResponse)
async def get_greeting(
    db: Session = Depends(get_db_session),
    user: User = Depends(get_current_user)
):
    """
    Get a personalized greeting for the authenticated user.

    This endpoint:
    1. Gets the student's profile
    2. Creates a personalized agent
    3. Generates a warm, encouraging greeting

    Args:
        db: Database session
        user: Current authenticated user

    Returns:
        GreetingResponse with personalized greeting

    Example:
        GET /api/chat/greeting
        Headers: Authorization: Bearer <token>
    """
    # Get student profile
    profile = db.query(StudentProfile).filter(
        StudentProfile.user_id == user.id
    ).first()

    if not profile:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Student profile not found"
        )

    # Create personalized agent
    agent = create_tutor_agent(
        current_chapter=profile.current_chapter,
        current_lesson=profile.current_lesson,
        student_level=profile.level,
        student_name=user.name,
        learning_style=profile.learning_style,
        completed_lessons=profile.completed_lessons or [],
        difficulty_topics=profile.difficulty_topics or []
    )

    # Generate greeting
    greeting = await agent.greet_student()

    # Update last active time
    profile.last_active_at = datetime.utcnow()
    try:
        db.commit()
    except Exception:
        db.rollback()

    return GreetingResponse(
        greeting=greeting,
        user_name=user.name,
        student_level=profile.level,
        completed_lessons_count=len(profile.completed_lessons or [])
    )


@router.get("/status")
async def get_chat_status(
    db: Session = Depends(get_db_session),
    user: User = Depends(get_current_user)
):
    """
    Get current chat/learning status for the user.

    Returns:
        User's profile info and learning stats
    """
    profile = db.query(StudentProfile).filter(
        StudentProfile.user_id == user.id
    ).first()

    if not profile:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Student profile not found"
        )

    return {
        "user_name": user.name,
        "email": user.email,
        "level": profile.level,
        "current_chapter": profile.current_chapter,
        "current_lesson": profile.current_lesson,
        "learning_style": profile.learning_style,
        "total_questions_asked": profile.total_questions_asked or 0,
        "completed_lessons_count": len(profile.completed_lessons or []),
        "difficulty_topics": profile.difficulty_topics or [],
        "last_active_at": profile.last_active_at
    }
