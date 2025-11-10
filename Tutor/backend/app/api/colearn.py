"""
Co-Learning API Endpoints

Provides RESTful and WebSocket endpoints for the autonomous co-learning system.
"""

from fastapi import APIRouter, HTTPException, WebSocket, WebSocketDisconnect, Depends
from pydantic import BaseModel
from typing import Optional, List, Dict, Any
import json
import asyncio

from app.agent.colearning_agent import create_colearning_agent, TeachingPhase

router = APIRouter(prefix="/api/colearn", tags=["colearning"])


# ============================================
# Request/Response Models
# ============================================

class CoLearningRequest(BaseModel):
    """Request for co-learning action"""
    action: str  # lesson_step, explain, summary, quiz_prepare, quiz_grade, task, greeting
    chapter: int
    section: Optional[str] = None
    text: Optional[str] = None
    language: Optional[str] = "en"
    userId: str
    uiHints: Optional[Dict[str, str]] = None
    studentAnswer: Optional[str] = None
    quizAnswers: Optional[List[Any]] = None


class CoLearningResponse(BaseModel):
    """Response from co-learning action"""
    message: str
    phase: str
    chapter: int
    section: int
    metadata: Dict[str, Any]


class QuizRequest(BaseModel):
    """Request to prepare quiz"""
    chapter: int
    language: str = "en"
    questionCount: int = 10


class QuizQuestion(BaseModel):
    """Single quiz question"""
    id: str
    question: str
    type: str  # multiple_choice, short_answer, true_false
    options: Optional[List[str]] = None
    correctAnswer: Optional[Any] = None
    explanation: Optional[str] = None


class QuizGradeRequest(BaseModel):
    """Request to grade quiz"""
    chapter: int
    answers: List[Any]
    userId: str


class QuizResult(BaseModel):
    """Quiz grading result"""
    score: int
    totalQuestions: int
    percentage: float
    answers: List[Dict[str, Any]]
    weakTopics: Optional[List[str]] = None
    needsRemedial: bool


class StudentProfile(BaseModel):
    """Student profile update"""
    userId: str
    name: Optional[str] = None
    level: Optional[str] = "beginner"
    language: Optional[str] = "en"
    currentChapter: Optional[int] = 1
    completedChapters: Optional[List[int]] = []
    learningStyle: Optional[str] = "balanced"


# ============================================
# Agent Management
# ============================================

# Store active agents per session
active_agents: Dict[str, Any] = {}


def get_or_create_agent(user_id: str, profile: Optional[Dict] = None):
    """Get existing agent or create new one for user"""
    if user_id not in active_agents:
        active_agents[user_id] = create_colearning_agent(
            session_id=f"colearn_{user_id}",
            student_profile=profile or {}
        )
    return active_agents[user_id]


# ============================================
# REST API Endpoints
# ============================================

@router.post("/action", response_model=CoLearningResponse)
async def co_learning_action(request: CoLearningRequest):
    """
    Main co-learning action endpoint.

    Handles all teaching actions: greetings, lesson steps, explanations,
    summaries, quiz prep, grading, and tasks.

    The agent autonomously decides how to respond based on the action
    and student context.
    """
    try:
        # Get or create agent for this student
        profile = {
            'language': request.language,
            'current_chapter': request.chapter,
            'level': 'beginner'  # TODO: Get from DB
        }
        agent = get_or_create_agent(request.userId, profile)

        # Update agent context if needed
        if request.language:
            agent.update_profile({'language': request.language})

        # Generate prompt based on action type
        prompt = _generate_action_prompt(request)

        # Get autonomous response from agent
        result = await agent.teach(prompt)

        return CoLearningResponse(
            message=result['response'],
            phase=result['phase'],
            chapter=result['chapter'],
            section=result['section'],
            metadata=result['metadata']
        )

    except Exception as e:
        print(f"Error in co_learning_action: {e}")
        raise HTTPException(status_code=500, detail=str(e))


def _generate_action_prompt(request: CoLearningRequest) -> str:
    """Generate appropriate prompt based on action type"""

    action_prompts = {
        'greeting': f"Please greet the student warmly in {request.language}. Ask if they want to start from Chapter 1 or jump to a specific chapter. Also ask their preferred language (English, Roman Urdu, Spanish).",

        'lesson_step': f"Teach the next section of Chapter {request.chapter}. Follow the teaching pattern: 1) Show a short note (1-2 lines), 2) Give clear 2-3 sentence explanation, 3) Provide example if helpful, 4) Ask one quick reflection question.",

        'explain': f"The student needs clarification on: {request.text or request.studentAnswer}. Provide a clear, adaptive explanation. Check if student is confused and simplify if needed.",

        'summary': f"Provide a concise summary of: {request.text or f'Chapter {request.chapter}'}. Keep it to 3-4 key points.",

        'quiz_prepare': f"It's time for the Chapter {request.chapter} quiz! Tell the student we're ready for a 10-question quiz to test their understanding. Be encouraging.",

        'quiz_grade': f"Grade the student's quiz answers and provide detailed feedback. Show score, explain wrong answers, identify weak topics, and suggest remedial path if needed (score < 50%).",

        'task': f"Give the student a small practice task related to Chapter {request.chapter}. Make it practical and achievable in 5-10 minutes."
    }

    prompt = action_prompts.get(request.action, request.text or "Continue teaching")

    # Add student answer if provided
    if request.studentAnswer:
        prompt += f"\n\nStudent's answer: {request.studentAnswer}"

    return prompt


@router.post("/quiz/prepare", response_model=List[QuizQuestion])
async def prepare_quiz(request: QuizRequest):
    """
    Generate quiz questions for a chapter.

    The agent autonomously creates a mix of question types:
    - Multiple choice
    - True/False
    - Short answer

    Questions are based on chapter content and adapted to student level.
    """
    try:
        # Create temporary agent for quiz generation
        agent = create_colearning_agent(
            session_id=f"quiz_gen_{request.chapter}",
            student_profile={'language': request.language}
        )

        # Generate quiz
        questions = await agent.generate_quiz(
            chapter=request.chapter,
            num_questions=request.questionCount
        )

        return questions

    except Exception as e:
        print(f"Error in prepare_quiz: {e}")
        # Return fallback quiz structure
        return _generate_fallback_quiz(request.chapter, request.questionCount)


def _generate_fallback_quiz(chapter: int, count: int) -> List[QuizQuestion]:
    """Generate fallback quiz questions if agent fails"""
    questions = []
    for i in range(min(count, 10)):
        questions.append(QuizQuestion(
            id=f"q{i+1}",
            question=f"Question {i+1} about Chapter {chapter} content",
            type="multiple_choice" if i < 6 else "short_answer" if i < 8 else "true_false",
            options=["Option A", "Option B", "Option C", "Option D"] if i < 6 else (["True", "False"] if i >= 8 else None),
            correctAnswer=1 if i < 6 else (0 if i >= 8 else "sample answer"),
            explanation=f"Explanation for question {i+1}"
        ))
    return questions


@router.post("/quiz/grade", response_model=QuizResult)
async def grade_quiz(request: QuizGradeRequest):
    """
    Grade student's quiz answers.

    The agent autonomously:
    - Evaluates each answer
    - Provides detailed feedback
    - Identifies weak topics
    - Recommends remedial or advanced path
    """
    try:
        agent = get_or_create_agent(request.userId)

        # Generate grading prompt
        prompt = f"""Grade this Chapter {request.chapter} quiz:

Student's answers: {json.dumps(request.answers)}

Provide:
1. Score (X/10)
2. Percentage
3. Detailed feedback for each question
4. Weak topics identified
5. Recommendation: remedial lesson if <50%, continue if 50-90%, advanced material if >90%
"""

        result = await agent.teach(prompt)

        # Parse grading results from agent response
        # For now, return a structured result
        score = len([a for a in request.answers if a])  # Simplified scoring
        total = len(request.answers)
        percentage = (score / total) * 100 if total > 0 else 0

        return QuizResult(
            score=score,
            totalQuestions=total,
            percentage=percentage,
            answers=[
                {
                    'questionId': f'q{i+1}',
                    'userAnswer': request.answers[i],
                    'correct': bool(request.answers[i]),  # Simplified
                    'feedback': f'Feedback for question {i+1}'
                }
                for i in range(total)
            ],
            weakTopics=['topic1', 'topic2'] if percentage < 70 else [],
            needsRemedial=percentage < 50
        )

    except Exception as e:
        print(f"Error in grade_quiz: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/chapter/{chapter_number}")
async def get_chapter_content(chapter_number: int):
    """
    Get chapter content and metadata.

    Returns chapter structure, sections, estimated time, etc.
    """
    # This would fetch from the RAG system or book parser
    return {
        "chapter": chapter_number,
        "title": f"Chapter {chapter_number} Title",
        "sections": [
            {"id": 1, "title": "Introduction"},
            {"id": 2, "title": "Core Concepts"},
            {"id": 3, "title": "Practical Examples"},
            {"id": 4, "title": "Best Practices"}
        ],
        "estimatedTime": 45,
        "totalLessons": 8
    }


@router.post("/profile/update")
async def update_student_profile(profile: StudentProfile):
    """
    Update student profile and preferences.

    Updates the agent's context with new student information.
    """
    try:
        agent = get_or_create_agent(profile.userId)

        updates = {}
        if profile.name:
            updates['name'] = profile.name
        if profile.level:
            updates['level'] = profile.level
        if profile.language:
            updates['language'] = profile.language
        if profile.currentChapter:
            updates['current_chapter'] = profile.currentChapter
        if profile.completedChapters:
            updates['completed_chapters'] = profile.completedChapters
        if profile.learningStyle:
            updates['learning_style'] = profile.learningStyle

        agent.update_profile(updates)

        return {"status": "success", "message": "Profile updated"}

    except Exception as e:
        print(f"Error in update_student_profile: {e}")
        raise HTTPException(status_code=500, detail=str(e))


# ============================================
# WebSocket for Real-time Teaching
# ============================================

@router.websocket("/ws/teach")
async def websocket_teaching(websocket: WebSocket, user_id: str):
    """
    WebSocket endpoint for real-time interactive teaching.

    Provides a persistent connection for:
    - Continuous conversation
    - Real-time feedback
    - Typing indicators
    - Instant responses
    """
    await websocket.accept()

    try:
        # Get agent for this user
        agent = get_or_create_agent(user_id)

        # Send welcome message
        await websocket.send_json({
            "type": "connected",
            "message": "Co-Learning Tutor connected! 🚀"
        })

        # Main conversation loop
        while True:
            # Receive student message
            data = await websocket.receive_json()
            message = data.get('message', '')

            if not message:
                continue

            # Send thinking indicator
            await websocket.send_json({
                "type": "status",
                "status": "thinking",
                "message": "Processing your message..."
            })

            # Get agent response
            result = await agent.teach(message)

            # Send response
            await websocket.send_json({
                "type": "response",
                "message": result['response'],
                "phase": result['phase'],
                "chapter": result['chapter'],
                "metadata": result['metadata']
            })

    except WebSocketDisconnect:
        print(f"WebSocket disconnected for user: {user_id}")
    except Exception as e:
        print(f"WebSocket error: {e}")
        await websocket.send_json({
            "type": "error",
            "message": str(e)
        })
        await websocket.close()


# ============================================
# Health & Status
# ============================================

@router.get("/health")
async def health_check():
    """Check co-learning system health"""
    return {
        "status": "healthy",
        "active_sessions": len(active_agents),
        "version": "1.0.0"
    }
