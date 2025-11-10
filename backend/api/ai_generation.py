from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import Literal
from backend.services.gemini_service import GeminiService

router = APIRouter()

class SelectionRequest(BaseModel):
    selectionType: Literal["text", "diagram", "code"]
    selectionContent: str
    selectionContext: str | None = None

class AIContentResponse(BaseModel):
    contextOfSelection: str
    basicTheory: str
    instructions: str
    example: str

async def get_gemini_service():
    return GeminiService()

@router.post("/generate-ai-content", response_model=AIContentResponse)
async def generate_ai_content(request: SelectionRequest, gemini_service: GeminiService = Depends(get_gemini_service)):
    try:
        ai_response = await gemini_service.generate_content(
            request.selectionType,
            request.selectionContent,
            request.selectionContext
        )
        return AIContentResponse(**ai_response)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"AI content generation failed: {str(e)}")
