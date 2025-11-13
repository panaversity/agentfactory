from fastapi import APIRouter, HTTPException, Request, BackgroundTasks
from fastapi.responses import JSONResponse, StreamingResponse
import asyncio
import logging
from typing import Dict, Any
import os
import json

from ..services.gemini_service import GeminiService

router = APIRouter()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@router.post("/api/ai/query")
async def ai_query(request: Request):
    """Handle AI query requests with highlighted text."""
    try:
        data = await request.json()
        highlighted_text = data.get("highlighted_text", "")
        query = data.get("query", "")
        api_key = data.get("api_key")  # Get API key from the request
        
        if not query:
            raise HTTPException(status_code=400, detail="Query is required")
        
        # Use provided API key or fall back to environment variable
        if not api_key:
            api_key = os.getenv("GEMINI_API_KEY")
            if not api_key:
                raise HTTPException(status_code=500, detail="No API key available")
        
        gemini_service = GeminiService(api_key=api_key)
        
        # Generate response
        # Structure the query to prompt for comprehensive but simple explanation, implementation, and examples
        structured_query = f"{query}\n\nPlease structure your response in the following format:\n\n1. Explanation: Provide a comprehensive yet simple explanation of the concept. Do not just restate the highlighted text. Explain the underlying principles, why it matters, how it works, and any relevant context. Use simple language but be thorough and detailed. If it's a concept, explain what it is, why it exists, how it works, and when/where to use it. If it's code, explain what each part does and why it's needed.\n2. Implementation: If applicable, provide specific implementation steps, configuration details, or code examples with explanations. If not applicable, write 'No implementation required.'\n3. Examples: Provide up to 3 relevant examples if possible. If no examples can be provided, write 'No examples available.'\n\nHighlighted text: {highlighted_text}"
        response_text = gemini_service.generate_response(structured_query)
        
        # Handle potential None response
        if response_text is None:
            response_text = "The AI service did not return a response. Please try again."
        
        return JSONResponse(content={"response": response_text})
    
    except Exception as e:
        logger.error(f"Error in AI query: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing AI query: {str(e)}")

@router.post("/api/config/gemini-key")
async def set_gemini_key(request: Request):
    """Configure the Gemini API key. For now, this just validates that a key is provided."""
    try:
        data = await request.json()
        api_key = data.get("api_key", "")
        
        if not api_key:
            raise HTTPException(status_code=400, detail="API key is required")
        
        # For now, simply acknowledge the key is provided
        # In a future version, this would store the user's key for their session
        print("API key received and acknowledged")
        return JSONResponse(content={"status": "success"})
    
    except Exception as e:
        logger.error(f"Error setting API key: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error configuring API key: {str(e)}")

@router.get("/api/config/status")
async def get_config_status():
    """Check the status of the API key configuration."""
    try:
        api_key = os.getenv("GEMINI_API_KEY")
        is_configured = bool(api_key)
        
        result = {
            "is_configured": is_configured,
            "is_valid": None,
            "message": None
        }
        
        if is_configured:
            try:
                gemini_service = GeminiService(api_key=api_key)
                is_valid = gemini_service.validate_api_key()
                result["is_valid"] = is_valid
                if not is_valid:
                    result["message"] = "API key is invalid"
            except Exception as e:
                result["is_valid"] = False
                result["message"] = f"Error validating API key: {str(e)}"
        
        return JSONResponse(content=result)
    
    except Exception as e:
        logger.error(f"Error getting config status: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error getting config status: {str(e)}")

@router.get("/health")
async def health_check():
    """Basic health check endpoint."""
    return {"status": "healthy", "service": "Highlight Selection AI Dialog Backend"}

# For streaming response functionality (if needed)
@router.post("/api/ai/query-stream")
async def ai_query_stream(request: Request):
    """Handle AI query requests with streaming response."""
    try:
        data = await request.json()
        highlighted_text = data.get("highlighted_text", "")
        query = data.get("query", "")
        
        if not query:
            raise HTTPException(status_code=400, detail="Query is required")
        
        api_key = os.getenv("GEMINI_API_KEY")
        if not api_key:
            raise HTTPException(status_code=500, detail="API key not configured")
        
        # Generator for streaming response
        async def generate_stream():
            try:
                gemini_service = GeminiService(api_key=api_key)
                full_query = f"{query} {highlighted_text}" if highlighted_text else query
                
                # In a real implementation, this would use the actual OpenAI streaming API
                # For now, we'll simulate streaming by breaking up the response
                response_text = gemini_service.generate_response(full_query)
                
                # Simulate streaming by sending chunks
                words = response_text.split()
                for i, word in enumerate(words):
                    if i > 0:
                        yield f" {word}"
                    else:
                        yield word
                    await asyncio.sleep(0.01)  # Small delay to simulate streaming
            except Exception as e:
                yield f"\n\nError: {str(e)}"
        
        return StreamingResponse(generate_stream(), media_type="text/plain")
    
    except Exception as e:
        logger.error(f"Error in AI query stream: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing AI query: {str(e)}")