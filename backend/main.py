from fastapi import FastAPI, Request
from fastapi.responses import StreamingResponse
from fastapi.middleware.cors import CORSMiddleware
import asyncio
import os
import sys
import importlib.util
from pathlib import Path

# Add the backend directory to the path
sys.path.append(str(Path(__file__).parent))

from src.api import routes

app = FastAPI()

# Include the routes
app.include_router(routes.router)

# Allow CORS for local development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins for development (change to specific origins in production)
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

async def generate_ai_response_stream(prompt: str, api_key: str):
    # In a real application, you would integrate with OpenAI or Gemini API here
    # For now, simulate streaming a response
    full_response = f"AI is processing your request for: \"{prompt}\". This is a simulated streamed response from the backend using API key: {api_key[:5]}...\n\n"
    words = "The actual AI integration would happen here, calling an LLM like OpenAI GPT or Google Gemini. The response would be broken into chunks and yielded to the client in real-time." \
            "This demonstrates how runner.run_streamed would be used on the server side." \
            "You can configure your API key and custom prompt in the AI Config page." \
            .split(' ')

    for word in words:
        yield word + ' '
        await asyncio.sleep(0.05) # Simulate network delay
    yield "\n\n--- End of simulated response ---"

@app.post("/ai-stream")
async def ai_stream(request: Request):
    data = await request.json()
    highlighted_text = data.get("highlightedText")
    api_key = data.get("apiKey")
    custom_prompt = data.get("customPrompt")

    if not highlighted_text or not api_key:
        return StreamingResponse(iter(["Error: Missing highlighted text or API key."]), media_type="text/event-stream")

    prompt = custom_prompt + " " + highlighted_text if custom_prompt else highlighted_text

    return StreamingResponse(generate_ai_response_stream(prompt, api_key), media_type="text/event-stream")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
