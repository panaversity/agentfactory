from fastapi import FastAPI, HTTPException
from fastapi.responses import StreamingResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import os
from dotenv import load_dotenv
import google.generativeai as genai
import asyncio

# Load API key from .env file
load_dotenv()

# Configure google.generativeai with the API key
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
if not GEMINI_API_KEY:
    print("WARNING: GEMINI_API_KEY not found in environment variables.")
else:
    genai.configure(api_key=GEMINI_API_KEY)

# --- Data Models ---
class ChatRequest(BaseModel):
    message: str
    history: list = []

class SummarizeRequest(BaseModel):
    text: str

class TranslateRequest(BaseModel):
    text: str
    language: str

# --- FastAPI App Initialization ---
app = FastAPI()

# CORS Middleware to allow frontend to call this API
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, restrict this to your frontend's URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- API Endpoints ---
@app.get("/")
def read_root():
    return {"Status": "Backend is running"}

async def stream_translation(text: str, language: str):
    """
    An async generator that yields translated text chunks.
    """
    try:
        model = genai.GenerativeModel('gemini-2.5-flash')
        prompt = f"""You are an expert translator. Your task is to translate the following text into {language}.
Do not provide any explanations, annotations, or the original text.
Only output the translated text.

Original Text:
---
{text}
---

Translated Text:
"""
        response = model.generate_content(prompt, stream=True)
        for chunk in response:
            yield chunk.text
            await asyncio.sleep(0.1)  # Small delay to simulate streaming)
    except Exception as e:
        print(f"Error during streaming: {e}")
        yield f"Error: {e}"

@app.post("/api/translate")
async def translate(request: TranslateRequest):
    """
    Receives text and a target language, and streams the translation.
    """
    print(f"\n--- [INFO] Received translation request for language: {request.language} ---")
    return StreamingResponse(stream_translation(request.text, request.language), media_type="text/plain")

async def stream_summary(text: str):
    """
    An async generator that yields summary text chunks.
    """
    try:
        model = genai.GenerativeModel('gemini-2.5-flash')
        prompt = f"Summarize the following text in a concise and easy-to-understand way:\n\n{text}"
        response = model.generate_content(prompt, stream=True)
        for chunk in response:
            yield chunk.text
            await asyncio.sleep(0.1)  # Small delay to simulate streaming
    except Exception as e:
        print(f"Error during streaming: {e}")
        yield f"Error: {e}"

@app.post("/api/summarize")
async def summarize(request: SummarizeRequest):
    """
    Receives text and streams the summary.
    """
    print("\n--- [INFO] Received summarization request ---")
    return StreamingResponse(stream_summary(request.text), media_type="text/plain")

@app.get("/api/models")
async def list_available_models():
    """
    Lists all available Gemini models for the configured API key.
    """
    if not GEMINI_API_KEY:
        raise HTTPException(status_code=401, detail="GEMINI_API_KEY not configured.")
    
    try:
        models = []
        for m in genai.list_models():
            models.append({
                "name": m.name,
                "display_name": m.display_name,
                "supported_generation_methods": m.supported_generation_methods,
                "input_token_limit": m.input_token_limit,
                "output_token_limit": m.output_token_limit,
            })
        return {"available_models": models}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to list models: {str(e)}")

@app.post("/api/chat")
async def chat(request: ChatRequest):
    """
    Receives a chat message, gets a response from Gemini via google-generativeai,
    and returns the response.
    """
    # For direct google-generativeai, history needs to be formatted differently
    # For simplicity in debugging, we'll just send the latest message for now.
    # A full chat history implementation would require converting LiteLLM's format
    # to genai's format (e.g., {'role': 'user', 'parts': [{'text': '...'}]})
    
    print("\n--- [INFO] Received request, attempting to call google-generativeai directly with model 'gemini-2.5-flash' ---")
    
    try:
        model = genai.GenerativeModel('gemini-2.5-flash')
        chat_session = model.start_chat(history=[]) # Start a new session for each request for simplicity
        
        response = chat_session.send_message(request.message) # Removed await
        
        ai_reply = response.text
        print(f"--- [SUCCESS] google-generativeai call successful. ---")
        return {"reply": ai_reply}
    
    except Exception as e:
        print(f"\n!!!!!!!! [ERROR] GOOGLE-GENERATIVEAI FAILED !!!!!!!!")
        print(f"Error Type: {type(e).__name__}")
        print(f"Error Details: {e}")
        print(f"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n")
        # Raise a proper HTTP exception that the frontend can catch
        raise HTTPException(status_code=500, detail=str(e))

# To run this app:
# 1. Make sure you have a .env file with your GEMINI_API_KEY.
# 2. Run in your terminal: uvicorn main:app --reload --port 8000
